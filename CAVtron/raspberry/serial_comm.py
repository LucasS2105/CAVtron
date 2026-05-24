"""
communication/serial_comm.py
============================
Protocolo de comunicação serial bidirecional com o Arduino.

Implementa o mesmo protocolo framed com CRC-8 definido na camada Arduino,
garantindo interoperabilidade direta sem camada de adaptação.

Formato do frame:
    <TIPO,PARAM1,...,PARAMN,HEXCRC>

Arquitetura de threading:
    Thread de leitura dedicada (daemon) lê bytes continuamente e
    alimenta uma fila thread-safe (queue.Queue). A thread principal
    consome a fila e despacha callbacks registrados, garantindo que
    a recepção serial não seja bloqueada por processamento de alto nível.

Referência CRC-8:
    Polinômio 0x07 (Dallas/Maxim) — idêntico ao implementado no Arduino.
"""

import serial
import threading
import queue
import time
import logging
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional

logger = logging.getLogger(__name__)


# ============================================================
#  ESTRUTURAS DE DADOS
# ============================================================

@dataclass
class ParsedMessage:
    """Mensagem decodificada e validada pelo parser serial."""
    msg_type: str = ''
    params: List[str] = field(default_factory=list)
    received_crc: int = 0
    calculated_crc: int = 0
    valid: bool = False
    timestamp: float = field(default_factory=time.monotonic)


@dataclass
class SensorData:
    """Dados de sensores recebidos do Arduino via protocolo SENSOR."""
    line_position: float = 0.0      # [-1.0, +1.0]
    line_detected: bool = False
    victim_detected: bool = False
    victim_x: float = 0.0           # [-1.0, +1.0] posição normalizada
    victim_y: float = 0.0           # [-1.0, +1.0] posição normalizada
    timestamp: float = field(default_factory=time.monotonic)


# ============================================================
#  CRC-8 (polinômio 0x07, Dallas/Maxim)
# ============================================================

def crc8(data: bytes) -> int:
    """
    Calcula CRC-8 com polinômio gerador x^8 + x^2 + x + 1 (0x07).

    Algoritmo bit-a-bit idêntico ao firmware Arduino. Sem lookup table
    para reduzir footprint de memória e garantir portabilidade.

    Args:
        data: Bytes sobre os quais calcular o CRC.
    Returns:
        Byte de CRC calculado [0-255].
    """
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


def crc8_str(s: str) -> int:
    """Wrapper de crc8 para strings ASCII."""
    return crc8(s.encode('ascii'))


# ============================================================
#  CLASSE PRINCIPAL
# ============================================================

class SerialComm:
    """
    Gerenciador do protocolo serial Raspberry Pi <-> Arduino.

    Args:
        port       : Porta serial (ex: '/dev/serial0', '/dev/ttyUSB0')
        baudrate   : Taxa de transmissão (deve coincidir com Arduino)
        timeout_s  : Timeout de leitura serial em segundos
        watchdog_s : Intervalo máximo sem mensagem antes de alarme
    """

    MSG_START = ord('<')
    MSG_END   = ord('>')
    MSG_SEP   = ','

    def __init__(self,
                 port: str = '/dev/serial0',
                 baudrate: int = 115200,
                 timeout_s: float = 1.0,
                 watchdog_s: float = 2.0):
        self._port       = port
        self._baudrate   = baudrate
        self._timeout_s  = timeout_s
        self._watchdog_s = watchdog_s

        self._serial: Optional[serial.Serial] = None
        self._rx_queue: "queue.Queue[ParsedMessage]" = queue.Queue(maxsize=128)
        self._callbacks: Dict[str, List[Callable]] = {}
        self._rx_buffer = bytearray()
        self._reading   = False

        self._last_rx_time: float = time.monotonic()
        self._connected: bool     = False
        self._tx_lock = threading.Lock()

        self._rx_thread = threading.Thread(
            target=self._rx_loop,
            name='SerialRxThread',
            daemon=True
        )

    # ----------------------------------------------------------
    #  CICLO DE VIDA
    # ----------------------------------------------------------

    def connect(self) -> bool:
        """
        Abre a porta serial e inicia a thread de recepção.

        Returns:
            True se conexão estabelecida com sucesso.
        """
        try:
            self._serial = serial.Serial(
                port=self._port,
                baudrate=self._baudrate,
                timeout=self._timeout_s,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            time.sleep(0.1)   # Aguarda estabilização do UART
            self._serial.reset_input_buffer()
            self._connected    = True
            self._last_rx_time = time.monotonic()
            self._rx_thread.start()
            logger.info(f"[SerialComm] Conectado em {self._port} @ {self._baudrate} baud")
            return True
        except serial.SerialException as e:
            logger.error(f"[SerialComm] Falha ao abrir porta: {e}")
            return False

    def disconnect(self):
        """Encerra a comunicação serial de forma segura."""
        self._connected = False
        if self._serial and self._serial.is_open:
            self._serial.close()
        logger.info("[SerialComm] Desconectado.")

    # ----------------------------------------------------------
    #  REGISTRO DE CALLBACKS
    # ----------------------------------------------------------

    def register_callback(self, msg_type: str,
                          callback: Callable[["ParsedMessage"], None]):
        """
        Registra callback para um tipo de mensagem.

        O callback é invocado na thread principal via process_queue().
        Múltiplos callbacks podem ser registrados para o mesmo tipo.

        Args:
            msg_type : Tipo da mensagem em maiúsculas (ex: 'SENSOR')
            callback : Função que recebe ParsedMessage como argumento
        """
        self._callbacks.setdefault(msg_type, []).append(callback)
        logger.debug(f"[SerialComm] Callback registrado: '{msg_type}'")

    # ----------------------------------------------------------
    #  PROCESSAMENTO DA FILA (LOOP PRINCIPAL)
    # ----------------------------------------------------------

    def process_queue(self, max_messages: int = 20):
        """
        Consome mensagens da fila e despacha callbacks.

        Deve ser chamado periodicamente no loop principal.
        Limita max_messages por chamada para não monopolizar CPU.

        Args:
            max_messages: Limite de mensagens processadas por invocação.
        """
        for _ in range(max_messages):
            try:
                msg = self._rx_queue.get_nowait()
                self._dispatch(msg)
            except queue.Empty:
                break

    def _dispatch(self, msg: ParsedMessage):
        for cb in self._callbacks.get(msg.msg_type, []):
            try:
                cb(msg)
            except Exception as e:
                logger.error(f"[SerialComm] Erro no callback '{msg.msg_type}': {e}")

    # ----------------------------------------------------------
    #  ENVIO DE COMANDOS
    # ----------------------------------------------------------

    def send_move(self, direction: str, speed: int):
        """
        Envia comando de movimento ao Arduino.

        Args:
            direction : 'FWD' | 'BWD' | 'ROT_L' | 'ROT_R' | 'STOP'
            speed     : Velocidade PWM [0-255]
        """
        self._send_frame(f"MOVE,{direction},{int(speed)}")

    def send_stop(self):
        """Para todos os motores imediatamente."""
        self._send_frame("STOP")

    def send_grip(self, action: str):
        """
        Envia comando de acionamento da garra.

        Args:
            action : 'OPEN' | 'CLOSE' | 'CAPTURE'
        """
        self._send_frame(f"GRIP,{action}")

    def send_lift(self, action: str):
        """
        Envia comando de elevação do braço.

        Args:
            action : 'UP' | 'DOWN'
        """
        self._send_frame(f"LIFT,{action}")

    def send_state(self, state_id: int):
        """Comanda adoção de novo estado FSM no Arduino."""
        self._send_frame(f"STATE,{state_id}")

    def send_ping(self):
        """Envia PING para verificar conectividade e resetar watchdog."""
        self._send_frame("PING")

    # ----------------------------------------------------------
    #  DIAGNÓSTICO
    # ----------------------------------------------------------

    def is_watchdog_expired(self) -> bool:
        """Retorna True se não houve RX válido por mais de watchdog_s."""
        return (time.monotonic() - self._last_rx_time) > self._watchdog_s

    def is_connected(self) -> bool:
        return self._connected and self._serial is not None and self._serial.is_open

    # ----------------------------------------------------------
    #  THREAD DE RECEPÇÃO (DAEMON)
    # ----------------------------------------------------------

    def _rx_loop(self):
        """
        Loop de leitura serial em thread daemon.

        FSM de parsing incremental:
          Aguarda '<' -> acumula bytes -> ao receber '>' processa frame.
        """
        logger.debug("[SerialRxThread] Iniciada.")
        while self._connected:
            try:
                if not (self._serial and self._serial.is_open):
                    time.sleep(0.05)
                    continue

                raw_byte = self._serial.read(1)
                if not raw_byte:
                    continue

                b = raw_byte[0]

                if b == self.MSG_START:
                    self._rx_buffer.clear()
                    self._reading = True
                elif b == self.MSG_END and self._reading:
                    self._reading = False
                    raw = self._rx_buffer.decode('ascii', errors='ignore')
                    msg = self._parse(raw)
                    if msg.valid:
                        self._last_rx_time = time.monotonic()
                        try:
                            self._rx_queue.put_nowait(msg)
                        except queue.Full:
                            logger.warning("[SerialComm] Fila RX cheia.")
                elif self._reading:
                    if len(self._rx_buffer) < 128:
                        self._rx_buffer.append(b)
                    else:
                        self._rx_buffer.clear()
                        self._reading = False
                        logger.warning("[SerialComm] Buffer overflow.")

            except serial.SerialException as e:
                logger.error(f"[SerialRxThread] Erro serial: {e}")
                time.sleep(0.1)
            except Exception as e:
                logger.error(f"[SerialRxThread] Exceção: {e}")
                time.sleep(0.1)

    # ----------------------------------------------------------
    #  PARSER
    # ----------------------------------------------------------

    def _parse(self, raw: str) -> ParsedMessage:
        """
        Faz o parse do conteúdo de um frame recebido.

        1. Encontra o último ',' (separador antes do campo CRC)
        2. Extrai e converte o CRC hexadecimal
        3. Valida CRC sobre o conteúdo anterior
        4. Tokeniza tipo e parâmetros

        Args:
            raw: String entre '<' e '>' sem os delimitadores.
        Returns:
            ParsedMessage com valid=True se CRC confere.
        """
        msg = ParsedMessage()
        last_comma = raw.rfind(self.MSG_SEP)
        if last_comma == -1:
            return msg

        content = raw[:last_comma]
        crc_str = raw[last_comma + 1:].strip()

        try:
            msg.received_crc = int(crc_str, 16)
        except ValueError:
            return msg

        msg.calculated_crc = crc8_str(content)
        if msg.received_crc != msg.calculated_crc:
            logger.warning(
                f"[SerialComm] CRC mismatch | "
                f"rx=0x{msg.received_crc:02X} calc=0x{msg.calculated_crc:02X}"
            )
            return msg

        tokens = content.split(self.MSG_SEP)
        if not tokens:
            return msg

        msg.msg_type = tokens[0].strip().upper()
        msg.params   = [t.strip() for t in tokens[1:]]
        msg.valid    = True
        return msg

    # ----------------------------------------------------------
    #  ENVIO INTERNO
    # ----------------------------------------------------------

    def _send_frame(self, content: str):
        """
        Formata e envia frame com CRC ao Arduino.

        Formato: <CONTENT,HEXCRC>\n
        """
        if not self.is_connected():
            logger.warning("[SerialComm] Envio sem conexão ativa.")
            return

        crc   = crc8_str(content)
        frame = f"<{content},{crc:02X}>\n".encode('ascii')
        with self._tx_lock:
            try:
                self._serial.write(frame)
                logger.debug(f"[SerialComm] TX: {frame.decode().strip()}")
            except serial.SerialException as e:
                logger.error(f"[SerialComm] Erro ao enviar: {e}")
