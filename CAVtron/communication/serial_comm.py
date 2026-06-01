"""
communication/serial_comm.py
=============================
Interface de comunicação serial UART para o RPi Zero 2 W.

Este módulo serve dois propósitos distintos, selecionáveis por configuração:

MODO 1 — Controle externo via USB/UART (modo debug/PC):
    O PC conectado via USB envia comandos ao RPi pelo mesmo protocolo
    framed com CRC-8. Útil para:
    - Testes em bancada sem controle autônomo ativo
    - Teleoperação manual durante desenvolvimento
    - Monitoramento de telemetria em tempo real

MODO 2 — Comunicação com microcontrolador externo:
    Se o RPi Zero 2W for usado em conjunto com um Arduino (para tarefas
    de controle de tempo real mais exigentes), este módulo implementa
    o protocolo completo de comunicação bidirecional com CRC-8 — o mesmo
    protocolo do firmware Arduino original.

Arquitetura de threading:
    Thread de leitura (daemon): lê bytes continuamente da porta serial e
    alimenta uma fila thread-safe (queue.Queue), desacoplando a recepção
    do processamento de alto nível.

    Loop principal: consome a fila via process_queue() e despacha
    callbacks registrados por tipo de mensagem.

Portas UART no RPi Zero 2 W:
    /dev/ttyS0     → UART nativo GPIO (pinos TX=GPIO14/pin8, RX=GPIO15/pin10)
                     Requer: enable_uart=1 em /boot/config.txt
                     Requer: console serial desabilitada (raspi-config)
    /dev/ttyAMA0   → UART primário (pode estar em uso pelo Bluetooth)
    /dev/ttyUSB0   → Adaptador USB-Serial (mais simples, sem conflitos)

Recomendação: usar /dev/ttyUSB0 com adaptador CP2102/CH340 para PC,
e /dev/ttyS0 para comunicação GPIO com Arduino.
"""

import serial
import threading
import queue
import time
import logging
from typing import Callable, Dict, List, Optional

from .protocol import (
    Message, FrameParser, crc8_str,
    build_frame, build_move, build_stop, build_grip,
    build_lift, build_state, build_ping, build_telemetry,
    FRAME_START, FRAME_END
)

logger = logging.getLogger(__name__)


# ============================================================
#  CLASSE PRINCIPAL
# ============================================================

class SerialComm:
    """
    Gerenciador de comunicação serial com protocolo framed + CRC-8.

    Suporta comunicação com PC (modo debug) ou microcontrolador externo
    (modo ponte Arduino) através do mesmo protocolo.

    Args:
        port         : Porta serial (ex: '/dev/ttyUSB0', '/dev/ttyS0')
        baudrate     : Taxa de transmissão (padrão: 115200 baud)
        timeout_s    : Timeout de leitura bloqueante em segundos
        watchdog_s   : Intervalo máximo sem RX antes de alarme de watchdog
        mode         : 'debug' (PC) | 'bridge' (Arduino) | 'monitor' (só leitura)
    """

    MODES = ('debug', 'bridge', 'monitor')

    def __init__(self,
                 port: str       = '/dev/ttyUSB0',
                 baudrate: int   = 115200,
                 timeout_s: float = 1.0,
                 watchdog_s: float = 3.0,
                 mode: str       = 'debug'):
        if mode not in self.MODES:
            raise ValueError(f"mode deve ser um de {self.MODES}, recebido: '{mode}'")

        self._port       = port
        self._baudrate   = baudrate
        self._timeout_s  = timeout_s
        self._watchdog_s = watchdog_s
        self._mode       = mode

        self._serial: Optional[serial.Serial] = None
        self._rx_queue: "queue.Queue[Message]" = queue.Queue(maxsize=256)
        self._callbacks: Dict[str, List[Callable[[Message], None]]] = {}
        self._parser = FrameParser()

        self._last_rx_time: float = time.monotonic()
        self._connected: bool     = False
        self._tx_lock             = threading.Lock()

        self._rx_thread = threading.Thread(
            target=self._rx_loop,
            name=f'SerialRx-{port.split("/")[-1]}',
            daemon=True
        )

        # Estatísticas de operação
        self._stats = {
            'frames_rx'    : 0,
            'frames_tx'    : 0,
            'crc_errors'   : 0,
            'rx_overflows' : 0,
        }

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
                stopbits=serial.STOPBITS_ONE,
                xonxoff=False,
                rtscts=False
            )
            # Aguarda estabilização do UART (especialmente após reset Arduino)
            time.sleep(0.1)
            self._serial.reset_input_buffer()
            self._serial.reset_output_buffer()

            self._connected    = True
            self._last_rx_time = time.monotonic()
            self._rx_thread.start()

            logger.info(
                f"[SerialComm] Conectado: {self._port} @ {self._baudrate} baud "
                f"| modo={self._mode}"
            )
            return True

        except serial.SerialException as e:
            logger.error(f"[SerialComm] Falha ao abrir {self._port}: {e}")
            return False

    def disconnect(self):
        """Encerra a comunicação serial de forma segura."""
        self._connected = False
        if self._serial and self._serial.is_open:
            try:
                self._serial.close()
            except Exception:
                pass
        logger.info(f"[SerialComm] Desconectado. Stats: {self._stats}")

    # ----------------------------------------------------------
    #  REGISTRO DE CALLBACKS
    # ----------------------------------------------------------

    def register_callback(self, msg_type: str,
                          callback: Callable[[Message], None]):
        """
        Registra função de callback para um tipo de mensagem.

        O callback é invocado na thread principal via process_queue().
        Múltiplos callbacks podem ser registrados para o mesmo tipo.

        Args:
            msg_type : Tipo da mensagem em maiúsculas (ex: 'SENSOR', 'PONG')
            callback : Função que recebe Message como argumento único
        """
        key = msg_type.upper()
        self._callbacks.setdefault(key, []).append(callback)
        logger.debug(f"[SerialComm] Callback registrado para '{key}'")

    def unregister_callbacks(self, msg_type: str):
        """Remove todos os callbacks de um tipo de mensagem."""
        self._callbacks.pop(msg_type.upper(), None)

    # ----------------------------------------------------------
    #  PROCESSAMENTO DA FILA (LOOP PRINCIPAL)
    # ----------------------------------------------------------

    def process_queue(self, max_messages: int = 32):
        """
        Consome mensagens da fila e despacha callbacks registrados.

        Deve ser chamado periodicamente no loop principal.
        Processa no máximo max_messages por chamada para não
        monopolizar o ciclo de controle.

        Args:
            max_messages: Limite de mensagens processadas por invocação.
        """
        for _ in range(max_messages):
            try:
                msg: Message = self._rx_queue.get_nowait()
                self._dispatch(msg)
            except queue.Empty:
                break

    def _dispatch(self, msg: Message):
        """Despacha mensagem para os callbacks registrados."""
        callbacks = self._callbacks.get(msg.msg_type, [])
        if not callbacks:
            logger.debug(f"[SerialComm] Sem callback para '{msg.msg_type}'")
            return

        for cb in callbacks:
            try:
                cb(msg)
            except Exception as e:
                logger.error(
                    f"[SerialComm] Erro no callback '{msg.msg_type}': {e}",
                    exc_info=True
                )

    # ----------------------------------------------------------
    #  ENVIO DE COMANDOS
    # ----------------------------------------------------------

    def send_move(self, direction: str, speed: int):
        """
        Envia comando de movimento.

        Args:
            direction : 'FWD' | 'BWD' | 'ROT_L' | 'ROT_R' | 'STOP'
            speed     : Velocidade PWM [0–255]
        """
        self._send_raw(build_move(direction, speed))

    def send_stop(self):
        """Para todos os motores imediatamente."""
        self._send_raw(build_stop())

    def send_grip(self, action: str):
        """
        Envia comando de garra.

        Args:
            action : 'OPEN' | 'CLOSE' | 'CAPTURE'
        """
        self._send_raw(build_grip(action))

    def send_lift(self, action: str):
        """
        Envia comando de braço.

        Args:
            action : 'UP' | 'DOWN'
        """
        self._send_raw(build_lift(action))

    def send_state(self, state_id: int):
        """Envia comando de mudança de estado."""
        self._send_raw(build_state(state_id))

    def send_ping(self):
        """Envia PING para verificar conectividade."""
        self._send_raw(build_ping())

    def send_telemetry(self, state: str, line_pos: float,
                       pid_out: float, victim: bool):
        """
        Envia frame de telemetria (para monitoramento externo).

        Args:
            state    : Nome do estado atual
            line_pos : Posição da linha [−1.0, +1.0]
            pid_out  : Saída do PID
            victim   : True se vítima detectada
        """
        self._send_raw(build_telemetry(state, line_pos, pid_out, victim))

    def send_raw_frame(self, content: str):
        """
        Envia um frame arbitrário formatado com CRC.

        Útil para comandos customizados não cobertos pelos métodos acima.

        Args:
            content: Conteúdo sem delimitadores e sem CRC.
                     Exemplo: "CUSTOM,PARAM1,PARAM2"
        """
        self._send_raw(build_frame(content))

    # ----------------------------------------------------------
    #  DIAGNÓSTICO
    # ----------------------------------------------------------

    def is_watchdog_expired(self) -> bool:
        """
        Verifica se o watchdog de comunicação expirou.

        Returns:
            True se não houve RX válido por mais de watchdog_s segundos.
        """
        return (time.monotonic() - self._last_rx_time) > self._watchdog_s

    @property
    def connected(self) -> bool:
        return self._connected and bool(self._serial and self._serial.is_open)

    @property
    def mode(self) -> str:
        return self._mode

    @property
    def stats(self) -> dict:
        """Retorna cópia das estatísticas de operação."""
        return dict(self._stats)

    @property
    def queue_size(self) -> int:
        """Número de mensagens aguardando na fila."""
        return self._rx_queue.qsize()

    # ----------------------------------------------------------
    #  THREAD DE RECEPÇÃO (DAEMON)
    # ----------------------------------------------------------

    def _rx_loop(self):
        """
        Loop de leitura serial em thread daemon.

        Usa o FrameParser incremental para processar bytes um a um,
        sem bloqueio e sem risco de perda de dados entre leituras.
        Equivalente à FSM de parsing do serial_comm.cpp do Arduino.
        """
        logger.debug(f"[SerialRx] Thread iniciada para {self._port}.")

        while self._connected:
            try:
                if not (self._serial and self._serial.is_open):
                    time.sleep(0.05)
                    continue

                # Lê até 64 bytes por iteração (batch para melhor throughput)
                data = self._serial.read(64)
                if not data:
                    continue

                for byte in data:
                    msg = self._parser.feed(byte)
                    if msg is None:
                        continue

                    # Frame válido recebido
                    self._last_rx_time = time.monotonic()
                    self._stats['frames_rx'] += 1

                    try:
                        self._rx_queue.put_nowait(msg)
                    except queue.Full:
                        self._stats['rx_overflows'] += 1
                        logger.warning("[SerialComm] Fila RX cheia — frame descartado.")

            except serial.SerialException as e:
                logger.error(f"[SerialRx] Erro serial: {e}")
                time.sleep(0.1)

            except Exception as e:
                logger.error(f"[SerialRx] Exceção inesperada: {e}", exc_info=True)
                time.sleep(0.1)

        logger.debug("[SerialRx] Thread encerrada.")

    # ----------------------------------------------------------
    #  ENVIO INTERNO
    # ----------------------------------------------------------

    def _send_raw(self, frame: bytes):
        """
        Envia bytes brutos pela porta serial com lock de thread.

        Args:
            frame: Bytes do frame completo (incluindo delimitadores e CRC).
        """
        if not self.connected:
            logger.warning("[SerialComm] Tentativa de envio sem conexão ativa.")
            return

        with self._tx_lock:
            try:
                self._serial.write(frame)
                self._stats['frames_tx'] += 1
                logger.debug(f"[SerialComm] TX: {frame.decode('ascii').strip()}")
            except serial.SerialException as e:
                logger.error(f"[SerialComm] Erro ao enviar: {e}")


# ============================================================
#  MONITOR SERIAL SIMPLES (modo somente leitura)
# ============================================================

class SerialMonitor:
    """
    Monitor serial passivo — exibe frames recebidos sem processamento.

    Útil para debug em bancada sem lógica de negócio acoplada.
    Usa SerialComm internamente em modo 'monitor'.

    Uso:
        monitor = SerialMonitor('/dev/ttyUSB0')
        monitor.start()
        # ... aguarda Ctrl+C
        monitor.stop()
    """

    def __init__(self, port: str = '/dev/ttyUSB0', baudrate: int = 115200):
        self._comm = SerialComm(port, baudrate, mode='monitor')
        self._running = False

    def start(self):
        """Inicia o monitor e imprime todos os frames recebidos."""
        if not self._comm.connect():
            logger.error("[SerialMonitor] Falha ao conectar.")
            return

        # Registra callback genérico para todos os tipos
        for msg_type in ('SENSOR', 'STATE_CHG', 'ACK', 'PONG', 'ERROR',
                         'MOVE', 'STOP', 'GRIP', 'LIFT', 'STATE', 'PING'):
            self._comm.register_callback(
                msg_type,
                lambda m: print(f"[RX] {m.msg_type} | params={m.params} | t={m.timestamp:.3f}")
            )

        self._running = True
        logger.info("[SerialMonitor] Monitorando... (Ctrl+C para parar)")

        try:
            while self._running:
                self._comm.process_queue()
                time.sleep(0.01)
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()

    def stop(self):
        self._running = False
        self._comm.disconnect()
        logger.info("[SerialMonitor] Parado.")
