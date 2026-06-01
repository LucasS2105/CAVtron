"""
communication/protocol.py
==========================
Definições do protocolo de comunicação serial bidirecional.

Este módulo centraliza TODAS as constantes, tipos e funções do protocolo,
garantindo que qualquer futuro módulo de transporte (UART, WiFi, Bluetooth)
use a mesma estrutura de mensagens sem duplicação de código.

Formato do frame (espelho exato do protocolo C++ do Arduino):
    <TIPO,PARAM1,...,PARAMN,HEXCRC>\n

Onde:
    < >       = delimitadores de início e fim de frame
    TIPO      = string identificadora do comando (ex: "MOVE", "SENSOR")
    HEXCRC    = CRC-8 em hexadecimal (2 dígitos) sobre o conteúdo
    \n        = terminador de linha (LF)

Exemplos de frames válidos:
    Saída (RPi → destino):  <MOVE,FWD,150,A3>
    Saída (RPi → destino):  <GRIP,CAPTURE,7E>
    Entrada (destino → RPi): <SENSOR,-0.25,1,1,0.45,0.12,B7>
    Entrada (destino → RPi): <STATE_CHG,2,C1>
    Entrada (destino → RPi): <ERROR,4,09>

CRC-8, polinômio 0x07 (Dallas/Maxim):
    Calculado sobre o conteúdo entre '<' e a última ',' (excluindo o CRC).
    Ex: frame <MOVE,FWD,150,A3> → CRC calculado sobre "MOVE,FWD,150".

Referência:
    Williams, R.N. (1993). A Painless Guide to CRC Error Detection Algorithms.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum
from typing import List, Optional
import time


# ============================================================
#  CONSTANTES DO FRAME
# ============================================================

FRAME_START: str = '<'
FRAME_END:   str = '>'
FRAME_SEP:   str = ','
FRAME_TERM:  str = '\n'
FRAME_MAX_LEN: int = 128   # Tamanho máximo do buffer de recepção


# ============================================================
#  TIPOS DE MENSAGEM
# ============================================================

class MsgType(IntEnum):
    """
    Tipos de mensagem do protocolo — espelho de MessageType_t do C++.

    Prefixo CMD_  = comandos enviados pelo RPi (saída)
    Prefixo MSG_  = mensagens recebidas pelo RPi (entrada)
    """
    # --- Comandos de saída (RPi → exterior) ---
    CMD_MOVE      = 0    # <MOVE,DIR,SPEED,CRC>
    CMD_STOP      = 1    # <STOP,CRC>
    CMD_GRIP      = 2    # <GRIP,ACTION,CRC>   ACTION: OPEN|CLOSE|CAPTURE
    CMD_LIFT      = 3    # <LIFT,ACTION,CRC>   ACTION: UP|DOWN
    CMD_SET_STATE = 4    # <STATE,ID,CRC>
    CMD_PING      = 5    # <PING,CRC>

    # --- Mensagens de entrada (exterior → RPi) ---
    MSG_SENSOR    = 10   # <SENSOR,LINE_POS,LINE_DET,VIC_DET,VIC_X,VIC_Y,CRC>
    MSG_STATE_CHG = 11   # <STATE_CHG,NEW_STATE,CRC>
    MSG_ACK       = 12   # <ACK,CMD_TYPE,CRC>
    MSG_PONG      = 13   # <PONG,CRC>
    MSG_ERROR     = 14   # <ERROR,CODE,CRC>

    # --- Telemetria interna (WiFi/console) ---
    MSG_TELEMETRY = 20   # <TELEM,STATE,LINE_POS,PID_OUT,VICTIM,CRC>
    MSG_LOG       = 21   # <LOG,LEVEL,TEXT,CRC>


# Mapeamento string → MsgType para parsing de frames recebidos
MSG_TYPE_MAP: dict = {t.name.split('_', 1)[-1]: t for t in MsgType}

# Strings de saída para cada tipo de comando
CMD_STRINGS = {
    MsgType.CMD_MOVE      : "MOVE",
    MsgType.CMD_STOP      : "STOP",
    MsgType.CMD_GRIP      : "GRIP",
    MsgType.CMD_LIFT      : "LIFT",
    MsgType.CMD_SET_STATE : "STATE",
    MsgType.CMD_PING      : "PING",
    MsgType.MSG_TELEMETRY : "TELEM",
    MsgType.MSG_LOG       : "LOG",
}

# Strings de entrada que o parser deve reconhecer
INCOMING_TYPES = {
    "SENSOR"    : MsgType.MSG_SENSOR,
    "STATE_CHG" : MsgType.MSG_STATE_CHG,
    "ACK"       : MsgType.MSG_ACK,
    "PONG"      : MsgType.MSG_PONG,
    "ERROR"     : MsgType.MSG_ERROR,
}


# ============================================================
#  CÓDIGOS DE ERRO
# ============================================================

class ErrorCode(IntEnum):
    """Códigos de erro — espelho de ErrorCode_t do C++."""
    NONE            = 0
    SERIAL_TIMEOUT  = 1
    CRC_MISMATCH    = 2
    UNKNOWN_CMD     = 3
    LINE_LOST       = 4
    HUSKYLENS_FAIL  = 5
    WATCHDOG        = 6
    HARDWARE_FAIL   = 7
    BUFFER_OVERFLOW = 8


# ============================================================
#  ESTRUTURA DE MENSAGEM PARSEADA
# ============================================================

@dataclass
class Message:
    """
    Mensagem decodificada e validada pelo parser de protocolo.

    Espelho de ParsedMessage_t do firmware C++.
    """
    msg_type: str           = ''       # String do tipo (ex: 'SENSOR', 'ACK')
    params: List[str]       = field(default_factory=list)
    received_crc: int       = 0        # CRC recebido no frame
    calculated_crc: int     = 0        # CRC calculado localmente
    valid: bool             = False    # True se CRC confere
    raw: str                = ''       # Frame original (para debug)
    timestamp: float        = field(default_factory=time.monotonic)

    def get_param(self, index: int, default: str = '') -> str:
        """Retorna parâmetro por índice com valor padrão seguro."""
        return self.params[index] if index < len(self.params) else default

    def get_float(self, index: int, default: float = 0.0) -> float:
        """Retorna parâmetro como float com fallback seguro."""
        try:
            return float(self.params[index])
        except (IndexError, ValueError):
            return default

    def get_int(self, index: int, default: int = 0) -> int:
        """Retorna parâmetro como inteiro com fallback seguro."""
        try:
            return int(self.params[index])
        except (IndexError, ValueError):
            return default

    def get_bool(self, index: int, default: bool = False) -> bool:
        """Retorna parâmetro como booleano (0/1) com fallback seguro."""
        try:
            return bool(int(self.params[index]))
        except (IndexError, ValueError):
            return default


# ============================================================
#  CRC-8 (polinômio 0x07, Dallas/Maxim)
# ============================================================

def crc8(data: bytes) -> int:
    """
    Calcula CRC-8 com polinômio gerador x^8 + x^2 + x + 1 (0x07).

    Algoritmo bit-a-bit idêntico ao firmware Arduino (serial_comm.cpp).
    Sem lookup table — menor footprint, portabilidade garantida.

    Complexidade: O(N × 8) onde N = len(data).

    Args:
        data: Bytes sobre os quais calcular o CRC.
    Returns:
        Byte de CRC calculado [0–255].

    Exemplo:
        >>> crc8(b'MOVE,FWD,150')
        163  # = 0xA3
    """
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


def crc8_str(s: str) -> int:
    """Wrapper de crc8() para strings ASCII."""
    return crc8(s.encode('ascii'))


# ============================================================
#  MONTAGEM DE FRAMES
# ============================================================

def build_frame(content: str) -> bytes:
    """
    Monta um frame completo com delimitadores e CRC.

    Formato: <CONTENT,HEXCRC>\n

    Args:
        content: Conteúdo da mensagem sem delimitadores e sem CRC.
                 Exemplo: "MOVE,FWD,150"
    Returns:
        Frame completo em bytes prontos para transmissão.

    Exemplo:
        >>> build_frame("MOVE,FWD,150")
        b'<MOVE,FWD,150,A3>\\n'
    """
    crc   = crc8_str(content)
    frame = f"{FRAME_START}{content}{FRAME_SEP}{crc:02X}{FRAME_END}{FRAME_TERM}"
    return frame.encode('ascii')


def build_move(direction: str, speed: int) -> bytes:
    """
    Monta frame de comando de movimento.

    Args:
        direction : 'FWD' | 'BWD' | 'ROT_L' | 'ROT_R' | 'STOP'
        speed     : Velocidade PWM [0–255]
    """
    return build_frame(f"MOVE,{direction},{int(speed)}")


def build_stop() -> bytes:
    """Monta frame de parada imediata."""
    return build_frame("STOP")


def build_grip(action: str) -> bytes:
    """
    Monta frame de comando de garra.

    Args:
        action : 'OPEN' | 'CLOSE' | 'CAPTURE'
    """
    return build_frame(f"GRIP,{action}")


def build_lift(action: str) -> bytes:
    """
    Monta frame de comando de elevação do braço.

    Args:
        action : 'UP' | 'DOWN'
    """
    return build_frame(f"LIFT,{action}")


def build_state(state_id: int) -> bytes:
    """Monta frame de comando de mudança de estado."""
    return build_frame(f"STATE,{state_id}")


def build_ping() -> bytes:
    """Monta frame de PING para healthcheck."""
    return build_frame("PING")


def build_telemetry(state: str, line_pos: float,
                    pid_out: float, victim: bool) -> bytes:
    """
    Monta frame de telemetria para envio via WiFi/console.

    Args:
        state    : Nome do estado atual da FSM
        line_pos : Posição da linha [−1.0, +1.0]
        pid_out  : Saída do controlador PID
        victim   : True se vítima detectada
    """
    return build_frame(
        f"TELEM,{state},{line_pos:.3f},{pid_out:.1f},{int(victim)}"
    )


# ============================================================
#  PARSER DE FRAMES
# ============================================================

def parse_frame(raw: str) -> Message:
    """
    Faz o parse de um frame recebido (conteúdo entre '<' e '>').

    Pipeline:
    1. Encontra o último ',' (separador antes do campo CRC)
    2. Extrai e converte o CRC hexadecimal
    3. Calcula CRC sobre o conteúdo anterior ao último ','
    4. Valida integridade (CRC recebido == CRC calculado)
    5. Tokeniza tipo e parâmetros

    Args:
        raw: String entre '<' e '>' sem os delimitadores.
    Returns:
        Message preenchida, com valid=True se CRC confere.

    Exemplo:
        >>> parse_frame("MOVE,FWD,150,A3")
        Message(msg_type='MOVE', params=['FWD', '150'], valid=True, ...)
    """
    msg = Message(raw=raw)

    # Separa conteúdo do campo CRC pelo último separador
    last_comma = raw.rfind(FRAME_SEP)
    if last_comma == -1:
        return msg   # Frame malformado (sem separador)

    content = raw[:last_comma]
    crc_str = raw[last_comma + 1:].strip()

    # Converte CRC hexadecimal
    try:
        msg.received_crc = int(crc_str, 16)
    except ValueError:
        return msg   # CRC não é hexadecimal válido

    # Valida CRC
    msg.calculated_crc = crc8_str(content)
    if msg.received_crc != msg.calculated_crc:
        return msg   # CRC mismatch — mensagem corrompida

    # Tokeniza tipo e parâmetros
    tokens = content.split(FRAME_SEP)
    if not tokens:
        return msg

    msg.msg_type = tokens[0].strip().upper()
    msg.params   = [t.strip() for t in tokens[1:]]
    msg.valid    = True

    return msg


class FrameParser:
    """
    Parser incremental de frames — processa bytes um a um.

    Equivalente à FSM de parsing do serial_comm.cpp do Arduino.
    Mantém estado entre chamadas, permitindo processar bytes
    recebidos de forma fragmentada (streams).

    Estados internos:
        WAIT_START : Descarta bytes até encontrar '<'
        READING    : Acumula bytes até encontrar '>'

    Uso:
        parser = FrameParser()
        for byte in stream:
            msg = parser.feed(byte)
            if msg is not None and msg.valid:
                handle(msg)
    """

    def __init__(self):
        self._buffer: bytearray = bytearray()
        self._reading: bool     = False

    def feed(self, byte: int) -> Optional[Message]:
        """
        Alimenta um byte ao parser.

        Args:
            byte: Byte recebido (inteiro 0–255).
        Returns:
            Message se um frame completo e válido foi parseado,
            None caso contrário.
        """
        ch = chr(byte)

        if ch == FRAME_START:
            self._buffer.clear()
            self._reading = True
            return None

        if ch == FRAME_END and self._reading:
            self._reading = False
            raw = self._buffer.decode('ascii', errors='ignore')
            self._buffer.clear()
            msg = parse_frame(raw)
            return msg if msg.valid else None

        if self._reading:
            if len(self._buffer) < FRAME_MAX_LEN:
                self._buffer.append(byte)
            else:
                # Buffer overflow — descarta frame corrompido
                self._buffer.clear()
                self._reading = False

        return None

    def reset(self):
        """Reinicia o estado do parser."""
        self._buffer.clear()
        self._reading = False
