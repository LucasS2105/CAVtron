"""
sensors/huskylens.py
=====================
Interface com a câmera HuskyLens via I2C — conversão de huskylens.h/.cpp.

A HuskyLens possui processador dedicado (FPGA + DSP) que executa os
algoritmos de visão internamente, entregando resultados estruturados
(bounding boxes com ID) via I2C ou UART.

Protocolo I2C HuskyLens (DFRobot):
    Frame de comando (RPi → câmera):
        [0x55, 0xAA, 0x11, DATA_LEN, COMMAND, ...DATA, CHECKSUM]
        CHECKSUM = soma de todos os bytes do frame (módulo 256)

    Frame de resposta (câmera → RPi):
        [0x55, 0xAA, 0x11, DATA_LEN, 0x2E, ...DATA, CHECKSUM]
        Cada "block" ocupa 10 bytes: xCenter(2), yCenter(2),
        width(2), height(2), ID(2)

Comandos usados:
    0x20 → REQUEST       : Solicita todos os objetos detectados
    0x2D → SET_ALGORITHM : Define o algoritmo ativo

Algoritmos:
    0x01 → FACE_RECOGNITION
    0x03 → OBJECT_TRACKING
    0x04 → OBJECT_RECOGNITION   ← padrão neste módulo
    0x05 → LINE_TRACKING
    0x06 → COLOR_RECOGNITION
    0x07 → TAG_RECOGNITION

Referência: DFRobot HUSKYLENS Arduino Library (open source)
    https://github.com/DFRobot/HUSKYLENS
"""

import time
import logging
import smbus2
from dataclasses import dataclass

from config import (
    I2C_BUS, HUSKYLENS_I2C_ADDR,
    HUSKYLENS_POLL_S,
    HUSKYLENS_IMG_WIDTH, HUSKYLENS_IMG_HEIGHT
)

logger = logging.getLogger(__name__)


# ============================================================
#  CONSTANTES DO PROTOCOLO
# ============================================================

_HEADER       = [0x55, 0xAA, 0x11]
_CMD_REQUEST  = 0x20   # Solicita todos os resultados detectados
_CMD_ALGORITHM = 0x2D  # Define o algoritmo ativo

_ALGO_OBJECT_RECOGNITION = 0x04   # Valor padrão

_RESPONSE_OK  = 0x2E   # Byte de resposta OK da câmera
_BLOCK_SIZE   = 10     # Bytes por objeto (block) na resposta


# ============================================================
#  ESTRUTURA DE DADOS
# ============================================================

@dataclass
class VictimData:
    """
    Dados de um objeto detectado pela HuskyLens.
    Espelho de VictimData_t do firmware C++.
    """
    detected: bool       = False
    x_center: int        = 0      # Pixel X [0, 320]
    y_center: int        = 0      # Pixel Y [0, 240]
    width: int           = 0      # Largura da bounding box
    height: int          = 0      # Altura da bounding box
    object_id: int       = 0      # ID do objeto treinado
    normalized_x: float  = 0.0   # X normalizado [−1.0, +1.0]
    normalized_y: float  = 0.0   # Y normalizado [−1.0, +1.0]
    area: float          = 0.0   # Área relativa (proxy de distância)
    timestamp: float     = 0.0   # time.monotonic() da última detecção


# ============================================================
#  INTERFACE HUSKYLENS
# ============================================================

class HuskyLensInterface:
    """
    Wrapper de alto nível para a câmera HuskyLens via I2C/smbus2.

    Args:
        i2c_bus  : Número do barramento I2C (padrão: 1 → /dev/i2c-1).
        address  : Endereço I2C da câmera (padrão: 0x32).
        algorithm: Algoritmo de visão a configurar.
    """

    def __init__(self,
                 i2c_bus: int   = I2C_BUS,
                 address: int   = HUSKYLENS_I2C_ADDR,
                 algorithm: int = _ALGO_OBJECT_RECOGNITION):
        self._bus_num   = i2c_bus
        self._addr      = address
        self._algorithm = algorithm

        self._bus: smbus2.SMBus = None
        self._data = VictimData()
        self._connected   = False
        self._last_poll   = 0.0
        self._last_frame  = 0.0

    # ----------------------------------------------------------
    #  CICLO DE VIDA
    # ----------------------------------------------------------

    def begin(self) -> bool:
        """
        Inicializa o barramento I2C e configura o algoritmo da câmera.

        Realiza até 3 tentativas de conexão.
        Requer que I2C esteja habilitado: sudo raspi-config →
        Interface Options → I2C → Enable.

        Returns:
            True se câmera respondeu com sucesso.
        """
        try:
            self._bus = smbus2.SMBus(self._bus_num)
        except Exception as e:
            logger.error(f"[HuskyLens] Falha ao abrir I2C bus {self._bus_num}: {e}")
            return False

        # Tenta conectar com até 3 retries
        for attempt in range(3):
            if self._ping():
                self._connected = True
                break
            time.sleep(0.2)

        if not self._connected:
            logger.error("[HuskyLens] Câmera não encontrada no I2C!")
            return False

        # Configura o algoritmo de reconhecimento
        self._set_algorithm(self._algorithm)

        logger.info(f"[HuskyLens] Inicializada. Algoritmo: {self._algorithm:#04x}")
        self._last_frame = time.monotonic()
        return True

    def cleanup(self):
        """Fecha o barramento I2C."""
        if self._bus:
            self._bus.close()

    # ----------------------------------------------------------
    #  ATUALIZAÇÃO (POLLING)
    # ----------------------------------------------------------

    def update(self):
        """
        Solicita e processa os dados mais recentes da câmera.

        Equivalente ao update() do Arduino — deve ser chamado
        a cada HUSKYLENS_POLL_S segundos.
        Mantém o último estado se a câmera não responder.
        """
        if not self._connected:
            return

        now = time.monotonic()
        if (now - self._last_poll) < HUSKYLENS_POLL_S:
            return
        self._last_poll = now

        try:
            blocks = self._request_all()
            if blocks:
                # Usa o primeiro bloco (objeto de maior relevância ou menor ID)
                self._parse_block(blocks[0])
                self._last_frame = now
            else:
                self._clear_data()
        except Exception as e:
            logger.debug(f"[HuskyLens] Erro na leitura: {e}")

    # ----------------------------------------------------------
    #  ACESSO AOS DADOS
    # ----------------------------------------------------------

    @property
    def data(self) -> VictimData:
        return self._data

    @property
    def victim_detected(self) -> bool:
        return self._data.detected

    @property
    def connected(self) -> bool:
        return self._connected

    def learn_object(self, obj_id: int):
        """
        Envia comando de aprendizado à câmera.

        Args:
            obj_id: ID do objeto a aprender [1–255].
        """
        if not self._connected:
            return
        cmd = [0x36]   # COMMAND_LEARN
        data = [obj_id & 0xFF, (obj_id >> 8) & 0xFF]
        self._send_command(cmd[0], data)
        logger.info(f"[HuskyLens] Aprendendo objeto ID={obj_id}")

    # ----------------------------------------------------------
    #  PROTOCOLO I2C INTERNO
    # ----------------------------------------------------------

    def _ping(self) -> bool:
        """
        Verifica se a câmera responde no barramento I2C.

        Envia o comando KNOCK (0x2C) e aguarda resposta OK (0x2E).
        """
        try:
            self._send_command(0x2C, [])
            time.sleep(0.05)
            response = self._read_response()
            return response is not None and len(response) >= 5 and response[4] == 0x2E
        except Exception:
            return False

    def _set_algorithm(self, algorithm: int):
        """
        Configura o algoritmo de visão ativo na câmera.

        Args:
            algorithm: Código do algoritmo (ex: 0x04 = object recognition).
        """
        data = [algorithm & 0xFF, (algorithm >> 8) & 0xFF]
        self._send_command(_CMD_ALGORITHM, data)
        time.sleep(0.3)  # Câmera precisa de tempo para trocar de modo

    def _request_all(self) -> list:
        """
        Solicita todos os objetos detectados e retorna lista de blocos.

        Protocolo:
        1. Envia comando REQUEST (0x20)
        2. Lê cabeçalho da resposta para obter número de blocos
        3. Parseia cada bloco de 10 bytes

        Returns:
            Lista de dicionários com os campos de cada objeto detectado.
        """
        self._send_command(_CMD_REQUEST, [])
        time.sleep(0.01)

        response = self._read_response()
        if not response or len(response) < 6:
            return []

        # Byte 3 = DATA_LEN (tamanho dos dados após o cabeçalho fixo)
        data_len = response[3]

        # Número de objetos = data_len / _BLOCK_SIZE
        num_blocks = data_len // _BLOCK_SIZE
        if num_blocks == 0:
            return []

        blocks = []
        for i in range(num_blocks):
            offset = 5 + i * _BLOCK_SIZE   # Pula [0x55, 0xAA, 0x11, LEN, 0x2E]
            if offset + _BLOCK_SIZE > len(response):
                break

            raw = response[offset: offset + _BLOCK_SIZE]
            block = {
                'x_center' : raw[0] | (raw[1] << 8),
                'y_center' : raw[2] | (raw[3] << 8),
                'width'    : raw[4] | (raw[5] << 8),
                'height'   : raw[6] | (raw[7] << 8),
                'id'       : raw[8] | (raw[9] << 8),
            }
            blocks.append(block)

        return blocks

    def _send_command(self, command: int, data: list):
        """
        Envia um frame de comando à câmera via I2C.

        Frame: [0x55, 0xAA, 0x11, DATA_LEN, COMMAND, ...DATA, CHECKSUM]
        CHECKSUM = soma de todos os bytes & 0xFF.

        Args:
            command: Byte de comando.
            data   : Lista de bytes de dados (pode ser vazia).
        """
        frame = _HEADER + [len(data), command] + data
        checksum = sum(frame) & 0xFF
        frame.append(checksum)

        # smbus2: write_i2c_block_data(addr, register, data)
        # Como HuskyLens usa I2C raw (sem register), usamos write_bytes
        msg = smbus2.i2c_msg.write(self._addr, frame)
        self._bus.i2c_rdwr(msg)

    def _read_response(self, max_bytes: int = 128) -> list:
        """
        Lê a resposta da câmera via I2C.

        Args:
            max_bytes: Número máximo de bytes a ler.
        Returns:
            Lista de bytes lidos, ou None em caso de erro.
        """
        try:
            msg = smbus2.i2c_msg.read(self._addr, max_bytes)
            self._bus.i2c_rdwr(msg)
            return list(msg)
        except Exception as e:
            logger.debug(f"[HuskyLens] Erro na leitura I2C: {e}")
            return None

    # ----------------------------------------------------------
    #  PROCESSAMENTO DE RESULTADO
    # ----------------------------------------------------------

    def _parse_block(self, block: dict):
        """
        Preenche VictimData a partir de um bloco de resultado.

        Normalização de posição (espelho de _parseResult do C++):
            normX = (xCenter − 160) / 160.0  ∈ [−1.0, +1.0]
            normY = (yCenter − 120) / 120.0  ∈ [−1.0, +1.0]

        Área relativa (proxy de distância):
            area = (width × height) / (320 × 240)  ∈ (0.0, 1.0]
        """
        cx = block['x_center']
        cy = block['y_center']
        w  = block['width']
        h  = block['height']

        self._data.detected     = True
        self._data.x_center     = cx
        self._data.y_center     = cy
        self._data.width        = w
        self._data.height       = h
        self._data.object_id    = block['id']
        self._data.normalized_x = (cx - HUSKYLENS_IMG_WIDTH  // 2) / (HUSKYLENS_IMG_WIDTH  / 2)
        self._data.normalized_y = (cy - HUSKYLENS_IMG_HEIGHT // 2) / (HUSKYLENS_IMG_HEIGHT / 2)
        self._data.area         = (w * h) / (HUSKYLENS_IMG_WIDTH * HUSKYLENS_IMG_HEIGHT)
        self._data.timestamp    = time.monotonic()

    def _clear_data(self):
        """Reseta VictimData para estado 'não detectado'."""
        self._data.detected     = False
        self._data.x_center     = 0
        self._data.y_center     = 0
        self._data.width        = 0
        self._data.height       = 0
        self._data.object_id    = 0
        self._data.normalized_x = 0.0
        self._data.normalized_y = 0.0
        self._data.area         = 0.0
