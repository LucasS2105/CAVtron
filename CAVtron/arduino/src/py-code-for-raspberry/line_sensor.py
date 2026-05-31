"""
sensors/line_sensor.py
=======================
Array de sensores de linha IR com ADC externo MCP3008 via SPI.
Conversão de line_sensor.h/.cpp para Raspberry Pi Zero 2 W.

DIFERENÇA CRÍTICA: O Raspberry Pi NÃO possui ADC nativo.
O Arduino usa analogRead() (ADC 10-bit interno). Aqui, usamos o
MCP3008 — ADC 8 canais, 10-bit, interface SPI — que é o equivalente
mais comum e econômico para projetos RPi.

Conexão MCP3008 ↔ RPi Zero 2 W:
    MCP3008 VDD  → 3.3V (pino 1)
    MCP3008 VREF → 3.3V (pino 1)
    MCP3008 AGND → GND
    MCP3008 DGND → GND
    MCP3008 CLK  → GPIO 11 (SCLK, pino 23)
    MCP3008 DOUT → GPIO 9  (MISO, pino 21)
    MCP3008 DIN  → GPIO 10 (MOSI, pino 19)
    MCP3008 CS   → GPIO 8  (CE0,  pino 24)
    CH0–CH7      → sensores IR (sensor 0=esq, sensor 7=dir)

ATENÇÃO: Sensores IR com saída 5V precisam de divisor de tensão
(ex: 10kΩ + 20kΩ) antes do MCP3008, que opera em 3.3V.

Algoritmo de posição — Centroide ponderado:
    position = Σ(weight_i × active_i) / Σ(active_i)
    weights = [−3.5, −2.5, −1.5, −0.5, +0.5, +1.5, +2.5, +3.5]
    resultado normalizado em [−1.0, +1.0]
    −1.0 = linha no extremo esquerdo
    +1.0 = linha no extremo direito
     0.0 = linha centralizada (setpoint do PID)
"""

import time
import logging
import spidev
from dataclasses import dataclass, field
from typing import List

from config import (
    SPI_BUS, SPI_DEVICE, SPI_SPEED_HZ,
    LINE_SENSOR_COUNT, LINE_SENSOR_CHANNELS,
    LINE_THRESHOLD, LINE_LOST_TIMEOUT_S
)

logger = logging.getLogger(__name__)

# Pesos do centroide — simetria em relação ao centro
# N=8: pesos de −3.5 a +3.5 com passo 1.0
SENSOR_WEIGHTS: List[float] = [
    -3.5, -2.5, -1.5, -0.5,
     0.5,  1.5,  2.5,  3.5
]
WEIGHT_NORMALIZER: float = 3.5  # = (N/2) − 0.5


@dataclass
class LineSensorData:
    """Dados processados do array de sensores — espelho de LineSensorData_t."""
    position: float           = 0.0      # Posição da linha [−1.0, +1.0]
    line_detected: bool        = False
    crossing_detected: bool    = False
    line_lost: bool            = False
    lost_timestamp: float      = 0.0     # time.monotonic() quando perdeu a linha
    raw_binary: int            = 0x00    # Bitmask dos sensores ativos
    raw_analog: List[int]      = field(default_factory=lambda: [0]*LINE_SENSOR_COUNT)


class LineSensor:
    """
    Gerenciador do array de sensores de linha IR com MCP3008.

    Args:
        spi_bus    : Barramento SPI (padrão: 0 → /dev/spidev0.x).
        spi_device : Dispositivo CE (padrão: 0 → CE0 = GPIO 8).
        threshold  : Limiar de binarização ADC [0–1023].
    """

    def __init__(self,
                 spi_bus: int    = SPI_BUS,
                 spi_device: int = SPI_DEVICE,
                 threshold: int  = LINE_THRESHOLD):
        self._threshold = threshold
        self._spi = spidev.SpiDev()
        self._spi_bus    = spi_bus
        self._spi_device = spi_device

        self._data = LineSensorData()
        self._last_valid_position = 0.0
        self._was_detected = False

    # ----------------------------------------------------------
    #  INICIALIZAÇÃO
    # ----------------------------------------------------------

    def begin(self):
        """
        Abre o barramento SPI e configura o MCP3008.

        Deve ser chamado uma vez antes de update().
        Requer que SPI esteja habilitado: sudo raspi-config →
        Interface Options → SPI → Enable.
        """
        self._spi.open(self._spi_bus, self._spi_device)
        self._spi.max_speed_hz = SPI_SPEED_HZ
        self._spi.mode = 0b00   # SPI mode 0 (CPOL=0, CPHA=0)
        logger.info(f"[LineSensor] SPI aberto: bus={self._spi_bus}, device={self._spi_device}")

    def cleanup(self):
        """Fecha o barramento SPI."""
        self._spi.close()
        logger.info("[LineSensor] SPI fechado.")

    # ----------------------------------------------------------
    #  ATUALIZAÇÃO (CHAMADA A CADA CICLO)
    # ----------------------------------------------------------

    def update(self):
        """
        Lê todos os sensores, calcula posição e atualiza flags.

        Equivalente ao update() do Arduino — deve ser chamado
        a cada ciclo de controle (~10ms).
        """
        self._read_analog()
        self._process_position()
        self._update_flags()

    # ----------------------------------------------------------
    #  LEITURA ANALÓGICA VIA MCP3008
    # ----------------------------------------------------------

    def _read_analog(self):
        """
        Lê os 8 canais do MCP3008 via SPI.

        Protocolo MCP3008 (SPI mode 0):
          Byte 1: start bit = 0b00000001
          Byte 2: SGL/DIFF | D2 | D1 | D0 | x | x | x | x
                  SGL=1 (single-ended), D2:D0 = canal (0–7)
          Byte 3: don't care (clock para leitura)

          Resposta: adc[1] bits 1:0 (MSB) + adc[2] bits 7:0 (LSB)
          Valor: 0–1023 (10-bit)

        Equivalente a analogRead(A0..A7) do Arduino.
        """
        for i, ch in enumerate(LINE_SENSOR_CHANNELS):
            # Constrói o comando de leitura para o canal ch
            cmd = [0x01, (0x08 | ch) << 4, 0x00]
            response = self._spi.xfer2(cmd)
            # Extrai os 10 bits de resultado
            value = ((response[1] & 0x03) << 8) | response[2]
            self._data.raw_analog[i] = value

    # ----------------------------------------------------------
    #  PROCESSAMENTO DE POSIÇÃO (CENTROIDE PONDERADO)
    # ----------------------------------------------------------

    def _process_position(self):
        """
        Binariza leituras e calcula posição por centroide.

        Binarização: sensor ativo se ADC < threshold
        (superfície escura → menos reflexão IR → valor ADC menor).

        Centroide ponderado:
            position = Σ(w_i) / N_ativos / WEIGHT_NORMALIZER
        """
        weighted_sum = 0.0
        total_weight = 0.0
        active_count = 0
        binary_mask  = 0x00

        for i in range(LINE_SENSOR_COUNT):
            active = self._data.raw_analog[i] < self._threshold

            if active:
                binary_mask  |= (1 << i)
                active_count += 1
                weighted_sum += SENSOR_WEIGHTS[i]
                total_weight += 1.0

        self._data.raw_binary = binary_mask

        # Cruzamento: todos (ou quase todos) os sensores ativos
        self._data.crossing_detected = (active_count >= LINE_SENSOR_COUNT - 1)

        if active_count == 0:
            # Linha não detectada — retém última posição válida
            # para indicar ao PID a direção de recuperação
            self._data.line_detected = False
            self._data.position = self._last_valid_position
        else:
            self._data.line_detected = True
            raw_pos = (weighted_sum / total_weight) / WEIGHT_NORMALIZER
            self._data.position = max(-1.0, min(1.0, raw_pos))
            self._last_valid_position = self._data.position

    # ----------------------------------------------------------
    #  ATUALIZAÇÃO DE FLAGS DE ESTADO
    # ----------------------------------------------------------

    def _update_flags(self):
        """
        Atualiza flags de linha perdida com debounce por timeout.

        Equivalente a _updateFlags() do Arduino, usando time.monotonic()
        em vez de millis().
        """
        now = time.monotonic()

        if self._data.line_detected:
            self._data.line_lost      = False
            self._data.lost_timestamp = 0.0
            self._was_detected        = True
        else:
            if self._was_detected and self._data.lost_timestamp == 0.0:
                self._data.lost_timestamp = now

            if (self._data.lost_timestamp > 0.0 and
                    (now - self._data.lost_timestamp) >= LINE_LOST_TIMEOUT_S):
                self._data.line_lost = True

    # ----------------------------------------------------------
    #  CALIBRAÇÃO
    # ----------------------------------------------------------

    def calibrate(self, duration_s: float = 2.0):
        """
        Calibra o threshold automaticamente coletando amostras.

        Mova o robô sobre a linha e o chão durante duration_s.
        O threshold é definido como a média entre os extremos globais.

        Args:
            duration_s: Duração da calibração em segundos.
        """
        logger.info(f"[LineSensor] Calibração iniciada ({duration_s:.0f}s)...")

        min_vals = [1023] * LINE_SENSOR_COUNT
        max_vals = [0]    * LINE_SENSOR_COUNT

        end_time = time.monotonic() + duration_s
        while time.monotonic() < end_time:
            self._read_analog()
            for i in range(LINE_SENSOR_COUNT):
                v = self._data.raw_analog[i]
                if v < min_vals[i]: min_vals[i] = v
                if v > max_vals[i]: max_vals[i] = v
            time.sleep(0.005)

        global_min = sum(min_vals) / LINE_SENSOR_COUNT
        global_max = sum(max_vals) / LINE_SENSOR_COUNT
        self._threshold = int((global_min + global_max) / 2)

        logger.info(f"[LineSensor] Threshold calibrado: {self._threshold}")

    def set_threshold(self, threshold: int):
        """Define manualmente o threshold de binarização."""
        self._threshold = threshold

    # ----------------------------------------------------------
    #  ACESSO AOS DADOS
    # ----------------------------------------------------------

    @property
    def data(self) -> LineSensorData:
        return self._data

    @property
    def position(self) -> float:
        return self._data.position

    @property
    def line_detected(self) -> bool:
        return self._data.line_detected

    @property
    def line_lost(self) -> bool:
        return self._data.line_lost

    @property
    def is_crossing(self) -> bool:
        return self._data.crossing_detected
