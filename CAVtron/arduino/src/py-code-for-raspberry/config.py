"""
config.py
=========
Configurações globais do sistema para Raspberry Pi Zero 2 W.

Equivalente direto ao config.h do firmware Arduino, adaptado para:
  - Numeração de pinos BCM (Broadcom) do Raspberry Pi
  - ADC externo MCP3008 via SPI (substitui analogRead do Arduino)
  - PWM por software via RPi.GPIO (frequência configurável)
  - I2C via smbus2 (substitui Wire.h)

Diferenças críticas em relação ao Arduino:
  - RPi NÃO possui ADC nativo — sensores analógicos requerem MCP3008
  - GPIO opera em 3.3V (Arduino em 5V) — verificar compatibilidade
  - PWM por software tem jitter (~50-100µs) — para servos de precisão,
    usar pigpio (hardware-timed DMA PWM)
  - Sistema operacional não é RTOS — loop de controle pode sofrer
    preempção de até ~1-5ms em carga normal

Pinout RPi Zero 2 W (BCM):
    +--------+--------+-----------+
    | BCM    | Função | Uso       |
    +--------+--------+-----------+
    | GPIO 12| PWM0   | Motor Esq |
    | GPIO 13| PWM1   | Motor Dir |
    | GPIO 18| PWM0*  | Servo Grip|
    | GPIO 23| GPO    | Servo Lift|
    | GPIO  2| SDA    | I2C       |
    | GPIO  3| SCL    | I2C       |
    | GPIO  8| SPI CE0| MCP3008   |
    | GPIO  9| SPI MISO| MCP3008  |
    | GPIO 10| SPI MOSI| MCP3008  |
    | GPIO 11| SPI CLK| MCP3008   |
    +--------+--------+-----------+
    *GPIO 18 compartilha PWM0 com GPIO 12 — usar somente um por vez.
"""

# ============================================================
#  TEMPORIZAÇÃO
# ============================================================

LOOP_PERIOD_S: float = 0.010          # 10ms → 100 Hz
WATCHDOG_TIMEOUT_S: float = 1.5       # Timeout de comunicação (não usado nesta arch.)

# ============================================================
#  PINOS — MOTORES DC (Driver L298N)
#  BCM numbering (GPIO.setmode(GPIO.BCM))
# ============================================================

MOTOR_LEFT_PWM: int  = 12   # Hardware PWM0 (recomendado: pigpio)
MOTOR_LEFT_DIR1: int = 20
MOTOR_LEFT_DIR2: int = 21

MOTOR_RIGHT_PWM: int  = 13  # Hardware PWM1
MOTOR_RIGHT_DIR1: int = 19
MOTOR_RIGHT_DIR2: int = 26

# Frequência PWM do driver de motor em Hz
# L298N: recomendado 1–20 kHz. 1 kHz via software é estável no RPi.
MOTOR_PWM_FREQ_HZ: int = 1000

MOTOR_BASE_SPEED: int   = 140   # Velocidade base [0–255, normalizado internamente]
MOTOR_MAX_SPEED: int    = 255
MOTOR_DEADBAND: int     = 30    # Duty mínimo abaixo do qual o motor não gira

# ============================================================
#  ADC EXTERNO — MCP3008 via SPI (substitui analogRead)
#  Sensor de linha: 8 canais analógicos CH0–CH7
# ============================================================

SPI_BUS: int    = 0     # /dev/spidev0.x
SPI_DEVICE: int = 0     # CE0 = GPIO 8
SPI_SPEED_HZ: int = 1_000_000   # 1 MHz — MCP3008 suporta até 3.6 MHz @ 5V

# Mapeamento canal MCP3008 → sensor de linha
# Canal 0 = sensor extremo esquerdo, Canal 7 = extremo direito
LINE_SENSOR_COUNT: int = 8
LINE_SENSOR_CHANNELS: list = [0, 1, 2, 3, 4, 5, 6, 7]

# Limiar de binarização (ADC 10-bit: 0–1023)
# Abaixo = linha detectada (superfície escura → menor reflexão IR)
LINE_THRESHOLD: int = 512

# Timeout sem detecção de linha antes de manobra de recuperação
LINE_LOST_TIMEOUT_S: float = 0.8

# ============================================================
#  PINOS — SERVOMOTORES (Garra)
# ============================================================

SERVO_GRIP_PIN: int = 18   # PWM por software 50 Hz
SERVO_LIFT_PIN: int = 23

# Frequência dos pulsos PWM de servo (padrão universal: 50 Hz)
SERVO_PWM_FREQ_HZ: int = 50

# Ângulos de operação (graus)
SERVO_GRIP_OPEN_DEG: int  = 90
SERVO_GRIP_CLOSE_DEG: int = 10
SERVO_LIFT_UP_DEG: int    = 90
SERVO_LIFT_DOWN_DEG: int  = 15

# Mapeamento de ângulo para duty cycle (%)
# Pulso: 0.5ms (2.5%) → 0° | 1.5ms (7.5%) → 90° | 2.5ms (12.5%) → 180°
SERVO_DUTY_MIN: float = 2.5    # % duty cycle para 0°
SERVO_DUTY_MAX: float = 12.5   # % duty cycle para 180°

# Intervalo entre passos de sweep em segundos (slew rate limiting)
SERVO_SWEEP_DELAY_S: float = 0.015   # 15ms/grau

# ============================================================
#  CONTROLADOR PID
# ============================================================

PID_KP: float = 32.0
PID_KI: float = 0.4
PID_KD: float = 22.0

PID_SAMPLE_TIME_S: float = LOOP_PERIOD_S

PID_OUTPUT_MAX: float  =  255.0
PID_OUTPUT_MIN: float  = -255.0
PID_INTEGRAL_MAX: float = 80.0
PID_DERIVATIVE_ALPHA: float = 0.2   # Filtro EMA no derivativo

# ============================================================
#  HUSKYLENS (I2C)
# ============================================================

I2C_BUS: int              = 1       # /dev/i2c-1 (pinos SDA=GPIO2, SCL=GPIO3)
HUSKYLENS_I2C_ADDR: int   = 0x32
HUSKYLENS_POLL_S: float   = 0.05   # 50ms → 20 Hz

# Resolução da câmera HuskyLens (pixels)
HUSKYLENS_IMG_WIDTH: int  = 320
HUSKYLENS_IMG_HEIGHT: int = 240

# ============================================================
#  ESTADOS DA FSM (espelho do RobotState_t do Arduino)
# ============================================================

from enum import IntEnum

class RobotState(IntEnum):
    IDLE        = 0
    FOLLOW_LINE = 1
    OBSTACLE    = 2
    GRIP_OPEN   = 3
    GRIP_CLOSE  = 4
    GRIP_LIFT   = 5
    GRIP_DROP   = 6
    EMERGENCY   = 7

# ============================================================
#  UTILITÁRIOS INLINE
# ============================================================

def millis() -> float:
    """Retorna o tempo monotônico em milissegundos (equivale ao millis() do Arduino)."""
    import time
    return time.monotonic() * 1000.0

def constrain(val: float, min_val: float, max_val: float) -> float:
    """Satura val no intervalo [min_val, max_val] (equivale ao constrain() do Arduino)."""
    return max(min_val, min(max_val, val))
