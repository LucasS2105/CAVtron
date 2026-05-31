"""
drivers/motor_driver.py
========================
Driver de tração diferencial para motores DC via L298N.
Conversão direta de motor_driver.h/.cpp para RPi.GPIO.

Diferenças em relação ao Arduino:
  - analogWrite(0–255) → GPIO.PWM.ChangeDutyCycle(0–100%)
    Conversão: duty = (pwm_value / 255.0) * 100.0
  - Frequência PWM configurável (padrão: 1 kHz via software)
  - Para PWM de maior precisão e menor jitter, usar pigpio

Modelo cinemático — Tração Diferencial:
    v_esq = BASE_SPEED − pidOutput
    v_dir = BASE_SPEED + pidOutput
    pidOutput > 0 → curva à direita (acelera dir, freia esq)
    pidOutput < 0 → curva à esquerda (acelera esq, freia dir)
"""

import logging
import RPi.GPIO as GPIO
from enum import IntEnum
from config import (
    MOTOR_LEFT_PWM, MOTOR_LEFT_DIR1, MOTOR_LEFT_DIR2,
    MOTOR_RIGHT_PWM, MOTOR_RIGHT_DIR1, MOTOR_RIGHT_DIR2,
    MOTOR_BASE_SPEED, MOTOR_MAX_SPEED, MOTOR_DEADBAND,
    MOTOR_PWM_FREQ_HZ
)

logger = logging.getLogger(__name__)


class MotorDirection(IntEnum):
    """Sentido de rotação do motor — espelho de MotorDirection_t."""
    FORWARD  = 0
    BACKWARD = 1
    BRAKE    = 2   # Freio ativo: DIR1=HIGH, DIR2=HIGH → curto nas bobinas
    COAST    = 3   # Desaceleração livre: DIR1=LOW, DIR2=LOW


class MotorDriver:
    """
    Controlador de tração diferencial para dois motores DC via L298N.

    Args:
        pwm_freq_hz: Frequência do sinal PWM em Hz (padrão: 1000 Hz).
    """

    def __init__(self, pwm_freq_hz: int = MOTOR_PWM_FREQ_HZ):
        self._freq = pwm_freq_hz
        self._pwm_left  = None
        self._pwm_right = None
        self._left_duty  = 0.0
        self._right_duty = 0.0
        self._initialized = False

    # ----------------------------------------------------------
    #  INICIALIZAÇÃO
    # ----------------------------------------------------------

    def begin(self):
        """
        Configura pinos GPIO e instancia os objetos PWM.

        GPIO.setmode(GPIO.BCM) deve ter sido chamado antes de begin().
        """
        # Configura pinos de direção como saídas digitais
        for pin in (MOTOR_LEFT_DIR1, MOTOR_LEFT_DIR2,
                    MOTOR_RIGHT_DIR1, MOTOR_RIGHT_DIR2):
            GPIO.setup(pin, GPIO.OUT)

        # Configura pinos PWM como saídas
        GPIO.setup(MOTOR_LEFT_PWM,  GPIO.OUT)
        GPIO.setup(MOTOR_RIGHT_PWM, GPIO.OUT)

        # Instancia objetos PWM por software
        self._pwm_left  = GPIO.PWM(MOTOR_LEFT_PWM,  self._freq)
        self._pwm_right = GPIO.PWM(MOTOR_RIGHT_PWM, self._freq)

        # Inicia com duty cycle zero (motores parados)
        self._pwm_left.start(0)
        self._pwm_right.start(0)

        self._initialized = True
        self.stop_all()  # Estado inicial seguro
        logger.info("[MotorDriver] Inicializado.")

    def cleanup(self):
        """Para os PWM e libera os pinos."""
        if self._pwm_left:
            self._pwm_left.stop()
        if self._pwm_right:
            self._pwm_right.stop()
        logger.info("[MotorDriver] Cleanup realizado.")

    # ----------------------------------------------------------
    #  CONTROLE INDIVIDUAL
    # ----------------------------------------------------------

    def set_left(self, speed: int, direction: MotorDirection):
        """
        Define velocidade e direção do motor esquerdo.

        Args:
            speed     : Velocidade [0–255] (escala Arduino).
            direction : Sentido de rotação (MotorDirection).
        """
        self._set_motor(
            MOTOR_LEFT_DIR1, MOTOR_LEFT_DIR2,
            self._pwm_left, speed, direction
        )
        self._left_duty = self._to_duty(speed)

    def set_right(self, speed: int, direction: MotorDirection):
        """
        Define velocidade e direção do motor direito.

        Args:
            speed     : Velocidade [0–255] (escala Arduino).
            direction : Sentido de rotação (MotorDirection).
        """
        self._set_motor(
            MOTOR_RIGHT_DIR1, MOTOR_RIGHT_DIR2,
            self._pwm_right, speed, direction
        )
        self._right_duty = self._to_duty(speed)

    # ----------------------------------------------------------
    #  STEERING DIFERENCIAL (saída do PID)
    # ----------------------------------------------------------

    def apply_pid_steering(self, pid_output: float):
        """
        Aplica a correção PID ao par de motores.

        Modelo diferencial:
            v_esq = BASE_SPEED − pid_output
            v_dir = BASE_SPEED + pid_output

        Args:
            pid_output: Saída do controlador PID [−255, +255].
        """
        left_raw  = int(MOTOR_BASE_SPEED - pid_output)
        right_raw = int(MOTOR_BASE_SPEED + pid_output)

        left_dir  = MotorDirection.FORWARD if left_raw  >= 0 else MotorDirection.BACKWARD
        right_dir = MotorDirection.FORWARD if right_raw >= 0 else MotorDirection.BACKWARD

        left_spd  = self._constrain_speed(abs(left_raw))
        right_spd = self._constrain_speed(abs(right_raw))

        self.set_left(left_spd,   left_dir)
        self.set_right(right_spd, right_dir)

    # ----------------------------------------------------------
    #  MOVIMENTOS PRÉ-DEFINIDOS
    # ----------------------------------------------------------

    def stop_all(self):
        """Para ambos os motores com freio ativo."""
        self._set_motor(MOTOR_LEFT_DIR1,  MOTOR_LEFT_DIR2,  self._pwm_left,  0, MotorDirection.BRAKE)
        self._set_motor(MOTOR_RIGHT_DIR1, MOTOR_RIGHT_DIR2, self._pwm_right, 0, MotorDirection.BRAKE)
        self._left_duty = self._right_duty = 0.0

    def rotate_left(self, speed: int):
        """
        Rotação in-loco à esquerda (CCW).
        Motor esquerdo recua, motor direito avança.
        """
        self.set_left(speed,  MotorDirection.BACKWARD)
        self.set_right(speed, MotorDirection.FORWARD)

    def rotate_right(self, speed: int):
        """
        Rotação in-loco à direita (CW).
        Motor esquerdo avança, motor direito recua.
        """
        self.set_left(speed,  MotorDirection.FORWARD)
        self.set_right(speed, MotorDirection.BACKWARD)

    def move_forward(self, speed: int):
        """Move para frente com velocidade simétrica."""
        self.set_left(speed,  MotorDirection.FORWARD)
        self.set_right(speed, MotorDirection.FORWARD)

    def move_backward(self, speed: int):
        """Move para trás com velocidade simétrica."""
        self.set_left(speed,  MotorDirection.BACKWARD)
        self.set_right(speed, MotorDirection.BACKWARD)

    # ----------------------------------------------------------
    #  PROPRIEDADES
    # ----------------------------------------------------------

    @property
    def left_speed(self) -> float:
        """Duty cycle atual do motor esquerdo [0–100%]."""
        return self._left_duty

    @property
    def right_speed(self) -> float:
        """Duty cycle atual do motor direito [0–100%]."""
        return self._right_duty

    # ----------------------------------------------------------
    #  MÉTODOS PRIVADOS
    # ----------------------------------------------------------

    def _set_motor(self, dir1_pin: int, dir2_pin: int,
                   pwm: 'GPIO.PWM',
                   speed: int, direction: MotorDirection):
        """
        Define estado de um motor individual via pinos L298N.

        L298N Truth Table:
          IN1=H, IN2=L → Forward
          IN1=L, IN2=H → Backward
          IN1=H, IN2=H → Brake (short circuit)
          IN1=L, IN2=L → Coast (free wheel)
        """
        if not self._initialized:
            return

        duty = self._to_duty(self._constrain_speed(speed))

        if direction == MotorDirection.FORWARD:
            GPIO.output(dir1_pin, GPIO.HIGH)
            GPIO.output(dir2_pin, GPIO.LOW)
            pwm.ChangeDutyCycle(duty)

        elif direction == MotorDirection.BACKWARD:
            GPIO.output(dir1_pin, GPIO.LOW)
            GPIO.output(dir2_pin, GPIO.HIGH)
            pwm.ChangeDutyCycle(duty)

        elif direction == MotorDirection.BRAKE:
            GPIO.output(dir1_pin, GPIO.HIGH)
            GPIO.output(dir2_pin, GPIO.HIGH)
            pwm.ChangeDutyCycle(0)

        else:  # COAST
            GPIO.output(dir1_pin, GPIO.LOW)
            GPIO.output(dir2_pin, GPIO.LOW)
            pwm.ChangeDutyCycle(0)

    @staticmethod
    def _to_duty(speed_255: int) -> float:
        """
        Converte escala Arduino [0–255] para duty cycle [0–100%].

        Arduino usa resolução 8-bit (0–255). RPi.GPIO usa porcentagem.
        Conversão linear: duty = speed / 255.0 * 100.0
        """
        return (speed_255 / 255.0) * 100.0

    @staticmethod
    def _constrain_speed(raw: int) -> int:
        """
        Aplica zona morta e limita ao intervalo válido.

        Abaixo de MOTOR_DEADBAND: retorna 0 (motor não gira).
        Acima de MOTOR_MAX_SPEED: retorna MOTOR_MAX_SPEED.
        """
        if raw < MOTOR_DEADBAND:
            return 0
        return min(raw, MOTOR_MAX_SPEED)
