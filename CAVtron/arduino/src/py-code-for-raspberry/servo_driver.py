"""
drivers/servo_driver.py
========================
Controle de servomotores — conversão de servo_driver.h/.cpp para RPi.GPIO.

Mapeamento de ângulo → duty cycle para servo padrão (50 Hz):
    Pulso mínimo : 0.5ms → duty = 0.5/20.0 × 100 = 2.5%   → 0°
    Pulso central: 1.5ms → duty = 1.5/20.0 × 100 = 7.5%   → 90°
    Pulso máximo : 2.5ms → duty = 2.5/20.0 × 100 = 12.5%  → 180°

    duty(angle) = SERVO_DUTY_MIN + (angle / 180.0) × (SERVO_DUTY_MAX − SERVO_DUTY_MIN)

O sweep assíncrono é implementado via threading.Thread em vez do
loop não-bloqueante do Arduino — mesma semântica de não bloquear o
ciclo de controle principal.

Nota: Para servos que exigem alta precisão de temporização, substituir
RPi.GPIO.PWM por pigpio (hardware DMA PWM) para eliminar jitter de software.
"""

import time
import logging
import threading
import RPi.GPIO as GPIO
from enum import IntEnum
from config import (
    SERVO_GRIP_PIN, SERVO_LIFT_PIN,
    SERVO_GRIP_OPEN_DEG, SERVO_GRIP_CLOSE_DEG,
    SERVO_LIFT_UP_DEG,   SERVO_LIFT_DOWN_DEG,
    SERVO_PWM_FREQ_HZ, SERVO_SWEEP_DELAY_S,
    SERVO_DUTY_MIN, SERVO_DUTY_MAX
)

logger = logging.getLogger(__name__)


class GripAction(IntEnum):
    """Ações do manipulador — espelho de GripAction_t."""
    OPEN         = 0
    CLOSE        = 1
    LIFT_UP      = 2
    LIFT_DOWN    = 3
    FULL_CAPTURE = 4   # Sequência atômica: descer → fechar → subir


class ServoDriver:
    """
    Controlador de servomotores para o manipulador do robô.

    O sweep (movimento suave) é executado em thread daemon separada,
    garantindo que o loop de controle principal não seja bloqueado.

    Args:
        freq_hz       : Frequência PWM dos servos em Hz (padrão: 50).
        sweep_delay_s : Intervalo entre passos de 1° em segundos.
    """

    def __init__(self,
                 freq_hz: float = SERVO_PWM_FREQ_HZ,
                 sweep_delay_s: float = SERVO_SWEEP_DELAY_S):
        self._freq        = freq_hz
        self._sweep_delay = sweep_delay_s

        self._pwm_grip = None
        self._pwm_lift = None

        # Posições atuais e alvos (em graus)
        self._grip_current: int = SERVO_GRIP_OPEN_DEG
        self._grip_target: int  = SERVO_GRIP_OPEN_DEG
        self._lift_current: int = SERVO_LIFT_UP_DEG
        self._lift_target: int  = SERVO_LIFT_UP_DEG

        # Flags de ocupação
        self._grip_busy: bool = False
        self._lift_busy: bool = False
        self._busy_lock = threading.Lock()

        # Thread de sweep (daemon: termina com o processo principal)
        self._sweep_thread = threading.Thread(
            target=self._sweep_loop,
            name='ServoSweepThread',
            daemon=True
        )
        self._running = False

    # ----------------------------------------------------------
    #  INICIALIZAÇÃO
    # ----------------------------------------------------------

    def begin(self):
        """
        Acopla os servos e os posiciona nas posições iniciais seguras.

        Garra aberta, braço elevado — evita colisões na partida.
        GPIO.setmode(GPIO.BCM) deve ter sido chamado antes.
        """
        GPIO.setup(SERVO_GRIP_PIN, GPIO.OUT)
        GPIO.setup(SERVO_LIFT_PIN, GPIO.OUT)

        self._pwm_grip = GPIO.PWM(SERVO_GRIP_PIN, self._freq)
        self._pwm_lift = GPIO.PWM(SERVO_LIFT_PIN, self._freq)

        self._pwm_grip.start(self._angle_to_duty(SERVO_GRIP_OPEN_DEG))
        self._pwm_lift.start(self._angle_to_duty(SERVO_LIFT_UP_DEG))

        time.sleep(0.5)  # Aguarda servos atingirem posição inicial

        self._running = True
        self._sweep_thread.start()

        logger.info("[ServoDriver] Inicializado. Garra=ABERTA, Braço=CIMA.")

    def cleanup(self):
        """Para os PWM e libera os pinos."""
        self._running = False
        if self._pwm_grip:
            self._pwm_grip.stop()
        if self._pwm_lift:
            self._pwm_lift.stop()
        logger.info("[ServoDriver] Cleanup realizado.")

    # ----------------------------------------------------------
    #  EXECUÇÃO DE AÇÕES
    # ----------------------------------------------------------

    def execute_action(self, action: GripAction):
        """
        Aciona uma operação do manipulador.

        Ações simples (OPEN, CLOSE, LIFT_UP, LIFT_DOWN) iniciam o sweep
        assíncrono e retornam imediatamente.

        FULL_CAPTURE executa a sequência atômica de forma bloqueante
        (o thread de chamada aguarda a conclusão), garantindo
        o sequenciamento correto: descer → fechar → subir.

        Args:
            action: Ação a executar (GripAction).
        """
        if action == GripAction.OPEN:
            self._set_grip_target(SERVO_GRIP_OPEN_DEG)

        elif action == GripAction.CLOSE:
            self._set_grip_target(SERVO_GRIP_CLOSE_DEG)

        elif action == GripAction.LIFT_UP:
            self._set_lift_target(SERVO_LIFT_UP_DEG)

        elif action == GripAction.LIFT_DOWN:
            self._set_lift_target(SERVO_LIFT_DOWN_DEG)

        elif action == GripAction.FULL_CAPTURE:
            self._full_capture_sequence()

    def _full_capture_sequence(self):
        """
        Sequência atômica de captura (bloqueante).

        1. Abaixa o braço
        2. Fecha a garra
        3. Eleva o braço

        Bloqueante por design para garantir sequenciamento correto.
        Deve ser chamado em thread separada se não quiser bloquear o loop.
        """
        logger.info("[ServoDriver] Iniciando sequência FULL_CAPTURE.")
        self.set_lift_direct(SERVO_LIFT_DOWN_DEG)
        time.sleep(0.5)
        self.set_grip_direct(SERVO_GRIP_CLOSE_DEG)
        time.sleep(0.4)
        self.set_lift_direct(SERVO_LIFT_UP_DEG)
        time.sleep(0.5)
        logger.info("[ServoDriver] Sequência FULL_CAPTURE concluída.")

    # ----------------------------------------------------------
    #  CONTROLE DIRETO (sem sweep — movimento imediato)
    # ----------------------------------------------------------

    def set_grip_direct(self, angle: int):
        """
        Move a garra diretamente para o ângulo especificado (sem sweep).

        Args:
            angle: Ângulo em graus [0–180].
        """
        angle = max(0, min(180, angle))
        with self._busy_lock:
            self._grip_current = angle
            self._grip_target  = angle
            self._grip_busy    = False
        if self._pwm_grip:
            self._pwm_grip.ChangeDutyCycle(self._angle_to_duty(angle))

    def set_lift_direct(self, angle: int):
        """
        Move o braço diretamente para o ângulo especificado (sem sweep).

        Args:
            angle: Ângulo em graus [0–180].
        """
        angle = max(0, min(180, angle))
        with self._busy_lock:
            self._lift_current = angle
            self._lift_target  = angle
            self._lift_busy    = False
        if self._pwm_lift:
            self._pwm_lift.ChangeDutyCycle(self._angle_to_duty(angle))

    # ----------------------------------------------------------
    #  PROPRIEDADES
    # ----------------------------------------------------------

    @property
    def is_busy(self) -> bool:
        """True se algum servo está em movimento (sweep em andamento)."""
        with self._busy_lock:
            return self._grip_busy or self._lift_busy

    @property
    def grip_angle(self) -> int:
        return self._grip_current

    @property
    def lift_angle(self) -> int:
        return self._lift_current

    # ----------------------------------------------------------
    #  THREAD DE SWEEP (DAEMON)
    # ----------------------------------------------------------

    def _sweep_loop(self):
        """
        Loop da thread de sweep — executa um passo por intervalo.

        Move 1° por ciclo em direção ao ângulo alvo.
        Slew rate = 1° / sweep_delay_s

        Equivalente ao método update() do Arduino, mas em thread
        dedicada em vez de ser chamado no loop principal.
        """
        while self._running:
            with self._busy_lock:
                # Passo da garra
                if self._grip_busy and self._grip_current != self._grip_target:
                    step = 1 if self._grip_current < self._grip_target else -1
                    self._grip_current += step
                    if self._pwm_grip:
                        self._pwm_grip.ChangeDutyCycle(
                            self._angle_to_duty(self._grip_current)
                        )
                    if self._grip_current == self._grip_target:
                        self._grip_busy = False

                # Passo do braço
                if self._lift_busy and self._lift_current != self._lift_target:
                    step = 1 if self._lift_current < self._lift_target else -1
                    self._lift_current += step
                    if self._pwm_lift:
                        self._pwm_lift.ChangeDutyCycle(
                            self._angle_to_duty(self._lift_current)
                        )
                    if self._lift_current == self._lift_target:
                        self._lift_busy = False

            time.sleep(self._sweep_delay)

    # ----------------------------------------------------------
    #  UTILITÁRIOS INTERNOS
    # ----------------------------------------------------------

    def _set_grip_target(self, angle: int):
        with self._busy_lock:
            self._grip_target = max(0, min(180, angle))
            self._grip_busy   = True

    def _set_lift_target(self, angle: int):
        with self._busy_lock:
            self._lift_target = max(0, min(180, angle))
            self._lift_busy   = True

    @staticmethod
    def _angle_to_duty(angle: int) -> float:
        """
        Converte ângulo em graus para duty cycle (%).

        Mapeamento linear entre SERVO_DUTY_MIN (0°) e SERVO_DUTY_MAX (180°).
        duty = DUTY_MIN + (angle / 180.0) × (DUTY_MAX − DUTY_MIN)

        Args:
            angle: Ângulo em graus [0, 180].
        Returns:
            Duty cycle em porcentagem [DUTY_MIN, DUTY_MAX].
        """
        angle = max(0, min(180, angle))
        return SERVO_DUTY_MIN + (angle / 180.0) * (SERVO_DUTY_MAX - SERVO_DUTY_MIN)
