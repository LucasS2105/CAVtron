"""
main.py
=======
Ponto de entrada principal — conversão de main.ino para Raspberry Pi Zero 2 W.

MUDANÇA ARQUITETURAL FUNDAMENTAL:
    No projeto original, o código era dividido entre Arduino (controle RT)
    e Raspberry Pi (processamento). Aqui, TODO o sistema roda em um único
    dispositivo: o Raspberry Pi Zero 2 W.

    Isso elimina a necessidade do módulo serial_comm (comunicação entre
    placas) e centraliza todo o controle no RPi.

Estrutura de threads:
    Thread principal   : Loop de controle a 100 Hz (10ms)
    ServoSweepThread   : Movimentação suave dos servos (daemon)

LIMITAÇÃO IMPORTANTE:
    O RPi roda Linux (não RTOS). O loop de controle pode sofrer
    jitter de até ~1-5ms por preempção do scheduler. Para aplicações
    críticas de controle, usar:
    - nice -20 (prioridade máxima do processo)
    - chrt -f 99 (SCHED_FIFO — real-time scheduling)
    - ou migrar o controle RT de volta para um microcontrolador

Uso:
    python3 main.py
    sudo python3 main.py             (necessário para GPIO)
    sudo chrt -f 99 python3 main.py  (real-time scheduling)
"""

import sys
import time
import signal
import logging

# Importação condicional do GPIO — permite execução em ambiente
# sem hardware para debug (dry-run)
try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except (ImportError, RuntimeError):
    GPIO_AVAILABLE = False
    print("[WARN] RPi.GPIO não disponível — modo simulado ativo.")

from config import (
    LOOP_PERIOD_S, RobotState,
    PID_KP, PID_KI, PID_KD, PID_SAMPLE_TIME_S,
    PID_OUTPUT_MAX, PID_OUTPUT_MIN, PID_INTEGRAL_MAX, PID_DERIVATIVE_ALPHA,
    MOTOR_BASE_SPEED, HUSKYLENS_POLL_S
)
from control.pid_controller   import PIDController, PIDConfig
from drivers.motor_driver      import MotorDriver, MotorDirection
from drivers.servo_driver      import ServoDriver, GripAction
from sensors.line_sensor       import LineSensor
from sensors.huskylens         import HuskyLensInterface
from utils.filters             import EMAFilter, EdgeDetector

# ============================================================
#  CONFIGURAÇÃO DE LOGGING
# ============================================================

logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s] [%(levelname)-8s] [%(name)s] %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger('main')


# ============================================================
#  CLASSE PRINCIPAL
# ============================================================

class RobotController:
    """
    Orquestrador principal — equivalente ao main.ino do Arduino.

    Integra todos os módulos e executa o loop de controle a 100 Hz.
    """

    def __init__(self):
        # Instancia módulos com configuração do config.py
        pid_cfg = PIDConfig(
            kp=PID_KP,
            ki=PID_KI,
            kd=PID_KD,
            sample_time_s=PID_SAMPLE_TIME_S,
            output_max=PID_OUTPUT_MAX,
            output_min=PID_OUTPUT_MIN,
            integral_max=PID_INTEGRAL_MAX,
            derivative_alpha=PID_DERIVATIVE_ALPHA
        )

        self._pid      = PIDController(pid_cfg)
        self._motors   = MotorDriver()
        self._servo    = ServoDriver()
        self._line     = LineSensor()
        self._husky    = HuskyLensInterface()

        # Filtros e detectores de borda
        self._line_pos_filter = EMAFilter(alpha=0.3)
        self._line_lost_edge  = EdgeDetector()
        self._victim_edge     = EdgeDetector()

        # Estado da FSM
        self._state: RobotState = RobotState.IDLE
        self._state_entry_time: float = time.monotonic()

        # Controle do loop
        self._running = False

        # Temporizadores
        self._last_husky_poll = 0.0
        self._last_debug_log  = 0.0
        self._debug_counter   = 0

        # Instala handlers de sinal para shutdown limpo
        signal.signal(signal.SIGINT,  self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    # ----------------------------------------------------------
    #  INICIALIZAÇÃO
    # ----------------------------------------------------------

    def begin(self) -> bool:
        """
        Inicializa GPIO e todos os módulos de hardware.

        Returns:
            True se todos os módulos foram inicializados.
        """
        logger.info("=" * 55)
        logger.info("  Robot Controller — Raspberry Pi Zero 2 W")
        logger.info("=" * 55)

        if GPIO_AVAILABLE:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
        else:
            logger.warning("[INIT] GPIO não disponível — modo simulado.")

        # Motores
        if GPIO_AVAILABLE:
            self._motors.begin()
        logger.info("[INIT] MotorDriver OK")

        # Servos
        if GPIO_AVAILABLE:
            self._servo.begin()
        logger.info("[INIT] ServoDriver OK")

        # Sensores de linha (MCP3008 via SPI)
        try:
            self._line.begin()
            logger.info("[INIT] LineSensor OK (MCP3008 SPI)")
        except Exception as e:
            logger.warning(f"[INIT] LineSensor falhou: {e} — modo simulado.")

        # HuskyLens (I2C)
        try:
            if not self._husky.begin():
                logger.warning("[INIT] HuskyLens não encontrada — continuando sem visão.")
            else:
                logger.info("[INIT] HuskyLens OK (I2C)")
        except Exception as e:
            logger.warning(f"[INIT] HuskyLens falhou: {e}")

        self._pid.reset()
        logger.info("[INIT] PIDController OK")

        self._set_state(RobotState.IDLE)
        logger.info("[INIT] Sistema pronto.")
        return True

    # ----------------------------------------------------------
    #  LOOP PRINCIPAL (100 Hz)
    # ----------------------------------------------------------

    def run(self):
        """
        Loop de controle principal — equivalente ao loop() do Arduino.

        Executa a 100 Hz usando busy-wait adaptativo para manter
        o período. Em sistemas não-RT como Linux, pode sofrer jitter.
        Para precisão crítica, usar sudo chrt -f 99 python3 main.py.
        """
        self._running = True
        logger.info("[Main] Loop principal iniciado @ 100 Hz.")

        while self._running:
            t_start = time.monotonic()

            # ---- 1. Leitura de sensores ----
            try:
                self._line.update()
            except Exception as e:
                logger.debug(f"[Loop] LineSensor error: {e}")

            # ---- 2. Polling da HuskyLens (20 Hz) ----
            now = time.monotonic()
            if (now - self._last_husky_poll) >= HUSKYLENS_POLL_S:
                self._last_husky_poll = now
                try:
                    self._husky.update()
                except Exception as e:
                    logger.debug(f"[Loop] HuskyLens error: {e}")

            # ---- 3. Executa FSM ----
            self._run_fsm()

            # ---- 4. Detectores de borda e notificações ----
            self._line_lost_edge.update(self._line.line_lost)
            self._victim_edge.update(self._husky.victim_detected)

            if self._line_lost_edge.rising_edge:
                logger.warning("[FSM] Linha perdida!")

            # ---- 5. Log de telemetria a 1 Hz ----
            self._debug_counter += 1
            if self._debug_counter >= 100:
                self._debug_counter = 0
                self._log_telemetry()

            # ---- 6. Mantém período de 10ms ----
            elapsed  = time.monotonic() - t_start
            sleep_t  = LOOP_PERIOD_S - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    # ----------------------------------------------------------
    #  FSM — MÁQUINA DE ESTADOS
    # ----------------------------------------------------------

    def _run_fsm(self):
        """
        Executa o estado atual da FSM.
        Espelho exato da função runFSM() do main.ino.
        """
        state = self._state

        if state == RobotState.IDLE:
            if GPIO_AVAILABLE:
                self._motors.stop_all()

        elif state == RobotState.FOLLOW_LINE:
            self._run_line_following()

        elif state == RobotState.OBSTACLE:
            if GPIO_AVAILABLE:
                self._motors.stop_all()
            self._pid.reset()

        elif state == RobotState.GRIP_OPEN:
            if not self._servo.is_busy:
                logger.info("[FSM] Garra aberta.")
                self._set_state(RobotState.FOLLOW_LINE)

        elif state == RobotState.GRIP_CLOSE:
            if not self._servo.is_busy:
                self._set_state(RobotState.GRIP_LIFT)

        elif state == RobotState.GRIP_LIFT:
            if not self._servo.is_busy:
                logger.info("[FSM] Captura concluída.")
                self._set_state(RobotState.FOLLOW_LINE)

        elif state == RobotState.GRIP_DROP:
            if not self._servo.is_busy:
                if GPIO_AVAILABLE:
                    self._servo.execute_action(GripAction.OPEN)
                self._set_state(RobotState.GRIP_OPEN)

        elif state == RobotState.EMERGENCY:
            self._handle_emergency()

    # ----------------------------------------------------------
    #  SEGUIMENTO DE LINHA
    # ----------------------------------------------------------

    def _run_line_following(self):
        """
        Controle PID de seguimento de linha.
        Equivalente a runLineFollowing() do main.ino.
        """
        if self._line.line_lost:
            self._handle_line_lost()
            return

        # Filtra posição antes de enviar ao PID
        filtered_pos = self._line_pos_filter.update(self._line.position)

        # PID: setpoint=0.0 (linha centrada), PV=posição filtrada
        pid_output = self._pid.compute(0.0, filtered_pos)

        if GPIO_AVAILABLE:
            self._motors.apply_pid_steering(pid_output)

    def _handle_line_lost(self):
        """
        Manobra de recuperação de linha.
        Rotaciona na direção da última posição válida.
        Equivalente a handleLineLost() do main.ino.
        """
        self._pid.reset()
        last_pos = self._line.data.position
        if GPIO_AVAILABLE:
            if last_pos > 0.0:
                self._motors.rotate_right(120)
            else:
                self._motors.rotate_left(120)

    # ----------------------------------------------------------
    #  CAPTURA DE VÍTIMA
    # ----------------------------------------------------------

    def capture_victim(self):
        """
        Inicia a sequência de captura de vítima.

        Equivalente ao handler CMD_GRIP com FULL_CAPTURE do main.ino.
        A sequência roda em thread separada para não bloquear o loop.
        """
        import threading

        def _capture():
            if GPIO_AVAILABLE:
                self._servo.execute_action(GripAction.FULL_CAPTURE)
            self._set_state(RobotState.GRIP_LIFT)

        threading.Thread(target=_capture, daemon=True).start()
        self._set_state(RobotState.GRIP_CLOSE)

    # ----------------------------------------------------------
    #  PARADA DE EMERGÊNCIA
    # ----------------------------------------------------------

    def _handle_emergency(self):
        """
        Parada de emergência — para tudo e aguarda reset.
        Equivalente a handleEmergencyStop() do main.ino.
        """
        if GPIO_AVAILABLE:
            self._motors.stop_all()
            self._servo.set_grip_direct(90)  # Abre garra por segurança
        self._pid.reset()

        # Retorna ao IDLE após 500ms
        if (time.monotonic() - self._state_entry_time) > 0.5:
            self._set_state(RobotState.IDLE)

    # ----------------------------------------------------------
    #  CONTROLE EXTERNO (API pública)
    # ----------------------------------------------------------

    def set_state(self, state: RobotState):
        """Define estado da FSM externamente."""
        self._set_state(state)

    def send_move(self, direction: str, speed: int):
        """
        Executa um comando de movimento.

        Args:
            direction: 'FWD' | 'BWD' | 'ROT_L' | 'ROT_R' | 'STOP'
            speed    : Velocidade [0–255]
        """
        if not GPIO_AVAILABLE:
            return
        if   direction == 'FWD':   self._motors.move_forward(speed)
        elif direction == 'BWD':   self._motors.move_backward(speed)
        elif direction == 'ROT_L': self._motors.rotate_left(speed)
        elif direction == 'ROT_R': self._motors.rotate_right(speed)
        elif direction == 'STOP':  self._motors.stop_all()

    def send_grip(self, action: str):
        """
        Aciona a garra.

        Args:
            action: 'OPEN' | 'CLOSE' | 'CAPTURE'
        """
        if not GPIO_AVAILABLE:
            return
        mapping = {
            'OPEN'   : GripAction.OPEN,
            'CLOSE'  : GripAction.CLOSE,
            'CAPTURE': GripAction.FULL_CAPTURE,
            'UP'     : GripAction.LIFT_UP,
            'DOWN'   : GripAction.LIFT_DOWN,
        }
        grip_action = mapping.get(action.upper(), GripAction.OPEN)
        self._servo.execute_action(grip_action)

    # ----------------------------------------------------------
    #  UTILITÁRIOS
    # ----------------------------------------------------------

    def _set_state(self, new_state: RobotState):
        """Transiciona a FSM para um novo estado."""
        if new_state == self._state:
            return
        logger.info(f"[FSM] {self._state.name} → {new_state.name}")
        self._state            = new_state
        self._state_entry_time = time.monotonic()

    def _log_telemetry(self):
        """Log de telemetria a 1 Hz (a cada 100 iterações)."""
        line = self._line.data
        victim = self._husky.data
        pid = self._pid

        logger.info(
            f"[TELEM] Estado={self._state.name} | "
            f"Linha={line.position:+.2f}({'OK' if line.line_detected else 'LOST'}) | "
            f"P={pid.term_p:+.1f} I={pid.term_i:+.1f} D={pid.term_d:+.1f} | "
            f"Vítima={'SIM' if victim.detected else 'NÃO'}"
            + (f" [x={victim.normalized_x:+.2f}]" if victim.detected else "")
        )

    def _signal_handler(self, signum, frame):
        """Handler de SIGINT/SIGTERM — dispara shutdown limpo."""
        logger.info(f"\n[Main] Sinal {signum} recebido. Encerrando...")
        self._running = False

    # ----------------------------------------------------------
    #  SHUTDOWN
    # ----------------------------------------------------------

    def shutdown(self):
        """Encerramento limpo de todos os módulos."""
        logger.info("[Main] Shutdown iniciado...")
        self._running = False

        if GPIO_AVAILABLE:
            self._motors.stop_all()
            self._motors.cleanup()
            self._servo.cleanup()

        try:
            self._line.cleanup()
        except Exception:
            pass

        try:
            self._husky.cleanup()
        except Exception:
            pass

        if GPIO_AVAILABLE:
            GPIO.cleanup()

        logger.info("[Main] Shutdown concluído.")


# ============================================================
#  ENTRY POINT
# ============================================================

def main():
    robot = RobotController()

    if not robot.begin():
        logger.critical("[Main] Falha na inicialização.")
        sys.exit(1)

    # Inicia em modo FOLLOW_LINE automaticamente
    robot.set_state(RobotState.FOLLOW_LINE)

    try:
        robot.run()
    except Exception as e:
        logger.critical(f"[Main] Exceção não tratada: {e}", exc_info=True)
    finally:
        robot.shutdown()


if __name__ == '__main__':
    main()
