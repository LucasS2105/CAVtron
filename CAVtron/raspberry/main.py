"""
main.py
=======
Ponto de entrada principal da Camada de Processamento (Raspberry Pi).

Instancia, configura e orquestra todos os módulos do sistema:
    - SerialComm       : Comunicação com o Arduino
    - LidarInterface   : Aquisição de varreduras LIDAR
    - OccupancyGrid    : Mapeamento probabilístico da arena
    - Odometry         : Estimação de pose por dead reckoning
    - ObstacleDetector : Detecção e classificação de obstáculos
    - StateMachine     : FSM de controle de missão
    - DecisionSystem   : Núcleo de decisão integrado

Loop principal:
    O loop roda a ~50 Hz (período = 20ms), suficiente para processar
    varreduras LIDAR (~10Hz) e dados seriais (~20Hz) com margem.
    A thread de recepção serial (daemon) opera independentemente.

Sinais de sistema:
    SIGINT (Ctrl+C) e SIGTERM disparam shutdown limpo:
    motores parados, LIDAR desligado, serial fechada.

Configuração:
    Todos os parâmetros de hardware são lidos de config/robot_config.yaml.
    Fallback para valores padrão se o arquivo não existir.
"""

import sys
import time
import signal
import logging
import threading
from pathlib import Path

# Ajusta o path para importar módulos locais corretamente
sys.path.insert(0, str(Path(__file__).parent))

from communication.serial_comm import SerialComm
from mapping.lidar_interface import LidarInterface
from mapping.mapping import OccupancyGrid, Odometry
from mapping.obstacle_detection import ObstacleDetector
from control.state_machine import StateMachine, MissionState, FSMEvent
from decision.decision_system import DecisionSystem
from utils.helpers import load_config, setup_logging, LoopTimer

# ============================================================
#  CONFIGURAÇÃO DE LOGGING
# ============================================================

setup_logging(level=logging.INFO)
logger = logging.getLogger(__name__)


# ============================================================
#  CONFIGURAÇÕES (com fallback para defaults)
# ============================================================

CONFIG_PATH = Path(__file__).parent.parent.parent / 'config' / 'robot_config.yaml'

def load_robot_config() -> dict:
    """Carrega configuração do YAML ou retorna defaults."""
    defaults = {
        'serial': {
            'port'        : '/dev/serial0',
            'baudrate'    : 115200,
            'watchdog_s'  : 2.0
        },
        'lidar': {
            'port'           : '/dev/ttyUSB0',
            'obstacle_dist_mm': 350.0,
            'warning_dist_mm' : 600.0
        },
        'arena': {
            'width_m'     : 3.0,
            'height_m'    : 3.0,
            'resolution_m': 0.05
        },
        'mission': {
            'victims_target'  : 1,
            'loop_period_ms'  : 20,
            'ping_interval_s' : 1.0
        }
    }

    if CONFIG_PATH.exists():
        loaded = load_config(str(CONFIG_PATH))
        # Merge com defaults (loaded sobrescreve)
        for section, values in loaded.items():
            if section in defaults:
                defaults[section].update(values)
        logger.info(f"[Main] Configuração carregada de {CONFIG_PATH}")
    else:
        logger.warning(f"[Main] {CONFIG_PATH} não encontrado. Usando defaults.")

    return defaults


# ============================================================
#  CLASSE PRINCIPAL DO SISTEMA
# ============================================================

class RobotSystem:
    """
    Orquestrador do sistema completo do Raspberry Pi.

    Gerencia o ciclo de vida de todos os módulos e o loop principal.
    """

    def __init__(self):
        self._cfg     = load_robot_config()
        self._running = False
        self._shutdown_event = threading.Event()

        # Instancia módulos
        self._comm = SerialComm(
            port=self._cfg['serial']['port'],
            baudrate=self._cfg['serial']['baudrate'],
            watchdog_s=self._cfg['serial']['watchdog_s']
        )

        self._lidar = LidarInterface(
            port=self._cfg['lidar']['port'],
            obstacle_dist_mm=self._cfg['lidar']['obstacle_dist_mm']
        )

        self._grid = OccupancyGrid(
            width_m=self._cfg['arena']['width_m'],
            height_m=self._cfg['arena']['height_m'],
            resolution_m=self._cfg['arena']['resolution_m']
        )

        self._odometry = Odometry()

        self._detector = ObstacleDetector(
            obstacle_dist_mm=self._cfg['lidar']['obstacle_dist_mm'],
            warning_dist_mm=self._cfg['lidar']['warning_dist_mm']
        )

        self._fsm = StateMachine(initial_state=MissionState.IDLE)

        self._decision = DecisionSystem(
            comm=self._comm,
            lidar=self._lidar,
            detector=self._detector,
            fsm=self._fsm
        )

        # Configura callbacks FSM para logging e integração com mapeamento
        self._setup_fsm_callbacks()

        # Registra handlers de sinal do sistema
        signal.signal(signal.SIGINT,  self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

        # Temporizadores auxiliares
        self._ping_interval  = self._cfg['mission']['ping_interval_s']
        self._loop_period_ms = self._cfg['mission']['loop_period_ms']
        self._last_ping_time = 0.0
        self._last_map_update = 0.0

    # ----------------------------------------------------------
    #  INICIALIZAÇÃO
    # ----------------------------------------------------------

    def start(self) -> bool:
        """
        Inicializa todos os módulos e inicia o loop principal.

        Returns:
            True se todos os módulos foram inicializados com sucesso.
        """
        logger.info("=" * 50)
        logger.info("  Robot Rescue System — Raspberry Pi Layer")
        logger.info("=" * 50)

        # Conexão serial com o Arduino
        if not self._comm.connect():
            logger.error("[Main] Falha na conexão serial com Arduino. Abortando.")
            return False

        # Aguarda Arduino inicializar e responder ao PING
        logger.info("[Main] Aguardando Arduino...")
        for attempt in range(5):
            self._comm.send_ping()
            time.sleep(0.5)
            self._comm.process_queue()
            if not self._comm.is_watchdog_expired():
                logger.info("[Main] Arduino responsivo.")
                break
            if attempt == 4:
                logger.error("[Main] Arduino não respondeu. Continuando sem confirmação.")

        # Inicializa o LIDAR
        if not self._lidar.start():
            logger.warning("[Main] LIDAR não disponível. Continuando sem mapeamento.")

        # Aguarda primeira varredura do LIDAR
        logger.info("[Main] Aguardando primeira varredura LIDAR...")
        for _ in range(20):
            time.sleep(0.1)
            if self._lidar.get_latest_scan() is not None:
                logger.info("[Main] LIDAR: primeira varredura recebida.")
                break

        self._running = True
        logger.info("[Main] Sistema iniciado. Iniciando missão...")

        # Inicia a missão automaticamente
        self._decision.start_mission()

        return True

    # ----------------------------------------------------------
    #  LOOP PRINCIPAL
    # ----------------------------------------------------------

    def run(self):
        """
        Loop principal de controle (~50 Hz).

        Pipeline por iteração:
        1. Timing: garante período fixo
        2. Decisão: executa um ciclo do sistema de decisão
        3. Mapeamento: integra varredura na grade de ocupação (10Hz)
        4. Healthcheck: PING periódico ao Arduino
        5. Telemetria: log de estado (1Hz)
        """
        loop_timer  = LoopTimer(self._loop_period_ms / 1000.0)
        log_counter = 0

        logger.info("[Main] Loop principal iniciado.")

        while self._running and not self._shutdown_event.is_set():
            loop_timer.start()

            now = time.monotonic()

            # --- Ciclo de Decisão ---
            self._decision.update()

            # --- Atualização do Mapa (10 Hz) ---
            if (now - self._last_map_update) >= 0.1:
                self._last_map_update = now
                scan = self._lidar.get_latest_scan()
                if scan and not scan.is_empty():
                    self._grid.update(scan, self._odometry.pose)

            # --- PING de healthcheck (1 Hz) ---
            if (now - self._last_ping_time) >= self._ping_interval:
                self._last_ping_time = now
                self._comm.send_ping()

            # --- Log de telemetria (1 Hz ≈ a cada 50 iterações) ---
            log_counter += 1
            if log_counter >= int(1000 / self._loop_period_ms):
                log_counter = 0
                self._log_telemetry()

            # --- Aguarda fim do período ---
            loop_timer.sleep()

        logger.info("[Main] Loop principal encerrado.")

    # ----------------------------------------------------------
    #  SHUTDOWN
    # ----------------------------------------------------------

    def shutdown(self):
        """Encerramento limpo de todos os módulos."""
        logger.info("[Main] Iniciando shutdown...")
        self._running = False

        # Para motores imediatamente
        try:
            self._comm.send_stop()
            time.sleep(0.1)
        except Exception:
            pass

        # Para o LIDAR
        self._lidar.stop()

        # Fecha serial
        self._comm.disconnect()

        logger.info("[Main] Shutdown concluído.")

    def _signal_handler(self, signum, frame):
        """Handler para SIGINT/SIGTERM — dispara shutdown limpo."""
        logger.info(f"\n[Main] Sinal {signum} recebido. Encerrando...")
        self._shutdown_event.set()
        self._running = False

    # ----------------------------------------------------------
    #  CONFIGURAÇÃO DE CALLBACKS FSM
    # ----------------------------------------------------------

    def _setup_fsm_callbacks(self):
        """Registra callbacks de entrada/saída nos estados da FSM."""
        fsm = self._fsm

        # Ao entrar em EMERGENCY: para tudo
        @fsm.on_enter(MissionState.EMERGENCY)
        def on_emergency(state, ctx):
            logger.critical("[FSM] EMERGÊNCIA ATIVADA — parando todos os sistemas.")
            self._comm.send_stop()

        # Ao entrar em MISSION_COMPLETE: log de conclusão
        @fsm.on_enter(MissionState.MISSION_COMPLETE)
        def on_complete(state, ctx):
            logger.info(
                f"[FSM] MISSÃO CONCLUÍDA! "
                f"Vítimas resgatadas: {ctx.victims_captured}"
            )
            self._comm.send_stop()

        # Ao entrar em OBSTACLE_AVOID: para Arduino e assume controle
        @fsm.on_enter(MissionState.OBSTACLE_AVOID)
        def on_obstacle(state, ctx):
            self._comm.send_state(2)  # STATE_OBSTACLE no Arduino

    # ----------------------------------------------------------
    #  TELEMETRIA
    # ----------------------------------------------------------

    def _log_telemetry(self):
        """Loga estado atual do sistema para diagnóstico."""
        ctx   = self._fsm.context
        pose  = self._odometry.pose
        scans = self._lidar.get_scan_count()

        logger.info(
            f"[TELEM] Estado={self._fsm.state.name} | "
            f"Linha={ctx.line_position:+.2f}({'OK' if ctx.line_detected else 'LOST'}) | "
            f"Vítima={'SIM' if ctx.victim_detected else 'NÃO'} | "
            f"Pose=({pose.x_m:.2f}m, {pose.y_m:.2f}m, "
            f"{pose.yaw_rad * 57.3:.0f}°) | "
            f"Scans={scans}"
        )


# ============================================================
#  ENTRY POINT
# ============================================================

def main():
    system = RobotSystem()

    if not system.start():
        logger.critical("[Main] Falha na inicialização. Encerrando.")
        sys.exit(1)

    try:
        system.run()
    except Exception as e:
        logger.critical(f"[Main] Exceção não tratada: {e}", exc_info=True)
    finally:
        system.shutdown()


if __name__ == '__main__':
    main()
