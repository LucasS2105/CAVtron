"""
decision/decision_system.py
============================
Sistema de Decisão — integração de múltiplos sensores e geração de comandos.

Implementa a lógica central de tomada de decisão do Raspberry Pi,
integrando dados de:
    - Sensores de linha (via Arduino/Serial)
    - LIDAR (mapeamento e obstáculos)
    - HuskyLens (detecção de vítimas)

O sistema opera com uma hierarquia de prioridade explícita, garantindo
comportamento determinístico em situações de conflito entre sensores:

    PRIORIDADE (decrescente):
    1. EMERGÊNCIA (watchdog, falha de hardware)
    2. OBSTÁCULO CRÍTICO (colisão iminente < obstacle_dist)
    3. VÍTIMA DETECTADA (captura disponível)
    4. DESVIO DE OBSTÁCULO (obstáculo próximo, manobra em andamento)
    5. SEGUIMENTO DE LINHA (comportamento padrão)

Essa hierarquia é análoga ao conceito de Behavior Trees (BT) simplificado,
onde behaviors com maior prioridade "preemptam" behaviors de menor prioridade.

Referência:
    Colledanchise, M., Ogren, P. (2018). Behavior Trees in Robotics
    and AI. CRC Press.
"""

import time
import logging
from typing import Optional

from control.state_machine import StateMachine, MissionState, FSMEvent, StateContext
from mapping.obstacle_detection import ObstacleDetector, AvoidanceCommand
from mapping.lidar_interface import LidarInterface
from communication.serial_comm import SerialComm, SensorData, ParsedMessage

logger = logging.getLogger(__name__)


# ============================================================
#  PARÂMETROS DO SISTEMA DE DECISÃO
# ============================================================

class DecisionConfig:
    """Parâmetros configuráveis do sistema de decisão."""

    # Velocidades de operação (PWM, 0-255)
    BASE_SPEED        = 140   # Velocidade de cruzeiro no seguimento
    APPROACH_SPEED    = 90    # Velocidade de aproximação de vítima
    AVOIDANCE_SPEED   = 110   # Velocidade durante desvio
    ROTATION_SPEED    = 120   # Velocidade de rotação in-loco

    # Limiares de distância
    OBSTACLE_DIST_MM  = 350.0   # Parada/desvio imediato
    WARNING_DIST_MM   = 600.0   # Redução de velocidade

    # Limiar de captura: área da bounding box da vítima (proxy de distância)
    CAPTURE_AREA_THRESHOLD = 0.08   # 8% da área da imagem ≈ vítima próxima

    # Tempo mínimo para confirmar "obstáculo limpo" (debounce)
    OBSTACLE_CLEAR_DEBOUNCE_S = 0.5

    # Tempo máximo de manobra de desvio antes de abortar
    AVOIDANCE_TIMEOUT_S = 8.0

    # Orientação de aproximação da vítima (normalizado [-1, +1])
    # Alinha o robô com a vítima antes de capturar
    VICTIM_CENTER_DEADBAND = 0.15


# ============================================================
#  SISTEMA DE DECISÃO
# ============================================================

class DecisionSystem:
    """
    Núcleo de decisão que integra sensores e comanda o Arduino.

    Args:
        comm     : Instância de SerialComm para envio de comandos
        lidar    : Instância de LidarInterface para dados de varredura
        detector : Instância de ObstacleDetector
        fsm      : Instância de StateMachine com o estado de missão
        config   : Parâmetros de configuração (DecisionConfig)
    """

    def __init__(self,
                 comm: SerialComm,
                 lidar: LidarInterface,
                 detector: ObstacleDetector,
                 fsm: StateMachine,
                 config: DecisionConfig = None):
        self._comm     = comm
        self._lidar    = lidar
        self._detector = detector
        self._fsm      = fsm
        self._cfg      = config or DecisionConfig()

        # Último dado de sensor recebido do Arduino
        self._sensor_data = SensorData()

        # Timestamps para debounce e timeout
        self._obstacle_clear_start: Optional[float] = None
        self._avoidance_start_time: Optional[float] = None

        # Registra callbacks para mensagens do Arduino
        self._register_arduino_callbacks()

        logger.info("[DecisionSystem] Inicializado.")

    # ----------------------------------------------------------
    #  CICLO PRINCIPAL DE DECISÃO
    # ----------------------------------------------------------

    def update(self):
        """
        Executa um ciclo do sistema de decisão.

        Deve ser chamado periodicamente no loop principal (~20Hz).
        Pipeline:
        1. Processa mensagens pendentes do Arduino
        2. Analisa dados do LIDAR
        3. Avalia hierarquia de prioridade
        4. Gera e envia comandos
        5. Dispara eventos na FSM se necessário
        """
        # 1. Processa fila de mensagens do Arduino
        self._comm.process_queue()

        # 2. Verifica watchdog de comunicação
        if self._comm.is_watchdog_expired():
            logger.warning("[DecisionSystem] Watchdog de comunicação expirado!")
            self._comm.send_stop()
            self._fsm.trigger(FSMEvent.EMERGENCY)
            return

        # 3. Obtém análise do LIDAR
        scan = self._lidar.get_latest_scan()
        avoidance_cmd = None
        if scan and not scan.is_empty():
            avoidance_cmd = self._detector.analyze(scan)

        # 4. Executa decisão baseada no estado atual da FSM
        state = self._fsm.state
        ctx   = self._fsm.context

        if state == MissionState.IDLE:
            self._decide_idle()
        elif state == MissionState.EXPLORING:
            self._decide_exploring(avoidance_cmd)
        elif state == MissionState.OBSTACLE_AVOID:
            self._decide_obstacle_avoid(avoidance_cmd)
        elif state == MissionState.VICTIM_APPROACH:
            self._decide_victim_approach()
        elif state == MissionState.VICTIM_CAPTURE:
            self._decide_victim_capture()
        elif state == MissionState.VICTIM_TRANSPORT:
            self._decide_victim_transport()
        elif state == MissionState.EMERGENCY:
            self._decide_emergency()

    # ----------------------------------------------------------
    #  DECISÕES POR ESTADO
    # ----------------------------------------------------------

    def _decide_idle(self):
        """IDLE: mantém motores parados."""
        self._comm.send_stop()

    def _decide_exploring(self, avoidance: Optional[AvoidanceCommand]):
        """
        EXPLORING: segue linha com PID e monitora obstáculos e vítimas.

        Prioridade neste estado:
        1. Obstáculo crítico → transição OBSTACLE_AVOID
        2. Vítima detectada  → transição VICTIM_APPROACH
        3. Padrão            → seguimento de linha (Arduino PID)
        """
        ctx = self._fsm.context

        # Prioridade 1: Obstáculo crítico
        if avoidance and avoidance.action in ('STOP', 'TURN_LEFT', 'TURN_RIGHT'):
            if avoidance.urgency > 0.3:
                logger.info(f"[Decision] Obstáculo detectado: {avoidance.reason}")
                self._fsm.trigger(FSMEvent.OBSTACLE_DETECTED)
                self._avoidance_start_time = time.monotonic()
                return

        # Prioridade 2: Vítima detectada
        if ctx.victim_detected:
            logger.info("[Decision] Vítima detectada — iniciando aproximação.")
            self._fsm.trigger(FSMEvent.VICTIM_DETECTED)
            return

        # Padrão: delega seguimento de linha ao Arduino (PID ativo)
        # Envia SET_STATE=FOLLOW_LINE (estado 1 conforme RobotState_t)
        # Apenas reenvia se a linha estiver detectada
        if not ctx.line_detected:
            logger.warning("[Decision] Linha perdida durante exploração.")

    def _decide_obstacle_avoid(self, avoidance: Optional[AvoidanceCommand]):
        """
        OBSTACLE_AVOID: executa manobra de desvio.

        Estratégia Bug-0 simplificada:
        - Para o Arduino
        - Identifica o lado com mais espaço
        - Rotaciona até o caminho frontal estar livre
        - Retoma exploração

        Timeout de segurança evita loop infinito.
        """
        if avoidance is None:
            return

        # Timeout de desvio
        if (self._avoidance_start_time and
                (time.monotonic() - self._avoidance_start_time)
                > self._cfg.AVOIDANCE_TIMEOUT_S):
            logger.warning("[Decision] Timeout de desvio — retomando exploração.")
            self._fsm.trigger(FSMEvent.OBSTACLE_CLEARED)
            self._avoidance_start_time = None
            return

        # Caminho frontal livre?
        if avoidance.action == 'NONE':
            # Debounce: confirma livre por OBSTACLE_CLEAR_DEBOUNCE_S segundos
            if self._obstacle_clear_start is None:
                self._obstacle_clear_start = time.monotonic()
            elif (time.monotonic() - self._obstacle_clear_start
                  >= self._cfg.OBSTACLE_CLEAR_DEBOUNCE_S):
                logger.info("[Decision] Caminho livre — retomando exploração.")
                self._obstacle_clear_start = None
                self._avoidance_start_time = None
                self._fsm.trigger(FSMEvent.OBSTACLE_CLEARED)
            return
        else:
            # Obstáculo ainda presente — reseta debounce
            self._obstacle_clear_start = None

        # Executa a manobra de desvio
        if avoidance.action == 'STOP':
            self._comm.send_stop()
        elif avoidance.action == 'TURN_LEFT':
            self._comm.send_move('ROT_L', self._cfg.ROTATION_SPEED)
        elif avoidance.action == 'TURN_RIGHT':
            self._comm.send_move('ROT_R', self._cfg.ROTATION_SPEED)
        else:
            self._comm.send_move('FWD', avoidance.speed)

    def _decide_victim_approach(self):
        """
        VICTIM_APPROACH: move em direção à vítima usando posição da HuskyLens.

        Usa a posição normalizada X da vítima para corrigir a trajetória
        (steering proporcional), enquanto avança a velocidade reduzida.
        Quando a área da vítima (proxy de distância) supera o threshold,
        transiciona para captura.
        """
        ctx = self._fsm.context

        if not ctx.victim_detected:
            logger.info("[Decision] Vítima perdida de vista — retomando exploração.")
            self._fsm.trigger(FSMEvent.VICTIM_LOST)
            return

        # Verifica se vítima está próxima o suficiente para capturar
        if ctx.victim_area >= self._cfg.CAPTURE_AREA_THRESHOLD:
            logger.info(f"[Decision] Vítima em alcance (área={ctx.victim_area:.3f}).")
            self._comm.send_stop()
            self._fsm.trigger(FSMEvent.VICTIM_IN_RANGE)
            return

        # Steering proporcional baseado na posição X da vítima
        # victim_x ∈ [-1.0, +1.0]: negativo=esquerda, positivo=direita
        victim_x = ctx.victim_x

        if abs(victim_x) < self._cfg.VICTIM_CENTER_DEADBAND:
            # Vítima centralizada — avança direto
            self._comm.send_move('FWD', self._cfg.APPROACH_SPEED)
        elif victim_x < 0:
            # Vítima à esquerda — gira levemente
            self._comm.send_move('ROT_L', self._cfg.ROTATION_SPEED // 2)
        else:
            # Vítima à direita
            self._comm.send_move('ROT_R', self._cfg.ROTATION_SPEED // 2)

    def _decide_victim_capture(self):
        """
        VICTIM_CAPTURE: comanda sequência de captura ao Arduino.

        Envia GRIP,CAPTURE (sequência atômica: baixar → fechar → subir).
        Aguarda ACK do Arduino para confirmar conclusão.
        """
        logger.info("[Decision] Iniciando sequência de captura.")
        self._comm.send_stop()
        time.sleep(0.2)
        self._comm.send_grip('CAPTURE')
        # A transição para VICTIM_TRANSPORT ocorre via callback
        # quando o ACK 'GRIP_CAPTURE' for recebido do Arduino

    def _decide_victim_transport(self):
        """
        VICTIM_TRANSPORT: retorna à base com vítima capturada.

        Estratégia simplificada: segue a linha de volta.
        A lógica de navegação de retorno depende da arena específica.
        """
        ctx = self._fsm.context

        # Retoma seguimento de linha — Arduino assume controle PID
        self._comm.send_state(1)  # STATE_FOLLOW_LINE

        # Verifica se chegou à base (pode usar LIDAR ou marcador visual)
        # Placeholder: por agora incrementa contador e retorna a EXPLORING
        if self._is_at_base():
            self._comm.send_grip('OPEN')
            ctx.victims_captured += 1
            logger.info(
                f"[Decision] Vítima depositada. "
                f"Total: {ctx.victims_captured}/{ctx.victims_target}"
            )
            if ctx.victims_captured >= ctx.victims_target:
                self._fsm.trigger(FSMEvent.MISSION_COMPLETE)
            else:
                self._fsm.trigger(FSMEvent.RETURN_COMPLETE)

    def _decide_emergency(self):
        """EMERGENCY: para tudo e aguarda reset manual."""
        self._comm.send_stop()

    # ----------------------------------------------------------
    #  CALLBACKS DO ARDUINO
    # ----------------------------------------------------------

    def _register_arduino_callbacks(self):
        """Registra handlers para mensagens recebidas do Arduino."""
        self._comm.register_callback('SENSOR',    self._on_sensor_data)
        self._comm.register_callback('STATE_CHG', self._on_arduino_state_change)
        self._comm.register_callback('ACK',       self._on_ack)
        self._comm.register_callback('ERROR',     self._on_error)
        self._comm.register_callback('PONG',      self._on_pong)

    def _on_sensor_data(self, msg: ParsedMessage):
        """
        Callback: atualiza contexto da FSM com dados de sensores.

        Formato esperado: SENSOR,LINE_POS,LINE_DET,VICTIM_DET,VIC_X,VIC_Y
        """
        if len(msg.params) < 5:
            return

        try:
            line_pos    = float(msg.params[0])
            line_det    = bool(int(msg.params[1]))
            victim_det  = bool(int(msg.params[2]))
            victim_x    = float(msg.params[3])
            victim_y    = float(msg.params[4])
        except (ValueError, IndexError):
            logger.warning(f"[Decision] Dados de sensor malformados: {msg.params}")
            return

        self._fsm.update_context(
            line_position    = line_pos,
            line_detected    = line_det,
            victim_detected  = victim_det,
            victim_x         = victim_x,
            victim_y         = victim_y
        )

    def _on_arduino_state_change(self, msg: ParsedMessage):
        """Callback: Arduino reportou mudança de estado."""
        if msg.params:
            try:
                arduino_state = int(msg.params[0])
                self._fsm.update_context(arduino_state=arduino_state)
                logger.debug(f"[Decision] Arduino estado: {arduino_state}")
            except ValueError:
                pass

    def _on_ack(self, msg: ParsedMessage):
        """Callback: ACK de comando concluído pelo Arduino."""
        if not msg.params:
            return

        ack_type = msg.params[0].upper()
        logger.debug(f"[Decision] ACK recebido: {ack_type}")

        # GRIP_CAPTURE concluída → transiciona para transporte
        if ack_type == 'GRIP_CAPTURE':
            self._fsm.trigger(FSMEvent.CAPTURE_COMPLETE)

    def _on_error(self, msg: ParsedMessage):
        """Callback: erro reportado pelo Arduino."""
        code = msg.params[0] if msg.params else 'UNKNOWN'
        logger.warning(f"[Decision] Erro do Arduino: código={code}")

        if code == '6':  # ERR_WATCHDOG
            self._fsm.trigger(FSMEvent.EMERGENCY)

    def _on_pong(self, msg: ParsedMessage):
        """Callback: resposta ao PING de healthcheck."""
        logger.debug("[Decision] PONG recebido — Arduino responsivo.")

    # ----------------------------------------------------------
    #  UTILITÁRIOS
    # ----------------------------------------------------------

    def _is_at_base(self) -> bool:
        """
        Determina se o robô chegou à base de depósito.

        Placeholder: implementar com marcador LIDAR ou visual.
        Retorna False por padrão (requer implementação específica da arena).
        """
        return False

    def start_mission(self):
        """Dispara o evento de início de missão."""
        self._fsm.trigger(FSMEvent.START)
        self._comm.send_state(1)  # Arduino: FOLLOW_LINE
        logger.info("[DecisionSystem] Missão iniciada.")

    def stop_mission(self):
        """Para a missão e coloca o sistema em IDLE."""
        self._comm.send_stop()
        self._fsm.trigger(FSMEvent.STOP)
        logger.info("[DecisionSystem] Missão interrompida.")
