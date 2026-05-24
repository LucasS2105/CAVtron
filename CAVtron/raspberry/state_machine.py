"""
control/state_machine.py
=========================
Máquina de Estados Finita (FSM) de alto nível do Raspberry Pi.

Define os estados da missão e as transições entre eles, com base
nos eventos recebidos dos sensores (LIDAR, HuskyLens via Arduino)
e nas decisões do sistema de decisão.

A FSM do Raspberry Pi opera em nível de missão, complementando
a FSM de baixo nível do Arduino (que opera em nível de atuação).

Hierarquia de controle:
    RPi FSM (missão)  →  Decisão  →  SerialComm  →  Arduino FSM  →  Motores

Estados de missão:
    IDLE            : Sistema parado, aguardando ativação
    EXPLORING       : Seguindo linha, mapeando arena
    OBSTACLE_AVOID  : Executando manobra de desvio
    VICTIM_APPROACH : Aproximando-se de vítima detectada
    VICTIM_CAPTURE  : Executando sequência de captura
    VICTIM_TRANSPORT: Transportando vítima para base
    MISSION_COMPLETE: Missão concluída
    EMERGENCY       : Parada de emergência sistêmica

Referência FSM:
    Miro, J.V. et al. (2007). Hierarchical state machine architecture
    for autonomous robot control. ICRA 2007.
"""

import time
import logging
from enum import IntEnum, auto
from dataclasses import dataclass, field
from typing import Optional, Callable, Dict, List

logger = logging.getLogger(__name__)


# ============================================================
#  ENUMERAÇÃO DE ESTADOS
# ============================================================

class MissionState(IntEnum):
    IDLE             = 0
    EXPLORING        = 1
    OBSTACLE_AVOID   = 2
    VICTIM_APPROACH  = 3
    VICTIM_CAPTURE   = 4
    VICTIM_TRANSPORT = 5
    MISSION_COMPLETE = 6
    EMERGENCY        = 7


# ============================================================
#  EVENTOS
# ============================================================

class FSMEvent(IntEnum):
    """Eventos que disparam transições de estado."""
    START              = auto()   # Comando de início de missão
    STOP               = auto()   # Comando de parada
    EMERGENCY          = auto()   # Falha crítica de hardware/comunicação
    OBSTACLE_DETECTED  = auto()   # Obstáculo frontal identificado
    OBSTACLE_CLEARED   = auto()   # Caminho livre após desvio
    VICTIM_DETECTED    = auto()   # HuskyLens identificou vítima
    VICTIM_LOST        = auto()   # Vítima saiu do campo de visão
    VICTIM_IN_RANGE    = auto()   # Vítima próxima o suficiente para captura
    CAPTURE_COMPLETE   = auto()   # Sequência de captura concluída
    RETURN_COMPLETE    = auto()   # Retorno à base concluído
    MISSION_COMPLETE   = auto()   # Todas as vítimas resgatadas


# ============================================================
#  DADOS DE ESTADO
# ============================================================

@dataclass
class StateContext:
    """
    Contexto compartilhado entre estados — dados da missão atual.
    """
    # Dados da linha
    line_position: float = 0.0
    line_detected: bool  = False

    # Dados de vítima
    victim_detected: bool  = False
    victim_x: float        = 0.0
    victim_y: float        = 0.0
    victim_area: float     = 0.0  # Proxy de distância

    # Dados de obstáculo
    obstacle_action: str   = 'NONE'
    obstacle_urgency: float = 0.0
    front_dist_mm: float   = float('inf')

    # Estado do Arduino
    arduino_state: int     = 0

    # Contadores de missão
    victims_captured: int  = 0
    victims_target: int    = 1

    # Timestamps de estado
    state_entry_time: float = field(default_factory=time.monotonic)


# ============================================================
#  MÁQUINA DE ESTADOS
# ============================================================

class StateMachine:
    """
    FSM hierárquica para controle de missão de resgate.

    Implementa o padrão State com callbacks de entrada/saída de estado
    e handlers de transição configuráveis.

    Args:
        initial_state : Estado inicial da FSM (padrão: IDLE)
    """

    def __init__(self, initial_state: MissionState = MissionState.IDLE):
        self._state   = initial_state
        self._context = StateContext()

        # Callbacks: on_enter[state] e on_exit[state]
        self._on_enter: Dict[MissionState, List[Callable]] = \
            {s: [] for s in MissionState}
        self._on_exit:  Dict[MissionState, List[Callable]] = \
            {s: [] for s in MissionState}

        # Tabela de transições: (estado_atual, evento) → estado_destino
        self._transitions: Dict[tuple, MissionState] = self._build_transition_table()

        # Histórico de transições (para debug e análise pós-missão)
        self._history: List[Dict] = []

        logger.info(f"[StateMachine] Inicializada em {self._state.name}")

    # ----------------------------------------------------------
    #  TABELA DE TRANSIÇÕES
    # ----------------------------------------------------------

    def _build_transition_table(self) -> Dict[tuple, MissionState]:
        """
        Define todas as transições válidas da FSM.

        Transições não definidas são silenciosamente ignoradas,
        mantendo o estado atual (comportamento conservador).

        Returns:
            Dict mapeando (estado, evento) → estado_destino
        """
        S = MissionState
        E = FSMEvent

        return {
            # De IDLE
            (S.IDLE, E.START)             : S.EXPLORING,
            (S.IDLE, E.EMERGENCY)         : S.EMERGENCY,

            # De EXPLORING (seguindo linha)
            (S.EXPLORING, E.OBSTACLE_DETECTED): S.OBSTACLE_AVOID,
            (S.EXPLORING, E.VICTIM_DETECTED)  : S.VICTIM_APPROACH,
            (S.EXPLORING, E.STOP)             : S.IDLE,
            (S.EXPLORING, E.EMERGENCY)        : S.EMERGENCY,
            (S.EXPLORING, E.MISSION_COMPLETE) : S.MISSION_COMPLETE,

            # De OBSTACLE_AVOID
            (S.OBSTACLE_AVOID, E.OBSTACLE_CLEARED): S.EXPLORING,
            (S.OBSTACLE_AVOID, E.VICTIM_DETECTED) : S.VICTIM_APPROACH,
            (S.OBSTACLE_AVOID, E.EMERGENCY)       : S.EMERGENCY,
            (S.OBSTACLE_AVOID, E.STOP)            : S.IDLE,

            # De VICTIM_APPROACH
            (S.VICTIM_APPROACH, E.VICTIM_IN_RANGE): S.VICTIM_CAPTURE,
            (S.VICTIM_APPROACH, E.VICTIM_LOST)    : S.EXPLORING,
            (S.VICTIM_APPROACH, E.OBSTACLE_DETECTED): S.OBSTACLE_AVOID,
            (S.VICTIM_APPROACH, E.EMERGENCY)      : S.EMERGENCY,
            (S.VICTIM_APPROACH, E.STOP)           : S.IDLE,

            # De VICTIM_CAPTURE
            (S.VICTIM_CAPTURE, E.CAPTURE_COMPLETE): S.VICTIM_TRANSPORT,
            (S.VICTIM_CAPTURE, E.EMERGENCY)       : S.EMERGENCY,

            # De VICTIM_TRANSPORT
            (S.VICTIM_TRANSPORT, E.RETURN_COMPLETE): S.EXPLORING,
            (S.VICTIM_TRANSPORT, E.MISSION_COMPLETE): S.MISSION_COMPLETE,
            (S.VICTIM_TRANSPORT, E.EMERGENCY)      : S.EMERGENCY,

            # De EMERGENCY (apenas reset manual pode sair)
            (S.EMERGENCY, E.START)         : S.IDLE,

            # De MISSION_COMPLETE
            (S.MISSION_COMPLETE, E.START)  : S.IDLE,
        }

    # ----------------------------------------------------------
    #  DISPARO DE EVENTOS
    # ----------------------------------------------------------

    def trigger(self, event: FSMEvent) -> bool:
        """
        Dispara um evento e executa a transição correspondente.

        Se a transição existe: executa on_exit do estado atual,
        muda de estado e executa on_enter do novo estado.

        Args:
            event: Evento a disparar.
        Returns:
            True se transição ocorreu, False se evento não mapeado.
        """
        key = (self._state, event)
        next_state = self._transitions.get(key)

        if next_state is None:
            logger.debug(
                f"[FSM] Evento {event.name} ignorado em {self._state.name}"
            )
            return False

        # Registra transição no histórico
        self._history.append({
            'from'     : self._state.name,
            'event'    : event.name,
            'to'       : next_state.name,
            'timestamp': time.monotonic()
        })

        logger.info(
            f"[FSM] {self._state.name} --[{event.name}]--> {next_state.name}"
        )

        # Callbacks de saída do estado atual
        for cb in self._on_exit.get(self._state, []):
            try:
                cb(self._state, self._context)
            except Exception as e:
                logger.error(f"[FSM] Erro em on_exit({self._state.name}): {e}")

        # Transição
        prev_state    = self._state
        self._state   = next_state
        self._context.state_entry_time = time.monotonic()

        # Callbacks de entrada do novo estado
        for cb in self._on_enter.get(self._state, []):
            try:
                cb(self._state, self._context)
            except Exception as e:
                logger.error(f"[FSM] Erro em on_enter({self._state.name}): {e}")

        return True

    # ----------------------------------------------------------
    #  REGISTRO DE CALLBACKS
    # ----------------------------------------------------------

    def on_enter(self, state: MissionState,
                 callback: Callable[[MissionState, StateContext], None]):
        """Registra callback executado ao entrar no estado."""
        self._on_enter[state].append(callback)

    def on_exit(self, state: MissionState,
                callback: Callable[[MissionState, StateContext], None]):
        """Registra callback executado ao sair do estado."""
        self._on_exit[state].append(callback)

    # ----------------------------------------------------------
    #  ACESSO AO ESTADO
    # ----------------------------------------------------------

    @property
    def state(self) -> MissionState:
        return self._state

    @property
    def context(self) -> StateContext:
        return self._context

    def is_in(self, *states: MissionState) -> bool:
        """Verifica se a FSM está em algum dos estados fornecidos."""
        return self._state in states

    def time_in_state(self) -> float:
        """Retorna o tempo em segundos no estado atual."""
        return time.monotonic() - self._context.state_entry_time

    def get_history(self) -> List[Dict]:
        """Retorna o histórico completo de transições."""
        return list(self._history)

    def update_context(self, **kwargs):
        """Atualiza campos do contexto por keyword arguments."""
        for key, value in kwargs.items():
            if hasattr(self._context, key):
                setattr(self._context, key, value)
            else:
                logger.warning(f"[FSM] Campo desconhecido no contexto: '{key}'")
