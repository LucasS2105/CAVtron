"""
mapping/mapping.py
==================
Mapeamento da arena por Grade de Ocupação (Occupancy Grid).

Implementa o modelo probabilístico de Occupancy Grid proposto por
Elfes (1987) e formalizado por Thrun et al. (2005), onde cada célula
da grade mantém a probabilidade log-odds de estar ocupada:

    l(m_i) = log( P(m_i=1) / P(m_i=0) )

Atualização Bayesiana (log-odds update rule):
    l_t(m_i) = l_{t-1}(m_i)
               + l_occ  (se raio do sensor passa pela célula)
               - l_free  (se raio passa sem obstrução)

Vantagem da representação log-odds:
    - Numericamente estável (evita underflow/overflow de probabilidades)
    - Atualização incremental O(1) por célula
    - Conversão trivial para probabilidade: P = 1 - 1/(1 + exp(l))

A pose do robô é estimada por odometria incremental baseada nos
comandos enviados aos motores (dead reckoning), com integração futura
para filtro de partículas (AMCL) ou SLAM.

Referência:
    Thrun, S., Burgard, W., Fox, D. (2005). Probabilistic Robotics.
    MIT Press. Cap. 9 — Occupancy Grid Mapping.
"""

import math
import logging
import numpy as np
from dataclasses import dataclass, field
from typing import Optional, Tuple, List

from .lidar_interface import LidarScan, LidarPoint

logger = logging.getLogger(__name__)


# ============================================================
#  POSE DO ROBÔ
# ============================================================

@dataclass
class RobotPose:
    """
    Pose 2D do robô no referencial da arena.

    Attributes:
        x_m   : Posição X em metros
        y_m   : Posição Y em metros
        yaw_rad: Orientação (yaw) em radianos [-π, +π]
    """
    x_m: float = 0.0
    y_m: float = 0.0
    yaw_rad: float = 0.0

    def translate(self, dx: float, dy: float):
        """Translada a pose por (dx, dy) em metros."""
        self.x_m += dx
        self.y_m += dy

    def rotate(self, dtheta: float):
        """Rotaciona a pose por dtheta radianos (normalizado em [-π, π])."""
        self.yaw_rad = math.atan2(
            math.sin(self.yaw_rad + dtheta),
            math.cos(self.yaw_rad + dtheta)
        )

    def to_map_coords(self, resolution_m: float,
                      origin_x: int, origin_y: int) -> Tuple[int, int]:
        """
        Converte pose em coordenadas de célula da grade.

        Args:
            resolution_m : Tamanho de cada célula em metros
            origin_x     : Coluna da célula de origem (0,0) na grade
            origin_y     : Linha da célula de origem (0,0) na grade
        Returns:
            Tupla (col, row) na grade de ocupação.
        """
        col = int(self.x_m / resolution_m) + origin_x
        row = int(self.y_m / resolution_m) + origin_y
        return col, row


# ============================================================
#  GRADE DE OCUPAÇÃO
# ============================================================

class OccupancyGrid:
    """
    Grade de ocupação probabilística 2D.

    Args:
        width_m      : Largura da arena mapeada em metros
        height_m     : Altura da arena mapeada em metros
        resolution_m : Resolução da grade (tamanho de cada célula) em metros
        l_occ        : Incremento log-odds para célula ocupada
        l_free       : Decremento log-odds para célula livre
        l_max        : Saturação superior do acumulador log-odds
        l_min        : Saturação inferior do acumulador log-odds
    """

    # Valores log-odds do modelo de sensor (ajustar conforme precisão do LIDAR)
    L_OCC_DEFAULT  =  0.85   # log(P_occ/P_free) para hit
    L_FREE_DEFAULT =  0.40   # log(P_free/P_occ) para miss
    L_MAX          =  5.0    # Saturação (evita "fossilização" de células)
    L_MIN          = -5.0

    # Limiar de probabilidade para classificar célula como ocupada
    OCCUPIED_THRESHOLD = 0.65   # P(ocupada) > 65%

    def __init__(self,
                 width_m: float = 4.0,
                 height_m: float = 4.0,
                 resolution_m: float = 0.05,
                 l_occ: float = L_OCC_DEFAULT,
                 l_free: float = L_FREE_DEFAULT):
        self._resolution = resolution_m
        self._l_occ  = l_occ
        self._l_free = l_free

        # Dimensões em células
        self._cols = int(width_m / resolution_m)
        self._rows = int(height_m / resolution_m)

        # Origem da grade (robô começa no centro)
        self._origin_col = self._cols // 2
        self._origin_row = self._rows // 2

        # Grade log-odds inicializada em 0.0 (incerteza total: P=0.5)
        self._log_odds = np.zeros((self._rows, self._cols), dtype=np.float32)

        logger.info(
            f"[OccupancyGrid] {self._rows}x{self._cols} células | "
            f"resolução={resolution_m*100:.1f}cm | "
            f"arena={width_m}x{height_m}m"
        )

    # ----------------------------------------------------------
    #  ATUALIZAÇÃO COM VARREDURA LIDAR
    # ----------------------------------------------------------

    def update(self, scan: LidarScan, pose: RobotPose):
        """
        Integra uma varredura LIDAR na grade de ocupação.

        Para cada ponto da varredura:
        1. Converte coordenadas polares para mapa global
        2. Traça raio da pose ao ponto (ray casting)
        3. Marca células ao longo do raio como livres (decremento)
        4. Marca célula do ponto como ocupada (incremento)

        Args:
            scan : Varredura LIDAR processada
            pose : Pose atual do robô na arena
        """
        if scan.is_empty():
            return

        robot_col, robot_row = pose.to_map_coords(
            self._resolution, self._origin_col, self._origin_row
        )

        for point in scan.points:
            # Transforma ponto do referencial do sensor para o da arena
            # (aplica rotação do yaw do robô)
            angle_global = point.angle_deg * math.pi / 180.0 + pose.yaw_rad
            endpoint_x = pose.x_m + point.distance_mm / 1000.0 * math.cos(angle_global)
            endpoint_y = pose.y_m + point.distance_mm / 1000.0 * math.sin(angle_global)

            # Converte endpoint para coordenadas de grade
            end_col = int(endpoint_x / self._resolution) + self._origin_col
            end_row = int(endpoint_y / self._resolution) + self._origin_row

            # Ray casting: marca células ao longo do raio
            free_cells = self._bresenham(robot_col, robot_row, end_col, end_row)

            # Células livres (todas exceto a última)
            for col, row in free_cells[:-1]:
                if self._in_bounds(col, row):
                    self._log_odds[row, col] = max(
                        self.L_MIN,
                        self._log_odds[row, col] - self._l_free
                    )

            # Célula ocupada (endpoint)
            if self._in_bounds(end_col, end_row):
                self._log_odds[end_row, end_col] = min(
                    self.L_MAX,
                    self._log_odds[end_row, end_col] + self._l_occ
                )

    # ----------------------------------------------------------
    #  CONSULTAS
    # ----------------------------------------------------------

    def get_probability_map(self) -> np.ndarray:
        """
        Converte a grade log-odds para probabilidades [0.0, 1.0].

        P(ocupada) = 1 - 1 / (1 + exp(l))

        Returns:
            Array 2D numpy float32 com probabilidades de ocupação.
        """
        return 1.0 - 1.0 / (1.0 + np.exp(self._log_odds))

    def is_occupied(self, col: int, row: int) -> bool:
        """
        Verifica se uma célula é classificada como ocupada.

        Args:
            col, row: Coordenadas da célula na grade.
        Returns:
            True se P(ocupada) > OCCUPIED_THRESHOLD.
        """
        if not self._in_bounds(col, row):
            return False
        prob = 1.0 - 1.0 / (1.0 + math.exp(float(self._log_odds[row, col])))
        return prob > self.OCCUPIED_THRESHOLD

    def get_log_odds(self) -> np.ndarray:
        """Retorna referência à grade log-odds (para visualização/debug)."""
        return self._log_odds

    def world_to_grid(self, x_m: float, y_m: float) -> Tuple[int, int]:
        """Converte coordenadas do mundo (metros) para índices da grade."""
        col = int(x_m / self._resolution) + self._origin_col
        row = int(y_m / self._resolution) + self._origin_row
        return col, row

    def grid_to_world(self, col: int, row: int) -> Tuple[float, float]:
        """Converte índices da grade para coordenadas do mundo (metros)."""
        x = (col - self._origin_col) * self._resolution
        y = (row - self._origin_row) * self._resolution
        return x, y

    def get_shape(self) -> Tuple[int, int]:
        return self._rows, self._cols

    def reset(self):
        """Reinicia a grade para estado de incerteza total."""
        self._log_odds.fill(0.0)

    # ----------------------------------------------------------
    #  UTILITÁRIOS INTERNOS
    # ----------------------------------------------------------

    def _in_bounds(self, col: int, row: int) -> bool:
        return 0 <= col < self._cols and 0 <= row < self._rows

    @staticmethod
    def _bresenham(x0: int, y0: int, x1: int, y1: int) -> List[Tuple[int, int]]:
        """
        Algoritmo de Bresenham para traçado de linha em grade discreta.

        Gera todos os pixels (células) ao longo da linha entre
        (x0,y0) e (x1,y1) com custo O(max(|dx|,|dy|)).

        Referência:
            Bresenham, J.E. (1965). Algorithm for computer control of a
            digital plotter. IBM Systems Journal, 4(1), 25-30.

        Returns:
            Lista de tuplas (col, row) ao longo da linha.
        """
        cells = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        x, y = x0, y0
        max_steps = max(dx, dy) + 1

        for _ in range(max_steps):
            cells.append((x, y))
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

        return cells


# ============================================================
#  ODOMETRIA (DEAD RECKONING)
# ============================================================

class Odometry:
    """
    Estimação de pose por dead reckoning (integração de velocidades).

    Modelo cinemático do robô diferencial:
        dx    = v * cos(θ) * dt
        dy    = v * sin(θ) * dt
        dθ    = ω * dt

    Onde v = (v_r + v_l) / 2  e  ω = (v_r - v_l) / L
    (L = distância entre rodas = wheelbase)

    Obs: Dead reckoning acumula erro de odometria ao longo do tempo.
    Para missões longas, fusão com LIDAR (SLAM) ou IMU é necessária.

    Args:
        wheelbase_m    : Distância entre as rodas em metros
        wheel_radius_m : Raio das rodas em metros
        max_speed_rpm  : RPM máxima dos motores (para normalização PWM)
    """

    def __init__(self,
                 wheelbase_m: float = 0.18,
                 wheel_radius_m: float = 0.033,
                 max_speed_rpm: float = 200.0):
        self._wheelbase    = wheelbase_m
        self._wheel_radius = wheel_radius_m
        self._max_rpm      = max_speed_rpm

        # Velocidade máxima linear em m/s
        self._v_max = (2 * math.pi * wheel_radius_m * max_speed_rpm) / 60.0

        self.pose      = RobotPose()
        self._last_time = time.monotonic()

    def update(self, left_pwm: int, right_pwm: int):
        """
        Integra as velocidades dos motores para atualizar a pose.

        Converte PWM [0-255] para velocidade linear normalizada,
        integra pelo intervalo de tempo desde a última chamada.

        Args:
            left_pwm  : PWM do motor esquerdo [0-255]
            right_pwm : PWM do motor direito  [0-255]
        """
        now = time.monotonic()
        dt  = now - self._last_time
        self._last_time = now

        if dt <= 0 or dt > 1.0:
            return  # Intervalo inválido

        # Normalização PWM para velocidade linear [m/s]
        v_l = (left_pwm  / 255.0) * self._v_max
        v_r = (right_pwm / 255.0) * self._v_max

        # Velocidade linear e angular do robô
        v = (v_r + v_l) / 2.0
        omega = (v_r - v_l) / self._wheelbase

        # Integração de Euler (simplificada — usar Runge-Kutta para maior precisão)
        dx    = v * math.cos(self.pose.yaw_rad) * dt
        dy    = v * math.sin(self.pose.yaw_rad) * dt
        dtheta = omega * dt

        self.pose.translate(dx, dy)
        self.pose.rotate(dtheta)

    def reset(self, x: float = 0.0, y: float = 0.0, yaw: float = 0.0):
        """Reinicia a pose para uma posição conhecida."""
        self.pose = RobotPose(x_m=x, y_m=y, yaw_rad=yaw)
        self._last_time = time.monotonic()
