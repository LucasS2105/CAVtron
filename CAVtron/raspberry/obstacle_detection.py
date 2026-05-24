"""
mapping/obstacle_detection.py
==============================
Detecção e classificação de obstáculos a partir de varreduras LIDAR.

Implementa dois níveis de análise:

1. Análise Setorial (reativa, baixa latência):
   Divide os 360° em setores e verifica distâncias mínimas.
   Resposta em O(N) por varredura. Adequado para reação imediata.

2. Análise de Agrupamento por DBSCAN simplificado:
   Agrupa pontos próximos em clusters (obstáculos individuais).
   Extrai centro de massa, dimensões e distância de cada obstáculo.
   Resposta mais lenta (~10ms), executada em ciclos mais espaçados.

A estratégia de desvio é determinada pela posição relativa e tamanho
do obstáculo, gerando um comando direcional para o sistema de decisão.

Referência desvio (Bug Algorithm):
    Lumelsky, V.J., Stepanov, A.A. (1987). Path-planning strategies
    for a point mobile automaton moving amidst unknown obstacles.
    Algorithmica, 2(1-4), 403-430.
"""

import math
import logging
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from .lidar_interface import LidarScan, LidarPoint, SectorAnalysis

logger = logging.getLogger(__name__)


# ============================================================
#  ESTRUTURAS DE DADOS
# ============================================================

@dataclass
class Obstacle:
    """Obstáculo detectado e caracterizado por clustering."""
    center_x_mm: float     # X do centroide no referencial do robô
    center_y_mm: float     # Y do centroide no referencial do robô
    distance_mm: float     # Distância do centro ao robô
    angle_deg: float       # Ângulo do centroide em relação à frente [0-360)
    width_mm: float        # Largura estimada do obstáculo
    point_count: int       # Número de pontos LIDAR no cluster
    is_blocking: bool = False  # True se obstáculo está no caminho direto

    @property
    def side(self) -> str:
        """Lado relativo ao robô: 'LEFT', 'RIGHT' ou 'FRONT'."""
        if self.angle_deg < 30 or self.angle_deg >= 330:
            return 'FRONT'
        elif 30 <= self.angle_deg < 180:
            return 'LEFT'
        else:
            return 'RIGHT'


@dataclass
class AvoidanceCommand:
    """
    Comando de desvio gerado pelo sistema de detecção.

    Attributes:
        action    : 'NONE'|'SLOW'|'STOP'|'TURN_LEFT'|'TURN_RIGHT'|'REVERSE'
        speed     : Velocidade sugerida [0-255]
        urgency   : Nível de urgência [0.0-1.0], 1.0 = parada imediata
        reason    : Descrição textual para logging
    """
    action: str = 'NONE'
    speed: int = 150
    urgency: float = 0.0
    reason: str = ''


# ============================================================
#  DETECTOR DE OBSTÁCULOS
# ============================================================

class ObstacleDetector:
    """
    Sistema de detecção e classificação de obstáculos via LIDAR.

    Args:
        obstacle_dist_mm  : Distância de alerta (obstáculo perigoso) em mm
        warning_dist_mm   : Distância de aviso (reduzir velocidade) em mm
        cluster_gap_mm    : Separação máxima entre pontos do mesmo cluster
        min_cluster_size  : Mínimo de pontos para considerar cluster válido
        front_half_angle  : Semiângulo do setor frontal em graus
    """

    def __init__(self,
                 obstacle_dist_mm: float = 350.0,
                 warning_dist_mm: float = 600.0,
                 cluster_gap_mm: float = 150.0,
                 min_cluster_size: int = 3,
                 front_half_angle: float = 40.0):
        self._obstacle_dist   = obstacle_dist_mm
        self._warning_dist    = warning_dist_mm
        self._cluster_gap     = cluster_gap_mm
        self._min_cluster     = min_cluster_size
        self._front_half_deg  = front_half_angle

        # Estado interno
        self._last_obstacles: List[Obstacle]           = []
        self._last_avoidance: Optional[AvoidanceCommand] = None

    # ----------------------------------------------------------
    #  ANÁLISE PRINCIPAL
    # ----------------------------------------------------------

    def analyze(self, scan: LidarScan) -> AvoidanceCommand:
        """
        Analisa uma varredura e gera comando de desvio se necessário.

        Pipeline:
        1. Análise setorial rápida (resposta imediata para obstáculos frontais)
        2. Clustering de pontos (caracterização de obstáculos individuais)
        3. Geração de comando de desvio baseado na análise combinada

        Args:
            scan: Varredura LIDAR processada.
        Returns:
            AvoidanceCommand com a ação recomendada.
        """
        if scan.is_empty():
            return AvoidanceCommand(action='NONE', reason='Sem dados LIDAR')

        # Análise setorial rápida
        sector = self._sector_analysis(scan)

        # Clustering e caracterização dos obstáculos
        self._last_obstacles = self._cluster_points(scan.points)

        # Geração do comando de desvio
        command = self._generate_avoidance(sector)
        self._last_avoidance = command

        return command

    # ----------------------------------------------------------
    #  ANÁLISE SETORIAL
    # ----------------------------------------------------------

    def _sector_analysis(self, scan: LidarScan) -> SectorAnalysis:
        """
        Calcula distância mínima por setor angular.

        Setores:
          Frontal  : ±front_half_angle° em relação a 0°
          Esquerdo : front_half_angle° a 180°
          Traseiro : 180° a (360° - front_half_angle°)
          Direito  : (360° - front_half_angle°) a 360°
        """
        analysis = SectorAnalysis()

        front_min = float('inf')
        left_min  = float('inf')
        right_min = float('inf')
        rear_min  = float('inf')

        half = self._front_half_deg

        for pt in scan.points:
            d = pt.distance_mm
            a = pt.angle_deg

            if a < half or a >= (360.0 - half):
                front_min = min(front_min, d)
            elif half <= a < 180.0:
                left_min = min(left_min, d)
            elif 180.0 <= a < (360.0 - half):
                right_min = min(right_min, d)
            else:
                rear_min = min(rear_min, d)

        analysis.front_min_mm = front_min
        analysis.left_min_mm  = left_min
        analysis.right_min_mm = right_min
        analysis.rear_min_mm  = rear_min

        return analysis

    # ----------------------------------------------------------
    #  CLUSTERING (DBSCAN 1D SIMPLIFICADO)
    # ----------------------------------------------------------

    def _cluster_points(self, points: List[LidarPoint]) -> List[Obstacle]:
        """
        Agrupa pontos LIDAR em clusters por adjacência angular.

        Algoritmo: DBSCAN 1D simplificado — pontos consecutivos
        (ordenados por ângulo) são agrupados se a distância euclidiana
        entre eles for menor que cluster_gap_mm.

        Para cada cluster válido, calcula:
        - Centroide (média dos pontos)
        - Largura (span cartesiano)
        - Distância ao robô
        - Ângulo do centroide

        Returns:
            Lista de Obstacle com clusters válidos.
        """
        if not points:
            return []

        # Ordena pontos por ângulo
        sorted_pts = sorted(points, key=lambda p: p.angle_deg)

        clusters: List[List[LidarPoint]] = []
        current_cluster: List[LidarPoint] = [sorted_pts[0]]

        for i in range(1, len(sorted_pts)):
            prev = sorted_pts[i - 1]
            curr = sorted_pts[i]

            # Distância euclidiana entre pontos consecutivos
            dist = math.sqrt(
                (curr.x_mm - prev.x_mm) ** 2 +
                (curr.y_mm - prev.y_mm) ** 2
            )

            if dist < self._cluster_gap:
                current_cluster.append(curr)
            else:
                if len(current_cluster) >= self._min_cluster:
                    clusters.append(current_cluster)
                current_cluster = [curr]

        if len(current_cluster) >= self._min_cluster:
            clusters.append(current_cluster)

        return [self._characterize_cluster(c) for c in clusters]

    def _characterize_cluster(self, cluster: List[LidarPoint]) -> Obstacle:
        """
        Extrai características de um cluster de pontos.

        Args:
            cluster: Lista de LidarPoint do mesmo cluster.
        Returns:
            Obstacle com centroide, distância, ângulo e largura.
        """
        xs = [p.x_mm for p in cluster]
        ys = [p.y_mm for p in cluster]

        cx = sum(xs) / len(xs)
        cy = sum(ys) / len(ys)

        distance = math.sqrt(cx ** 2 + cy ** 2)
        angle    = math.degrees(math.atan2(cy, cx)) % 360.0

        # Largura: span máximo cartesiano
        x_range = max(xs) - min(xs)
        y_range = max(ys) - min(ys)
        width   = math.sqrt(x_range ** 2 + y_range ** 2)

        return Obstacle(
            center_x_mm=cx,
            center_y_mm=cy,
            distance_mm=distance,
            angle_deg=angle,
            width_mm=width,
            point_count=len(cluster)
        )

    # ----------------------------------------------------------
    #  GERAÇÃO DO COMANDO DE DESVIO
    # ----------------------------------------------------------

    def _generate_avoidance(self, sector: SectorAnalysis) -> AvoidanceCommand:
        """
        Determina a manobra de desvio baseada na análise setorial.

        Lógica de prioridade:
        1. Obstáculo frontal crítico (<obstacle_dist)  → STOP
        2. Obstáculo frontal próximo (<warning_dist)   → TURN (menor distância lateral)
        3. Sem obstáculo significativo                 → NONE

        A escolha de girar à esquerda ou direita é baseada em qual lado
        tem mais espaço disponível (maior distância mínima).

        Args:
            sector: Análise setorial da varredura atual.
        Returns:
            AvoidanceCommand com ação e velocidade recomendadas.
        """
        front = sector.front_min_mm
        left  = sector.left_min_mm
        right = sector.right_min_mm

        # PARADA DE EMERGÊNCIA: obstáculo muito próximo na frente
        if front < self._obstacle_dist:
            urgency = 1.0 - (front / self._obstacle_dist)
            urgency = min(urgency, 1.0)

            # Determina o melhor lado para girar
            if left > right:
                action = 'TURN_LEFT'
            else:
                action = 'TURN_RIGHT'

            return AvoidanceCommand(
                action='STOP' if front < self._obstacle_dist * 0.5 else action,
                speed=0,
                urgency=urgency,
                reason=f"Obstáculo frontal a {front:.0f}mm | esq={left:.0f}mm dir={right:.0f}mm"
            )

        # AVISO: obstáculo no raio de atenção
        if front < self._warning_dist:
            # Velocidade reduzida proporcional à proximidade
            speed_factor = (front - self._obstacle_dist) / \
                           (self._warning_dist - self._obstacle_dist)
            reduced_speed = int(80 + speed_factor * 70)  # 80-150

            if left > right:
                action = 'TURN_LEFT'
            else:
                action = 'TURN_RIGHT'

            return AvoidanceCommand(
                action=action,
                speed=reduced_speed,
                urgency=(1.0 - speed_factor) * 0.5,
                reason=f"Obstáculo em zona de aviso: {front:.0f}mm"
            )

        return AvoidanceCommand(
            action='NONE',
            speed=150,
            urgency=0.0,
            reason='Caminho livre'
        )

    # ----------------------------------------------------------
    #  ACESSO AO ESTADO
    # ----------------------------------------------------------

    def get_obstacles(self) -> List[Obstacle]:
        """Retorna a lista de obstáculos da última análise."""
        return self._last_obstacles

    def get_last_command(self) -> Optional[AvoidanceCommand]:
        """Retorna o último comando de desvio gerado."""
        return self._last_avoidance

    def get_closest_obstacle(self) -> Optional[Obstacle]:
        """Retorna o obstáculo mais próximo do robô, ou None."""
        if not self._last_obstacles:
            return None
        return min(self._last_obstacles, key=lambda o: o.distance_mm)
