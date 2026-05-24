"""
mapping/lidar_interface.py
===========================
Interface com sensor LIDAR para aquisição de varreduras 2D.

Suporta a família RPLidar (A1/A2/A3) via biblioteca rplidar-roboticia.
O sensor realiza varreduras rotativas de 360°, entregando pares
(ângulo, distância) que alimentam o sistema de mapeamento.

Modelo de dados de uma varredura (scan):
    Lista de tuplas (quality, angle_deg, distance_mm) onde:
    - quality   : qualidade do ponto [0-15], 0 = inválido
    - angle_deg : ângulo em graus [0.0, 360.0)
    - distance_mm: distância em milímetros [0, max_range]

O processamento da nuvem de pontos converte para coordenadas
cartesianas e polares normalizadas, servindo ao mapeamento por
grade de ocupação (Occupancy Grid) e à detecção de obstáculos.

Referência:
    Thrun, S., Burgard, W., Fox, D. (2005). Probabilistic Robotics.
    MIT Press. Cap. 6 — Occupancy Grid Maps.
"""

import time
import math
import logging
import threading
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

logger = logging.getLogger(__name__)

# Importação condicional — permite testes em ambiente sem hardware
try:
    from rplidar import RPLidar, RPLidarException
    RPLIDAR_AVAILABLE = True
except ImportError:
    logger.warning("[LidarInterface] rplidar não disponível. Modo simulado ativo.")
    RPLIDAR_AVAILABLE = False


# ============================================================
#  ESTRUTURAS DE DADOS
# ============================================================

@dataclass
class LidarPoint:
    """Ponto de uma varredura LIDAR em coordenadas polares e cartesianas."""
    angle_deg: float     # Ângulo em graus [0, 360)
    distance_mm: float   # Distância em milímetros
    quality: int         # Qualidade do retorno [0-15]
    x_mm: float = 0.0    # Coordenada X cartesiana (relativa ao robô)
    y_mm: float = 0.0    # Coordenada Y cartesiana (relativa ao robô)

    def __post_init__(self):
        """Calcula coordenadas cartesianas a partir de polares."""
        rad = math.radians(self.angle_deg)
        self.x_mm = self.distance_mm * math.cos(rad)
        self.y_mm = self.distance_mm * math.sin(rad)


@dataclass
class LidarScan:
    """Varredura completa — 360° de pontos filtrados."""
    points: List[LidarPoint] = field(default_factory=list)
    timestamp: float = field(default_factory=time.monotonic)
    scan_id: int = 0

    def is_empty(self) -> bool:
        return len(self.points) == 0


@dataclass
class SectorAnalysis:
    """
    Análise setorial da varredura — distâncias mínimas por setor.

    O espaço ao redor do robô é dividido em setores angulares.
    O setor frontal é o mais crítico para navegação direta.

    Convenção de ângulos (referencial do robô):
        0°   = frente
        90°  = esquerda
        180° = atrás
        270° = direita (equivalente a -90°)
    """
    front_min_mm: float = float('inf')    # 330°-30° (frente ±30°)
    left_min_mm: float = float('inf')     # 30°-150°
    rear_min_mm: float = float('inf')     # 150°-210°
    right_min_mm: float = float('inf')    # 210°-330°
    obstacle_detected: bool = False
    obstacle_direction: str = 'NONE'      # 'FRONT'|'LEFT'|'RIGHT'|'REAR'|'NONE'
    timestamp: float = field(default_factory=time.monotonic)


# ============================================================
#  INTERFACE LIDAR
# ============================================================

class LidarInterface:
    """
    Interface de alto nível para sensor LIDAR RPLidar.

    Executa aquisição de varreduras em thread dedicada, disponibilizando
    a varredura mais recente de forma thread-safe para consumo pelo
    sistema de mapeamento e decisão.

    Args:
        port          : Porta serial do LIDAR (ex: '/dev/ttyUSB0')
        min_quality   : Qualidade mínima de ponto aceita [0-15]
        min_dist_mm   : Distância mínima válida (filtra reflexos próximos)
        max_dist_mm   : Distância máxima útil do sensor
        obstacle_dist : Distância de alerta de obstáculo em mm
    """

    def __init__(self,
                 port: str = '/dev/ttyUSB0',
                 min_quality: int = 5,
                 min_dist_mm: float = 100.0,
                 max_dist_mm: float = 6000.0,
                 obstacle_dist_mm: float = 400.0):
        self._port           = port
        self._min_quality    = min_quality
        self._min_dist_mm    = min_dist_mm
        self._max_dist_mm    = max_dist_mm
        self._obstacle_dist  = obstacle_dist_mm

        self._lidar = None
        self._running = False
        self._scan_lock = threading.Lock()

        # Última varredura válida
        self._latest_scan: Optional[LidarScan] = None
        self._scan_count: int = 0

        self._scan_thread = threading.Thread(
            target=self._scan_loop,
            name='LidarScanThread',
            daemon=True
        )

    # ----------------------------------------------------------
    #  CICLO DE VIDA
    # ----------------------------------------------------------

    def start(self) -> bool:
        """
        Inicializa o sensor e inicia a thread de varredura.

        Returns:
            True se o sensor foi inicializado com sucesso.
        """
        if not RPLIDAR_AVAILABLE:
            logger.warning("[LidarInterface] Operando em modo simulado.")
            self._running = True
            self._scan_thread.start()
            return True

        try:
            self._lidar = RPLidar(self._port, baudrate=115200)
            info = self._lidar.get_info()
            health = self._lidar.get_health()
            logger.info(f"[LidarInterface] Sensor conectado: {info}")
            logger.info(f"[LidarInterface] Saúde do sensor: {health}")

            self._running = True
            self._scan_thread.start()
            return True

        except Exception as e:
            logger.error(f"[LidarInterface] Falha ao inicializar: {e}")
            return False

    def stop(self):
        """Para a aquisição e desliga o motor do LIDAR."""
        self._running = False
        if self._lidar:
            try:
                self._lidar.stop()
                self._lidar.stop_motor()
                self._lidar.disconnect()
            except Exception:
                pass
        logger.info("[LidarInterface] Parado.")

    # ----------------------------------------------------------
    #  ACESSO AOS DADOS
    # ----------------------------------------------------------

    def get_latest_scan(self) -> Optional[LidarScan]:
        """
        Retorna a varredura mais recente de forma thread-safe.

        Returns:
            LidarScan ou None se nenhuma varredura disponível.
        """
        with self._scan_lock:
            return self._latest_scan

    def get_sector_analysis(self) -> Optional[SectorAnalysis]:
        """
        Analisa a varredura mais recente por setores angulares.

        Divide os 360° em 4 setores e encontra a distância mínima
        em cada um. Detecta obstáculos baseado em obstacle_dist_mm.

        Returns:
            SectorAnalysis preenchido, ou None se sem dados.
        """
        scan = self.get_latest_scan()
        if not scan or scan.is_empty():
            return None

        analysis = SectorAnalysis()

        for pt in scan.points:
            d = pt.distance_mm
            a = pt.angle_deg

            # Setor frontal: 330°–360° e 0°–30°
            if a >= 330.0 or a < 30.0:
                analysis.front_min_mm = min(analysis.front_min_mm, d)
            # Setor esquerdo: 30°–150°
            elif 30.0 <= a < 150.0:
                analysis.left_min_mm = min(analysis.left_min_mm, d)
            # Setor traseiro: 150°–210°
            elif 150.0 <= a < 210.0:
                analysis.rear_min_mm = min(analysis.rear_min_mm, d)
            # Setor direito: 210°–330°
            else:
                analysis.right_min_mm = min(analysis.right_min_mm, d)

        # Determina se há obstáculo e em qual direção
        min_front = analysis.front_min_mm
        min_side  = min(analysis.left_min_mm, analysis.right_min_mm)

        if min_front < self._obstacle_dist:
            analysis.obstacle_detected  = True
            analysis.obstacle_direction = 'FRONT'
        elif analysis.left_min_mm < self._obstacle_dist:
            analysis.obstacle_detected  = True
            analysis.obstacle_direction = 'LEFT'
        elif analysis.right_min_mm < self._obstacle_dist:
            analysis.obstacle_detected  = True
            analysis.obstacle_direction = 'RIGHT'

        return analysis

    def get_front_distance_mm(self) -> float:
        """
        Retorna a menor distância na zona frontal (±30°).

        Atalho conveniente para verificação rápida de obstáculo frontal.

        Returns:
            Distância mínima em mm, ou inf se sem dados.
        """
        analysis = self.get_sector_analysis()
        if analysis is None:
            return float('inf')
        return analysis.front_min_mm

    def is_running(self) -> bool:
        return self._running

    def get_scan_count(self) -> int:
        return self._scan_count

    # ----------------------------------------------------------
    #  THREAD DE VARREDURA
    # ----------------------------------------------------------

    def _scan_loop(self):
        """
        Thread de aquisição contínua de varreduras.

        Para hardware real: itera sobre os scans do RPLidar.
        Para modo simulado: gera varredura vazia com delay.
        """
        logger.debug("[LidarScanThread] Iniciada.")

        if not RPLIDAR_AVAILABLE or self._lidar is None:
            self._scan_loop_simulated()
            return

        try:
            for scan_raw in self._lidar.iter_scans(max_buf_meas=500):
                if not self._running:
                    break

                processed = self._process_scan(scan_raw)
                with self._scan_lock:
                    self._latest_scan = processed
                    self._scan_count += 1

        except Exception as e:
            logger.error(f"[LidarScanThread] Erro na aquisição: {e}")
        finally:
            logger.debug("[LidarScanThread] Encerrada.")

    def _scan_loop_simulated(self):
        """Modo simulado — gera varredura vazia para testes sem hardware."""
        logger.info("[LidarInterface] Modo simulado ativo.")
        while self._running:
            with self._scan_lock:
                self._latest_scan = LidarScan(scan_id=self._scan_count)
                self._scan_count += 1
            time.sleep(0.1)

    # ----------------------------------------------------------
    #  PROCESSAMENTO DE VARREDURA
    # ----------------------------------------------------------

    def _process_scan(self, raw_scan: list) -> LidarScan:
        """
        Converte lista bruta de medições em LidarScan filtrado.

        Filtragem aplicada:
        - Remove pontos com qualidade abaixo de min_quality
        - Remove distâncias fora do intervalo [min_dist, max_dist]
        - Converte para coordenadas cartesianas (LidarPoint)

        Args:
            raw_scan: Lista de tuplas (quality, angle, distance) do RPLidar.
        Returns:
            LidarScan com pontos filtrados e processados.
        """
        points = []
        for quality, angle, distance in raw_scan:
            if quality < self._min_quality:
                continue
            if distance < self._min_dist_mm or distance > self._max_dist_mm:
                continue

            point = LidarPoint(
                angle_deg=float(angle) % 360.0,
                distance_mm=float(distance),
                quality=int(quality)
            )
            points.append(point)

        return LidarScan(points=points, scan_id=self._scan_count)
