"""
utils/helpers.py
=================
Utilitários gerais do sistema Raspberry Pi.

Inclui:
    - setup_logging   : Configuração padronizada do logger
    - load_config     : Carregamento de YAML com fallback
    - LoopTimer       : Controle preciso de período de loop
    - RateMonitor     : Monitoramento de taxa de execução
    - EMAFilter       : Filtro exponencial (espelho do Arduino)
"""

import time
import logging
import logging.handlers
from pathlib import Path
from typing import Optional

logger = logging.getLogger(__name__)


# ============================================================
#  LOGGING
# ============================================================

def setup_logging(level: int = logging.INFO,
                  log_dir: str = 'logs',
                  log_file: str = 'system.log'):
    """
    Configura o sistema de logging com saída para console e arquivo.

    Formato de log:
        [2025-06-01 10:23:45] [INFO    ] [main] Mensagem

    Args:
        level    : Nível de logging (ex: logging.DEBUG, logging.INFO)
        log_dir  : Diretório para arquivos de log
        log_file : Nome do arquivo de log principal
    """
    log_path = Path(log_dir)
    log_path.mkdir(parents=True, exist_ok=True)

    fmt = logging.Formatter(
        fmt='[%(asctime)s] [%(levelname)-8s] [%(name)s] %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )

    root_logger = logging.getLogger()
    root_logger.setLevel(level)

    # Handler de console
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(fmt)
    console_handler.setLevel(level)
    root_logger.addHandler(console_handler)

    # Handler de arquivo com rotação (máx 5MB, 3 backups)
    try:
        file_handler = logging.handlers.RotatingFileHandler(
            filename=log_path / log_file,
            maxBytes=5 * 1024 * 1024,
            backupCount=3,
            encoding='utf-8'
        )
        file_handler.setFormatter(fmt)
        file_handler.setLevel(logging.DEBUG)
        root_logger.addHandler(file_handler)
    except (OSError, PermissionError) as e:
        logging.warning(f"[Helpers] Não foi possível criar arquivo de log: {e}")


# ============================================================
#  CARREGAMENTO DE CONFIGURAÇÃO
# ============================================================

def load_config(path: str) -> dict:
    """
    Carrega configuração de um arquivo YAML.

    Args:
        path : Caminho para o arquivo YAML.
    Returns:
        Dicionário com as configurações, ou {} em caso de erro.
    """
    try:
        import yaml
        with open(path, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)
            return config if isinstance(config, dict) else {}
    except ImportError:
        logger.warning("[Helpers] PyYAML não disponível. Instale: pip install pyyaml")
        return {}
    except FileNotFoundError:
        logger.warning(f"[Helpers] Arquivo de configuração não encontrado: {path}")
        return {}
    except Exception as e:
        logger.error(f"[Helpers] Erro ao carregar config '{path}': {e}")
        return {}


# ============================================================
#  TEMPORIZADOR DE LOOP
# ============================================================

class LoopTimer:
    """
    Controla o período de execução de um loop com precisão.

    Usa sleep adaptativo para manter a frequência alvo sem
    acúmulo de drift ao longo do tempo.

    Args:
        period_s : Período alvo do loop em segundos.

    Uso:
        timer = LoopTimer(0.02)   # 50 Hz
        while running:
            timer.start()
            do_work()
            timer.sleep()         # Aguarda o tempo restante do período
    """

    def __init__(self, period_s: float):
        self._period  = period_s
        self._t_start = time.monotonic()

    def start(self):
        """Marca o início da iteração atual."""
        self._t_start = time.monotonic()

    def sleep(self):
        """
        Aguarda o tempo restante para completar o período.

        Se o processamento já excedeu o período, retorna imediatamente
        (sem sleep negativo) e loga um warning de overrun.
        """
        elapsed  = time.monotonic() - self._t_start
        remaining = self._period - elapsed

        if remaining > 0:
            time.sleep(remaining)
        elif remaining < -0.005:
            logger.debug(
                f"[LoopTimer] Overrun de {abs(remaining)*1000:.1f}ms "
                f"(período={self._period*1000:.0f}ms)"
            )

    def elapsed_ms(self) -> float:
        """Retorna o tempo decorrido desde o início da iteração em ms."""
        return (time.monotonic() - self._t_start) * 1000.0


# ============================================================
#  MONITOR DE TAXA
# ============================================================

class RateMonitor:
    """
    Monitora a taxa de execução real de um processo.

    Calcula a frequência média das últimas N chamadas.

    Args:
        window_size : Número de amostras para a média móvel

    Uso:
        monitor = RateMonitor()
        while True:
            monitor.tick()
            print(f"Taxa: {monitor.get_rate_hz():.1f} Hz")
    """

    def __init__(self, window_size: int = 50):
        self._window  = window_size
        self._times   = []
        self._last_t  = None

    def tick(self):
        """Registra uma nova ocorrência."""
        now = time.monotonic()
        if self._last_t is not None:
            dt = now - self._last_t
            if dt > 0:
                self._times.append(dt)
                if len(self._times) > self._window:
                    self._times.pop(0)
        self._last_t = now

    def get_rate_hz(self) -> float:
        """
        Retorna a taxa média de execução em Hz.

        Returns:
            Taxa em Hz, ou 0.0 se dados insuficientes.
        """
        if not self._times:
            return 0.0
        avg_dt = sum(self._times) / len(self._times)
        return 1.0 / avg_dt if avg_dt > 0 else 0.0

    def reset(self):
        """Reinicia o monitor."""
        self._times  = []
        self._last_t = None


# ============================================================
#  FILTRO EMA (espelho do Arduino)
# ============================================================

class EMAFilter:
    """
    Filtro de Média Exponencial Móvel para Python.

    y(k) = α*x(k) + (1-α)*y(k-1)

    Espelho do EMAFilter implementado no firmware Arduino,
    permitindo comportamento simétrico entre as camadas.

    Args:
        alpha : Coeficiente de suavização ∈ (0,1]
    """

    def __init__(self, alpha: float = 0.2):
        self._alpha   = alpha
        self._output  = 0.0
        self._init    = False

    def update(self, value: float) -> float:
        """
        Processa nova amostra.

        Args:
            value: Valor bruto da amostra.
        Returns:
            Valor filtrado.
        """
        if not self._init:
            self._output = value
            self._init   = True
        else:
            self._output = self._alpha * value + (1.0 - self._alpha) * self._output
        return self._output

    def get(self) -> float:
        return self._output

    def reset(self, value: float = 0.0):
        self._output = value
        self._init   = (value != 0.0)
