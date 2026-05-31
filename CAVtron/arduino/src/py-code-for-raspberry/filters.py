"""
utils/filters.py
================
Filtros digitais — conversão direta de filters.h (C++) para Python.

Filtros implementados:
  EMAFilter    : IIR 1ª ordem — y(k) = α·x(k) + (1−α)·y(k−1)
  SMAFilter    : FIR média simples com buffer circular (substitui template C++)
  EdgeDetector : Detector de borda rising/falling para sinais booleanos

O SMAFilter<N> do C++ usava template com N fixo em compilação.
Em Python, N é parâmetro do construtor — mesma semântica, sem alocação
dinâmica (deque com maxlen cumpre o mesmo papel do buffer circular).
"""

from collections import deque
from typing import Optional


# ============================================================
#  FILTRO EMA (IIR de 1ª ordem)
# ============================================================

class EMAFilter:
    """
    Filtro de Média Exponencial Móvel (Exponential Moving Average).

    Equivalente a um filtro passa-baixa de 1ª ordem em tempo discreto.
    Frequência de corte equivalente:
        fc = −ln(1−α) / (2π·Ts)   [Hz]

    Args:
        alpha : Coeficiente de suavização ∈ (0, 1]
                0.1 → alta filtragem (muito suave, mais lag)
                0.5 → filtragem média
                0.9 → filtragem mínima (quase sem suavização)
    """

    def __init__(self, alpha: float = 0.2):
        if not (0.0 < alpha <= 1.0):
            raise ValueError(f"alpha deve ser em (0, 1], recebido: {alpha}")
        self._alpha: float = alpha
        self._output: float = 0.0
        self._initialized: bool = False

    def update(self, value: float) -> float:
        """
        Processa uma nova amostra e retorna o valor filtrado.

        Na primeira chamada inicializa a saída com o valor de entrada
        (evita transiente inicial de zero para o sinal real).

        Args:
            value: Amostra bruta do sinal.
        Returns:
            Valor filtrado.
        """
        if not self._initialized:
            self._output = value
            self._initialized = True
        else:
            self._output = self._alpha * value + (1.0 - self._alpha) * self._output
        return self._output

    @property
    def value(self) -> float:
        """Último valor filtrado calculado."""
        return self._output

    def reset(self, initial: float = 0.0):
        """
        Reinicia o filtro para um valor inicial.

        Útil para evitar transientes ao retomar controle após pausa.

        Args:
            initial: Valor de pré-carga do filtro.
        """
        self._output = initial
        self._initialized = (initial != 0.0)

    def set_alpha(self, alpha: float):
        """Atualiza o coeficiente de suavização em tempo de execução."""
        if not (0.0 < alpha <= 1.0):
            raise ValueError(f"alpha deve ser em (0, 1], recebido: {alpha}")
        self._alpha = alpha


# ============================================================
#  FILTRO SMA (FIR — Média Simples Móvel)
# ============================================================

class SMAFilter:
    """
    Filtro de Média Simples Móvel (Simple Moving Average) com buffer circular.

    Substitui o template SMAFilter<N> do C++.
    Usa collections.deque(maxlen=N) que implementa internamente
    o buffer circular com remoção automática do elemento mais antigo.

    Args:
        window_size : Tamanho da janela N [1, 64].
    """

    def __init__(self, window_size: int = 5):
        if not (1 <= window_size <= 64):
            raise ValueError(f"window_size deve ser [1, 64], recebido: {window_size}")
        self._n: int = window_size
        self._buffer: deque = deque(maxlen=window_size)
        self._sum: float = 0.0

    def update(self, value: float) -> float:
        """
        Insere nova amostra e retorna a média da janela atual.

        Quando o buffer ainda não está cheio, a média é calculada
        sobre as amostras disponíveis (comportamento idêntico ao C++).

        Args:
            value: Nova amostra.
        Returns:
            Média das últimas N amostras.
        """
        if len(self._buffer) == self._n:
            # Remove a amostra mais antiga da soma acumulada
            self._sum -= self._buffer[0]

        self._buffer.append(value)
        self._sum += value

        return self._sum / len(self._buffer)

    @property
    def value(self) -> float:
        """Última média calculada."""
        if not self._buffer:
            return 0.0
        return self._sum / len(self._buffer)

    @property
    def count(self) -> int:
        """Número de amostras atualmente no buffer."""
        return len(self._buffer)

    def reset(self):
        """Reinicia o buffer e o acumulador."""
        self._buffer.clear()
        self._sum = 0.0


# ============================================================
#  DETECTOR DE BORDA
# ============================================================

class EdgeDetector:
    """
    Detector de transições de sinal booleano (rising/falling edge).

    Útil para detectar mudanças de estado em sinais discretos
    sem polling de nível — ex: linha detectada → linha perdida.

    Uso:
        detector = EdgeDetector()
        detector.update(sensor.is_line_detected())
        if detector.rising_edge:
            print("Linha acabou de ser detectada!")
    """

    def __init__(self):
        self._prev_state: bool = False
        self._rising: bool     = False
        self._falling: bool    = False

    def update(self, current: bool):
        """
        Atualiza o detector com o estado atual do sinal.

        Args:
            current: Estado atual do sinal booleano.
        """
        self._rising  = current and not self._prev_state
        self._falling = not current and self._prev_state
        self._prev_state = current

    @property
    def rising_edge(self) -> bool:
        """True se houve transição LOW → HIGH neste ciclo."""
        return self._rising

    @property
    def falling_edge(self) -> bool:
        """True se houve transição HIGH → LOW neste ciclo."""
        return self._falling

    @property
    def state(self) -> bool:
        """Estado atual do sinal."""
        return self._prev_state
