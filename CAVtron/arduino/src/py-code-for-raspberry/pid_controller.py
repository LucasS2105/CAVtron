"""
control/pid_controller.py
==========================
Controlador PID discreto — conversão direta de pid_controller.h/.cpp.

Implementação na forma posicional discreta:
    u(k) = Kp·e(k) + Ki·Ts·Σe(i) + Kd/Ts·(e(k) − e(k−1))

Recursos mantidos da versão C++:
  - Derivada sobre a medição (Derivative on Measurement):
      D = −Kd · Δmeasurement / Ts
    Elimina "derivative kick" em mudanças abruptas de setpoint.
  - Anti-windup por clamping condicional:
      Integrador só acumula se a saída não está saturada OU se o
      erro contribui para reduzir a saturação.
  - Filtro EMA no termo derivativo:
      y_D(k) = α·D(k) + (1−α)·y_D(k−1)
  - Saturação simétrica configurável.
  - Reset em tempo de execução (bumpless transfer).

Referência:
    Åström, K.J. & Hägglund, T. (2006). Advanced PID Control.
    ISA — The Instrumentation, Systems, and Automation Society.
"""

from dataclasses import dataclass
import logging

logger = logging.getLogger(__name__)


@dataclass
class PIDConfig:
    """Parâmetros de configuração do controlador PID."""
    kp: float             = 32.0
    ki: float             = 0.4
    kd: float             = 22.0
    sample_time_s: float  = 0.010    # Ts — deve coincidir com o período do loop
    output_max: float     = 255.0
    output_min: float     = -255.0
    integral_max: float   = 80.0     # Limite anti-windup do acumulador
    derivative_alpha: float = 0.2    # Coeficiente filtro EMA derivativo


class PIDController:
    """
    Controlador PID discreto com recursos de robustez industrial.

    Args:
        config: Parâmetros do controlador (PIDConfig).
    """

    def __init__(self, config: PIDConfig = None):
        self._cfg = config or PIDConfig()

        # Estado interno
        self._integral: float        = 0.0
        self._prev_measurement: float = 0.0
        self._filtered_deriv: float  = 0.0
        self._last_error: float      = 0.0
        self._output: float          = 0.0

        # Termos individuais (para telemetria/debug)
        self._term_p: float = 0.0
        self._term_i: float = 0.0
        self._term_d: float = 0.0

        self._first_run: bool = True

    # ----------------------------------------------------------
    #  CÁLCULO DA SAÍDA DE CONTROLE
    # ----------------------------------------------------------

    def compute(self, setpoint: float, measurement: float) -> float:
        """
        Calcula a saída de controle u(k) para o ciclo atual.

        Deve ser chamado periodicamente com intervalo igual a sample_time_s.

        Args:
            setpoint    : Valor de referência desejado (SP).
            measurement : Valor medido atual do processo (PV).
        Returns:
            Saída de controle saturada u(k) ∈ [output_min, output_max].
        """
        cfg = self._cfg

        # Inicialização na primeira execução
        if self._first_run:
            self._prev_measurement = measurement
            self._first_run = False

        # ---- Erro ----
        error = setpoint - measurement
        self._last_error = error

        # ---- Termo Proporcional ----
        self._term_p = cfg.kp * error

        # ---- Termo Derivativo (sobre a medição, não sobre o erro) ----
        # Derivada: D_raw = −(measurement − prev_measurement) / Ts
        # Sinal negativo: aumento da medição → redução do esforço de controle.
        # Isso evita "derivative kick" quando o setpoint muda abruptamente.
        raw_deriv = (measurement - self._prev_measurement) / cfg.sample_time_s

        # Filtro EMA para atenuar ruído de alta frequência antes de derivar
        self._filtered_deriv = (cfg.derivative_alpha * raw_deriv
                                + (1.0 - cfg.derivative_alpha) * self._filtered_deriv)

        self._term_d = -cfg.kd * self._filtered_deriv

        # ---- Saída parcial (P + D, sem integrador) ----
        # Usada pelo anti-windup para verificar saturação
        output_pre_integral = self._term_p + self._term_d

        # ---- Termo Integral com Anti-Windup (Clamping Condicional) ----
        # O integrador acumula apenas quando não há saturação ativa,
        # ou quando o erro contribui para reduzir a saturação existente.
        output_test = output_pre_integral + self._integral * cfg.ki * cfg.sample_time_s
        if self._anti_windup_clear(output_test, error):
            self._integral += error * cfg.sample_time_s
            # Limitação do acumulador (back-calculation complementar)
            self._integral = max(-cfg.integral_max,
                                 min(cfg.integral_max, self._integral))

        self._term_i = cfg.ki * self._integral

        # ---- Saída total e saturação ----
        raw_output = self._term_p + self._term_i + self._term_d
        self._output = self._saturate(raw_output)

        # ---- Atualiza estado para próxima iteração ----
        self._prev_measurement = measurement

        return self._output

    # ----------------------------------------------------------
    #  RESET
    # ----------------------------------------------------------

    def reset(self):
        """
        Reinicia o estado interno do controlador.

        Deve ser chamado ao retomar o controle após período de inatividade
        para evitar transientes indesejados (bumpless transfer).
        Reseta integral e derivativo, mas mantém os ganhos.
        """
        self._integral       = 0.0
        self._filtered_deriv = 0.0
        self._last_error     = 0.0
        self._output         = 0.0
        self._term_p         = 0.0
        self._term_i         = 0.0
        self._term_d         = 0.0
        self._first_run      = True

    def set_gains(self, kp: float, ki: float, kd: float):
        """
        Atualiza os ganhos em tempo de execução.

        Reseta o integrador ao alterar ganhos para evitar transientes.

        Args:
            kp, ki, kd: Novos ganhos proporcional, integral, derivativo.
        """
        self._cfg.kp = kp
        self._cfg.ki = ki
        self._cfg.kd = kd
        self._integral = 0.0  # Evita transiente na troca de ganhos
        logger.debug(f"[PID] Ganhos atualizados: Kp={kp}, Ki={ki}, Kd={kd}")

    # ----------------------------------------------------------
    #  PROPRIEDADES DE TELEMETRIA
    # ----------------------------------------------------------

    @property
    def last_error(self) -> float:
        return self._last_error

    @property
    def output(self) -> float:
        return self._output

    @property
    def term_p(self) -> float:
        return self._term_p

    @property
    def term_i(self) -> float:
        return self._term_i

    @property
    def term_d(self) -> float:
        return self._term_d

    # ----------------------------------------------------------
    #  MÉTODOS PRIVADOS
    # ----------------------------------------------------------

    def _saturate(self, value: float) -> float:
        """Satura o valor entre os limites configurados."""
        return max(self._cfg.output_min, min(self._cfg.output_max, value))

    def _anti_windup_clear(self, output: float, error: float) -> bool:
        """
        Decide se o integrador deve acumular neste ciclo.

        Clamping condicional:
          - Saturação superior (output ≥ max) e erro positivo → bloqueia
            (integrar mais só pioraria a saturação)
          - Saturação inferior (output ≤ min) e erro negativo → bloqueia
          - Caso contrário → permite integração

        Args:
            output: Saída pré-saturação atual.
            error : Erro atual e(k).
        Returns:
            True se integração deve prosseguir.
        """
        if output >= self._cfg.output_max and error > 0.0:
            return False
        if output <= self._cfg.output_min and error < 0.0:
            return False
        return True
