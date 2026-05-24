/**
 * @file pid_controller.cpp
 * @brief Implementação do controlador PID discreto
 *
 * Referência teórica:
 *  Åström, K.J. & Hägglund, T. (2006). Advanced PID Control.
 *  ISA — The Instrumentation, Systems, and Automation Society.
 */

#include "pid_controller.h"
#include <math.h>

// ============================================================
//  CONSTRUTOR
// ============================================================

PIDController::PIDController(const PIDConfig_t& config)
    : _cfg(config),
      _integral(0.0f),
      _prevMeasurement(0.0f),
      _filteredDeriv(0.0f),
      _lastError(0.0f),
      _output(0.0f),
      _termP(0.0f),
      _termI(0.0f),
      _termD(0.0f),
      _firstRun(true)
{
    // Nenhum hardware inicializado aqui — responsabilidade do chamador
}

// ============================================================
//  CÁLCULO DA SAÍDA DE CONTROLE
// ============================================================

float PIDController::compute(float setpoint, float measurement) {
    // --- Inicialização na primeira execução -----------------------
    if (_firstRun) {
        _prevMeasurement = measurement;
        _firstRun = false;
    }

    // --- Cálculo do erro ------------------------------------------
    const float error = setpoint - measurement;
    _lastError = error;

    // --- Termo Proporcional ----------------------------------------
    _termP = _cfg.kp * error;

    // --- Termo Derivativo (sobre a medição, não sobre o erro) ------
    // Derivada sobre medição: D = -Kd/Ts * Δmeasurement
    // Sinal negativo porque aumento na medição → redução do esforço
    // (evita "derivative kick" em mudanças de setpoint)
    const float rawDeriv = (measurement - _prevMeasurement)
                           / (_cfg.sampleTimeMs / 1000.0f);

    // Filtro EMA: y(k) = α*x(k) + (1-α)*y(k-1)
    _filteredDeriv = _cfg.derivativeAlpha * rawDeriv
                   + (1.0f - _cfg.derivativeAlpha) * _filteredDeriv;

    _termD = -_cfg.kd * _filteredDeriv;

    // --- Cálculo da saída sem integrador (para teste anti-windup) --
    float outputPreIntegral = _termP + _termD;

    // --- Termo Integral com Anti-Windup (Clamping Condicional) -----
    // O integrador só acumula se a saída não está saturada OU se
    // o erro "desfaz" a saturação (contribuição contrária).
    if (_antiWindupClear(outputPreIntegral + _integral * _cfg.ki * (_cfg.sampleTimeMs / 1000.0f), error)) {
        _integral += error * (_cfg.sampleTimeMs / 1000.0f);

        // Limitação do acumulador integral (back-calculation complementar)
        _integral = constrain(_integral, -_cfg.integralMax, _cfg.integralMax);
    }
    _termI = _cfg.ki * _integral;

    // --- Saída total e saturação -----------------------------------
    const float rawOutput = _termP + _termI + _termD;
    _output = _saturate(rawOutput);

    // --- Atualiza estado para próxima iteração ---------------------
    _prevMeasurement = measurement;

    return _output;
}

// ============================================================
//  RESET DE ESTADO INTERNO
// ============================================================

void PIDController::reset() {
    _integral       = 0.0f;
    _filteredDeriv  = 0.0f;
    _lastError      = 0.0f;
    _output         = 0.0f;
    _termP          = 0.0f;
    _termI          = 0.0f;
    _termD          = 0.0f;
    _firstRun       = true;
}

// ============================================================
//  ATUALIZAÇÃO DE GANHOS EM TEMPO DE EXECUÇÃO
// ============================================================

void PIDController::setGains(float kp, float ki, float kd) {
    _cfg.kp = kp;
    _cfg.ki = ki;
    _cfg.kd = kd;
    // Reseta integrador ao alterar ganhos para evitar transientes
    _integral = 0.0f;
}

// ============================================================
//  MÉTODOS PRIVADOS
// ============================================================

float PIDController::_saturate(float value) const {
    if (value > _cfg.outputMax) return _cfg.outputMax;
    if (value < _cfg.outputMin) return _cfg.outputMin;
    return value;
}

bool PIDController::_antiWindupClear(float output, float error) const {
    // Saturação superior: só integra se erro é negativo (redução)
    if (output >= _cfg.outputMax && error > 0.0f) return false;
    // Saturação inferior: só integra se erro é positivo (redução)
    if (output <= _cfg.outputMin && error < 0.0f) return false;
    return true;
}
