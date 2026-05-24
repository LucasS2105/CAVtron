/**
 * @file pid_controller.h
 * @brief Controlador PID discreto com anti-windup e filtro derivativo
 *
 * Implementação do algoritmo PID na forma posicional discreta:
 *
 *   u(k) = Kp*e(k) + Ki*Ts*Σe(i) + Kd/Ts*(e(k) - e(k-1))
 *
 * Recursos implementados:
 *  - Anti-windup por clamping condicional do integrador
 *  - Derivada aplicada à medição (Derivative on Measurement),
 *    evitando "derivative kick" em mudanças abruptas de setpoint
 *  - Filtro EMA (Exponential Moving Average) no termo derivativo
 *  - Saturação simétrica configurável da saída
 *  - Reset em tempo de execução
 */

#pragma once

#include <Arduino.h>

/**
 * @struct PIDConfig_t
 * @brief Parâmetros de configuração do controlador PID
 */
typedef struct {
    float kp;               ///< Ganho proporcional
    float ki;               ///< Ganho integral
    float kd;               ///< Ganho derivativo
    float sampleTimeMs;     ///< Período de amostragem em ms
    float outputMax;        ///< Saturação superior da saída
    float outputMin;        ///< Saturação inferior da saída
    float integralMax;      ///< Limite anti-windup do integrador
    float derivativeAlpha;  ///< Coef. filtro EMA derivativo (0,1]
} PIDConfig_t;

/**
 * @class PIDController
 * @brief Controlador PID discreto com recursos de robustez industrial
 */
class PIDController {
public:
    /**
     * @brief Construtor parametrizado
     * @param config Estrutura com os parâmetros do controlador
     */
    explicit PIDController(const PIDConfig_t& config);

    /**
     * @brief Calcula a saída de controle para o erro atual
     *
     * Deve ser chamado periodicamente com intervalo igual a sampleTimeMs.
     * Internamente computa:
     *   P = Kp * error
     *   I = I_prev + Ki * Ts * error  (com clamping anti-windup)
     *   D = Kd/Ts * alpha*(measurement - prev_measurement)  (filtrado)
     *   output = sat(P + I - D)
     *
     * @param setpoint  Valor de referência desejado
     * @param measurement Valor medido atual do processo
     * @return Saída de controle saturada
     */
    float compute(float setpoint, float measurement);

    /**
     * @brief Reinicia o estado interno do integrador e derivativo
     *
     * Deve ser chamado ao retomar controle após período de inatividade
     * para evitar transientes indesejados (bumpless transfer).
     */
    void reset();

    /**
     * @brief Atualiza ganhos em tempo de execução (auto-tuning futuro)
     * @param kp Novo ganho proporcional
     * @param ki Novo ganho integral
     * @param kd Novo ganho derivativo
     */
    void setGains(float kp, float ki, float kd);

    /** @brief Retorna o erro atual (para telemetria) */
    float getLastError() const { return _lastError; }

    /** @brief Retorna o termo proporcional atual */
    float getTermP() const { return _termP; }

    /** @brief Retorna o termo integral atual */
    float getTermI() const { return _termI; }

    /** @brief Retorna o termo derivativo atual */
    float getTermD() const { return _termD; }

    /** @brief Retorna a saída de controle atual */
    float getOutput() const { return _output; }

private:
    PIDConfig_t _cfg;         ///< Configuração do controlador

    float _integral;          ///< Acumulador integral
    float _prevMeasurement;   ///< Medição anterior (derivada sobre medição)
    float _filteredDeriv;     ///< Derivativo filtrado pelo EMA
    float _lastError;         ///< Último erro calculado
    float _output;            ///< Última saída calculada

    float _termP;             ///< Contribuição proporcional atual
    float _termI;             ///< Contribuição integral atual
    float _termD;             ///< Contribuição derivativa atual

    bool  _firstRun;          ///< Flag de primeira execução

    /**
     * @brief Satura o valor entre os limites configurados
     * @param value Valor a saturar
     * @return Valor saturado
     */
    float _saturate(float value) const;

    /**
     * @brief Verifica se anti-windup deve suprimir a integração
     *
     * Clamping condicional: o integrador só é atualizado se:
     *   - A saída não está saturada, OU
     *   - O erro tem sinal oposto à saturação (contribuição de redução)
     *
     * @param output Saída pré-saturação
     * @param error  Erro atual
     * @return true se integração deve ocorrer
     */
    bool _antiWindupClear(float output, float error) const;
};
