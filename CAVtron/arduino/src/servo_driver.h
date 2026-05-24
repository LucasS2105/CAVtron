/**
 * @file servo_control.h
 * @brief Controle de servomotores para o sistema de garra (manipulador)
 *
 * Gerencia dois servomotores:
 *  - Servo de garra (grip): abertura e fechamento
 *  - Servo de elevação (lift): posicionamento vertical do braço
 *
 * Implementa movimentação suave por sweep incremental (slew rate limiting),
 * evitando correntes de pico e impactos mecânicos que prejudicam
 * a estrutura e a vida útil dos servos.
 *
 * Compatível com a biblioteca padrão Servo.h do Arduino.
 */

#pragma once

#include <Arduino.h>
#include <Servo.h>
#include "../config.h"

/**
 * @enum GripAction_t
 * @brief Ações disponíveis para o sistema de garra
 */
typedef enum {
    GRIP_OPEN   = 0,  ///< Abre a garra para posição de repouso
    GRIP_CLOSE  = 1,  ///< Fecha a garra para captura de objeto
    LIFT_UP     = 2,  ///< Eleva o braço para transporte
    LIFT_DOWN   = 3,  ///< Abaixa o braço para posição de captura
    GRIP_FULL_CAPTURE = 4  ///< Sequência completa: descer → fechar → subir
} GripAction_t;

/**
 * @class ServoControl
 * @brief Controlador de servomotores para o manipulador do robô
 */
class ServoControl {
public:
    /**
     * @brief Inicializa e acopla os servos aos pinos configurados
     *
     * Posiciona os servos nas posições iniciais seguras (garra aberta,
     * braço elevado) para evitar colisões durante a partida.
     */
    void begin();

    /**
     * @brief Atualiza o estado dos servos (deve ser chamado no loop principal)
     *
     * Executa um passo do sweep se houver movimento em andamento.
     * Não bloqueia o loop — o movimento é assíncrono e incremental.
     */
    void update();

    /**
     * @brief Aciona uma ação do manipulador
     *
     * Encaminha o servo para a posição-alvo correspondente à ação.
     * O movimento ocorre de forma suave via update().
     *
     * @param action Ação a executar (GRIP_OPEN, GRIP_CLOSE, etc.)
     */
    void executeAction(GripAction_t action);

    /**
     * @brief Verifica se algum servo está em movimento
     * @return true se sweep em andamento
     */
    bool isBusy() const;

    /**
     * @brief Força posição imediata do servo de garra (sem sweep)
     * @param angle Ângulo em graus [0–180]
     */
    void setGripDirect(uint8_t angle);

    /**
     * @brief Força posição imediata do servo de elevação (sem sweep)
     * @param angle Ângulo em graus [0–180]
     */
    void setLiftDirect(uint8_t angle);

    /** @brief Retorna ângulo atual do servo de garra */
    uint8_t getGripAngle()  const { return _gripCurrent; }

    /** @brief Retorna ângulo atual do servo de elevação */
    uint8_t getLiftAngle()  const { return _liftCurrent; }

private:
    Servo _servoGrip;     ///< Instância do servo de garra
    Servo _servoLift;     ///< Instância do servo de elevação

    uint8_t _gripCurrent  = SERVO_GRIP_OPEN_DEG;
    uint8_t _gripTarget   = SERVO_GRIP_OPEN_DEG;
    uint8_t _liftCurrent  = SERVO_LIFT_UP_DEG;
    uint8_t _liftTarget   = SERVO_LIFT_UP_DEG;

    uint32_t _lastSweepTime = 0;  ///< millis() do último passo de sweep

    bool _gripBusy = false;
    bool _liftBusy = false;

    /**
     * @brief Avança um passo no sweep de um servo
     * @param current  Posição atual (modificada in-place)
     * @param target   Posição alvo
     * @param servo    Referência ao objeto Servo
     * @param busyFlag Flag de ocupação (modificada in-place)
     */
    void _sweepStep(uint8_t& current,
                    uint8_t  target,
                    Servo&   servo,
                    bool&    busyFlag);
};
