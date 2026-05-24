/**
 * @file motor_driver.h
 * @brief Driver de acionamento para motores DC via ponte H L298N
 *
 * Abstrai o controle de velocidade e direção de dois motores DC
 * em configuração de tração diferencial (differential drive),
 * padrão para robótica móvel de duas rodas motrizes.
 *
 * A tração diferencial permite:
 *  - Translação: ambos os motores com mesma velocidade/direção
 *  - Rotação in-loco: motores em sentidos opostos
 *  - Curva: velocidades assimétricas entre os motores
 *
 * Diagrama de controle L298N:
 *   ENA=PWM, IN1=H, IN2=L → Motor A forward
 *   ENA=PWM, IN1=L, IN2=H → Motor A backward
 *   ENA=L   (qualquer IN)  → Motor A coast/stop
 */

#pragma once

#include <Arduino.h>
#include "../config.h"

/**
 * @enum MotorDirection_t
 * @brief Sentido de rotação do motor
 */
typedef enum {
    MOTOR_FORWARD  = 0,  ///< Rotação para frente
    MOTOR_BACKWARD = 1,  ///< Rotação para trás
    MOTOR_BRAKE    = 2,  ///< Freio ativo (curto-circuito nas bobinas)
    MOTOR_COAST    = 3   ///< Desaceleração livre (sem frenagem)
} MotorDirection_t;

/**
 * @struct MotorPins_t
 * @brief Mapeamento de pinos para um motor individual
 */
typedef struct {
    uint8_t pwmPin;   ///< Pino PWM (Enable do L298N)
    uint8_t dir1Pin;  ///< Pino de direção 1 (IN1/IN3)
    uint8_t dir2Pin;  ///< Pino de direção 2 (IN2/IN4)
} MotorPins_t;

/**
 * @class MotorDriver
 * @brief Controlador de tração diferencial para dois motores DC
 */
class MotorDriver {
public:
    /**
     * @brief Inicializa os pinos de controle dos motores
     *
     * Configura todos os pinos como OUTPUT e garante estado
     * inicial seguro (motores parados).
     */
    void begin();

    /**
     * @brief Define velocidade e direção do motor esquerdo
     * @param speed     Velocidade PWM [0–255]
     * @param direction Sentido de rotação
     */
    void setLeft(uint8_t speed, MotorDirection_t direction);

    /**
     * @brief Define velocidade e direção do motor direito
     * @param speed     Velocidade PWM [0–255]
     * @param direction Sentido de rotação
     */
    void setRight(uint8_t speed, MotorDirection_t direction);

    /**
     * @brief Aplica correção PID ao par de motores (seguimento de linha)
     *
     * Implementa steering diferencial:
     *   v_esq = BASE_SPEED - pidOutput
     *   v_dir = BASE_SPEED + pidOutput
     *
     * O sinal de pidOutput determina para qual lado o robô curva.
     * Saída positiva → curva à direita (acelera dir, freia esq).
     *
     * @param pidOutput Saída do controlador PID [-255, +255]
     */
    void applyPIDSteering(float pidOutput);

    /**
     * @brief Para ambos os motores com freio ativo
     */
    void stopAll();

    /**
     * @brief Rotação in-loco à esquerda (para desvio de obstáculo)
     * @param speed Velocidade de rotação [0–255]
     */
    void rotateLeft(uint8_t speed);

    /**
     * @brief Rotação in-loco à direita (para desvio de obstáculo)
     * @param speed Velocidade de rotação [0–255]
     */
    void rotateRight(uint8_t speed);

    /**
     * @brief Move ambos os motores para frente
     * @param speed Velocidade [0–255]
     */
    void moveForward(uint8_t speed);

    /**
     * @brief Move ambos os motores para trás
     * @param speed Velocidade [0–255]
     */
    void moveBackward(uint8_t speed);

    /**
     * @brief Retorna a velocidade atual do motor esquerdo (0–255)
     */
    uint8_t getLeftSpeed() const  { return _leftSpeed; }

    /**
     * @brief Retorna a velocidade atual do motor direito (0–255)
     */
    uint8_t getRightSpeed() const { return _rightSpeed; }

private:
    uint8_t _leftSpeed  = 0;
    uint8_t _rightSpeed = 0;

    /**
     * @brief Define o estado de um motor individual
     * @param pins      Estrutura de pinos do motor
     * @param speed     Velocidade PWM [0–255]
     * @param direction Sentido de rotação
     */
    void _setMotor(const MotorPins_t& pins,
                   uint8_t speed,
                   MotorDirection_t direction);

    /**
     * @brief Aplica zona morta e limita a velocidade ao intervalo válido
     * @param raw Velocidade bruta
     * @return Velocidade corrigida dentro de [DEADBAND, MAX_SPEED]
     */
    uint8_t _constrainSpeed(int16_t raw) const;
};
