/**
 * @file motor_driver.cpp
 * @brief Implementação do driver de tração diferencial via L298N
 */

#include "motor_driver.h"

// Definição estática dos pinos (evita alocação dinâmica)
static const MotorPins_t LEFT_MOTOR  = { MOTOR_LEFT_PWM,  MOTOR_LEFT_DIR1,  MOTOR_LEFT_DIR2  };
static const MotorPins_t RIGHT_MOTOR = { MOTOR_RIGHT_PWM, MOTOR_RIGHT_DIR1, MOTOR_RIGHT_DIR2 };

// ============================================================
//  INICIALIZAÇÃO
// ============================================================

void MotorDriver::begin() {
    // Configura todos os pinos como saída digital/analógica
    pinMode(MOTOR_LEFT_PWM,   OUTPUT);
    pinMode(MOTOR_LEFT_DIR1,  OUTPUT);
    pinMode(MOTOR_LEFT_DIR2,  OUTPUT);

    pinMode(MOTOR_RIGHT_PWM,  OUTPUT);
    pinMode(MOTOR_RIGHT_DIR1, OUTPUT);
    pinMode(MOTOR_RIGHT_DIR2, OUTPUT);

    // Estado inicial: motores parados com freio ativo
    stopAll();
}

// ============================================================
//  CONTROLE INDIVIDUAL DOS MOTORES
// ============================================================

void MotorDriver::setLeft(uint8_t speed, MotorDirection_t direction) {
    _leftSpeed = speed;
    _setMotor(LEFT_MOTOR, speed, direction);
}

void MotorDriver::setRight(uint8_t speed, MotorDirection_t direction) {
    _rightSpeed = speed;
    _setMotor(RIGHT_MOTOR, speed, direction);
}

// ============================================================
//  STEERING DIFERENCIAL PID
// ============================================================

void MotorDriver::applyPIDSteering(float pidOutput) {
    // Cálculo das velocidades por diferencial
    const int16_t leftRaw  = (int16_t)(MOTOR_BASE_SPEED - pidOutput);
    const int16_t rightRaw = (int16_t)(MOTOR_BASE_SPEED + pidOutput);

    // Determina direção e velocidade para motor esquerdo
    MotorDirection_t leftDir  = (leftRaw  >= 0) ? MOTOR_FORWARD : MOTOR_BACKWARD;
    MotorDirection_t rightDir = (rightRaw >= 0) ? MOTOR_FORWARD : MOTOR_BACKWARD;

    const uint8_t leftSpeed  = _constrainSpeed(abs(leftRaw));
    const uint8_t rightSpeed = _constrainSpeed(abs(rightRaw));

    _setMotor(LEFT_MOTOR,  leftSpeed,  leftDir);
    _setMotor(RIGHT_MOTOR, rightSpeed, rightDir);

    _leftSpeed  = leftSpeed;
    _rightSpeed = rightSpeed;
}

// ============================================================
//  MOVIMENTOS PRÉ-DEFINIDOS
// ============================================================

void MotorDriver::stopAll() {
    _setMotor(LEFT_MOTOR,  0, MOTOR_BRAKE);
    _setMotor(RIGHT_MOTOR, 0, MOTOR_BRAKE);
    _leftSpeed  = 0;
    _rightSpeed = 0;
}

void MotorDriver::rotateLeft(uint8_t speed) {
    // Motor esquerdo recua, motor direito avança → rotação CCW
    _setMotor(LEFT_MOTOR,  speed, MOTOR_BACKWARD);
    _setMotor(RIGHT_MOTOR, speed, MOTOR_FORWARD);
    _leftSpeed  = speed;
    _rightSpeed = speed;
}

void MotorDriver::rotateRight(uint8_t speed) {
    // Motor esquerdo avança, motor direito recua → rotação CW
    _setMotor(LEFT_MOTOR,  speed, MOTOR_FORWARD);
    _setMotor(RIGHT_MOTOR, speed, MOTOR_BACKWARD);
    _leftSpeed  = speed;
    _rightSpeed = speed;
}

void MotorDriver::moveForward(uint8_t speed) {
    _setMotor(LEFT_MOTOR,  speed, MOTOR_FORWARD);
    _setMotor(RIGHT_MOTOR, speed, MOTOR_FORWARD);
    _leftSpeed  = speed;
    _rightSpeed = speed;
}

void MotorDriver::moveBackward(uint8_t speed) {
    _setMotor(LEFT_MOTOR,  speed, MOTOR_BACKWARD);
    _setMotor(RIGHT_MOTOR, speed, MOTOR_BACKWARD);
    _leftSpeed  = speed;
    _rightSpeed = speed;
}

// ============================================================
//  MÉTODOS PRIVADOS
// ============================================================

void MotorDriver::_setMotor(const MotorPins_t& pins,
                             uint8_t speed,
                             MotorDirection_t direction)
{
    switch (direction) {
        case MOTOR_FORWARD:
            digitalWrite(pins.dir1Pin, HIGH);
            digitalWrite(pins.dir2Pin, LOW);
            analogWrite(pins.pwmPin, speed);
            break;

        case MOTOR_BACKWARD:
            digitalWrite(pins.dir1Pin, LOW);
            digitalWrite(pins.dir2Pin, HIGH);
            analogWrite(pins.pwmPin, speed);
            break;

        case MOTOR_BRAKE:
            // Freio ativo: ambas as entradas HIGH → curto nas bobinas
            digitalWrite(pins.dir1Pin, HIGH);
            digitalWrite(pins.dir2Pin, HIGH);
            analogWrite(pins.pwmPin, 0);
            break;

        case MOTOR_COAST:
        default:
            // Costa: PWM zero, entradas LOW → motor desacoplado
            digitalWrite(pins.dir1Pin, LOW);
            digitalWrite(pins.dir2Pin, LOW);
            analogWrite(pins.pwmPin, 0);
            break;
    }
}

uint8_t MotorDriver::_constrainSpeed(int16_t raw) const {
    if (raw < MOTOR_DEADBAND) return 0;              // Abaixo da zona morta
    if (raw > MOTOR_MAX_SPEED) return MOTOR_MAX_SPEED;
    return (uint8_t)raw;
}
