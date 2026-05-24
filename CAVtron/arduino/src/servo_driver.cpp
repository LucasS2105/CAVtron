/**
 * @file servo_driver.cpp
 * @brief Implementação do controle de servomotores para o manipulador
 */

#include "servo_driver.h"

// ============================================================
//  INICIALIZAÇÃO
// ============================================================

void ServoControl::begin() {
    _servoGrip.attach(SERVO_GRIP_PIN);
    _servoLift.attach(SERVO_LIFT_PIN);

    // Posições iniciais seguras
    _gripCurrent = SERVO_GRIP_OPEN_DEG;
    _gripTarget  = SERVO_GRIP_OPEN_DEG;
    _liftCurrent = SERVO_LIFT_UP_DEG;
    _liftTarget  = SERVO_LIFT_UP_DEG;

    _servoGrip.write(_gripCurrent);
    _servoLift.write(_liftCurrent);

    _gripBusy = false;
    _liftBusy = false;
    _lastSweepTime = millis();
}

// ============================================================
//  UPDATE (SWEEP ASSÍNCRONO — CHAMADO NO LOOP)
// ============================================================

void ServoControl::update() {
    const uint32_t now = millis();

    // Respeita o intervalo de sweep configurado
    if ((now - _lastSweepTime) < SERVO_SWEEP_DELAY_MS) return;
    _lastSweepTime = now;

    _sweepStep(_gripCurrent, _gripTarget, _servoGrip, _gripBusy);
    _sweepStep(_liftCurrent, _liftTarget, _servoLift, _liftBusy);
}

// ============================================================
//  EXECUÇÃO DE AÇÕES
// ============================================================

void ServoControl::executeAction(GripAction_t action) {
    switch (action) {
        case GRIP_OPEN:
            _gripTarget = SERVO_GRIP_OPEN_DEG;
            _gripBusy   = true;
            break;

        case GRIP_CLOSE:
            _gripTarget = SERVO_GRIP_CLOSE_DEG;
            _gripBusy   = true;
            break;

        case LIFT_UP:
            _liftTarget = SERVO_LIFT_UP_DEG;
            _liftBusy   = true;
            break;

        case LIFT_DOWN:
            _liftTarget = SERVO_LIFT_DOWN_DEG;
            _liftBusy   = true;
            break;

        case GRIP_FULL_CAPTURE:
            /**
             * Sequência atômica de captura:
             * 1. Abaixar braço
             * 2. Fechar garra
             * 3. Elevar braço
             * (Executado de forma sincrona para garantir sequenciamento)
             */
            setLiftDirect(SERVO_LIFT_DOWN_DEG);
            delay(500);
            setGripDirect(SERVO_GRIP_CLOSE_DEG);
            delay(400);
            setLiftDirect(SERVO_LIFT_UP_DEG);
            delay(500);
            break;

        default:
            break;
    }
}

// ============================================================
//  CONTROLE DIRETO (SEM SWEEP)
// ============================================================

void ServoControl::setGripDirect(uint8_t angle) {
    angle        = constrain(angle, 0, 180);
    _gripCurrent = angle;
    _gripTarget  = angle;
    _gripBusy    = false;
    _servoGrip.write(angle);
}

void ServoControl::setLiftDirect(uint8_t angle) {
    angle        = constrain(angle, 0, 180);
    _liftCurrent = angle;
    _liftTarget  = angle;
    _liftBusy    = false;
    _servoLift.write(angle);
}

// ============================================================
//  STATUS
// ============================================================

bool ServoControl::isBusy() const {
    return _gripBusy || _liftBusy;
}

// ============================================================
//  MÉTODO PRIVADO: PASSO DE SWEEP
// ============================================================

void ServoControl::_sweepStep(uint8_t& current,
                               uint8_t  target,
                               Servo&   servo,
                               bool&    busyFlag)
{
    if (!busyFlag || current == target) {
        busyFlag = false;
        return;
    }

    // Incrementa ou decrementa 1° por passo (slew rate = 1°/DELAY ms)
    if (current < target) {
        current++;
    } else {
        current--;
    }

    servo.write(current);

    if (current == target) {
        busyFlag = false;
    }
}
