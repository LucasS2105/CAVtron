/**
 * @file main.ino
 * @brief Ponto de entrada principal — Camada de Controle Arduino
 *
 * Orquestra todos os módulos do sistema em tempo real:
 *  - Controle PID de seguimento de linha
 *  - Leitura de sensores infravermelhos
 *  - Interface com HuskyLens (detecção de vítimas)
 *  - Controle de motores DC via L298N
 *  - Controle de servos (garra manipuladora)
 *  - Comunicação serial bidirecional com Raspberry Pi
 *  - Máquina de estados (FSM) de controle de missão
 *
 * Arquitetura de temporização:
 *  O loop principal roda a 100 Hz (período = 10ms).
 *  Cada módulo tem seu próprio período interno, executado
 *  com base em timestamps (millis()), garantindo concorrência
 *  cooperativa sem bloqueio.
 *
 * Plataforma: Arduino Mega 2560
 * Compilador: avr-g++ (C++11)
 * IDE:        Arduino IDE 2.x / PlatformIO
 *
 * Dependências externas:
 *  - HUSKYLENS by DFRobot  (Library Manager)
 *  - Servo.h               (built-in)
 *  - Wire.h                (built-in)
 */

// ============================================================
//  INCLUDES
// ============================================================

#include <Wire.h>
#include <Servo.h>
#include <avr/wdt.h>          // Watchdog Timer do hardware AVR

#include "config.h"
#include "control/pid_controller.h"
#include "drivers/motor_driver.h"
#include "drivers/servo_driver.h"
#include "sensors/line_sensor.h"
#include "sensors/huskylens.h"
#include "communication/serial_comm.h"
#include "utils/filters.h"

// ============================================================
//  INSTÂNCIAS DOS MÓDULOS
// ============================================================

// Configuração do controlador PID
static const PIDConfig_t PID_CFG = {
    .kp             = PID_KP,
    .ki             = PID_KI,
    .kd             = PID_KD,
    .sampleTimeMs   = (float)PID_SAMPLE_TIME_MS,
    .outputMax      = PID_OUTPUT_MAX,
    .outputMin      = PID_OUTPUT_MIN,
    .integralMax    = PID_INTEGRAL_MAX,
    .derivativeAlpha = PID_DERIVATIVE_ALPHA
};

PIDController      pid(PID_CFG);
MotorDriver        motors;
ServoControl       servo;
LineSensor         lineSensor;
HuskyLensInterface huskyLens;
SerialComm         comm;

// Filtros auxiliares
EMAFilter          linePositionFilter(0.3f); // Suaviza posição da linha
SMAFilter<5>       victimXFilter;            // Suaviza X da vítima

// Detecção de bordas
EdgeDetector       lineLostEdge;
EdgeDetector       victimEdge;

// ============================================================
//  ESTADO GLOBAL DA FSM
// ============================================================

static RobotState_t currentState     = STATE_IDLE;
static RobotState_t previousState    = STATE_IDLE;
static uint32_t     stateEntryTime   = 0;

// ============================================================
//  TEMPORIZADORES DE TAREFAS
// ============================================================

static uint32_t lastLoopTime         = 0;
static uint32_t lastSensorTxTime     = 0;
static uint32_t lastHuskyPollTime    = 0;

static const uint32_t SENSOR_TX_PERIOD_MS = 50;  // 20 Hz — envio de dados ao RPi

// ============================================================
//  PROTÓTIPOS
// ============================================================

void     setState(RobotState_t newState);
void     handleIncomingCommand(const ParsedMessage_t& msg);
void     runFSM();
void     runLineFollowing();
void     handleLineLost();
void     handleEmergencyStop();
GripAction_t parseGripAction(const char* actionStr);

// ============================================================
//  SETUP
// ============================================================

void setup() {
    // --- Hardware Watchdog AVR (reset em 2s sem wdt_reset()) ---
    wdt_enable(WDTO_2S);

    // --- Inicialização dos módulos ---
    comm.begin();
    SERIAL_DEBUG.println(F("=== Robot Controller - Arduino Layer ==="));
    SERIAL_DEBUG.println(F("[INIT] Inicializando modulos..."));

    motors.begin();
    SERIAL_DEBUG.println(F("[INIT] MotorDriver OK"));

    servo.begin();
    SERIAL_DEBUG.println(F("[INIT] ServoControl OK"));

    lineSensor.begin();
    SERIAL_DEBUG.println(F("[INIT] LineSensor OK"));

    if (!huskyLens.begin()) {
        SERIAL_DEBUG.println(F("[INIT] WARN: HuskyLens nao encontrada. Continuando sem visao."));
    } else {
        SERIAL_DEBUG.println(F("[INIT] HuskyLens OK"));
    }

    pid.reset();
    SERIAL_DEBUG.println(F("[INIT] PIDController OK"));

    // Estado inicial
    setState(STATE_IDLE);

    lastLoopTime     = millis();
    lastSensorTxTime = millis();

    SERIAL_DEBUG.println(F("[INIT] Sistema pronto. Aguardando comando do Raspberry Pi."));
    wdt_reset();
}

// ============================================================
//  LOOP PRINCIPAL (100 Hz)
// ============================================================

void loop() {
    const uint32_t now = millis();

    // Garante período fixo de 10ms (busy-wait controlado)
    if ((now - lastLoopTime) < LOOP_PERIOD_MS) return;
    lastLoopTime = now;

    // ---- Pet do watchdog de hardware AVR ----
    wdt_reset();

    // ---- 1. Leitura de sensores ----
    lineSensor.update();
    servo.update();     // Sweep assíncrono dos servos

    // ---- 2. Polling da HuskyLens (50ms) ----
    if ((now - lastHuskyPollTime) >= HUSKYLENS_POLL_MS) {
        huskyLens.update();
        lastHuskyPollTime = now;
    }

    // ---- 3. Processa comandos recebidos do Raspberry Pi ----
    if (comm.update()) {
        const ParsedMessage_t& msg = comm.getLastMessage();
        if (msg.valid) {
            handleIncomingCommand(msg);
            comm.resetWatchdog();
        }
    }

    // ---- 4. Verifica watchdog de comunicação ----
    if (comm.isWatchdogExpired() && currentState != STATE_IDLE) {
        SERIAL_DEBUG.println(F("[WATCHDOG] Timeout de comunicacao com RPi!"));
        comm.sendError(ERR_WATCHDOG);
        setState(STATE_EMERGENCY);
    }

    // ---- 5. Executa FSM ----
    runFSM();

    // ---- 6. Transmissão periódica de dados de sensores (20 Hz) ----
    if ((now - lastSensorTxTime) >= SENSOR_TX_PERIOD_MS) {
        lastSensorTxTime = now;

        const LineSensorData_t& lineData   = lineSensor.getData();
        const VictimData_t&     victimData = huskyLens.getData();

        comm.sendSensorData(
            lineData.position,
            lineData.lineDetected,
            victimData.detected,
            victimData.normalizedX,
            victimData.normalizedY
        );
    }

    // ---- 7. Detecção de bordas e notificações assíncronas ----
    lineLostEdge.update(lineSensor.isLineLost());
    victimEdge.update(huskyLens.isVictimDetected());

    // Notifica o RPi quando a linha é perdida (borda de subida)
    if (lineLostEdge.isRisingEdge()) {
        comm.sendError(ERR_LINE_LOST);
        SERIAL_DEBUG.println(F("[FSM] Linha perdida!"));
    }
}

// ============================================================
//  FSM — MÁQUINA DE ESTADOS
// ============================================================

void runFSM() {
    switch (currentState) {

        // ---- IDLE: Aguarda comando do Raspberry Pi ----
        case STATE_IDLE:
            motors.stopAll();
            break;

        // ---- FOLLOW_LINE: Controle PID ativo ----
        case STATE_FOLLOW_LINE:
            runLineFollowing();
            break;

        // ---- OBSTACLE: Parado, aguardando instrução do RPi ----
        case STATE_OBSTACLE:
            motors.stopAll();
            pid.reset();  // Reseta integrador para evitar windup acumulado
            break;

        // ---- GRIP_OPEN: Abre a garra ----
        case STATE_GRIP_OPEN:
            if (!servo.isBusy()) {
                // Ação concluída: retorna ao estado de seguimento
                comm.sendAck("GRIP_OPEN");
                setState(STATE_FOLLOW_LINE);
            }
            break;

        // ---- GRIP_CLOSE: Fecha a garra ----
        case STATE_GRIP_CLOSE:
            if (!servo.isBusy()) {
                comm.sendAck("GRIP_CLOSE");
                setState(STATE_GRIP_LIFT);
            }
            break;

        // ---- GRIP_LIFT: Eleva o braço após captura ----
        case STATE_GRIP_LIFT:
            if (!servo.isBusy()) {
                comm.sendAck("LIFT_UP");
                setState(STATE_FOLLOW_LINE);
            }
            break;

        // ---- GRIP_DROP: Abaixa o braço para soltar vítima ----
        case STATE_GRIP_DROP:
            if (!servo.isBusy()) {
                servo.executeAction(GRIP_OPEN);
                setState(STATE_GRIP_OPEN);
            }
            break;

        // ---- EMERGENCY: Parada de emergência total ----
        case STATE_EMERGENCY:
            handleEmergencyStop();
            break;

        default:
            setState(STATE_IDLE);
            break;
    }
}

// ============================================================
//  SEGUIMENTO DE LINHA COM PID
// ============================================================

void runLineFollowing() {
    const LineSensorData_t& lineData = lineSensor.getData();

    if (lineSensor.isLineLost()) {
        handleLineLost();
        return;
    }

    // Filtra posição antes de enviar ao PID
    const float filteredPos = linePositionFilter.update(lineData.position);

    /**
     * Setpoint = 0.0 (linha centralizada)
     * Variável de processo = posição filtrada [-1.0, +1.0]
     *
     * Erro > 0 → linha à direita → PID gera output positivo
     *          → motor direito acelera, esquerdo freia → curva à direita
     */
    const float pidOutput = pid.compute(0.0f, filteredPos);
    motors.applyPIDSteering(pidOutput);

    // Debug de telemetria (a 10Hz, não a 100Hz para não saturar serial)
    static uint8_t debugCounter = 0;
    if (++debugCounter >= 10) {
        debugCounter = 0;
        SERIAL_DEBUG.print(F("PID | pos="));
        SERIAL_DEBUG.print(filteredPos, 3);
        SERIAL_DEBUG.print(F(" out="));
        SERIAL_DEBUG.print(pidOutput, 1);
        SERIAL_DEBUG.print(F(" P="));
        SERIAL_DEBUG.print(pid.getTermP(), 1);
        SERIAL_DEBUG.print(F(" I="));
        SERIAL_DEBUG.print(pid.getTermI(), 1);
        SERIAL_DEBUG.print(F(" D="));
        SERIAL_DEBUG.println(pid.getTermD(), 1);
    }
}

// ============================================================
//  MANOBRA DE RECUPERAÇÃO DE LINHA
// ============================================================

void handleLineLost() {
    /**
     * Estratégia: rotaciona na direção da última posição conhecida.
     * Se última posição > 0 → linha estava à direita → rotaciona CW.
     * Caso contrário → rotaciona CCW.
     */
    pid.reset();

    const float lastPos = lineSensor.getData().position;
    if (lastPos > 0.0f) {
        motors.rotateRight(120);
    } else {
        motors.rotateLeft(120);
    }
}

// ============================================================
//  PARADA DE EMERGÊNCIA
// ============================================================

void handleEmergencyStop() {
    motors.stopAll();
    servo.setGripDirect(SERVO_GRIP_OPEN_DEG);  // Abre garra por segurança
    pid.reset();

    // Retorna a IDLE após 500ms (debounce)
    static uint32_t emergencyEntry = 0;
    if (emergencyEntry == 0) emergencyEntry = millis();

    if ((millis() - emergencyEntry) > 500) {
        emergencyEntry = 0;
        setState(STATE_IDLE);
        comm.sendStateChange(STATE_IDLE);
    }
}

// ============================================================
//  HANDLER DE COMANDOS RECEBIDOS DO RASPBERRY PI
// ============================================================

void handleIncomingCommand(const ParsedMessage_t& msg) {
    // ---- PING ----
    if (strcmp(msg.type, "PING") == 0) {
        comm.sendPong();
        return;
    }

    // ---- MOVE ----
    if (strcmp(msg.type, "MOVE") == 0 && msg.paramCount >= 2) {
        const char* dir   = msg.params[0];
        const uint8_t spd = (uint8_t)atoi(msg.params[1]);

        if      (strcmp(dir, "FWD")  == 0) motors.moveForward(spd);
        else if (strcmp(dir, "BWD")  == 0) motors.moveBackward(spd);
        else if (strcmp(dir, "ROT_L")== 0) motors.rotateLeft(spd);
        else if (strcmp(dir, "ROT_R")== 0) motors.rotateRight(spd);
        else if (strcmp(dir, "STOP") == 0) motors.stopAll();

        comm.sendAck("MOVE");
        return;
    }

    // ---- STOP ----
    if (strcmp(msg.type, "STOP") == 0) {
        motors.stopAll();
        pid.reset();
        setState(STATE_IDLE);
        comm.sendAck("STOP");
        return;
    }

    // ---- GRIP ----
    if (strcmp(msg.type, "GRIP") == 0 && msg.paramCount >= 1) {
        const GripAction_t action = parseGripAction(msg.params[0]);
        servo.executeAction(action);

        if (action == GRIP_OPEN)  setState(STATE_GRIP_OPEN);
        if (action == GRIP_CLOSE) setState(STATE_GRIP_CLOSE);
        if (action == GRIP_FULL_CAPTURE) {
            comm.sendAck("GRIP_CAPTURE");
        }
        return;
    }

    // ---- LIFT ----
    if (strcmp(msg.type, "LIFT") == 0 && msg.paramCount >= 1) {
        if (strcmp(msg.params[0], "UP") == 0) {
            servo.executeAction(LIFT_UP);
            setState(STATE_GRIP_LIFT);
        } else {
            servo.executeAction(LIFT_DOWN);
            setState(STATE_GRIP_DROP);
        }
        comm.sendAck("LIFT");
        return;
    }

    // ---- STATE ----
    if (strcmp(msg.type, "STATE") == 0 && msg.paramCount >= 1) {
        const RobotState_t newState = (RobotState_t)atoi(msg.params[0]);
        if (newState < STATE_COUNT) {
            setState(newState);
            comm.sendAck("STATE");
        }
        return;
    }

    // ---- Comando desconhecido ----
    SERIAL_DEBUG.print(F("[CMD] Comando desconhecido: "));
    SERIAL_DEBUG.println(msg.type);
    comm.sendError(ERR_UNKNOWN_CMD);
}

// ============================================================
//  UTILITÁRIOS
// ============================================================

void setState(RobotState_t newState) {
    if (newState == currentState) return;

    previousState  = currentState;
    currentState   = newState;
    stateEntryTime = millis();

    SERIAL_DEBUG.print(F("[FSM] Estado: "));
    SERIAL_DEBUG.print(previousState);
    SERIAL_DEBUG.print(F(" -> "));
    SERIAL_DEBUG.println(currentState);

    comm.sendStateChange(currentState);
}

GripAction_t parseGripAction(const char* actionStr) {
    if (strcmp(actionStr, "OPEN")    == 0) return GRIP_OPEN;
    if (strcmp(actionStr, "CLOSE")   == 0) return GRIP_CLOSE;
    if (strcmp(actionStr, "CAPTURE") == 0) return GRIP_FULL_CAPTURE;
    return GRIP_OPEN;  // Fallback seguro
}
