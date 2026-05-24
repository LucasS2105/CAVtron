/**
 * @file config.h
 * @brief Configurações globais do sistema — pinos, parâmetros e constantes
 *
 * Centraliza todas as definições de hardware e software do projeto,
 * permitindo ajuste sem modificação dos módulos individuais.
 *
 * Plataforma alvo: Arduino Mega 2560
 */

#pragma once

#include <Arduino.h>
#include <stdint.h>

// ============================================================
//  PLATAFORMA E TEMPORIZAÇÃO
// ============================================================

/** Período do loop principal de controle em milissegundos (100 Hz). */
#define LOOP_PERIOD_MS          10U

/**
 * Timeout do watchdog para comunicação com o Raspberry Pi.
 * Se nenhuma mensagem for recebida neste intervalo, o robô entra
 * em modo seguro (parada controlada).
 */
#define WATCHDOG_TIMEOUT_MS     1500U

// ============================================================
//  PINAGEM — MOTORES DC (Driver L298N)
// ============================================================
/**
 * Configuração de ponte H dupla para acionamento diferencial.
 * PWM nos pinos de enable; DIR1/DIR2 controlam sentido de rotação.
 * Lógica: DIR1=HIGH, DIR2=LOW → frente | DIR1=LOW, DIR2=HIGH → ré
 */
#define MOTOR_LEFT_PWM          5     ///< PWM motor esquerdo (Enable A)
#define MOTOR_LEFT_DIR1         22    ///< Direção A1 motor esquerdo
#define MOTOR_LEFT_DIR2         23    ///< Direção A2 motor esquerdo

#define MOTOR_RIGHT_PWM         6     ///< PWM motor direito (Enable B)
#define MOTOR_RIGHT_DIR1        24    ///< Direção B1 motor direito
#define MOTOR_RIGHT_DIR2        25    ///< Direção B2 motor direito

/** Velocidade base de cruzeiro (0–255, referência PWM 8-bit). */
#define MOTOR_BASE_SPEED        140
/** Velocidade máxima absoluta permitida. */
#define MOTOR_MAX_SPEED         255
/** Zona morta mínima do driver (abaixo disso o motor não gira). */
#define MOTOR_DEADBAND          30

// ============================================================
//  PINAGEM — SENSORES DE LINHA (Array IR Analógico/Digital)
// ============================================================
/** Quantidade de sensores no array de linha. */
#define LINE_SENSOR_COUNT       8

/**
 * Mapeamento físico dos pinos analógicos para cada sensor.
 * Sensor 0 = extremo esquerdo; Sensor 7 = extremo direito.
 */
static const uint8_t LINE_SENSOR_PINS[LINE_SENSOR_COUNT] = {
    A0, A1, A2, A3, A4, A5, A6, A7
};

/**
 * Limiar de binarização para detecção da linha.
 * Valor de 0–1023 (ADC 10-bit). Abaixo = LINHA detectada
 * (sensor sobre superfície escura reflete menos IR).
 * Ajustar conforme calibração do ambiente.
 */
#define LINE_THRESHOLD          512

/**
 * Timeout em ms sem detecção de linha antes de acionar
 * comportamento de recuperação (ex: rotação de busca).
 */
#define LINE_LOST_TIMEOUT_MS    800U

// ============================================================
//  PINAGEM — SERVOMOTORES (Garra)
// ============================================================
/** Servo responsável pelo fechamento/abertura da garra. */
#define SERVO_GRIP_PIN          9

/** Servo responsável pela elevação/abaixamento da garra. */
#define SERVO_LIFT_PIN          10

// Posições angulares em graus
#define SERVO_GRIP_OPEN_DEG     90    ///< Garra aberta
#define SERVO_GRIP_CLOSE_DEG    10    ///< Garra fechada (ajustar por calibração)
#define SERVO_LIFT_UP_DEG       90    ///< Braço elevado
#define SERVO_LIFT_DOWN_DEG     15    ///< Braço abaixado para captura

/** Velocidade de movimento do servo (ms entre incrementos de 1°). */
#define SERVO_SWEEP_DELAY_MS    15U

// ============================================================
//  PARÂMETROS DO CONTROLADOR PID
// ============================================================
/**
 * Controlador PID discreto para seguimento de linha.
 *
 * A saída do PID representa a diferença de velocidade entre
 * motor esquerdo e direito (steering correction):
 *   v_esq = BASE_SPEED - output
 *   v_dir = BASE_SPEED + output
 *
 * Kp, Ki, Kd devem ser ajustados experimentalmente (Ziegler-Nichols
 * ou método heurístico em malha fechada).
 */
#define PID_KP                  32.0f   ///< Ganho proporcional
#define PID_KI                   0.4f   ///< Ganho integral
#define PID_KD                  22.0f   ///< Ganho derivativo

/** Período de amostragem do PID — deve coincidir com LOOP_PERIOD_MS. */
#define PID_SAMPLE_TIME_MS      10U

/** Saturação simétrica da saída do PID. */
#define PID_OUTPUT_MAX          255.0f
#define PID_OUTPUT_MIN         -255.0f

/**
 * Limite anti-windup do termo integral.
 * Evita integrator windup em situações de saturação prolongada.
 */
#define PID_INTEGRAL_MAX        80.0f

/**
 * Coeficiente do filtro passa-baixa no termo derivativo.
 * α ∈ (0,1]: α=1 → derivada pura; α<1 → filtrado.
 * Recomendado: 0.1 – 0.3 para reduzir sensibilidade a ruído.
 */
#define PID_DERIVATIVE_ALPHA    0.2f

// ============================================================
//  PROTOCOLO DE COMUNICAÇÃO SERIAL (Arduino ↔ Raspberry Pi)
// ============================================================
/**
 * UART1 do Arduino Mega (pinos 18 TX1, 19 RX1) dedicada ao RPi.
 * UART0 (USB) reservada para debug.
 */
#define SERIAL_RPI              Serial1
#define SERIAL_DEBUG            Serial

#define BAUD_RATE_RPI           115200UL
#define BAUD_RATE_DEBUG         115200UL

/** Delimitadores do frame de mensagem: <TIPO,P1,P2,...,CRC8> */
#define MSG_START_CHAR          '<'
#define MSG_END_CHAR            '>'
#define MSG_DELIMITER           ','
#define MSG_MAX_LENGTH          64U
#define MSG_TIMEOUT_MS          150U

// ============================================================
//  HUSKYLENS (I2C)
// ============================================================
/**
 * Arduino Mega: I2C nos pinos SDA=20, SCL=21.
 * Endereço padrão HuskyLens: 0x32.
 */
#define HUSKYLENS_I2C_ADDR      0x32
#define HUSKYLENS_POLL_MS       50U   ///< Intervalo de polling da câmera

// ============================================================
//  ESTADOS DA MÁQUINA DE ESTADOS (Arduino-side)
// ============================================================
typedef enum {
    STATE_IDLE        = 0,  ///< Aguardando comando inicial
    STATE_FOLLOW_LINE = 1,  ///< Seguindo linha com PID
    STATE_OBSTACLE    = 2,  ///< Parado, aguardando instrução do RPi
    STATE_GRIP_OPEN   = 3,  ///< Executando abertura de garra
    STATE_GRIP_CLOSE  = 4,  ///< Executando fechamento de garra
    STATE_GRIP_LIFT   = 5,  ///< Elevando braço
    STATE_GRIP_DROP   = 6,  ///< Abaixando braço
    STATE_EMERGENCY   = 7,  ///< Parada de emergência
    STATE_COUNT       = 8
} RobotState_t;

// ============================================================
//  TIPOS DE MENSAGEM DO PROTOCOLO SERIAL
// ============================================================
typedef enum {
    // Recebidos do Raspberry Pi
    CMD_MOVE         = 0,  ///< <MOVE,DIR,SPEED,CRC>
    CMD_STOP         = 1,  ///< <STOP,CRC>
    CMD_GRIP         = 2,  ///< <GRIP,ACTION,CRC>  ACTION: OPEN|CLOSE
    CMD_LIFT         = 3,  ///< <LIFT,ACTION,CRC>  ACTION: UP|DOWN
    CMD_SET_STATE    = 4,  ///< <STATE,NEW_STATE,CRC>
    CMD_PING         = 5,  ///< <PING,CRC>

    // Enviados ao Raspberry Pi
    MSG_SENSOR_DATA  = 10, ///< <SENSOR,LINE_POS,LINE_VALID,VICTIM_X,VICTIM_Y,CRC>
    MSG_STATE_CHANGE = 11, ///< <STATE_CHG,NEW_STATE,CRC>
    MSG_ACK          = 12, ///< <ACK,CMD_ID,CRC>
    MSG_PONG         = 13, ///< <PONG,CRC>
    MSG_ERROR        = 14  ///< <ERROR,CODE,CRC>
} MessageType_t;

// ============================================================
//  CÓDIGOS DE ERRO
// ============================================================
typedef enum {
    ERR_NONE            = 0,
    ERR_SERIAL_TIMEOUT  = 1,
    ERR_CRC_MISMATCH    = 2,
    ERR_UNKNOWN_CMD     = 3,
    ERR_LINE_LOST       = 4,
    ERR_HUSKYLENS_FAIL  = 5,
    ERR_WATCHDOG        = 6
} ErrorCode_t;
