/**
 * @file serial_comm.cpp
 * @brief Implementação do protocolo de comunicação serial com CRC-8
 */

#include "serial_comm.h"
#include <string.h>
#include <stdio.h>

// ============================================================
//  INICIALIZAÇÃO
// ============================================================

void SerialComm::begin() {
    SERIAL_RPI.begin(BAUD_RATE_RPI);
    SERIAL_DEBUG.begin(BAUD_RATE_DEBUG);

    _parseState       = PARSE_WAIT_START;
    _rxIndex          = 0;
    _lastRxTimestamp  = millis();
    _lastMsgTimestamp = millis();

    memset(&_lastMsg, 0, sizeof(_lastMsg));

    SERIAL_DEBUG.println(F("[SerialComm] Inicializada."));
}

// ============================================================
//  UPDATE — PARSER INCREMENTAL (FSM)
// ============================================================

bool SerialComm::update() {
    bool newMessageReady = false;

    while (SERIAL_RPI.available() > 0) {
        const char c = (char)SERIAL_RPI.read();

        switch (_parseState) {
            // ---- Aguarda delimitador de início '<' ----
            case PARSE_WAIT_START:
                if (c == MSG_START_CHAR) {
                    _rxIndex    = 0;
                    _parseState = PARSE_READING;
                }
                break;

            // ---- Lê conteúdo até '>' ----
            case PARSE_READING:
                if (c == MSG_END_CHAR) {
                    // Fim do frame: null-termina e processa
                    _rxBuffer[_rxIndex] = '\0';
                    _parseState = PARSE_WAIT_START;

                    if (_parseBuffer()) {
                        _lastRxTimestamp  = millis();
                        _lastMsgTimestamp = millis();
                        newMessageReady   = true;
                    }
                } else if (_rxIndex < (MSG_MAX_LENGTH - 1)) {
                    _rxBuffer[_rxIndex++] = c;
                } else {
                    // Buffer overflow: descarta frame corrompido
                    SERIAL_DEBUG.println(F("[SerialComm] WARN: Buffer overflow, frame descartado."));
                    _parseState = PARSE_WAIT_START;
                    _rxIndex    = 0;
                }
                break;

            default:
                _parseState = PARSE_WAIT_START;
                break;
        }
    }

    return newMessageReady;
}

// ============================================================
//  PARSER DO BUFFER
// ============================================================

bool SerialComm::_parseBuffer() {
    /**
     * Buffer contém: TIPO,PARAM1,...,PARAMN,CRCXX
     * Onde CRCXX é o CRC-8 em hexadecimal (2 dígitos uppercase)
     *
     * Estratégia:
     * 1. Encontrar o último ',' (separador antes do CRC)
     * 2. Extrair e converter o CRC hexadecimal
     * 3. Calcular CRC sobre o conteúdo antes do último ','
     * 4. Tokenizar os campos
     */
    memset(&_lastMsg, 0, sizeof(_lastMsg));

    // Encontra a última vírgula (antes do campo CRC)
    char* lastComma = strrchr(_rxBuffer, MSG_DELIMITER);
    if (!lastComma) {
        SERIAL_DEBUG.println(F("[SerialComm] ERRO: Formato de frame inválido."));
        return false;
    }

    // Extrai CRC recebido (hexadecimal)
    const char* crcStr = lastComma + 1;
    _lastMsg.receivedCRC = (uint8_t)strtol(crcStr, nullptr, 16);

    // Calcula CRC sobre o conteúdo antes do último ','
    const uint8_t contentLen = (uint8_t)(lastComma - _rxBuffer);
    _lastMsg.calculatedCRC   = _crc8((const uint8_t*)_rxBuffer, contentLen);

    // Verifica integridade
    if (_lastMsg.receivedCRC != _lastMsg.calculatedCRC) {
        SERIAL_DEBUG.print(F("[SerialComm] ERRO CRC: recebido=0x"));
        SERIAL_DEBUG.print(_lastMsg.receivedCRC, HEX);
        SERIAL_DEBUG.print(F(" calculado=0x"));
        SERIAL_DEBUG.println(_lastMsg.calculatedCRC, HEX);
        _lastMsg.valid = false;
        return false;
    }

    _lastMsg.valid = true;

    // Tokeniza o conteúdo (sem o campo CRC)
    char content[MSG_MAX_LENGTH];
    strncpy(content, _rxBuffer, contentLen);
    content[contentLen] = '\0';

    // Extrai tipo (primeiro token)
    char* token = strtok(content, ",");
    if (!token) return false;
    strncpy(_lastMsg.type, token, sizeof(_lastMsg.type) - 1);

    // Extrai parâmetros restantes
    _lastMsg.paramCount = 0;
    while ((token = strtok(nullptr, ",")) != nullptr
           && _lastMsg.paramCount < 8)
    {
        strncpy(_lastMsg.params[_lastMsg.paramCount],
                token,
                sizeof(_lastMsg.params[0]) - 1);
        _lastMsg.paramCount++;
    }

    return true;
}

// ============================================================
//  ENVIO DE MENSAGENS
// ============================================================

void SerialComm::sendSensorData(float linePos,
                                bool  lineDetected,
                                bool  victimDetected,
                                float victimX,
                                float victimY)
{
    char content[MSG_MAX_LENGTH];
    snprintf(content, sizeof(content),
             "SENSOR,%.3f,%d,%d,%.3f,%.3f",
             linePos,
             (int)lineDetected,
             (int)victimDetected,
             victimX,
             victimY);
    _sendFrame(content);
}

void SerialComm::sendStateChange(RobotState_t newState) {
    char content[32];
    snprintf(content, sizeof(content), "STATE_CHG,%d", (int)newState);
    _sendFrame(content);
}

void SerialComm::sendAck(const char* cmdType) {
    char content[32];
    snprintf(content, sizeof(content), "ACK,%s", cmdType);
    _sendFrame(content);
}

void SerialComm::sendPong() {
    _sendFrame("PONG");
}

void SerialComm::sendError(ErrorCode_t errorCode) {
    char content[32];
    snprintf(content, sizeof(content), "ERROR,%d", (int)errorCode);
    _sendFrame(content);
}

// ============================================================
//  WATCHDOG
// ============================================================

bool SerialComm::isWatchdogExpired() const {
    return (millis() - _lastMsgTimestamp) > WATCHDOG_TIMEOUT_MS;
}

void SerialComm::resetWatchdog() {
    _lastMsgTimestamp = millis();
}

// ============================================================
//  MÉTODOS PRIVADOS
// ============================================================

void SerialComm::_sendFrame(const char* content) {
    const uint8_t crc = _crc8str(content);

    // Formato: <CONTENT,CRCXX>\n
    SERIAL_RPI.print(MSG_START_CHAR);
    SERIAL_RPI.print(content);
    SERIAL_RPI.print(MSG_DELIMITER);

    // CRC em hexadecimal com padding de 2 dígitos
    if (crc < 0x10) SERIAL_RPI.print('0');
    SERIAL_RPI.print(crc, HEX);

    SERIAL_RPI.print(MSG_END_CHAR);
    SERIAL_RPI.print('\n');
}

uint8_t SerialComm::_crc8(const uint8_t* data, uint8_t len) {
    /**
     * CRC-8, polinômio gerador: x^8 + x^2 + x + 1 → 0x07
     * Padrão Dallas/Maxim — amplamente utilizado em sistemas embarcados.
     *
     * Algoritmo bit-a-bit (sem lookup table para economizar RAM):
     *   Para cada byte: XOR com registrador, depois 8 iterações de
     *   deslocamento com XOR condicional pelo polinômio.
     */
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

uint8_t SerialComm::_crc8str(const char* str) {
    return _crc8((const uint8_t*)str, (uint8_t)strlen(str));
}
