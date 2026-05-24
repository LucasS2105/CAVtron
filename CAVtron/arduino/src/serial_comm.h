/**
 * @file serial_comm.h
 * @brief Protocolo de comunicação serial Arduino ↔ Raspberry Pi
 *
 * Implementa um protocolo de mensagens enquadradas (framed protocol)
 * com detecção de erros via CRC-8 (polinômio 0x07, estilo Dallas/Maxim).
 *
 * Formato do frame:
 *   <TIPO,PARAM1,PARAM2,...,PARAMN,CRC8>\n
 *
 * Onde:
 *   < >   = delimitadores de frame
 *   TIPO  = string identificadora do comando (ex: "MOVE", "GRIP")
 *   CRC8  = checksum hexadecimal de 2 dígitos sobre o conteúdo interno
 *           (tudo entre '<' e ',' antes do CRC)
 *   \n    = terminador de linha (LF)
 *
 * Exemplos:
 *   Raspberry → Arduino:  <MOVE,FWD,150,A3>
 *   Raspberry → Arduino:  <GRIP,OPEN,5F>
 *   Arduino → Raspberry:  <SENSOR,-0.25,1,0.45,0.12,B7>
 *   Arduino → Raspberry:  <STATE_CHG,2,C1>
 *
 * CRC-8 calculado sobre os bytes do conteúdo sem os delimitadores
 * e sem o próprio CRC (i.e., "MOVE,FWD,150").
 *
 * Referência CRC-8:
 *   Williams, R.N. (1993). A Painless Guide to CRC Error Detection Algorithms.
 */

#pragma once

#include <Arduino.h>
#include "../config.h"

/**
 * @struct ParsedMessage_t
 * @brief Mensagem decodificada pelo parser serial
 */
typedef struct {
    char     type[16];                     ///< Tipo/comando da mensagem
    char     params[8][16];               ///< Parâmetros extraídos (até 8)
    uint8_t  paramCount;                  ///< Quantidade de parâmetros
    uint8_t  receivedCRC;                 ///< CRC recebido na mensagem
    uint8_t  calculatedCRC;               ///< CRC calculado localmente
    bool     valid;                       ///< true se CRC confere
} ParsedMessage_t;

/**
 * @class SerialComm
 * @brief Gerenciador do protocolo de comunicação serial com o Raspberry Pi
 */
class SerialComm {
public:
    /**
     * @brief Inicializa as portas seriais e buffers internos
     */
    void begin();

    /**
     * @brief Processa bytes disponíveis na serial — deve ser chamado no loop
     *
     * Implementa uma máquina de estados para recepção incremental de frames.
     * Não bloqueia — processa apenas os bytes disponíveis no buffer nativo.
     *
     * @return true se uma mensagem completa foi recebida e parseada
     */
    bool update();

    /**
     * @brief Retorna a última mensagem recebida e válida
     * @return Referência à estrutura ParsedMessage_t interna
     */
    const ParsedMessage_t& getLastMessage() const { return _lastMsg; }

    /**
     * @brief Envia dados dos sensores ao Raspberry Pi
     *
     * Formato: <SENSOR,LINE_POS,LINE_VALID,VICTIM_DET,VIC_X,VIC_Y,CRC>
     *
     * @param linePos       Posição da linha [-1.0, +1.0]
     * @param lineDetected  true se linha detectada
     * @param victimDetected true se vítima detectada
     * @param victimX       Posição X normalizada da vítima
     * @param victimY       Posição Y normalizada da vítima
     */
    void sendSensorData(float linePos,
                        bool lineDetected,
                        bool victimDetected,
                        float victimX,
                        float victimY);

    /**
     * @brief Envia mudança de estado ao Raspberry Pi
     * @param newState Novo estado da FSM
     */
    void sendStateChange(RobotState_t newState);

    /**
     * @brief Envia ACK de confirmação de comando
     * @param cmdType String do tipo de comando confirmado
     */
    void sendAck(const char* cmdType);

    /**
     * @brief Envia resposta de PONG ao PING do Raspberry Pi
     */
    void sendPong();

    /**
     * @brief Envia notificação de erro
     * @param errorCode Código de erro definido em ErrorCode_t
     */
    void sendError(ErrorCode_t errorCode);

    /**
     * @brief Verifica se o watchdog expirou (sem mensagem por WATCHDOG_TIMEOUT_MS)
     * @return true se timeout de comunicação atingido
     */
    bool isWatchdogExpired() const;

    /**
     * @brief Reseta o temporizador do watchdog
     */
    void resetWatchdog();

    /**
     * @brief Retorna millis() do último frame válido recebido
     */
    uint32_t getLastRxTimestamp() const { return _lastRxTimestamp; }

private:
    // Estado do parser (FSM de recepção)
    typedef enum {
        PARSE_WAIT_START = 0,  ///< Aguardando delimitador '<'
        PARSE_READING    = 1,  ///< Lendo conteúdo do frame
        PARSE_COMPLETE   = 2   ///< Frame completo recebido
    } ParseState_t;

    ParseState_t  _parseState = PARSE_WAIT_START;
    char          _rxBuffer[MSG_MAX_LENGTH];
    uint8_t       _rxIndex   = 0;

    ParsedMessage_t _lastMsg;
    uint32_t        _lastRxTimestamp = 0;
    uint32_t        _lastMsgTimestamp = 0;

    /**
     * @brief Faz o parse do buffer recebido em _rxBuffer
     * @return true se parse bem-sucedido e CRC válido
     */
    bool _parseBuffer();

    /**
     * @brief Envia um frame formatado com CRC ao Raspberry Pi
     * @param content String do conteúdo sem delimitadores e sem CRC
     */
    void _sendFrame(const char* content);

    /**
     * @brief Calcula CRC-8 (polinômio 0x07) sobre uma string
     * @param data Ponteiro para os dados
     * @param len  Comprimento em bytes
     * @return Byte de CRC calculado
     */
    static uint8_t _crc8(const uint8_t* data, uint8_t len);

    /**
     * @brief Versão com string (wrapper de _crc8)
     */
    static uint8_t _crc8str(const char* str);
};
