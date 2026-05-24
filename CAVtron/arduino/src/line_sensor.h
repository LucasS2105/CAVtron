/**
 * @file line_sensor.h
 * @brief Leitura e processamento do array de sensores de linha IR
 *
 * Implementa a leitura de um array linear de sensores infravermelhos
 * e calcula a posição relativa da linha através da técnica de
 * média ponderada (centroide):
 *
 *   position = Σ(peso_i * leitura_i) / Σ(leitura_i)
 *
 * O resultado é normalizado no intervalo [-1.0, +1.0], onde:
 *   -1.0 → linha no extremo esquerdo
 *    0.0 → linha centralizada
 *   +1.0 → linha no extremo direito
 *
 * Este valor serve diretamente como variável de processo (PV)
 * do controlador PID, com setpoint = 0.0 (linha centrada).
 *
 * Recursos adicionais:
 *  - Detecção de linha perdida com timestamp
 *  - Histórico da última posição válida (para manobra de recuperação)
 *  - Binarização por limiar configurável
 *  - Detecção de cruzamento (todos os sensores ativos)
 */

#pragma once

#include <Arduino.h>
#include "../config.h"

/**
 * @struct LineSensorData_t
 * @brief Dados processados do array de sensores de linha
 */
typedef struct {
    float    position;           ///< Posição da linha [-1.0, +1.0]
    bool     lineDetected;       ///< true se linha está sendo detectada
    bool     crossingDetected;   ///< true se cruzamento foi detectado
    bool     lineLost;           ///< true se linha perdida por timeout
    uint32_t lostTimestamp;      ///< millis() quando linha foi perdida
    uint8_t  rawBinary;          ///< Bitmask dos sensores ativos
    uint16_t rawAnalog[LINE_SENSOR_COUNT]; ///< Valores ADC brutos
} LineSensorData_t;

/**
 * @class LineSensor
 * @brief Gerenciador do array de sensores infravermelhos de linha
 */
class LineSensor {
public:
    /**
     * @brief Inicializa os pinos dos sensores como entrada
     */
    void begin();

    /**
     * @brief Realiza leitura completa e processamento do array
     *
     * Deve ser chamado a cada ciclo de controle (10ms).
     * Atualiza o estado interno e os dados públicos acessíveis
     * por getData().
     */
    void update();

    /**
     * @brief Retorna os dados processados mais recentes
     * @return Estrutura LineSensorData_t com todos os campos atualizados
     */
    const LineSensorData_t& getData() const { return _data; }

    /**
     * @brief Posição da linha (atalho para o campo mais usado)
     * @return float em [-1.0, +1.0]
     */
    float getPosition() const { return _data.position; }

    /**
     * @brief Indica se a linha está sendo detectada neste ciclo
     */
    bool isLineDetected() const { return _data.lineDetected; }

    /**
     * @brief Indica se a linha foi perdida por tempo superior ao timeout
     */
    bool isLineLost() const { return _data.lineLost; }

    /**
     * @brief Indica se um cruzamento foi detectado (todos os sensores ativos)
     */
    bool isCrossing() const { return _data.crossingDetected; }

    /**
     * @brief Calibra o limiar de detecção automaticamente
     *
     * Move o robô sobre a linha e chão, coleta amostras e
     * define o threshold como a média entre os extremos.
     * (Método por polling — bloqueia por ~2s)
     */
    void calibrate();

    /**
     * @brief Define manualmente o limiar de binarização
     * @param threshold Valor ADC [0–1023]
     */
    void setThreshold(uint16_t threshold);

private:
    LineSensorData_t _data;
    uint16_t         _threshold = LINE_THRESHOLD;
    float            _lastValidPosition = 0.0f;
    bool             _wasDetected = false;

    /**
     * @brief Lê valores analógicos brutos de todos os sensores
     */
    void _readAnalog();

    /**
     * @brief Binariza as leituras e calcula posição por centroide
     *
     * Centroide com pesos proporcionais à distância do centro:
     * pesos = {-3.5, -2.5, -1.5, -0.5, +0.5, +1.5, +2.5, +3.5}
     * para 8 sensores
     */
    void _processPosition();

    /**
     * @brief Atualiza flags de estado (line lost, crossing, etc.)
     */
    void _updateFlags();
};
