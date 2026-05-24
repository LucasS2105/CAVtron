/**
 * @file line_sensor.cpp
 * @brief Implementação do processamento do array de sensores de linha
 */

#include "line_sensor.h"

/**
 * Pesos do centroide para N=8 sensores.
 * Simetria em relação ao centro: sensor 0 = -3.5, sensor 7 = +3.5
 * Divididos por (N/2) = 4 na normalização → resultado em [-1, +1]
 */
static const float SENSOR_WEIGHTS[LINE_SENSOR_COUNT] = {
    -3.5f, -2.5f, -1.5f, -0.5f,
     0.5f,  1.5f,  2.5f,  3.5f
};

/** Normalizador: peso máximo = (N/2) - 0.5 = 3.5 para N=8 */
static const float WEIGHT_NORMALIZER = 3.5f;

// ============================================================
//  INICIALIZAÇÃO
// ============================================================

void LineSensor::begin() {
    for (uint8_t i = 0; i < LINE_SENSOR_COUNT; i++) {
        pinMode(LINE_SENSOR_PINS[i], INPUT);
    }

    // Inicializa estrutura de dados
    _data.position         = 0.0f;
    _data.lineDetected     = false;
    _data.crossingDetected = false;
    _data.lineLost         = false;
    _data.lostTimestamp    = 0;
    _data.rawBinary        = 0x00;
    _lastValidPosition     = 0.0f;
    _wasDetected           = false;
}

// ============================================================
//  ATUALIZAÇÃO DO ESTADO (CHAMADA A CADA CICLO)
// ============================================================

void LineSensor::update() {
    _readAnalog();
    _processPosition();
    _updateFlags();
}

// ============================================================
//  LEITURA ANALÓGICA
// ============================================================

void LineSensor::_readAnalog() {
    for (uint8_t i = 0; i < LINE_SENSOR_COUNT; i++) {
        _data.rawAnalog[i] = analogRead(LINE_SENSOR_PINS[i]);
    }
}

// ============================================================
//  PROCESSAMENTO DE POSIÇÃO (CENTROIDE PONDERADO)
// ============================================================

void LineSensor::_processPosition() {
    float    weightedSum = 0.0f;
    float    totalWeight = 0.0f;
    uint8_t  activeSensors = 0;
    uint8_t  binaryMask = 0x00;

    for (uint8_t i = 0; i < LINE_SENSOR_COUNT; i++) {
        /**
         * Binarização: sensor ativo (sobre a linha) quando
         * leitura ADC está ABAIXO do threshold.
         * Sensores IR refletem menos em superfícies escuras → valor menor.
         */
        const bool active = (_data.rawAnalog[i] < _threshold);

        if (active) {
            binaryMask |= (1 << i);
            activeSensors++;

            // Contribuição para o centroide
            weightedSum += SENSOR_WEIGHTS[i];
            totalWeight += 1.0f;
        }
    }

    _data.rawBinary = binaryMask;

    // Cruzamento: todos (ou quase todos) os sensores ativos
    _data.crossingDetected = (activeSensors >= (LINE_SENSOR_COUNT - 1));

    if (activeSensors == 0) {
        // Nenhum sensor ativo: linha não detectada
        _data.lineDetected = false;
        // Mantém a última posição válida com sinal amplificado
        // para indicar ao PID para onde recuperar
        _data.position = _lastValidPosition;
    } else {
        _data.lineDetected = true;
        // Cálculo do centroide normalizado
        _data.position = (weightedSum / totalWeight) / WEIGHT_NORMALIZER;
        _data.position = constrain(_data.position, -1.0f, 1.0f);
        _lastValidPosition = _data.position;
    }
}

// ============================================================
//  ATUALIZAÇÃO DE FLAGS DE ESTADO
// ============================================================

void LineSensor::_updateFlags() {
    const uint32_t now = millis();

    if (_data.lineDetected) {
        // Linha detectada: reseta flags de perda
        _data.lineLost      = false;
        _data.lostTimestamp = 0;
        _wasDetected        = true;
    } else {
        if (_wasDetected && _data.lostTimestamp == 0) {
            // Marca o timestamp da perda da linha
            _data.lostTimestamp = now;
        }

        // Verifica timeout de linha perdida
        if (_data.lostTimestamp > 0 &&
            (now - _data.lostTimestamp) >= LINE_LOST_TIMEOUT_MS) {
            _data.lineLost = true;
        }
    }
}

// ============================================================
//  CALIBRAÇÃO
// ============================================================

void LineSensor::calibrate() {
    uint16_t minVals[LINE_SENSOR_COUNT];
    uint16_t maxVals[LINE_SENSOR_COUNT];

    // Inicializa com extremos opostos
    for (uint8_t i = 0; i < LINE_SENSOR_COUNT; i++) {
        minVals[i] = 1023;
        maxVals[i] = 0;
    }

    // Coleta amostras por 2 segundos
    const uint32_t start = millis();
    while (millis() - start < 2000) {
        for (uint8_t i = 0; i < LINE_SENSOR_COUNT; i++) {
            const uint16_t val = analogRead(LINE_SENSOR_PINS[i]);
            if (val < minVals[i]) minVals[i] = val;
            if (val > maxVals[i]) maxVals[i] = val;
        }
        delay(5);
    }

    // Threshold = média entre o mínimo e máximo globais
    uint32_t globalMin = 0, globalMax = 0;
    for (uint8_t i = 0; i < LINE_SENSOR_COUNT; i++) {
        globalMin += minVals[i];
        globalMax += maxVals[i];
    }
    _threshold = (uint16_t)((globalMin + globalMax) / (2 * LINE_SENSOR_COUNT));

    SERIAL_DEBUG.print(F("[LineSensor] Threshold calibrado: "));
    SERIAL_DEBUG.println(_threshold);
}

void LineSensor::setThreshold(uint16_t threshold) {
    _threshold = threshold;
}
