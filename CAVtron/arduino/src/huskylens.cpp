/**
 * @file huskylens.cpp
 * @brief Implementação da interface com HuskyLens via I2C
 */

#include "huskylens.h"

// ============================================================
//  INICIALIZAÇÃO
// ============================================================

bool HuskyLensInterface::begin() {
    Wire.begin();
    _clearData();

    // Tenta conectar com retries (até 3 tentativas)
    for (uint8_t attempt = 0; attempt < 3; attempt++) {
        if (_huskyLens.begin(Wire)) {
            _connected = true;
            break;
        }
        delay(200);
    }

    if (!_connected) {
        SERIAL_DEBUG.println(F("[HuskyLens] ERRO: Câmera não encontrada no I2C!"));
        return false;
    }

    /**
     * Configura o algoritmo de reconhecimento de objetos.
     * ALGORITHM_OBJECT_RECOGNITION reconhece objetos previamente treinados.
     * Alternativas: ALGORITHM_FACE_RECOGNITION, ALGORITHM_COLOR_RECOGNITION
     */
    _huskyLens.writeAlgorithm(ALGORITHM_OBJECT_RECOGNITION);

    SERIAL_DEBUG.println(F("[HuskyLens] Inicializada com sucesso."));
    _lastFrameMs = millis();
    return true;
}

// ============================================================
//  ATUALIZAÇÃO (POLLING)
// ============================================================

void HuskyLensInterface::update() {
    if (!_connected) return;

    const uint32_t now = millis();
    if ((now - _lastPollMs) < HUSKYLENS_POLL_MS) return;
    _lastPollMs = now;

    /**
     * request() envia um frame request à câmera e retorna true se
     * dados foram recebidos. Internamente usa Wire.requestFrom().
     */
    if (!_huskyLens.request()) {
        // Falha de comunicação — não atualiza dados (mantém último estado)
        return;
    }

    _lastFrameMs = now;

    // Verifica se há resultados no frame atual
    if (_huskyLens.isLearned() && _huskyLens.available()) {
        /**
         * Pega o primeiro resultado disponível.
         * Para múltiplas detecções, iterar com while(_huskyLens.available())
         * e selecionar o de maior área ou ID específico.
         */
        HUSKYLENSResult result = _huskyLens.read();
        _parseResult(result);
    } else {
        // Nenhum objeto reconhecido neste frame
        _clearData();
    }
}

// ============================================================
//  APRENDIZADO DE OBJETO
// ============================================================

void HuskyLensInterface::learnObject(uint8_t id) {
    if (!_connected) return;
    _huskyLens.writeLearn(id);
    SERIAL_DEBUG.print(F("[HuskyLens] Aprendendo objeto ID: "));
    SERIAL_DEBUG.println(id);
}

// ============================================================
//  MÉTODOS PRIVADOS
// ============================================================

void HuskyLensInterface::_parseResult(const HUSKYLENSResult& result) {
    _data.detected  = true;
    _data.xCenter   = result.xCenter;
    _data.yCenter   = result.yCenter;
    _data.width     = result.width;
    _data.height    = result.height;
    _data.objectID  = result.ID;

    /**
     * Normalização da posição para coordenadas de câmera:
     * Resolução HuskyLens: 320 x 240 pixels
     * Centro da imagem: (160, 120)
     * Resultado: [-1.0, +1.0] em ambos os eixos
     */
    _data.normalizedX = (result.xCenter - 160) / 160.0f;
    _data.normalizedY = (result.yCenter - 120) / 120.0f;

    /**
     * Área relativa como proxy de distância:
     * Área máxima = 320*240 = 76800 px²
     * área relativa ∈ (0, 1]: maior área → objeto mais próximo
     */
    _data.area = ((float)result.width * result.height) / 76800.0f;
}

void HuskyLensInterface::_clearData() {
    _data.detected    = false;
    _data.xCenter     = 0;
    _data.yCenter     = 0;
    _data.width       = 0;
    _data.height      = 0;
    _data.objectID    = 0;
    _data.normalizedX = 0.0f;
    _data.normalizedY = 0.0f;
    _data.area        = 0.0f;
}
