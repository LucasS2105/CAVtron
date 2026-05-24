/**
 * @file huskylens.h
 * @brief Interface com a câmera HuskyLens via protocolo I2C
 *
 * A HuskyLens é uma câmera de visão computacional embarcada equipada
 * com FPGA e processador dedicado, capaz de executar localmente
 * algoritmos de reconhecimento de face, objetos, cor, tags e linhas,
 * entregando resultados estruturados via I2C ou UART.
 *
 * Este módulo utiliza a biblioteca oficial HUSKYLENS.h para comunicação
 * I2C e abstrai o acesso aos resultados de detecção de objetos/vítimas.
 *
 * Protocolo de resultado (HuskyLens Object):
 *  - xCenter, yCenter: coordenadas do centroide na imagem (0–320, 0–240)
 *  - width, height: dimensões da bounding box
 *  - ID: ID do objeto aprendido
 *
 * A posição normalizada é calculada como:
 *   normX = (xCenter - 160) / 160.0  ∈ [-1, +1]
 *   normY = (yCenter - 120) / 120.0  ∈ [-1, +1]
 *
 * Requer biblioteca: HUSKYLENS by DFRobot (Arduino Library Manager)
 */

#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "HUSKYLENS.h"       // Biblioteca oficial DFRobot
#include "../config.h"

/**
 * @struct VictimData_t
 * @brief Dados de uma vítima detectada pela HuskyLens
 */
typedef struct {
    bool    detected;     ///< true se vítima visível no frame atual
    int16_t xCenter;      ///< Coordenada X do centroide [0–320]
    int16_t yCenter;      ///< Coordenada Y do centroide [0–240]
    int16_t width;        ///< Largura da bounding box em pixels
    int16_t height;       ///< Altura da bounding box em pixels
    int16_t objectID;     ///< ID do objeto treinado (1=vítima padrão)
    float   normalizedX;  ///< Posição X normalizada [-1.0, +1.0]
    float   normalizedY;  ///< Posição Y normalizada [-1.0, +1.0]
    float   area;         ///< Área relativa da bbox (proxy de distância)
} VictimData_t;

/**
 * @class HuskyLensInterface
 * @brief Wrapper de alto nível para a HuskyLens em modo seguimento de objetos
 */
class HuskyLensInterface {
public:
    /**
     * @brief Inicializa a comunicação I2C com a HuskyLens
     *
     * Tenta estabelecer conexão e configurar o algoritmo de
     * reconhecimento de objetos. Retorna false se a câmera
     * não for encontrada no barramento I2C.
     *
     * @return true  Inicialização bem-sucedida
     * @return false Câmera não encontrada ou falha de comunicação
     */
    bool begin();

    /**
     * @brief Atualiza os dados de detecção (polling da câmera)
     *
     * Deve ser chamado periodicamente (a cada HUSKYLENS_POLL_MS ms).
     * Atualiza o estado interno com a detecção mais recente.
     */
    void update();

    /**
     * @brief Retorna os dados da última detecção
     * @return Estrutura VictimData_t com todos os campos atualizados
     */
    const VictimData_t& getData() const { return _data; }

    /**
     * @brief Indica se vítima está detectada no frame atual
     */
    bool isVictimDetected() const { return _data.detected; }

    /**
     * @brief Indica se a câmera está operacional
     */
    bool isConnected() const { return _connected; }

    /**
     * @brief Envia requisição de "aprender" para a câmera (para treinamento)
     * @param id ID do objeto a aprender (1–255)
     */
    void learnObject(uint8_t id);

    /**
     * @brief Retorna millis() do último frame recebido com sucesso
     */
    uint32_t getLastFrameTimestamp() const { return _lastFrameMs; }

private:
    HUSKYLENS _huskyLens;       ///< Instância da biblioteca oficial
    VictimData_t _data;         ///< Dados atuais de detecção
    bool _connected = false;    ///< Estado da conexão
    uint32_t _lastPollMs  = 0;  ///< millis() do último polling
    uint32_t _lastFrameMs = 0;  ///< millis() do último frame válido

    /**
     * @brief Processa um resultado HUSKYLENSResult e preenche _data
     * @param result Objeto de resultado da biblioteca
     */
    void _parseResult(const HUSKYLENSResult& result);

    /**
     * @brief Reseta os dados de detecção para estado "não detectado"
     */
    void _clearData();
};
