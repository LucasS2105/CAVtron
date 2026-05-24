/**
 * @file filters.h
 * @brief Filtros digitais utilitários para sinais de sensores
 *
 * Implementa filtros IIR e FIR de uso geral para suavização
 * de sinais ruidosos provenientes de sensores analógicos e digitais.
 *
 * Filtros disponíveis:
 *
 * 1. EMA — Exponential Moving Average (IIR de 1ª ordem):
 *    y(k) = α*x(k) + (1-α)*y(k-1)
 *    Simples, baixo custo computacional, adequado para MCUs.
 *    α ∈ (0,1]: α=1 → sem filtro; α→0 → maior suavização (mais lag)
 *
 * 2. SMA — Simple Moving Average (FIR):
 *    y(k) = (1/N) * Σ x(k-i)  para i=0..N-1
 *    Resposta linear em fase, mas requer buffer circular.
 */

#pragma once

#include <Arduino.h>

// ============================================================
//  FILTRO EMA (IIR de 1ª ordem)
// ============================================================

/**
 * @class EMAFilter
 * @brief Filtro de média exponencial móvel (Exponential Moving Average)
 *
 * Equivalente a um filtro passa-baixa de 1ª ordem em tempo discreto.
 * Frequência de corte equivalente:
 *   fc = -ln(1-α) / (2π * Ts)   [Hz]
 */
class EMAFilter {
public:
    /**
     * @brief Construtor
     * @param alpha Coeficiente de suavização ∈ (0,1]
     *              Sugestões: 0.1=alta filtragem, 0.5=média, 0.9=mínima
     */
    explicit EMAFilter(float alpha = 0.2f)
        : _alpha(alpha), _output(0.0f), _initialized(false) {}

    /**
     * @brief Processa uma nova amostra
     * @param input Valor bruto da amostra
     * @return Valor filtrado
     */
    float update(float input) {
        if (!_initialized) {
            _output = input;
            _initialized = true;
        } else {
            _output = _alpha * input + (1.0f - _alpha) * _output;
        }
        return _output;
    }

    /** @brief Retorna a última saída filtrada */
    float get() const { return _output; }

    /** @brief Reinicia o filtro */
    void reset(float initialValue = 0.0f) {
        _output      = initialValue;
        _initialized = (initialValue != 0.0f);
    }

    /** @brief Altera o coeficiente alpha em tempo de execução */
    void setAlpha(float alpha) { _alpha = alpha; }

private:
    float _alpha;
    float _output;
    bool  _initialized;
};

// ============================================================
//  FILTRO SMA (FIR — Média Simples Móvel)
// ============================================================

/**
 * @tparam N Tamanho da janela (número de amostras)
 *
 * @class SMAFilter
 * @brief Filtro de média simples móvel com buffer circular
 *
 * Implementado com buffer circular de tamanho fixo N (template),
 * evitando alocação dinâmica. Adequado para N pequenos (≤ 32).
 */
template<uint8_t N>
class SMAFilter {
    static_assert(N > 0 && N <= 64, "N deve ser entre 1 e 64");

public:
    SMAFilter() : _head(0), _count(0), _sum(0.0f) {
        memset(_buffer, 0, sizeof(_buffer));
    }

    /**
     * @brief Insere nova amostra e retorna a média da janela atual
     * @param input Nova amostra
     * @return Média das últimas N amostras (ou menos, se buffer não cheio)
     */
    float update(float input) {
        // Remove a amostra mais antiga da soma (quando buffer está cheio)
        if (_count == N) {
            _sum -= _buffer[_head];
        } else {
            _count++;
        }

        // Insere nova amostra
        _buffer[_head] = input;
        _sum += input;

        // Avança o ponteiro circular
        _head = (_head + 1) % N;

        return _sum / (float)_count;
    }

    /** @brief Retorna a última média calculada */
    float get() const {
        return (_count > 0) ? (_sum / (float)_count) : 0.0f;
    }

    /** @brief Reinicia o buffer */
    void reset() {
        memset(_buffer, 0, sizeof(_buffer));
        _head  = 0;
        _count = 0;
        _sum   = 0.0f;
    }

    /** @brief Retorna quantas amostras estão no buffer */
    uint8_t getCount() const { return _count; }

private:
    float   _buffer[N];
    uint8_t _head;
    uint8_t _count;
    float   _sum;
};

// ============================================================
//  DETECTOR DE BORDA (Edge Detector)
// ============================================================

/**
 * @class EdgeDetector
 * @brief Detecta transições de sinal digital (rising/falling edge)
 *
 * Útil para detectar mudanças de estado em sensores booleanos
 * (ex: linha detectada → linha perdida) sem polling contínuo.
 */
class EdgeDetector {
public:
    EdgeDetector() : _prevState(false), _risingEdge(false), _fallingEdge(false) {}

    /**
     * @brief Atualiza o estado e detecta bordas
     * @param current Estado atual do sinal
     */
    void update(bool current) {
        _risingEdge  = (current && !_prevState);
        _fallingEdge = (!current && _prevState);
        _prevState   = current;
    }

    bool isRisingEdge()  const { return _risingEdge;  }
    bool isFallingEdge() const { return _fallingEdge; }
    bool getState()      const { return _prevState;   }

private:
    bool _prevState;
    bool _risingEdge;
    bool _fallingEdge;
};
