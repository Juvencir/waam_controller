/**
 * @file encoder.h
 * @brief Driver de baixo nível para leitura de encoder em modo quadratura via Timer Hardware
 * (32-bit).
 */

#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

#include "stm32f4xx_hal.h"

/**
 * @brief Estrutura de controle do driver do encoder.
 */
typedef struct {
    TIM_HandleTypeDef* htim; /**< Handle do Timer STM32 configurado no CubeMX em Quadrature Mode */
    uint32_t           last_cnt; /**< Leitura do registrador CNT da amostragem anterior */
} Encoder_t;

/**
 * @brief Inicializa a estrutura do encoder e ativa o periférico em modo Quadratura 4x.
 * @param encoder Ponteiro para a estrutura de controle `Encoder_t`.
 * @param htim Handle do Timer do STM32 de 32 bits (ex: &htim2 ou &htim5).
 */
void Encoder_Init(Encoder_t* encoder, TIM_HandleTypeDef* htim);

/**
 * @brief Zera o contador físico do encoder e sincroniza a referência interna.
 * @param encoder Ponteiro para a estrutura do encoder.
 */
void Encoder_ResetZero(Encoder_t* encoder);

/**
 * @brief Lê o registrador de hardware, calcula o delta de pulsos com sinal e atualiza a referência.
 * @param encoder Ponteiro para a estrutura do encoder.
 * @return int32_t Variação de pulsos com sinal no último ciclo (Positivo = Avanço, Negativo =
 * Recuo).
 */
int32_t Encoder_UpdateDelta(Encoder_t* encoder);

#endif  // ENCODER_H