/**
 * @file hbridge.h
 * @brief Driver de baixo nível para controle de Ponte H via PWM e pino de Direção (DIR/PWM).
 */

#ifndef HBRIDGE_H
#define HBRIDGE_H

#include <stm32f4xx_hal.h>

/**
 * @brief Estrutura de controle da Ponte H.
 */
typedef struct {
    TIM_HandleTypeDef* htim;        // Handle do Timer do PWM (ex: &htim3)
    uint32_t           channel;     // Canal do Timer (ex: TIM_CHANNEL_1)
    GPIO_TypeDef*      dir_port;    // Porta GPIO do pino de Direção (ex: GPIOB)
    uint16_t           dir_pin;     // Pino GPIO de Direção (ex: GPIO_PIN_0)
    float              duty_cycle;  // Esforço atual normalizado (-1.0f a 1.0f)
} HBridge_t;

/**
 * @brief Inicializa a Ponte H e ativa a saída PWM do Timer.
 * @param hb Ponteiro para a estrutura da Ponte H.
 * @param htim Handle do Timer configurado no CubeMX.
 * @param channel Canal do PWM.
 * @param dir_port Porta GPIO do pino de direção.
 * @param dir_pin Pino GPIO de direção.
 */
void HBridge_Init(HBridge_t* hb, TIM_HandleTypeDef* htim, uint32_t channel, GPIO_TypeDef* dir_port,
                  uint16_t dir_pin);

/**
 * @brief Define o esforço de saída da Ponte H.
 * @param hb Ponteiro para a estrutura da Ponte H.
 * @param speed Esforço de controle normalizado (-1.0f a 1.0f), 0.0f para parar.
 */
void HBridge_SetOutput(HBridge_t* hb, float speed);

#endif  // HBRIDGE_H