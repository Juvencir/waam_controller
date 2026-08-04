#include "hbridge.h"

void HBridge_Init(HBridge_t* hb, TIM_HandleTypeDef* htim, uint32_t channel, GPIO_TypeDef* dir_port,
                  uint16_t dir_pin) {
    hb->htim       = htim;
    hb->channel    = channel;
    hb->dir_port   = dir_port;
    hb->dir_pin    = dir_pin;
    hb->duty_cycle = 0.0f;

    // Estado inicial seguro: Direção em LOW e PWM em 0%
    HAL_GPIO_WritePin(hb->dir_port, hb->dir_pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(hb->htim, hb->channel, 0);

    // Inicia a geração de PWM no periférico
    HAL_TIM_PWM_Start(hb->htim, hb->channel);
}

void HBridge_SetOutput(HBridge_t* hb, float speed) {
    // 1. Saturação do sinal entre -1.0f e 1.0f
    if (speed > 1.0f) speed = 1.0f;
    if (speed < -1.0f) speed = -1.0f;

    hb->duty_cycle = speed;

    // 2. Comuta a direção física
    if (speed >= 0.0f) {
        HAL_GPIO_WritePin(hb->dir_port, hb->dir_pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(hb->dir_port, hb->dir_pin, GPIO_PIN_RESET);
        speed = -speed;  // Converte para magnitude positiva (0.0f a 1.0f)
    }

    // 3. Aplica diretamente a magnitude proporcional ao ARR
    uint32_t arr         = __HAL_TIM_GET_AUTORELOAD(hb->htim);
    uint32_t compare_val = (uint32_t)(speed * (float)arr);

    // 4. Atualiza registrador de comparação
    __HAL_TIM_SET_COMPARE(hb->htim, hb->channel, compare_val);
}