#include "encoder.h"

void Encoder_Init(Encoder_t* encoder, TIM_HandleTypeDef* htim) {
    // Vincula a instância do hardware
    encoder->htim     = htim;
    encoder->last_cnt = 0;

    // Garante zera do registrador do timer na inicialização
    __HAL_TIM_SET_COUNTER(encoder->htim, 0);

    // Inicializa as duas vias de quadratura (Canal A e Canal B) no hardware
    HAL_TIM_Encoder_Start(encoder->htim, TIM_CHANNEL_ALL);
}

int32_t Encoder_UpdateDelta(Encoder_t* encoder) {
    // Capture atômico do registrador CNT (32 bits sem sinal)
    uint32_t current_cnt = __HAL_TIM_GET_COUNTER(encoder->htim);

    // Subtração direta em uint32_t seguida de cast para int32_t:
    // O complemento de dois trata estouros de estouro/esvaziamento sem necessidade de 'if'.
    int32_t delta = (int32_t)(current_cnt - encoder->last_cnt);

    // Atualiza o estado da última leitura para o próximo ciclo
    encoder->last_cnt = current_cnt;

    return delta;
}