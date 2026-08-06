#include "Domain/axis_kinematics.h"

#include "FreeRTOS.h"
#include "task.h"

void AxisKinematics_Init(AxisKinematics_t* kin, Encoder_t* encoder_drv) {
    if (kin == NULL) return;

    kin->encoder_driver          = encoder_drv;
    kin->state.total_pulses      = 0;
    kin->state.last_delta_pulses = 0;
    kin->state.position_mm       = 0.0f;
    kin->state.velocity_mm_s     = 0.0f;
}

void AxisKinematics_UpdateISR(AxisKinematics_t* kin, float dt_s) {
    if (kin == NULL || kin->encoder_driver == NULL || dt_s <= 0.0f) return;

    // 1. Lê a variação física do contador do Timer (TIM2/TIM5) no último ms[cite: 4]
    int32_t delta                = Encoder_UpdateDelta(kin->encoder_driver);
    kin->state.last_delta_pulses = delta;

    // 2. Acumula os pulsos no acumulador de 64 bits (Aceita avanço/recuo)
    kin->state.total_pulses += delta;

    // 3. Executa a conversão para unidades de engenharia (mm e mm/s)[cite: 2, 4]
    kin->state.position_mm   = (float)kin->state.total_pulses * KIN_MM_PER_PULSE;
    kin->state.velocity_mm_s = ((float)delta * KIN_MM_PER_PULSE) / dt_s;
}

void AxisKinematics_ResetZero(AxisKinematics_t* kin) {
    if (kin == NULL) return;

    // Garante atomicidade: desabilita interrupções durante o zera-registro
    taskENTER_CRITICAL();
    if (kin->encoder_driver != NULL) {
        Encoder_ResetZero(kin->encoder_driver);
    }
    kin->state.total_pulses = 0;
    kin->state.last_delta_pulses = 0;
    kin->state.position_mm  = 0.0f;
    kin->state.velocity_mm_s = 0.0f;
    taskEXIT_CRITICAL();
}

void AxisKinematics_GetStateSnapshot(const AxisKinematics_t* kin, KinematicsState_t* out_state) {
    if (kin == NULL || out_state == NULL) return;

    // Entra em Seção Crítica do FreeRTOS para realizar a cópia atômica da struct[cite: 4]
    taskENTER_CRITICAL();
    *out_state = *(KinematicsState_t*)&kin->state;
    taskEXIT_CRITICAL();
}