/**
 * @file servo_drive.c
 * @brief Implementação do Servo Drive — controle unificado do motor como servo.
 */

#include "Control/servo_drive.h"

#include <stddef.h>

#include "Config/waam_params.h"

// ============================================================================
// INICIALIZAÇÃO
// ============================================================================

void Servo_Init(ServoDrive_t* servo, AxisKinematics_t* kin, HBridge_t* hb) {
    if (servo == NULL) return;

    // Vincula camadas inferiores
    servo->kinematics = kin;
    servo->hbridge    = hb;

    // Configurações padrão — vindas de Config/waam_params.h (100% ajustáveis)
    servo->pos_gain     = WAAM_SERVO_POS_GAIN;
    servo->max_velocity = WAAM_MAX_TRAVEL_SPEED_MM_S;
    servo->vel_ff       = WAAM_SERVO_VEL_FF;
    servo->duty_max     = WAAM_SERVO_DUTY_MAX;

    // Inicializa PID de velocidade (apenas termo P; sem I, sem D)
    PID_Init(&servo->vel_pid, WAAM_SERVO_VEL_KP, 0.0f, 0.0f, -1.0f, 1.0f);

    // Inicializa o algoritmo Direct interno (usado em MoveTo e no auto-hold)
    DirectAlg_Init(&servo->direct_alg, &servo->direct_ctx, servo->pos_gain, servo->max_velocity,
                   WAAM_SERVO_EPS_MM);

    // O algoritmo trapezoidal interno NÃO é inicializado aqui: seus parâmetros
    // (cruzeiro, rampas) são sempre fornecidos por parâmetro em Servo_MoveProfile,
    // que o (re)configura integralmente a cada chamada.

    // Estado inicial: IDLE
    servo->mode                = SERVO_MODE_IDLE;
    servo->velocity_sp_mm_s    = 0.0f;
    servo->active_alg          = NULL;
    servo->algorithm_target_mm = 0.0f;
    servo->alg_done            = false;
}

// ============================================================================
// CONFIGURAÇÃO
// ============================================================================

void Servo_SetPosGain(ServoDrive_t* servo, float gain) {
    if (servo == NULL) return;
    servo->pos_gain = gain;
    // Propaga para o algoritmo Direct interno
    servo->direct_ctx.kp_pos = gain;
}

void Servo_SetMaxVelocity(ServoDrive_t* servo, float max_vel) {
    if (servo == NULL) return;
    servo->max_velocity = max_vel;
    // Propaga para o algoritmo Direct interno
    servo->direct_ctx.max_vel_mm_s = max_vel;
}

void Servo_SetVelocityFF(ServoDrive_t* servo, float ff) {
    if (servo == NULL) return;
    if (ff < 0.0f) ff = 0.0f;
    if (ff > 1.0f) ff = 1.0f;
    servo->vel_ff = ff;
}

void Servo_SetDutyMax(ServoDrive_t* servo, float max_duty) {
    if (servo == NULL) return;
    if (max_duty < 0.0f) max_duty = 0.0f;
    if (max_duty > 1.0f) max_duty = 1.0f;
    servo->duty_max = max_duty;
}

// ============================================================================
// MODOS DE OPERAÇÃO
// ============================================================================

void Servo_SetIdle(ServoDrive_t* servo) {
    if (servo == NULL) return;
    servo->mode       = SERVO_MODE_IDLE;
    servo->active_alg = NULL;
    servo->alg_done   = false;
    // Zera saída imediatamente
    if (servo->hbridge != NULL) {
        HBridge_SetOutput(servo->hbridge, 0.0f);
    }
}

void Servo_SetVelocity(ServoDrive_t* servo, float vel_mm_s) {
    if (servo == NULL) return;
    // Satura velocidade ao limite configurado
    if (vel_mm_s > servo->max_velocity) vel_mm_s = servo->max_velocity;
    if (vel_mm_s < -servo->max_velocity) vel_mm_s = -servo->max_velocity;

    servo->mode             = SERVO_MODE_VELOCITY;
    servo->velocity_sp_mm_s = vel_mm_s;
    servo->active_alg       = NULL;
    servo->alg_done         = false;
}

void Servo_MoveTo(ServoDrive_t* servo, float target_mm) {
    if (servo == NULL) return;
    Servo_RunAlgorithm(servo, &servo->direct_alg, target_mm);
}

void Servo_MoveProfile(ServoDrive_t* servo, float target_mm, float cruise_vel, float ramp_up_s,
                       float ramp_down_s) {
    if (servo == NULL) return;

    // Reconfigura o algoritmo trapezoidal interno com os novos parâmetros
    TrapezoidalAlg_Init(&servo->trapezoidal_alg, &servo->trapezoidal_ctx, cruise_vel, ramp_up_s,
                        ramp_down_s, WAAM_SERVO_EPS_MM);

    // Posição atual como ponto de partida (lida do estado volatile da cinemática)
    float start_mm = 0.0f;
    if (servo->kinematics != NULL) {
        start_mm = servo->kinematics->state.position_mm;
    }

    // Inicia o algoritmo trapezoidal
    servo->trapezoidal_alg.start(&servo->trapezoidal_alg, start_mm, target_mm);

    // Ativa modo ALGORITHM com o trapezoidal
    servo->active_alg          = &servo->trapezoidal_alg;
    servo->algorithm_target_mm = target_mm;
    servo->mode                = SERVO_MODE_ALGORITHM;
    servo->alg_done            = false;
}

void Servo_RunAlgorithm(ServoDrive_t* servo, MotionAlgorithm_t* alg, float target_mm) {
    if (servo == NULL || alg == NULL) return;

    // Posição atual como ponto de partida
    float start_mm = 0.0f;
    if (servo->kinematics != NULL) {
        start_mm = servo->kinematics->state.position_mm;
    }

    // Inicia o algoritmo
    alg->start(alg, start_mm, target_mm);

    servo->active_alg          = alg;
    servo->algorithm_target_mm = target_mm;
    servo->mode                = SERVO_MODE_ALGORITHM;
    servo->alg_done            = false;
}

// ============================================================================
// CICLO DE CONTROLE (ISR — 1 kHz)
// ============================================================================

void Servo_Update(ServoDrive_t* servo, float dt_s) {
    if (servo == NULL || servo->kinematics == NULL || servo->hbridge == NULL) {
        return;
    }

    // --- 1. Atualiza cinemática (leitura do encoder) ---
    AxisKinematics_UpdateISR(servo->kinematics, dt_s);

    // --- 2. Lê estado atual diretamente do volatile (contexto ISR) ---
    float cur_pos_mm   = servo->kinematics->state.position_mm;
    float cur_vel_mm_s = servo->kinematics->state.velocity_mm_s;

    // --- 3. Determina a referência de velocidade conforme o modo ---
    float v_ref = 0.0f;

    switch (servo->mode) {
        case SERVO_MODE_IDLE:
            v_ref = 0.0f;
            break;

        case SERVO_MODE_VELOCITY:
            v_ref = servo->velocity_sp_mm_s;
            break;

        case SERVO_MODE_ALGORITHM:
            if (servo->active_alg != NULL && servo->active_alg->next != NULL) {
                bool alg_done = false;
                v_ref = servo->active_alg->next(servo->active_alg, dt_s, cur_pos_mm, &alg_done);

                if (alg_done) {
                    // --------------------------------------------------------
                    // Algoritmo concluído → auto-switch para Direct (hold)
                    // --------------------------------------------------------
                    servo->direct_alg.start(&servo->direct_alg, cur_pos_mm,
                                            servo->algorithm_target_mm);
                    servo->active_alg = &servo->direct_alg;
                    servo->alg_done   = true;

                    // Recalcula v_ref com o Direct para este mesmo ciclo
                    v_ref = servo->direct_alg.next(&servo->direct_alg, dt_s, cur_pos_mm, &alg_done);
                }
            }
            break;
    }

    // --- 4. Saturação da referência de velocidade ---
    if (v_ref > servo->max_velocity) v_ref = servo->max_velocity;
    if (v_ref < -servo->max_velocity) v_ref = -servo->max_velocity;

    // --- 5. Malha de velocidade (PID com feedforward) ---
    // PID de velocidade: P apenas (ki=0, kd=0). Corrige o erro de tracking.
    float vel_pid_out = PID_Compute(&servo->vel_pid, v_ref, cur_vel_mm_s, dt_s);

    // Feedforward: antecipa o esforço proporcional à velocidade desejada
    float duty = vel_pid_out + (v_ref * servo->vel_ff);

    // --- 6. Saturação do duty cycle ---
    if (duty > servo->duty_max) duty = servo->duty_max;
    if (duty < -servo->duty_max) duty = -servo->duty_max;

    // --- 7. Saída física para a Ponte H ---
    HBridge_SetOutput(servo->hbridge, duty);
}

// ============================================================================
// SEGURANÇA E TELEMETRIA
// ============================================================================

void Servo_EmergencyStop(ServoDrive_t* servo) {
    if (servo == NULL) return;
    servo->mode             = SERVO_MODE_IDLE;
    servo->velocity_sp_mm_s = 0.0f;
    servo->active_alg       = NULL;
    servo->alg_done         = false;
    if (servo->hbridge != NULL) {
        HBridge_SetOutput(servo->hbridge, 0.0f);
    }
    // Reseta estados internos do PID para evitar kicks na retomada
    PID_Reset(&servo->vel_pid);
}

bool Servo_IsDone(const ServoDrive_t* servo) {
    if (servo == NULL) return true;
    return servo->alg_done;
}

ServoMode_t Servo_GetMode(const ServoDrive_t* servo) {
    if (servo == NULL) return SERVO_MODE_IDLE;
    return servo->mode;
}
