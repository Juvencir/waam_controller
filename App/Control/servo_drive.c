/**
 * @file servo_drive.c
 * @brief Implementação do Servo Drive — controle unificado do motor como servo.
 */

#include "Control/servo_drive.h"

#include <math.h>
#include <stddef.h>

#include "Config/waam_params.h"

// ============================================================================
// HELPERS INTERNOS — PLANEJADOR DE TRAJETÓRIA
// ============================================================================

/**
 * @brief Inicializa o planejador trapezoidal com os parâmetros fornecidos.
 * @details Se max_vel <= 0, não há perfil temporal (t_total = 0) — o Update
 *          interpreta isso como "sem plano" e usa P-controller puro (homing).
 *
 *          Se max_vel > 0, calcula as fases (aceleração, cruzeiro, desaceleração).
 *          Distâncias curtas caem automaticamente em perfil triangular.
 *          max_accel/decel = 0 usam rampa default de WAAM_MOVE_DEFAULT_RAMP_S.
 */
static void plan_init(MovePlan_t* p, float start_mm, float target_mm, float max_vel,
                      float max_accel, float max_decel, float eps) {
    p->start_pos_mm    = start_mm;
    p->target_mm       = target_mm;
    p->elapsed_s       = 0.0f;
    p->max_vel_mm_s    = max_vel;
    p->max_accel_mm_s2 = max_accel;
    p->max_decel_mm_s2 = max_decel;
    p->active          = true;

    float dist = target_mm - start_mm;

    // Já no alvo ou sem limite de velocidade → sem plano temporal
    if (fabsf(dist) < eps || max_vel <= 0.0f) {
        p->v_peak        = 0.0f;
        p->t_accel_end   = 0.0f;
        p->t_decel_start = 0.0f;
        p->t_total       = 0.0f;
        p->accel_rate    = 0.0f;
        p->decel_rate    = 0.0f;
        return;
    }

    float abs_d = fabsf(dist);

    // Rampas: se não informadas, deriva da velocidade de cruzeiro com rampa default
    float a_acc = (max_accel > 0.0f) ? max_accel : (max_vel / WAAM_MOVE_DEFAULT_RAMP_S);
    float a_dec = (max_decel > 0.0f) ? max_decel : (max_vel / WAAM_MOVE_DEFAULT_RAMP_S);

    if (a_acc < 1.0f) a_acc = 1.0f;
    if (a_dec < 1.0f) a_dec = 1.0f;

    p->accel_rate = a_acc;
    p->decel_rate = a_dec;

    float d_accel = 0.5f * max_vel * max_vel / a_acc;
    float d_decel = 0.5f * max_vel * max_vel / a_dec;
    float d_min   = d_accel + d_decel;

    if (abs_d >= d_min) {
        // Perfil trapezoidal completo
        p->v_peak        = max_vel;
        p->t_accel_end   = max_vel / a_acc;
        float t_cruise   = (abs_d - d_min) / max_vel;
        p->t_decel_start = p->t_accel_end + t_cruise;
        p->t_total       = p->t_decel_start + (max_vel / a_dec);
    } else {
        // Perfil triangular (sem cruzeiro)
        float a_sum      = a_acc + a_dec;
        p->v_peak        = sqrtf(2.0f * abs_d * a_acc * a_dec / a_sum);
        p->t_accel_end   = p->v_peak / a_acc;
        p->t_decel_start = p->t_accel_end;
        p->t_total       = p->t_accel_end + (p->v_peak / a_dec);
    }
}

/**
 * @brief Avança um passo do perfil temporal e retorna a referência de velocidade.
 * @param done [out] true se o perfil temporal terminou (elapsed >= t_total).
 * @return Velocidade de referência em mm/s com sinal.
 */
static float plan_step(MovePlan_t* p, float dt_s, bool* done) {
    if (p->elapsed_s >= p->t_total) {
        *done = true;
        return 0.0f;
    }

    float sign = (p->target_mm > p->start_pos_mm) ? 1.0f : -1.0f;
    float t    = p->elapsed_s;
    float vel  = 0.0f;

    if (t < p->t_accel_end) {
        vel = p->accel_rate * t;
    } else if (t < p->t_decel_start) {
        vel = p->v_peak;
    } else {
        float t_decel = t - p->t_decel_start;
        vel           = p->v_peak - p->decel_rate * t_decel;
        if (vel < 0.0f) vel = 0.0f;
    }

    p->elapsed_s += dt_s;
    *done = false;
    return vel * sign;
}

// ============================================================================
// INICIALIZAÇÃO
// ============================================================================

void Servo_Init(ServoDrive_t* servo, AxisKinematics_t* kin, HBridge_t* hb) {
    if (servo == NULL) return;

    servo->kinematics = kin;
    servo->hbridge    = hb;

    servo->pos_gain     = WAAM_SERVO_POS_GAIN;
    servo->max_velocity = WAAM_MAX_TRAVEL_SPEED_MM_S;
    servo->vel_ff       = WAAM_SERVO_VEL_FF;
    servo->duty_max     = WAAM_SERVO_DUTY_MAX;

    PID_Init(&servo->vel_pid, WAAM_SERVO_VEL_KP, 0.0f, 0.0f, -1.0f, 1.0f);

    servo->mode             = SERVO_MODE_IDLE;
    servo->velocity_sp_mm_s = 0.0f;
    servo->hold_target_mm   = 0.0f;
    servo->move_done        = false;
    servo->plan.active      = false;
}

// ============================================================================
// CONFIGURAÇÃO
// ============================================================================

void Servo_SetPosGain(ServoDrive_t* servo, float gain) {
    if (servo == NULL) return;
    servo->pos_gain = gain;
}

void Servo_SetMaxVelocity(ServoDrive_t* servo, float max_vel) {
    if (servo == NULL) return;
    servo->max_velocity = max_vel;
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
    servo->mode        = SERVO_MODE_IDLE;
    servo->move_done   = false;
    servo->plan.active = false;
    if (servo->hbridge != NULL) {
        HBridge_SetOutput(servo->hbridge, 0.0f);
    }
}

void Servo_SetVelocity(ServoDrive_t* servo, float vel_mm_s) {
    if (servo == NULL) return;
    if (vel_mm_s > servo->max_velocity) vel_mm_s = servo->max_velocity;
    if (vel_mm_s < -servo->max_velocity) vel_mm_s = -servo->max_velocity;

    servo->mode             = SERVO_MODE_VELOCITY;
    servo->velocity_sp_mm_s = vel_mm_s;
    servo->move_done        = false;
    servo->plan.active      = false;
}

void Servo_Move(ServoDrive_t* servo, float target_mm, float max_vel_mm_s, float max_accel_mm_s2,
                float max_decel_mm_s2) {
    if (servo == NULL) return;

    servo->hold_target_mm = target_mm;
    servo->move_done      = false;
    servo->mode           = SERVO_MODE_MOVE;

    float start_mm = 0.0f;
    if (servo->kinematics != NULL) {
        start_mm = servo->kinematics->state.position_mm;
    }

    // plan_init trata max_vel=0 internamente: t_total=0 sinaliza "sem plano"
    plan_init(&servo->plan, start_mm, target_mm, max_vel_mm_s, max_accel_mm_s2, max_decel_mm_s2,
              WAAM_SERVO_EPS_MM);
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

        case SERVO_MODE_MOVE: {
            if (servo->plan.t_total > 0.0f) {
                // ------------------------------------------------------------
                // Existe perfil temporal → trapezoidal.
                // ------------------------------------------------------------
                bool done = false;
                v_ref     = plan_step(&servo->plan, dt_s, &done);

                if (servo->plan.max_vel_mm_s > 0.0f) {
                    if (v_ref > servo->max_velocity) v_ref = servo->max_velocity;
                    if (v_ref < -servo->max_velocity) v_ref = -servo->max_velocity;
                }

                if (done) {
                    servo->move_done = true;
                    // Auto-hold: correção fina de posição
                    float err = servo->hold_target_mm - cur_pos_mm;
                    v_ref     = err * servo->pos_gain;
                    if (v_ref > servo->max_velocity) v_ref = servo->max_velocity;
                    if (v_ref < -servo->max_velocity) v_ref = -servo->max_velocity;
                }
            } else {
                // ------------------------------------------------------------
                // Sem perfil temporal (max_vel=0) → P-controller puro.
                // Único limitante é duty_max → comportamento de homing.
                // ------------------------------------------------------------
                float err = servo->hold_target_mm - cur_pos_mm;
                v_ref     = err * servo->pos_gain;

                if (fabsf(err) < WAAM_SERVO_EPS_MM) {
                    servo->move_done = true;
                }
            }
            break;
        }
    }

    // --- 4. Malha de velocidade (PID com feedforward) ---
    float vel_pid_out = PID_Compute(&servo->vel_pid, v_ref, cur_vel_mm_s, dt_s);
    float duty        = vel_pid_out + (v_ref * servo->vel_ff);

    // --- 5. Saturação do duty cycle ---
    if (duty > servo->duty_max) duty = servo->duty_max;
    if (duty < -servo->duty_max) duty = -servo->duty_max;

    // --- 6. Saída física para a Ponte H ---
    HBridge_SetOutput(servo->hbridge, duty);
}

// ============================================================================
// SEGURANÇA E TELEMETRIA
// ============================================================================

void Servo_EmergencyStop(ServoDrive_t* servo) {
    if (servo == NULL) return;

    servo->mode             = SERVO_MODE_IDLE;
    servo->velocity_sp_mm_s = 0.0f;
    servo->move_done        = false;
    servo->plan.active      = false;

    if (servo->hbridge != NULL) {
        HBridge_SetOutput(servo->hbridge, 0.0f);
    }

    PID_Reset(&servo->vel_pid);
}

bool Servo_IsDone(const ServoDrive_t* servo) {
    if (servo == NULL) return true;

    if (servo->mode == SERVO_MODE_IDLE) return true;
    if (servo->mode == SERVO_MODE_VELOCITY) return false;
    return servo->move_done;
}

ServoMode_t Servo_GetMode(const ServoDrive_t* servo) {
    if (servo == NULL) return SERVO_MODE_IDLE;
    return servo->mode;
}
