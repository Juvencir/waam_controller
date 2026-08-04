/**
 * @file motion_algorithm.c
 * @brief Implementação dos algoritmos de movimento (Direct e Trapezoidal).
 */

#include "Algorithms/motion_algorithm.h"

#include <math.h>
#include <stdlib.h>

// ============================================================================
// ALGORITMO: DIRECT
// ============================================================================

static void direct_start(MotionAlgorithm_t* alg, float start_mm, float target_mm) {
    (void)start_mm;  // Não utilizado no Direct
    DirectAlgContext_t* ctx = (DirectAlgContext_t*)alg->ctx;
    ctx->target_mm          = target_mm;
}

static float direct_next(MotionAlgorithm_t* alg, float dt_s, float cur_mm, bool* done) {
    (void)dt_s;
    DirectAlgContext_t* ctx = (DirectAlgContext_t*)alg->ctx;

    float error = ctx->target_mm - cur_mm;
    float v_ref = error * ctx->kp_pos;

    // Saturação da referência de velocidade
    if (v_ref > ctx->max_vel_mm_s) {
        v_ref = ctx->max_vel_mm_s;
    } else if (v_ref < -ctx->max_vel_mm_s) {
        v_ref = -ctx->max_vel_mm_s;
    }

    // Direct nunca termina — é um hold contínuo
    *done = false;
    return v_ref;
}

void DirectAlg_Init(MotionAlgorithm_t* alg, DirectAlgContext_t* ctx, float kp_pos, float max_vel,
                    float eps) {
    if (alg == NULL || ctx == NULL) return;

    ctx->kp_pos       = kp_pos;
    ctx->max_vel_mm_s = max_vel;
    ctx->eps_mm       = eps;
    ctx->target_mm    = 0.0f;

    alg->ctx   = ctx;
    alg->start = direct_start;
    alg->next  = direct_next;
}

// ============================================================================
// ALGORITMO: TRAPEZOIDAL
// ============================================================================

static void trapezoidal_start(MotionAlgorithm_t* alg, float start_mm, float target_mm) {
    TrapezoidalAlgContext_t* ctx = (TrapezoidalAlgContext_t*)alg->ctx;

    ctx->target_mm    = target_mm;
    ctx->start_pos_mm = start_mm;
    ctx->elapsed_s    = 0.0f;

    float total_dist = target_mm - start_mm;

    // Distância zero ou desprezível: já está no alvo
    if (fabsf(total_dist) < ctx->eps_mm) {
        ctx->dir      = 0;
        ctx->v_peak   = 0.0f;
        ctx->t1       = 0.0f;
        ctx->t2       = 0.0f;
        ctx->t_total  = 0.0f;
        ctx->d_accel  = 0.0f;
        ctx->d_cruise = 0.0f;
        ctx->accel    = 0.0f;
        ctx->decel    = 0.0f;
        return;
    }

    ctx->dir   = (total_dist > 0.0f) ? 1 : -1;
    float dist = fabsf(total_dist);

    // Proteção contra parâmetros inválidos
    float ramp_up   = (ctx->ramp_up_s > 0.001f) ? ctx->ramp_up_s : 0.001f;
    float ramp_down = (ctx->ramp_down_s > 0.001f) ? ctx->ramp_down_s : 0.001f;
    float cruise    = (ctx->cruise_vel_mm_s > 0.001f) ? ctx->cruise_vel_mm_s : 0.001f;

    // Taxas de aceleração/desaceleração
    ctx->accel = cruise / ramp_up;
    ctx->decel = cruise / ramp_down;

    // Distâncias mínimas para perfil completo
    float d_accel_full = 0.5f * cruise * ramp_up;
    float d_decel_full = 0.5f * cruise * ramp_down;
    float d_min_full   = d_accel_full + d_decel_full;

    if (dist >= d_min_full) {
        // ================================================================
        // PERFIL TRAPEZOIDAL COMPLETO (aceleração + cruzeiro + desaceleração)
        // ================================================================
        ctx->v_peak   = cruise;
        ctx->d_accel  = d_accel_full;
        ctx->d_cruise = dist - d_min_full;
        ctx->t1       = ramp_up;
        ctx->t2       = ramp_up + (ctx->d_cruise / cruise);
        ctx->t_total  = ctx->t2 + ramp_down;
    } else {
        // ================================================================
        // PERFIL TRIANGULAR (sem cruzeiro — reduz velocidade de pico)
        // ================================================================
        // v_peak = sqrt(2 * D * a_acc * a_dec / (a_acc + a_dec))
        float a_sum = ctx->accel + ctx->decel;
        if (a_sum < 1e-6f) a_sum = 1e-6f;
        ctx->v_peak   = sqrtf(2.0f * dist * ctx->accel * ctx->decel / a_sum);
        ctx->d_accel  = 0.5f * ctx->v_peak * ctx->v_peak / ctx->accel;
        ctx->d_cruise = 0.0f;
        ctx->t1       = ctx->v_peak / ctx->accel;
        ctx->t2       = ctx->t1;  // sem fase de cruzeiro
        ctx->t_total  = ctx->t1 + (ctx->v_peak / ctx->decel);
    }
}

static float trapezoidal_next(MotionAlgorithm_t* alg, float dt_s, float cur_mm, bool* done) {
    (void)cur_mm;  // O perfil trapezoidal é puramente temporal; a correção de
                   // posição fina fica a cargo do algoritmo Direct após done.

    TrapezoidalAlgContext_t* ctx = (TrapezoidalAlgContext_t*)alg->ctx;

    // Distância zero ou perfil já concluído
    if (ctx->dir == 0 || ctx->elapsed_s >= ctx->t_total) {
        *done = true;
        return 0.0f;
    }

    *done     = false;
    float t   = ctx->elapsed_s;
    float vel = 0.0f;

    if (t < ctx->t1) {
        // ------------------------------------------------------------
        // FASE 1: ACELERAÇÃO (0 → t1)
        // ------------------------------------------------------------
        vel = ctx->accel * t;
    } else if (t < ctx->t2) {
        // ------------------------------------------------------------
        // FASE 2: CRUZEIRO (t1 → t2)
        // ------------------------------------------------------------
        vel = ctx->v_peak;
    } else {
        // ------------------------------------------------------------
        // FASE 3: DESACELERAÇÃO (t2 → t_total)
        // ------------------------------------------------------------
        float t_decel = t - ctx->t2;
        vel           = ctx->v_peak - ctx->decel * t_decel;
        if (vel < 0.0f) vel = 0.0f;
    }

    // Avança o tempo interno
    ctx->elapsed_s += dt_s;

    return vel * (float)ctx->dir;
}

void TrapezoidalAlg_Init(MotionAlgorithm_t* alg, TrapezoidalAlgContext_t* ctx, float cruise_vel,
                         float ramp_up_s, float ramp_down_s, float eps) {
    if (alg == NULL || ctx == NULL) return;

    ctx->cruise_vel_mm_s = cruise_vel;
    ctx->ramp_up_s       = ramp_up_s;
    ctx->ramp_down_s     = ramp_down_s;
    ctx->eps_mm          = eps;

    // Estado interno será preenchido em start()
    ctx->target_mm    = 0.0f;
    ctx->start_pos_mm = 0.0f;
    ctx->elapsed_s    = 0.0f;
    ctx->accel        = 0.0f;
    ctx->decel        = 0.0f;
    ctx->v_peak       = 0.0f;
    ctx->t1           = 0.0f;
    ctx->t2           = 0.0f;
    ctx->t_total      = 0.0f;
    ctx->d_accel      = 0.0f;
    ctx->d_cruise     = 0.0f;
    ctx->dir          = 0;

    alg->ctx   = ctx;
    alg->start = trapezoidal_start;
    alg->next  = trapezoidal_next;
}
