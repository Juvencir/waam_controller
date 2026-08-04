/**
 * @file motion_algorithm.h
 * @brief Interface injetável de algoritmos de movimento para o Servo Drive.
 * @details Cada algoritmo implementa start(next) -> v_ref + done.
 *          - Direct: controle P de posição com clamp de velocidade (hold contínuo).
 *          - Trapezoidal: perfil trapezoidal (aceleração → cruzeiro → desaceleração)
 *            com fallback para perfil triangular se a distância for curta.
 */

#ifndef MOTION_ALGORITHM_H
#define MOTION_ALGORITHM_H

#include <stdbool.h>
#include <stdint.h>

// ============================================================================
// INTERFACE ABSTRATA (VTABLE)
// ============================================================================

/** Forward declaration da estrutura base do algoritmo. */
typedef struct MotionAlgorithm_s MotionAlgorithm_t;

/**
 * @brief Estrutura base (vtable) de um algoritmo de movimento.
 * @details O usuário instancia um contexto tipado (ex: DirectAlgContext_t),
 *          preenche a vtable com as funções correspondentes e injeta no Servo.
 */
struct MotionAlgorithm_s {
    void* ctx; /**< Ponteiro para o contexto concreto do algoritmo */

    /**
     * @brief (Re)inicia o algoritmo para um novo alvo.
     * @param alg       Ponteiro para a vtable do algoritmo.
     * @param start_mm  Posição atual do eixo no momento do start.
     * @param target_mm Posição alvo desejada.
     */
    void (*start)(MotionAlgorithm_t* alg, float start_mm, float target_mm);

    /**
     * @brief Avança um passo do algoritmo e retorna a referência de velocidade.
     * @param alg       Ponteiro para a vtable do algoritmo.
     * @param dt_s      Intervalo de tempo desde a última chamada (segundos).
     * @param cur_mm    Posição atual do eixo (realimentação).
     * @param done      [out] true se o algoritmo atingiu o objetivo.
     * @return float    Velocidade de referência em mm/s com sinal.
     */
    float (*next)(MotionAlgorithm_t* alg, float dt_s, float cur_mm, bool* done);
};

// ============================================================================
// ALGORITMO: DIRECT (HOLD DE POSIÇÃO)
// ============================================================================

/**
 * @brief Contexto do algoritmo Direct — controle proporcional de posição.
 * @details Mantém o eixo no alvo indefinidamente (done nunca é true).
 *          v_ref = clamp((target - cur) * kp, ±max_vel).
 */
typedef struct {
    float target_mm;    /**< Posição alvo atual */
    float kp_pos;       /**< Ganho proporcional de posição (mm/s por mm de erro) */
    float max_vel_mm_s; /**< Limite de velocidade de referência (mm/s) */
    float eps_mm;       /**< Tolerância para considerar "no alvo" (apenas informativo) */
} DirectAlgContext_t;

/**
 * @brief Inicializa a vtable e o contexto do algoritmo Direct.
 * @param alg      Ponteiro para a vtable a ser preenchida.
 * @param ctx      Ponteiro para o contexto DirectAlgContext_t.
 * @param kp_pos   Ganho proporcional (ex: 8.0f).
 * @param max_vel  Velocidade máxima de referência em mm/s.
 * @param eps      Tolerância em mm (ex: 0.1f).
 */
void DirectAlg_Init(MotionAlgorithm_t* alg, DirectAlgContext_t* ctx, float kp_pos, float max_vel,
                    float eps);

// ============================================================================
// ALGORITMO: TRAPEZOIDAL (PERFIL DE VELOCIDADE)
// ============================================================================

/**
 * @brief Contexto do algoritmo Trapezoidal — perfil aceleração/cruzeiro/desaceleração.
 * @details Gera referência de velocidade no tempo. Se a distância for curta demais
 *          para o perfil completo, reduz automaticamente a velocidade de pico
 *          (perfil triangular, sem cruzeiro). Reporta done = true quando
 *          o perfil temporal termina E o erro de posição está abaixo de eps_mm.
 */
typedef struct {
    // Configuração (fixa após init)
    float cruise_vel_mm_s; /**< Velocidade de cruzeiro (mm/s) */
    float ramp_up_s;       /**< Tempo de rampa de aceleração (s) */
    float ramp_down_s;     /**< Tempo de rampa de desaceleração (s) */
    float eps_mm;          /**< Tolerância de posição para done (mm) */

    // Estado interno (atualizado em start e next)
    float  target_mm;
    float  start_pos_mm;
    float  elapsed_s;
    float  accel;    /**< Taxa de aceleração calculada (mm/s²) */
    float  decel;    /**< Taxa de desaceleração calculada (mm/s²) */
    float  v_peak;   /**< Velocidade de pico real (mm/s, ≤ cruise) */
    float  t1;       /**< Fim da fase de aceleração (s) */
    float  t2;       /**< Início da fase de desaceleração (s) */
    float  t_total;  /**< Duração total do perfil (s) */
    float  d_accel;  /**< Distância percorrida na aceleração (mm) */
    float  d_cruise; /**< Distância percorrida no cruzeiro (mm) */
    int8_t dir;      /**< +1 se target > start, -1 caso contrário */
} TrapezoidalAlgContext_t;

/**
 * @brief Inicializa a vtable e o contexto do algoritmo Trapezoidal.
 * @param alg         Ponteiro para a vtable a ser preenchida.
 * @param ctx         Ponteiro para o contexto TrapezoidalAlgContext_t.
 * @param cruise_vel  Velocidade de cruzeiro em mm/s (ex: 10.0f).
 * @param ramp_up_s   Tempo de subida (aceleração) em segundos (ex: 0.5f).
 * @param ramp_down_s Tempo de descida (desaceleração) em segundos (ex: 0.3f).
 * @param eps         Tolerância de posição em mm (ex: 0.1f).
 */
void TrapezoidalAlg_Init(MotionAlgorithm_t* alg, TrapezoidalAlgContext_t* ctx, float cruise_vel,
                         float ramp_up_s, float ramp_down_s, float eps);

#endif  // MOTION_ALGORITHM_H
