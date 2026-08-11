/**
 * @file waam_params.h
 * @brief Parâmetros Físicos e Geométricos do Equipamento WAAM.
 */

#ifndef WAAM_PARAMS_H
#define WAAM_PARAMS_H

// ============================================================================
// GEOMETRIA E MECÂNICA DA TARTÍLOPE
// ============================================================================
#define WAAM_ENCODER_PPR 1000U        /**< Pulsos Por Volta do encoder físico */
#define WAAM_QUADRATURE_FACTOR 4U     /**< Fator de quadratura 4x (TIM2) */
#define WAAM_PINION_DIAMETER_MM 30.0f /**< Diâmetro primitivo do pinhão (mm) */

// ============================================================================
// LIMITES E VELOCIDADES PADRÃO
// ============================================================================
// Velocidade máxima em mm/s.
#define WAAM_MAX_TRAVEL_SPEED_MM_S 14.0f

// ============================================================================
// PARÂMETROS DO SERVO DRIVE (DEFAULTS — sobrescrevíveis em runtime via API)
// ============================================================================
#define WAAM_SERVO_POS_GAIN 8.0f /**< Ganho P de posição (mm/s por mm de erro) */
#define WAAM_SERVO_VEL_KP 0.5f   /**< Ganho P da malha de velocidade */

// Feedforward: modelo linear duty = vel_ff * v_ref, base teórica vel_ff = 1/V_max.
// VEL_FF_SCALE = fator de "under-drive": <1.0 deixa margem de autoridade para o
// P/I corrigir sem saturar (ex: 0.8 = 80% do modelo → 20% de margem).
// Independente do limite de velocidade: mudar MM_S não exige mexer na escala.
#define WAAM_SERVO_VEL_FF_SCALE 0.8f
#define WAAM_SERVO_VEL_FF ((1.0f / WAAM_MAX_TRAVEL_SPEED_MM_S) * WAAM_SERVO_VEL_FF_SCALE)

#define WAAM_SERVO_DUTY_MAX 1.0f       /**< Duty cycle máximo (0.0–1.0) */
#define WAAM_SERVO_EPS_MM 0.1f         /**< Tolerância de posição para chegada (mm) */
#define WAAM_MOVE_DEFAULT_RAMP_S 0.05f /**< Rampa padrão quando max_accel/max_decel = 0 (s) */

#endif  // WAAM_PARAMS_H