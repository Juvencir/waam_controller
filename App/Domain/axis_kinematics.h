/**
 * @file axis_kinematics.h
 * @brief Gerenciador de Cinemática do Eixo.
 * @details Responsável pela conversão física de pulsos para milímetros (mm)
 *          e milímetros por segundo (mm/s), utilizando sistema de coordenada
 *          única com sinal e suporte a reset direto de zero.
 */

#ifndef AXIS_KINEMATICS_H
#define AXIS_KINEMATICS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "Config/waam_params.h"
#include "Drivers/encoder.h"

// ============================================================================
// CONSTANTES E GEOMETRIA DO PROCESSO WAAM
// ============================================================================
#ifndef KINEMATICS_PI
#define KINEMATICS_PI 3.14159265358979323846f
#endif

// Derivações matemáticas calculadas em tempo de compilação
#define KIN_TOTAL_PPR ((float)(WAAM_ENCODER_PPR * WAAM_QUADRATURE_FACTOR))
#define KIN_PERIMETER_MM (KINEMATICS_PI * WAAM_PINION_DIAMETER_MM)
#define KIN_PULSES_PER_MM (KIN_TOTAL_PPR / KIN_PERIMETER_MM)
#define KIN_MM_PER_PULSE (1.0f / KIN_PULSES_PER_MM)

// ============================================================================
// ESTRUTURAS DE DADOS UNIFICADAS
// ============================================================================

/**
 * @brief Estado físico instantâneo do eixo (Dados puros de cinemática).
 * @details Estrutura reutilizável tanto para o estado vivo na ISR quanto
 *          para cópias congeladas (snapshots) consumidas pela Telemetria.
 */
typedef struct {
    float   position_mm;       /**< Posição única atual com sinal (+/- mm) */
    float   velocity_mm_s;     /**< Velocidade instantânea em mm/s */
    int64_t total_pulses;      /**< Acumulador de pulsos em 64-bit com sinal */
    int32_t last_delta_pulses; /**< Delta de pulsos lido no último ms */
} KinematicsState_t;

/**
 * @brief Instância do Módulo AxisKinematics (Composição por aninhamento).
 */
typedef struct {
    Encoder_t*                 encoder_driver; /**< Ponteiro para o driver do Encoder[cite: 4] */
    volatile KinematicsState_t state;          /**< Estado físico vivo (Volatile para ISR) */
} AxisKinematics_t;

// ============================================================================
// APIS PÚBLICAS DO MÓDULO
// ============================================================================

/**
 * @brief Inicializa a estrutura de cinemática e vincula o driver de hardware.
 */
void AxisKinematics_Init(AxisKinematics_t* kin, Encoder_t* encoder_drv);

/**
 * @brief Atualiza a física do eixo. Executada estritamente na ISR do Timer (1 kHz).
 * @param dt_s Período de amostragem em segundos (ex: 0.001f para 1 ms).
 */
void AxisKinematics_UpdateISR(AxisKinematics_t* kin, float dt_s);

/**
 * @brief Reseta o ponto atual do motor para ser o NOVO ZERO ABSOLUTO (0.0 mm).
 * @details Executado via Seção Crítica do RTOS. Zera o acumulador de hardware.
 */
void AxisKinematics_ResetZero(AxisKinematics_t* kin);

/**
 * @brief Extrai uma cópia atômica (Thread-Safe) do estado atual para a Telemetria.
 * @param kin Ponteiro para a estrutura de cinemática.
 * @param out_state Ponteiro do buffer local da Task de Telemetria.
 */
void AxisKinematics_GetStateSnapshot(const AxisKinematics_t* kin, KinematicsState_t* out_state);

#endif  // AXIS_KINEMATICS_H