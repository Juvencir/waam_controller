/**
 * @file servo_drive.h
 * @brief API unificada do Servo Drive — controle de motor como servo completo.
 * @details Compõe AxisKinematics (feedback), HBridge (atuador), PID (controle)
 *          e MotionAlgorithm (planejamento de trajetória) em uma interface
 *          simples de alto nível com três modos de operação:
 *          - IDLE:      motor desligado.
 *          - VELOCITY:  controle direto de velocidade (jog).
 *          - ALGORITHM: controle por algoritmo de movimento (posição ou perfil).
 *
 *          A função Servo_Update() deve ser chamada na ISR do Timer de controle
 *          (1 kHz). Ela executa: leitura do encoder → atualização da cinemática →
 *          cálculo da lei de controle → saída PWM na Ponte H.
 */

#ifndef SERVO_DRIVE_H
#define SERVO_DRIVE_H

#include <stdbool.h>

#include "Algorithms/motion_algorithm.h"
#include "Algorithms/pid.h"
#include "Domain/axis_kinematics.h"
#include "Drivers/hbridge.h"

// ============================================================================
// TIPOS PÚBLICOS
// ============================================================================

/** Modos de operação do Servo Drive. */
typedef enum {
    SERVO_MODE_IDLE = 0,  /**< Motor desenergizado, saída em 0 */
    SERVO_MODE_VELOCITY,  /**< Controle direto de velocidade (jog manual) */
    SERVO_MODE_ALGORITHM, /**< Controle por algoritmo de movimento injetável */
} ServoMode_t;

/**
 * @brief Estrutura principal do Servo Drive (composição de todas as camadas).
 */
typedef struct {
    // -- Referências para camadas inferiores (injetadas) --
    AxisKinematics_t* kinematics; /**< Cinemática do eixo (feedback de pos/vel) */
    HBridge_t*        hbridge;    /**< Driver da Ponte H (atuador) */

    // -- Malhas de controle --
    PID_t vel_pid; /**< PID da malha de velocidade */

    // -- Configurações ajustáveis --
    float pos_gain;     /**< Ganho P de posição (mm/s por mm de erro). Default: 8.0 */
    float max_velocity; /**< Limite de velocidade de referência (mm/s). Default: 13.33 */
    float vel_ff;       /**< Ganho feedforward duty/velocidade (1/(mm/s)). Default: 1/V_max */
    float duty_max;     /**< Duty cycle máximo (0.0 a 1.0). Default: 1.0 */

    // -- Estado operacional --
    ServoMode_t        mode;                /**< Modo atual */
    float              velocity_sp_mm_s;    /**< Setpoint do modo VELOCITY */
    MotionAlgorithm_t* active_alg;          /**< Algoritmo ativo no modo ALGORITHM */
    float              algorithm_target_mm; /**< Alvo do algoritmo ativo (p/ auto-hold) */
    bool               alg_done;            /**< true se o algoritmo reportou done */

    // -- Algoritmos internos (pré-instanciados para conveniência) --
    DirectAlgContext_t      direct_ctx;
    TrapezoidalAlgContext_t trapezoidal_ctx;
    MotionAlgorithm_t       direct_alg;
    MotionAlgorithm_t       trapezoidal_alg;

} ServoDrive_t;

// ============================================================================
// API DE INICIALIZAÇÃO E CONFIGURAÇÃO
// ============================================================================

/**
 * @brief Inicializa o Servo Drive, vinculando as camadas e configurando defaults.
 * @param servo Ponteiro para a estrutura do Servo Drive.
 * @param kin   Ponteiro para a instância de AxisKinematics (já inicializada).
 * @param hb    Ponteiro para a instância de HBridge (já inicializada).
 */
void Servo_Init(ServoDrive_t* servo, AxisKinematics_t* kin, HBridge_t* hb);

/**
 * @brief Ajusta o ganho proporcional de posição.
 * @param servo Ponteiro para o Servo Drive.
 * @param gain  Ganho em (mm/s)/mm (ex: 8.0f significa 8 mm/s para cada 1 mm de erro).
 */
void Servo_SetPosGain(ServoDrive_t* servo, float gain);

/**
 * @brief Ajusta a velocidade máxima de referência.
 * @param servo    Ponteiro para o Servo Drive.
 * @param max_vel  Velocidade máxima em mm/s (ex: 13.33f).
 */
void Servo_SetMaxVelocity(ServoDrive_t* servo, float max_vel);

/**
 * @brief Ajusta o ganho de feedforward de velocidade.
 * @details Modelo linear: duty_ff = ff * v_ref. Calibrar medindo a velocidade
 *          em regime com duty conhecido (ex: duty=0.3 → v=4 mm/s ⇒ ff=0.075).
 * @param servo Ponteiro para o Servo Drive.
 * @param ff    Ganho feedforward em duty por mm/s (ex: 0.075f).
 */
void Servo_SetVelocityFF(ServoDrive_t* servo, float ff);

/**
 * @brief Ajusta o duty cycle máximo aplicado ao motor.
 * @param servo    Ponteiro para o Servo Drive.
 * @param max_duty Valor entre 0.0 e 1.0.
 */
void Servo_SetDutyMax(ServoDrive_t* servo, float max_duty);

// ============================================================================
// API DE MODO DE OPERAÇÃO
// ============================================================================

/**
 * @brief Coloca o servo em modo IDLE (motor desligado).
 * @param servo Ponteiro para o Servo Drive.
 */
void Servo_SetIdle(ServoDrive_t* servo);

/**
 * @brief Coloca o servo em modo VELOCITY (jog) com o setpoint informado.
 * @param servo      Ponteiro para o Servo Drive.
 * @param vel_mm_s   Velocidade desejada em mm/s (positiva = avanço, negativa = recuo).
 */
void Servo_SetVelocity(ServoDrive_t* servo, float vel_mm_s);

/**
 * @brief Move o eixo diretamente para uma posição alvo e mantém (hold).
 * @details Utiliza internamente o algoritmo Direct. A velocidade de aproximação
 *          é proporcional ao erro (pos_gain) e limitada por max_velocity.
 * @param servo    Ponteiro para o Servo Drive.
 * @param target_mm Posição alvo em mm.
 */
void Servo_MoveTo(ServoDrive_t* servo, float target_mm);

/**
 * @brief Move o eixo seguindo um perfil trapezoidal de velocidade.
 * @details Acelera até cruise_vel em ramp_up_s, mantém cruzeiro e desacelera
 *          em ramp_down_s, parando exatamente no alvo. Se a distância for
 *          curta demais, reduz automaticamente a velocidade de pico (triangular).
 * @param servo       Ponteiro para o Servo Drive.
 * @param target_mm   Posição alvo em mm.
 * @param cruise_vel  Velocidade de cruzeiro em mm/s.
 * @param ramp_up_s   Tempo de rampa de aceleração em segundos.
 * @param ramp_down_s Tempo de rampa de desaceleração em segundos.
 */
void Servo_MoveProfile(ServoDrive_t* servo, float target_mm, float cruise_vel, float ramp_up_s,
                       float ramp_down_s);

/**
 * @brief Injeta um algoritmo de movimento customizado.
 * @details O algoritmo será iniciado com start() e executado a cada Update.
 *          Quando o algoritmo reportar done = true, o servo volta automaticamente
 *          para hold na posição alvo (comportamento Direct).
 * @param servo  Ponteiro para o Servo Drive.
 * @param alg    Ponteiro para a vtable do algoritmo (já inicializado).
 * @param target_mm Posição alvo final (usada para hold após done).
 */
void Servo_RunAlgorithm(ServoDrive_t* servo, MotionAlgorithm_t* alg, float target_mm);

// ============================================================================
// API DE CICLO DE CONTROLE (ISR)
// ============================================================================

/**
 * @brief Executa um ciclo completo de controle. CHAMAR NA ISR DO TIMER (1 kHz).
 * @details Ordem interna: leitura do encoder → atualização cinemática →
 *          avaliação do algoritmo ativo → malha de velocidade → saída PWM.
 * @param servo Ponteiro para o Servo Drive.
 * @param dt_s  Intervalo de amostragem em segundos (tipicamente 0.001f).
 */
void Servo_Update(ServoDrive_t* servo, float dt_s);

// ============================================================================
// API DE SEGURANÇA E TELEMETRIA
// ============================================================================

/**
 * @brief Para o motor imediatamente (IDLE) e zera todas as referências.
 * @param servo Ponteiro para o Servo Drive.
 */
void Servo_EmergencyStop(ServoDrive_t* servo);

/**
 * @brief Verifica se o algoritmo ativo já concluiu seu objetivo.
 * @param servo Ponteiro para o Servo Drive.
 * @return true se o algoritmo reportou done (ou modo IDLE/VELOCITY).
 */
bool Servo_IsDone(const ServoDrive_t* servo);

/**
 * @brief Obtém o modo de operação atual.
 * @param servo Ponteiro para o Servo Drive.
 * @return ServoMode_t Modo atual.
 */
ServoMode_t Servo_GetMode(const ServoDrive_t* servo);

#endif  // SERVO_DRIVE_H
