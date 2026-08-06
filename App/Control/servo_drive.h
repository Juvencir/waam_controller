/**
 * @file servo_drive.h
 * @brief API unificada do Servo Drive — controle de motor como servo completo.
 * @details Compõe AxisKinematics (feedback), HBridge (atuador), PID (controle)
 *          e um planejador de trajetória trapezoidal interno em uma interface
 *          simples de alto nível com três modos de operação:
 *          - IDLE:      motor desligado.
 *          - VELOCITY:  controle direto de velocidade (jog).
 *          - MOVE:      move para posição alvo com perfil unificado.
 *
 *          A função Servo_Update() deve ser chamada na ISR do Timer de controle
 *          (1 kHz). Ela executa: leitura do encoder → atualização da cinemática →
 *          cálculo da lei de controle → saída PWM na Ponte H.
 */

#ifndef SERVO_DRIVE_H
#define SERVO_DRIVE_H

#include <stdbool.h>

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
    SERVO_MODE_MOVE,      /**< Movimento para posição alvo com perfil unificado */
} ServoMode_t;

/**
 * @brief Estado interno do planejador de trajetória (trapezoidal ou homing).
 */
typedef struct {
    float target_mm;       /**< Posição alvo (mm) */
    float start_pos_mm;    /**< Posição inicial no momento do start (mm) */
    float elapsed_s;       /**< Tempo decorrido desde o start (s) */
    float max_vel_mm_s;    /**< Velocidade máxima configurada (0 = homing) */
    float max_accel_mm_s2; /**< Aceleração máxima (0 = rampa default) */
    float max_decel_mm_s2; /**< Desaceleração máxima (0 = rampa default) */
    float accel_rate;      /**< Taxa de aceleração efetiva (mm/s²) */
    float decel_rate;      /**< Taxa de desaceleração efetiva (mm/s²) */
    float v_peak;          /**< Velocidade de pico real (≤ max_vel, mm/s) */
    float t_accel_end;     /**< Fim da fase de aceleração (s) */
    float t_decel_start;   /**< Início da fase de desaceleração (s) */
    float t_total;         /**< Duração total do perfil (s) */
    bool   active;         /**< true se o plano está ativo */
} MovePlan_t;

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
    ServoMode_t mode;             /**< Modo atual */
    float       velocity_sp_mm_s; /**< Setpoint do modo VELOCITY */

    // -- Planejador de trajetória --
    MovePlan_t plan;           /**< Estado da trajetória ativa (modo MOVE) */
    float      hold_target_mm; /**< Alvo para auto-hold após conclusão da trajetória */
    bool       move_done;      /**< true quando a trajetória concluiu (tempo ou tolerância) */

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
 * @brief Move o eixo para uma posição alvo com perfil de velocidade unificado.
 * @details Comportamento definido pelos parâmetros (0 = sem limite/sentinela):
 *
 *          | max_vel | max_accel | max_decel | Modo                                           |
 *          |---------|-----------|-----------|-------------------------------------------------|
 *          | 0       | 0         | 0         | **Homing**: P-controller, vai o mais rápido que o hardware permite |
 *          | >0      | 0         | 0         | **Velocidade constante**: trapezoidal com rampas default de 50 ms |
 *          | >0      | >0        | >0        | **Trapezoidal completo**: aceleração e desaceleração configuráveis |
 *
 *          Ao concluir a trajetória, o servo mantém hold na posição alvo
 *          automaticamente (P-controller com clamp de max_velocity).
 *
 * @param servo          Ponteiro para o Servo Drive.
 * @param target_mm      Posição alvo em mm.
 * @param max_vel_mm_s   Velocidade máxima em mm/s (0 = sem limite/homing).
 * @param max_accel_mm_s2 Aceleração máxima em mm/s² (0 = rampa default de 50 ms).
 * @param max_decel_mm_s2 Desaceleração máxima em mm/s² (0 = rampa default de 50 ms).
 */
void Servo_Move(ServoDrive_t* servo, float target_mm, float max_vel_mm_s, float max_accel_mm_s2,
                float max_decel_mm_s2);

// ============================================================================
// API DE CICLO DE CONTROLE (ISR)
// ============================================================================

/**
 * @brief Executa um ciclo completo de controle. CHAMAR NA ISR DO TIMER (1 kHz).
 * @details Ordem interna: leitura do encoder → atualização cinemática →
 *          avaliação do planejador ativo → malha de velocidade → saída PWM.
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
 * @brief Verifica se o movimento atual já concluiu seu objetivo.
 * @param servo Ponteiro para o Servo Drive.
 * @return true no modo IDLE; false no modo VELOCITY; no modo MOVE retorna move_done.
 */
bool Servo_IsDone(const ServoDrive_t* servo);

/**
 * @brief Obtém o modo de operação atual.
 * @param servo Ponteiro para o Servo Drive.
 * @return ServoMode_t Modo atual.
 */
ServoMode_t Servo_GetMode(const ServoDrive_t* servo);

#endif  // SERVO_DRIVE_H
