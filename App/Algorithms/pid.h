/**
 * @file pid.h
 * @brief Algoritmo de controle PID discreto em C puro (independente de hardware).
 */

#ifndef PID_H
#define PID_H

/**
 * @brief Estrutura de controle e estado interno do algoritmo PID.
 */
typedef struct {
    // Ganhos do controlador
    float kp;
    float ki;
    float kd;

    // Limites de saída (Saturação e Anti-windup)
    float out_min;
    float out_max;

    // Coeficiente do filtro passa-baixas derivativo (0.0f = sem filtro, 0.99f = filtragem máxima)
    float d_alpha;

    // Variáveis de estado interno
    float integrator;
    float prev_error;
    float prev_derivative;
} PID_t;

/**
 * @brief Inicializa a estrutura do controlador PID com seus ganhos e limites.
 * @param pid Ponteiro para a estrutura PID.
 * @param kp Ganho Proporcional.
 * @param ki Ganho Integrativo.
 * @param kd Ganho Derivativo.
 * @param out_min Limite mínimo do sinal de saída (ex: -1.0f).
 * @param out_max Limite máximo do sinal de saída (ex: 1.0f).
 */
void PID_Init(PID_t* pid, float kp, float ki, float kd, float out_min, float out_max);

/**
 * @brief Configura o coeficiente do filtro passa-baixas de 1ª ordem do termo derivativo.
 * @param pid Ponteiro para a estrutura PID.
 * @param alpha Fator de suavização entre 0.0f (desativado) e 0.99f (filtro pesado).
 */
void PID_SetDerivativeFilter(PID_t* pid, float alpha);

/**
 * @brief Reseta os estados internos acumulados (integrador, histórico de erros e derivadas).
 * @param pid Ponteiro para a estrutura PID.
 */
void PID_Reset(PID_t* pid);

/**
 * @brief Executa o cálculo da malha PID em tempo discreto.
 * @param pid Ponteiro para a estrutura PID.
 * @param target Valor desejado de referência.
 * @param feedback Valor medido atual.
 * @param dt Intervalo de tempo entre amostragens em segundos (ex: 0.001f para ISR de 1 kHz).
 * @return Sinal de controle calculado e saturado.
 */
float PID_Compute(PID_t* pid, float target, float feedback, float dt);

#endif  // PID_H