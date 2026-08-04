#include "pid.h"

void PID_Init(PID_t* pid, float kp, float ki, float kd, float out_min, float out_max) {
    pid->kp      = kp;
    pid->ki      = ki;
    pid->kd      = kd;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->d_alpha = 0.0f;  // Padrão: sem filtro no termo derivativo

    PID_Reset(pid);
}

void PID_SetDerivativeFilter(PID_t* pid, float alpha) {
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 0.99f) alpha = 0.99f;
    pid->d_alpha = alpha;
}

void PID_Reset(PID_t* pid) {
    pid->integrator      = 0.0f;
    pid->prev_error      = 0.0f;
    pid->prev_derivative = 0.0f;
}

float PID_Compute(PID_t* pid, float setpoint, float measurement, float dt) {
    // Proteção contra divisão por zero ou dt inválido
    if (dt <= 0.0f) {
        return 0.0f;
    }

    // 1. Cálculo do Erro de Malha
    float error = setpoint - measurement;

    // 2. Termo Proporcional
    float p_term = pid->kp * error;

    // 3. Termo Integrativo com trava de Saturação (Anti-Windup)
    pid->integrator += pid->ki * error * dt;

    if (pid->integrator > pid->out_max) {
        pid->integrator = pid->out_max;
    } else if (pid->integrator < pid->out_min) {
        pid->integrator = pid->out_min;
    }

    // 4. Termo Derivativo com Filtro Passa-Baixas de 1ª Ordem
    float raw_derivative = (error - pid->prev_error) / dt;
    float d_term_filtered =
        (pid->d_alpha * pid->prev_derivative) + ((1.0f - pid->d_alpha) * raw_derivative);

    // Atualiza estados para o próximo ciclo
    pid->prev_error      = error;
    pid->prev_derivative = d_term_filtered;

    float d_term = pid->kd * d_term_filtered;

    // 5. Soma dos Termos de Controle
    float output = p_term + pid->integrator + d_term;

    // 6. Saturação Final do Sinal de Controle
    if (output > pid->out_max) {
        output = pid->out_max;
    } else if (output < pid->out_min) {
        output = pid->out_min;
    }

    return output;
}