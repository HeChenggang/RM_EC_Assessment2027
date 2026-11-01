#include "pid.h"
#include <math.h>
#include <stddef.h>

static bool config_valid(const PID_Config *c)
{
    return c != NULL && isfinite(c->kp) && c->kp >= 0.0f &&
        isfinite(c->ki) && c->ki >= 0.0f && isfinite(c->kd) && c->kd >= 0.0f &&
        isfinite(c->output_limit) && c->output_limit > 0.0f &&
        isfinite(c->integral_limit) && c->integral_limit >= 0.0f &&
        isfinite(c->derivative_tau) && c->derivative_tau >= 0.0f &&
        isfinite(c->max_dt) && c->max_dt > 0.0f;
}

static float clamp(float value, float limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

void PID_Reset(PID_Controller *pid)
{
    if (pid == NULL) return;
    pid->p = pid->i = pid->d = pid->output = 0.0f;
    pid->previous_measurement = pid->filtered_derivative = 0.0f;
    pid->has_previous_measurement = false;
}

bool PID_Init(PID_Controller *pid, const PID_Config *config)
{
    if (pid == NULL) return false;
    /* Copy before clearing so PID_Init(&pid, &pid.config) is also supported. */
    const bool ok = config_valid(config);
    const PID_Config copy = ok ? *config : (PID_Config){0};
    pid->config = copy;
    pid->configured = ok;
    PID_Reset(pid);
    return ok;
}

bool PID_Update(PID_Controller *pid, float target, float measurement,
                float dt_seconds, float *output)
{
    if (output != NULL) *output = 0.0f;
    if (pid == NULL) return false;
    if (output == NULL || !pid->configured || !config_valid(&pid->config) ||
        !isfinite(target) || !isfinite(measurement) || !isfinite(dt_seconds) ||
        dt_seconds <= 0.0f || dt_seconds > pid->config.max_dt) goto invalid;

    const PID_Config *c = &pid->config;
    const float error = target - measurement;
    const float p = c->kp * error;
    const float integral_step = c->ki * error * dt_seconds;
    const float integral_sum = pid->i + integral_step;
    if (!isfinite(error) || !isfinite(p) || !isfinite(integral_step) ||
        !isfinite(integral_sum)) goto invalid;
    float integral = clamp(integral_sum, c->integral_limit);

    float derivative = 0.0f;
    if (pid->has_previous_measurement && c->kd > 0.0f) {
        const float raw = -(measurement - pid->previous_measurement) / dt_seconds;
        const float denominator = c->derivative_tau + dt_seconds;
        if (!isfinite(raw) || !isfinite(denominator)) goto invalid;
        const float alpha = dt_seconds / denominator;
        derivative = (1.0f - alpha) * pid->filtered_derivative + alpha * raw;
    }
    const float d = c->kd * derivative;
    const float trial = p + integral + d;
    if (!isfinite(derivative) || !isfinite(d) || !isfinite(trial)) goto invalid;

    /* Reject only increments that push further into saturation; allow unwind. */
    const float delta_i = integral - pid->i;
    if ((trial > c->output_limit && delta_i > 0.0f) ||
        (trial < -c->output_limit && delta_i < 0.0f)) integral = pid->i;
    const float sum = p + integral + d;
    if (!isfinite(sum)) goto invalid;
    pid->p = p;
    pid->i = integral;
    pid->d = d;
    pid->output = clamp(sum, c->output_limit);
    pid->filtered_derivative = derivative;
    pid->previous_measurement = measurement;
    pid->has_previous_measurement = true;
    *output = pid->output;
    return true;

invalid:
    PID_Reset(pid);
    return false;
}
