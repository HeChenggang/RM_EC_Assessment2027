#ifndef CONTROL_PID_H
#define CONTROL_PID_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp, ki, kd;       /* Continuous-time gains; dt is in seconds. */
    float output_limit;     /* Symmetric output bound, strictly positive. */
    float integral_limit;   /* Bound on I contribution (output units), >= 0. */
    float derivative_tau;   /* Measurement-derivative low-pass time constant, seconds. */
    float max_dt;           /* Reject stale control intervals; strictly positive. */
} PID_Config;

typedef struct {
    PID_Config config;
    float p, i, d, output;  /* Diagnostic terms from the last successful update. */
    float previous_measurement;
    float filtered_derivative;
    bool has_previous_measurement;
    bool configured;
} PID_Controller;

/* Zero-initialize before first use. Init rejects non-finite/negative gains,
 * invalid limits, and disables the controller on failure. No allocation/HAL.
 * Each loop needs its own instance; serialize access. Do not modify fields
 * directly: reinitialize to change gains (this also resets history). */
bool PID_Init(PID_Controller *pid, const PID_Config *config);
/* Clear history/output but retain configuration. Use on stop, offline or rearm. */
void PID_Reset(PID_Controller *pid);
/* Position-form PID, error = target - measurement. D is on measurement to
 * avoid setpoint kick; first sample has D=0. Conditional integration prevents
 * further windup while saturated. Invalid input/timestep/numeric overflow:
 * return false, reset history, write 0 to output (when non-NULL).
 * Caller must enforce remote validity, motor feedback freshness and arming. */
bool PID_Update(PID_Controller *pid, float target, float measurement,
                float dt_seconds, float *output);
/* Additional output contribution participates in BOTH saturation and
 * anti-windup. output includes it; diagnostic p/i/d remain PID-only. */
bool PID_UpdateWithFeedforward(PID_Controller *pid, float target, float measurement,
                               float feedforward, float dt_seconds, float *output);

#ifdef __cplusplus
}
#endif
#endif
