#include "attitude.h"
#include "motion_math.h"
#include <math.h>
#include <stddef.h>

static bool rotation_valid(const float r[3][3])
{
    for (unsigned i = 0; i < 3; ++i) {
        for (unsigned j = 0; j < 3; ++j) {
            if (!isfinite(r[i][j])) return false;
            float dot = 0;
            for (unsigned k = 0; k < 3; ++k) dot += r[i][k] * r[j][k];
            if (!isfinite(dot) || fabsf(dot - (i == j ? 1.0f : 0.0f)) > 0.001f) return false;
        }
    }
    const float det = r[0][0]*(r[1][1]*r[2][2]-r[1][2]*r[2][1]) -
        r[0][1]*(r[1][0]*r[2][2]-r[1][2]*r[2][0]) + r[0][2]*(r[1][0]*r[2][1]-r[1][1]*r[2][0]);
    return isfinite(det) && fabsf(det - 1) < 0.001f;
}
void Attitude_Reset(Attitude *s)
{
    if (s == NULL) return;
    s->q[0] = 1; s->q[1] = s->q[2] = s->q[3] = 0;
    s->initialized = false;
}
bool Attitude_Init(Attitude *s, const AttitudeConfig *config)
{
    if (s == NULL) return false;
    const AttitudeConfig c = config != NULL ? *config : (AttitudeConfig){0};
    *s = (Attitude){0};
    Attitude_Reset(s);
    if (!rotation_valid(c.sensor_to_body) || !isfinite(c.gravity) || c.gravity <= 0 ||
        !isfinite(c.accel_tolerance) || c.accel_tolerance <= 0 || c.accel_tolerance >= 1 ||
        !isfinite(c.correction_gain) || c.correction_gain < 0 ||
        !isfinite(c.max_dt) || c.max_dt <= 0 || !isfinite(c.max_gyro) || c.max_gyro <= 0) return false;
    s->config = c; s->configured = true;
    return true;
}
bool Attitude_Update(Attitude *s, const float accel[3], const float gyro[3], float dt, AttitudeOutput *out)
{
    if (out != NULL) *out = (AttitudeOutput){0};
    if (s == NULL) return false;
    if (!s->configured || accel == NULL || gyro == NULL || out == NULL ||
        !isfinite(dt) || dt <= 0 || dt > s->config.max_dt) goto invalid;
    float a[3] = {0}, w[3] = {0};
    for (unsigned i = 0; i < 3; ++i) {
        if (!isfinite(accel[i]) || !isfinite(gyro[i])) goto invalid;
        for (unsigned j = 0; j < 3; ++j) {
            a[i] += s->config.sensor_to_body[i][j] * accel[j];
            w[i] += s->config.sensor_to_body[i][j] * gyro[j];
        }
        if (!isfinite(a[i]) || !isfinite(w[i])) goto invalid;
    }
    const float gyro_norm = hypotf(hypotf(w[0], w[1]), w[2]);
    if (!isfinite(gyro_norm) || gyro_norm > s->config.max_gyro) goto invalid;
    const float norm = hypotf(hypotf(a[0], a[1]), a[2]);
    const bool use_accel = isfinite(norm) && norm > 0 &&
        fabsf(norm / s->config.gravity - 1) <= s->config.accel_tolerance;
    if (use_accel) for (unsigned i = 0; i < 3; ++i) a[i] /= norm;
    if (!s->initialized) {
        if (!use_accel) goto invalid;
        const float roll = atan2f(a[1], a[2]);
        const float pitch = atan2f(-a[0], hypotf(a[1], a[2]));
        const float cr = cosf(roll/2), sr = sinf(roll/2), cp = cosf(pitch/2), sp = sinf(pitch/2);
        s->q[0] = cr*cp; s->q[1] = sr*cp; s->q[2] = cr*sp; s->q[3] = -sr*sp;
        s->initialized = true;
    }
    const float *q = s->q;
    float corrected[3] = {w[0], w[1], w[2]};
    if (use_accel) {
        /* Predicted world-up in body coordinates, R(q)^T [0,0,1].
         * measured-up cross predicted-up gives body correction angular rate. */
        const float up[3] = {2*(q[1]*q[3]-q[0]*q[2]), 2*(q[2]*q[3]+q[0]*q[1]),
                            1-2*(q[1]*q[1]+q[2]*q[2])};
        for (unsigned i = 0; i < 3; ++i) {
            const unsigned j = (i+1)%3, k = (i+2)%3;
            corrected[i] += s->config.correction_gain * (a[j]*up[k]-a[k]*up[j]);
        }
    }
    const float speed = hypotf(hypotf(corrected[0], corrected[1]), corrected[2]);
    const float angle = speed*dt;
    if (!isfinite(angle) || angle >= MOTION_PI) goto invalid;
    const float k = speed > 1e-6f ? sinf(angle/2)/speed : dt/2;
    const float d[4] = {cosf(angle/2), corrected[0]*k, corrected[1]*k, corrected[2]*k};
    float next[4] = {q[0]*d[0]-q[1]*d[1]-q[2]*d[2]-q[3]*d[3],
                    q[0]*d[1]+q[1]*d[0]+q[2]*d[3]-q[3]*d[2],
                    q[0]*d[2]-q[1]*d[3]+q[2]*d[0]+q[3]*d[1],
                    q[0]*d[3]+q[1]*d[2]-q[2]*d[1]+q[3]*d[0]};
    const float length = sqrtf(next[0]*next[0]+next[1]*next[1]+next[2]*next[2]+next[3]*next[3]);
    if (!isfinite(length) || length < 1e-6f) goto invalid;
    for (unsigned i = 0; i < 4; ++i) s->q[i] = next[i]/length;
    AttitudeOutput result = {.accel_used = use_accel};
    result.roll = atan2f(2*(q[0]*q[1]+q[2]*q[3]), 1-2*(q[1]*q[1]+q[2]*q[2]));
    result.pitch = asinf(Motion_Clamp(2*(q[0]*q[2]-q[3]*q[1]), 1));
    result.yaw = atan2f(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]*q[2]+q[3]*q[3]));
    const float cp = cosf(result.pitch), sr = sinf(result.roll), cr = cosf(result.roll);
    if (fabsf(cp) < 0.05f) goto invalid;
    result.euler_rate[2] = (sr*w[1]+cr*w[2])/cp;
    result.euler_rate[1] = cr*w[1]-sr*w[2];
    result.euler_rate[0] = w[0]+sinf(result.pitch)*result.euler_rate[2];
    *out = result;
    return true;
invalid:
    Attitude_Reset(s);
    return false;
}
