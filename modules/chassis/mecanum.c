#include "mecanum.h"
#include <math.h>
#include <stddef.h>

#define TWO_PI 6.2831853071795864769f

const Mecanum_Config MECANUM_DEFAULT_CONFIG = {
    .track_m = 0.45f, .wheelbase_m = 0.55f, .radius_m = 0.07786f,
    .reduction = 14.0f, .motor_direction = {1, 1, 1, 1}, .max_motor_rpm = 0.0f
};

static bool factors(const Mecanum_Config *c, float *k, float *rpm_per_mps)
{
    if (c == NULL || !isfinite(c->track_m) || c->track_m <= 0 ||
        !isfinite(c->wheelbase_m) || c->wheelbase_m <= 0 ||
        !isfinite(c->radius_m) || c->radius_m <= 0 ||
        !isfinite(c->reduction) || c->reduction <= 0 ||
        !isfinite(c->max_motor_rpm) || c->max_motor_rpm < 0) return false;
    for (unsigned i = 0; i < 4; ++i)
        if (c->motor_direction[i] != 1 && c->motor_direction[i] != -1) return false;
    *k = c->wheelbase_m * 0.5f + c->track_m * 0.5f;
    *rpm_per_mps = 60.0f / (TWO_PI * c->radius_m) * c->reduction;
    return isfinite(*k) && *k > 0 && isfinite(*rpm_per_mps) && *rpm_per_mps > 0;
}

bool Mecanum_PrioritizeRotation(const Mecanum_Config *c, Mecanum_Velocity *v)
{
    float k, factor;
    if (!factors(c, &k, &factor) || v == NULL ||
        !isfinite(v->vx) || !isfinite(v->vy) || !isfinite(v->wz)) return false;
    if (c->max_motor_rpm == 0.0f) return true;

    /* Each translation term (vx +/- vy) is paired with both rotation signs,
     * so the peak wheel speed is |vx| + |vy| + k*|wz| before conversion to rpm. */
    const float wheel_limit = c->max_motor_rpm / factor;
    const float translation = fabsf(v->vx) + fabsf(v->vy);
    if (!isfinite(wheel_limit) || !isfinite(translation)) return false;
    const float rotation = k * fabsf(v->wz);
    const float remaining = fmaxf(0.0f, wheel_limit - rotation);
    if (translation > remaining) {
        const float scale = remaining / translation;
        v->vx *= scale;
        v->vy *= scale;
    }
    if (rotation > wheel_limit) v->wz = copysignf(wheel_limit / k, v->wz);
    return true;
}

bool Mecanum_Inverse(const Mecanum_Config *c, const Mecanum_Velocity *v,
                     Mecanum_Wheels *out)
{
    if (out == NULL) return false;
    *out = (Mecanum_Wheels){0};
    float k, factor;
    if (!factors(c, &k, &factor) || v == NULL ||
        !isfinite(v->vx) || !isfinite(v->vy) || !isfinite(v->wz)) return false;
    const float rotation = k * v->wz;
    float wheel[4] = {
        v->vx - v->vy - rotation, /* FL */
        v->vx + v->vy - rotation, /* RL */
        v->vx - v->vy + rotation, /* RR */
        v->vx + v->vy + rotation  /* FR */
    };
    float peak = 0;
    for (unsigned i = 0; i < 4; ++i) {
        wheel[i] *= factor * c->motor_direction[i];
        if (!isfinite(wheel[i])) return false;
        if (fabsf(wheel[i]) > peak) peak = fabsf(wheel[i]);
    }
    const float scale = c->max_motor_rpm > 0 && peak > c->max_motor_rpm
        ? c->max_motor_rpm / peak : 1.0f;
    for (unsigned i = 0; i < 4; ++i) out->motor_rpm[i] = wheel[i] * scale;
    out->scale = scale;
    return true;
}

bool Mecanum_Forward(const Mecanum_Config *c, const float motor_rpm[4],
                     Mecanum_Velocity *out)
{
    if (out == NULL) return false;
    *out = (Mecanum_Velocity){0};
    float k, factor;
    if (!factors(c, &k, &factor) || motor_rpm == NULL) return false;
    float s[4];
    for (unsigned i = 0; i < 4; ++i) {
        if (!isfinite(motor_rpm[i])) return false;
        /* Quarter terms keep the summation from overflowing unnecessarily. */
        s[i] = motor_rpm[i] / factor * c->motor_direction[i] * 0.25f;
        if (!isfinite(s[i])) return false;
    }
    const Mecanum_Velocity v = {
        .vx = s[0] + s[1] + s[2] + s[3],
        .vy = -s[0] + s[1] - s[2] + s[3],
        .wz = (-s[0] - s[1] + s[2] + s[3]) / k
    };
    if (!isfinite(v.vx) || !isfinite(v.vy) || !isfinite(v.wz)) return false;
    *out = v;
    return true;
}
