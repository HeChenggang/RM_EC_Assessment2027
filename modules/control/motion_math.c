#include "motion_math.h"
#include <math.h>
#include <stddef.h>

float Motion_Wrap(float radians)
{
    if (!isfinite(radians)) return NAN;
    float value = fmodf(radians, MOTION_TAU);
    if (value >= MOTION_PI) value -= MOTION_TAU;
    if (value < -MOTION_PI) value += MOTION_TAU;
    return value;
}

float Motion_Clamp(float value, float limit)
{
    return value > limit ? limit : (value < -limit ? -limit : value);
}

bool Motion_Rotate(float x, float y, float yaw, float *out_x, float *out_y)
{
    if (out_x == NULL || out_y == NULL || out_x == out_y) return false;
    *out_x = *out_y = 0;
    if (!isfinite(x) || !isfinite(y) || !isfinite(yaw)) return false;
    const float c = cosf(yaw), s = sinf(yaw);
    const float a = c * x - s * y, b = s * x + c * y;
    if (!isfinite(a) || !isfinite(b)) return false;
    *out_x = a; *out_y = b;
    return true;
}

bool AngleTracker_Update(AngleTracker *s, float wrapped, uint32_t now,
                         float max_rate, uint32_t timeout, float *out)
{
    if (out != NULL) *out = 0;
    if (s == NULL) return false;
    if (out == NULL || !isfinite(wrapped) || !isfinite(max_rate) || max_rate <= 0 ||
        timeout == 0 || timeout > 0x7fffffffU) goto invalid;
    wrapped = Motion_Wrap(wrapped);
    if (!s->initialized) {
        *s = (AngleTracker){wrapped, wrapped, now, true};
    } else {
        const uint32_t elapsed = now - s->last_ms;
        const float bound = max_rate * ((float)elapsed * 0.001f);
        const float delta = Motion_Wrap(wrapped - s->previous);
        if (elapsed == 0 || elapsed >= timeout || !isfinite(bound) ||
            bound >= MOTION_PI || fabsf(delta) > bound + 1e-5f ||
            !isfinite(s->continuous + delta)) goto invalid;
        s->continuous += delta;
        s->previous = wrapped;
        s->last_ms = now;
    }
    *out = s->continuous;
    return true;
invalid:
    *s = (AngleTracker){0};
    return false;
}
