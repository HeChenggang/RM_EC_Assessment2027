#ifndef MOTION_MATH_H
#define MOTION_MATH_H
#include <stdbool.h>
#include <stdint.h>
#define MOTION_PI 3.14159265358979323846f
#define MOTION_TAU (2.0f * MOTION_PI)

float Motion_Wrap(float radians); /* [-pi, pi), NAN on invalid input */
float Motion_Clamp(float value, float limit);
bool Motion_Rotate(float x, float y, float yaw, float *out_x, float *out_y);

typedef struct {
    float previous, continuous;
    uint32_t last_ms;
    bool initialized;
} AngleTracker;
/* max_rate*elapsed must be < pi to avoid ambiguous turn counting. On failure
 * invalidate the tracker; the caller must disarm and explicitly reacquire. */
bool AngleTracker_Update(AngleTracker *state, float wrapped, uint32_t now_ms,
                         float max_rate, uint32_t timeout_ms, float *continuous);
#endif
