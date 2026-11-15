#include "chassis_control.h"
#include "motion_math.h"
#include <math.h>
#include <stddef.h>

static bool positive(float v) { return isfinite(v) && v > 0; }
void Chassis_Reset(ChassisControl *s)
{
    if (s == NULL) return;
    s->previous = (Mecanum_Velocity){0};
    for (unsigned i = 0; i < 4; ++i) PID_Reset(&s->wheel[i]);
}
bool Chassis_Init(ChassisControl *s, const ChassisConfig *c)
{
    if (s == NULL) return false;
    ChassisConfig copy = c != NULL ? *c : (ChassisConfig){0};
    *s = (ChassisControl){0};
    c = &copy;
    Mecanum_Wheels ignored;
    const Mecanum_Velocity zero = {0};
    if (!positive(c->max_translation) || !positive(c->max_rotation) ||
        !isfinite(c->spin_rate) || c->spin_rate == 0 || fabsf(c->spin_rate) > c->max_rotation ||
        !positive(c->follow_kp) || !positive(c->translation_accel) ||
        !positive(c->rotation_accel) || !positive(c->max_dt) ||
        !positive(c->geometry.max_motor_rpm) || !Mecanum_Inverse(&c->geometry, &zero, &ignored)) return false;
    for (unsigned i = 0; i < 4; ++i) {
        if (!PID_Init(&s->wheel[i], &c->wheel_pid[i]) || c->wheel_pid[i].output_limit > 16384 ||
            c->wheel_pid[i].max_dt < c->max_dt) return false;
    }
    s->config = copy;
    s->configured = true;
    return true;
}
bool Chassis_Step(ChassisControl *s, const RobotCommand *cmd, float yaw,
                  const float rpm[4], const bool fresh[4], bool enabled,
                  float dt, ChassisOutput *out)
{
    if (out != NULL) *out = (ChassisOutput){0};
    if (s == NULL) return false;
    if (out == NULL || !s->configured || !enabled || !Command_Valid(cmd) ||
        !cmd->remote_valid || !cmd->arm || cmd->estop || !isfinite(yaw) ||
        rpm == NULL || fresh == NULL || !positive(dt) || dt > s->config.max_dt) goto stop;
    for (unsigned i = 0; i < 4; ++i)
        if (!fresh[i] || !isfinite(rpm[i])) goto stop;
    const ChassisConfig *c = &s->config;
    float x = cmd->forward, y = cmd->left;
    const float norm = hypotf(x, y);
    if (norm > 1) { x /= norm; y /= norm; }
    Mecanum_Velocity v = {0};
    if (!Motion_Rotate(x * c->max_translation, y * c->max_translation, yaw, &v.vx, &v.vy)) goto stop;
    /* Positive yaw means chassis must rotate CCW to follow the gimbal.
     * A wrapped proportional loop avoids derivative kick at +/-pi. */
    const float follow = c->follow_kp * Motion_Wrap(yaw);
    if (!isfinite(follow)) goto stop;
    v.wz = cmd->mode == MODE_FOLLOW ? Motion_Clamp(follow, c->max_rotation) :
        (cmd->mode == MODE_SPIN ? c->spin_rate : 0);
    const float dx = v.vx - s->previous.vx, dy = v.vy - s->previous.vy;
    const float distance = hypotf(dx, dy), step = c->translation_accel * dt;
    if (!isfinite(distance) || !isfinite(step) || !isfinite(c->rotation_accel * dt)) goto stop;
    if (distance > step) {
        v.vx = s->previous.vx + dx / distance * step;
        v.vy = s->previous.vy + dy / distance * step;
    }
    v.wz = s->previous.wz + Motion_Clamp(v.wz - s->previous.wz, c->rotation_accel * dt);
    ChassisOutput result = {.velocity = v};
    if (!Mecanum_Inverse(&c->geometry, &v, &result.wheels)) goto stop;
    for (unsigned i = 0; i < 4; ++i) {
        float current;
        if (!PID_Update(&s->wheel[i], result.wheels.motor_rpm[i], rpm[i], dt, &current)) goto stop;
        result.current[i] = (int16_t)current;
    }
    s->previous = v;
    *out = result;
    return true;
stop:
    Chassis_Reset(s);
    return false;
}
