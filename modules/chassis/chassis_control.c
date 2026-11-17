#include "chassis_control.h"
#include "motion_math.h"
#include <math.h>
#include <stddef.h>

static bool positive(float v) { return isfinite(v) && v > 0; }
void Chassis_Reset(ChassisControl *s)
{
    if (s == NULL) return;
    s->previous = (Mecanum_Velocity){0};
    s->previous_yaw = 0.0f;
    for (unsigned i = 0; i < 4; ++i) {
        PID_Reset(&s->wheel[i]);
        s->previous_target_rpm[i] = 0.0f;
    }
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
        !isfinite(c->spin_max_motor_rpm) || c->spin_max_motor_rpm < 0 ||
        !positive(c->follow_kp) || !positive(c->translation_accel) ||
        !positive(c->rotation_accel) || !positive(c->follow_rotation_accel) || !positive(c->max_dt) ||
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
    const bool spin = cmd->mode == MODE_SPIN;
    const float wrapped_yaw = Motion_Wrap(yaw);
    float x = cmd->forward, y = cmd->left;
    const float norm = hypotf(x, y);
    if (norm > 1) { x /= norm; y /= norm; }
    Mecanum_Velocity v = {0};
    if (!Motion_Rotate(x * c->max_translation, y * c->max_translation, yaw, &v.vx, &v.vy)) goto stop;
    float rotation_accel = c->rotation_accel;
    if (cmd->mode == MODE_FOLLOW) {
        const float error = wrapped_yaw;
        const float follow = c->follow_kp * error;
        if (!isfinite(follow)) goto stop;
        rotation_accel = c->follow_rotation_accel;
        /* Leave ramp headroom for braking and reserve 200 ms for feedback
         * and wheel response. Solve distance = speed * delay + speed^2 / (2 * decel). */
        const float decel = 0.75f * rotation_accel;
        const float lead = 0.20f * decel;
        const float brake_limit = sqrtf(lead * lead + 2.0f * decel * fabsf(error)) - lead;
        v.wz = Motion_Clamp(follow, fminf(c->max_rotation, fmaxf(0.0f, brake_limit)));
    } else if (spin) {
        v.wz = c->spin_rate;
    }
    float previous_vx = s->previous.vx, previous_vy = s->previous.vy;
    /* Express the previous translation in the current chassis frame before
     * comparing velocities. A fixed gimbal-frame command rotates with yaw. */
    if (spin && !Motion_Rotate(previous_vx, previous_vy, wrapped_yaw - s->previous_yaw,
                              &previous_vx, &previous_vy)) goto stop;
    const float dx = v.vx - previous_vx, dy = v.vy - previous_vy;
    const float distance = hypotf(dx, dy), step = c->translation_accel * dt;
    const float rotation_step = rotation_accel * dt;
    if (!isfinite(distance) || !isfinite(step) || !isfinite(rotation_step)) goto stop;
    if (distance > step) {
        v.vx = previous_vx + dx / distance * step;
        v.vy = previous_vy + dy / distance * step;
    }
    v.wz = s->previous.wz + Motion_Clamp(v.wz - s->previous.wz, rotation_step);
    Mecanum_Config geometry = c->geometry;
    if (spin) {
        if (c->spin_max_motor_rpm > 0.0f) geometry.max_motor_rpm = c->spin_max_motor_rpm;
        if (!Mecanum_PrioritizeRotation(&geometry, &v)) goto stop;
    }
    ChassisOutput result = {.velocity = v};
    if (!Mecanum_Inverse(&geometry, &v, &result.wheels)) goto stop;
    for (unsigned i = 0; i < 4; ++i) {
        const float target = result.wheels.motor_rpm[i];
        const float previous_target = s->previous_target_rpm[i];
        if ((target < 0.0f && previous_target > 0.0f) ||
            (target > 0.0f && previous_target < 0.0f)) {
            /* A reference can cross zero between ticks. Drop the old drive bias. */
            s->wheel[i].i = 0.0f;
        }
        float current;
        if (!PID_Update(&s->wheel[i], target, rpm[i], dt, &current)) goto stop;
        if (target == 0.0f) {
            /* Brake proportionally at zero target without accumulating bias. */
            current = Motion_Clamp(s->wheel[i].p, c->wheel_pid[i].output_limit);
            PID_Reset(&s->wheel[i]);
        }
        result.current[i] = (int16_t)current;
        s->previous_target_rpm[i] = target;
    }
    s->previous = v;
    s->previous_yaw = wrapped_yaw;
    *out = result;
    return true;
stop:
    Chassis_Reset(s);
    return false;
}
