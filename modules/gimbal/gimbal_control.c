#include "gimbal_control.h"
#include "motion_math.h"
#include <math.h>
#include <stddef.h>

void Gimbal_Reset(GimbalControl *s)
{
    if (s == NULL) return;
    s->tracking = false;
    s->target[0] = s->target[1] = 0;
    PID_Reset(&s->rate[0]); PID_Reset(&s->rate[1]);
}
bool Gimbal_Init(GimbalControl *s, const GimbalConfig *config)
{
    if (s == NULL) return false;
    GimbalConfig c = config != NULL ? *config : (GimbalConfig){0};
    *s = (GimbalControl){0};
    if (!isfinite(c.pitch_min) || !isfinite(c.pitch_max) || c.pitch_min >= c.pitch_max ||
        c.pitch_min <= -MOTION_PI / 2 || c.pitch_max >= MOTION_PI / 2 ||
        !isfinite(c.max_dt) || c.max_dt <= 0 || !isfinite(c.yaw_command_rate) ||
        c.yaw_command_rate <= 0 || !isfinite(c.pitch_command_rate) || c.pitch_command_rate <= 0)
        return false;
    for (unsigned i = 0; i < 2; ++i) {
        if ((c.space[i] != GIMBAL_JOINT && c.space[i] != GIMBAL_WORLD) ||
            !isfinite(c.angle_kp[i]) || c.angle_kp[i] <= 0 || !isfinite(c.max_rate[i]) ||
            c.max_rate[i] <= 0 || c.rate_pid[i].max_dt < c.max_dt ||
            !PID_Init(&s->rate[i], &c.rate_pid[i])) return false;
    }
    s->config = c; s->configured = true;
    return true;
}
bool Gimbal_Step(GimbalControl *s, const RobotCommand *cmd,
                 const GimbalFeedback *f, bool enabled, float dt, GimbalOutput *out)
{
    if (out != NULL) *out = (GimbalOutput){0};
    if (s == NULL) return false;
    if (!s->configured || out == NULL || !enabled || !Command_Valid(cmd) ||
        !cmd->remote_valid || !cmd->arm || cmd->estop || f == NULL || !f->joint_fresh ||
        !isfinite(dt) || dt <= 0 || dt > s->config.max_dt) goto stop;
    const GimbalConfig *c = &s->config;
    if (!isfinite(f->joint[1]) || f->joint[1] < c->pitch_min || f->joint[1] > c->pitch_max) goto stop;
    float measured[2], rate[2];
    for (unsigned i = 0; i < 2; ++i) {
        if (!isfinite(f->joint[i])) goto stop;
        const bool world = c->space[i] == GIMBAL_WORLD;
        if (world && !f->world_fresh) goto stop;
        measured[i] = world ? f->world[i] : f->joint[i];
        rate[i] = world ? f->world_rate[i] : f->joint_rate[i];
        if (!isfinite(measured[i]) || !isfinite(rate[i])) goto stop;
    }
    if (!s->tracking) {
        s->target[0] = Motion_Wrap(measured[0]); s->target[1] = measured[1];
        s->tracking = true;
    }
    s->target[0] = Motion_Wrap(s->target[0] + cmd->yaw * c->yaw_command_rate * dt);
    s->target[1] += cmd->pitch * c->pitch_command_rate * dt;
    /* Local aligned-axis relationship: change in world pitch = change in
     * joint pitch at this instant. Clamp the corresponding joint target. */
    float joint_target = f->joint[1] + (s->target[1] - measured[1]);
    if (!isfinite(joint_target)) goto stop;
    if (joint_target > c->pitch_max) joint_target = c->pitch_max;
    if (joint_target < c->pitch_min) joint_target = c->pitch_min;
    s->target[1] = measured[1] + (joint_target - f->joint[1]);
    GimbalOutput result = {0};
    for (unsigned i = 0; i < 2; ++i) {
        const float error = i == 0 ? Motion_Wrap(s->target[i] - measured[i]) : s->target[i] - measured[i];
        result.target[i] = s->target[i];
        const float requested = c->angle_kp[i] * error;
        if (!isfinite(requested)) goto stop;
        result.rate[i] = Motion_Clamp(requested, c->max_rate[i]);
        if (!PID_Update(&s->rate[i], result.rate[i], rate[i], dt, &result.torque[i])) goto stop;
    }
    if ((f->joint[1] >= c->pitch_max && result.torque[1] > 0) ||
        (f->joint[1] <= c->pitch_min && result.torque[1] < 0)) {
        result.torque[1] = 0;
        PID_Reset(&s->rate[1]);
    }
    *out = result;
    return true;
stop:
    Gimbal_Reset(s);
    return false;
}
