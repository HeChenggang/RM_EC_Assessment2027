#include "gimbal_control.h"
#include "motion_math.h"
#include <math.h>
#include <stddef.h>

void Gimbal_Reset(GimbalControl *s)
{
    if (s == NULL) return;
    s->tracking = false;
    s->yaw_input_direction = 0;
    s->yaw_previous_rate_limit = 0.0f;
    s->yaw_world = false;
    s->yaw_chassis_rate_valid = false;
    s->yaw_chassis_rate = 0;
    s->spin_elapsed = s->yaw_last = s->yaw_travel = 0;
    s->spin_limit = GIMBAL_SPIN_WITHIN_LIMITS;
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
        !isfinite(c.yaw_hold_max_rate) || c.yaw_hold_max_rate < 0 ||
        !isfinite(c.yaw_hold_angle_kp) || c.yaw_hold_angle_kp < 0 ||
        !isfinite(c.yaw_hold_integral_limit) || c.yaw_hold_integral_limit < 0 ||
        !isfinite(c.yaw_joint_rate_tau) || c.yaw_joint_rate_tau < 0 ||
        !isfinite(c.pitch_hold_kp) || c.pitch_hold_kp < 0 ||
        !isfinite(c.max_dt) || c.max_dt <= 0 || !isfinite(c.yaw_command_rate) ||
        c.yaw_command_rate <= 0 || !isfinite(c.pitch_command_rate) || c.pitch_command_rate <= 0 ||
        !isfinite(c.spin_max_time) || c.spin_max_time < 0 ||
        !isfinite(c.spin_max_travel) || c.spin_max_travel < 0)
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
    const bool spin = cmd->mode == MODE_SPIN;
    const bool yaw_world = spin || cmd->mode == MODE_FOLLOW || c->space[0] == GIMBAL_WORLD;
    if (!isfinite(f->joint[1]) || f->joint[1] < c->pitch_min || f->joint[1] > c->pitch_max) goto stop;
    float measured[2], rate[2];
    for (unsigned i = 0; i < 2; ++i) {
        if (!isfinite(f->joint[i])) goto stop;
        const bool world = i == 0 ? yaw_world : c->space[i] == GIMBAL_WORLD;
        if (world && !f->world_fresh) goto stop;
        measured[i] = world ? f->world[i] : f->joint[i];
        rate[i] = world ? f->world_rate[i] : f->joint_rate[i];
        if (!isfinite(measured[i]) || !isfinite(rate[i])) goto stop;
    }
    const bool use_gyro = !yaw_world && c->yaw_joint_rate_tau > 0.0f &&
        f->world_fresh && isfinite(f->world_rate[0]);
    if (use_gyro) {
        /* World rate minus joint rate estimates chassis rotation. Filter that
         * difference so gear oscillations in motor rate do not drive torque. */
        const float chassis_rate = f->world_rate[0] - rate[0];
        if (!isfinite(chassis_rate)) goto stop;
        if (s->yaw_chassis_rate_valid) {
            const float weight = dt / (c->yaw_joint_rate_tau + dt);
            s->yaw_chassis_rate += weight * (chassis_rate - s->yaw_chassis_rate);
        } else {
            s->yaw_chassis_rate = chassis_rate;
        }
        rate[0] = f->world_rate[0] - s->yaw_chassis_rate;
    }
    s->yaw_chassis_rate_valid = use_gyro;
    if (!s->tracking) {
        s->target[0] = Motion_Wrap(measured[0]); s->target[1] = measured[1];
        s->yaw_last = f->joint[0];
        s->yaw_world = yaw_world;
        s->tracking = true;
    }
    /* Keep the winding budget across mode switches. Wrapped feedback can
     * cross +/-pi; consecutive physical samples must differ by less than pi. */
    s->yaw_travel += Motion_Wrap(f->joint[0] - s->yaw_last);
    s->yaw_last = f->joint[0];
    if (spin) s->spin_elapsed += dt;
    if (!isfinite(s->yaw_travel) || !isfinite(s->spin_elapsed)) goto stop;
    if (s->spin_elapsed > 0) {
        const bool timed_out = c->spin_max_time > 0 && s->spin_elapsed >= c->spin_max_time;
        const bool travel_out = c->spin_max_travel > 0 && fabsf(s->yaw_travel) >= c->spin_max_travel;
        if (timed_out || travel_out) {
            Gimbal_Reset(s);
            s->spin_limit = timed_out ? GIMBAL_SPIN_TIME_LIMIT : GIMBAL_SPIN_TRAVEL_LIMIT;
            return false;
        }
    }
    if (s->yaw_world != yaw_world) {
        /* Capture the new frame's current heading without a target jump. */
        s->target[0] = Motion_Wrap(measured[0]);
        s->yaw_input_direction = 0;
        s->yaw_world = yaw_world;
        PID_Reset(&s->rate[0]);
    }
    const int yaw_direction = (cmd->yaw > 0.0f) - (cmd->yaw < 0.0f);
    const float yaw_angle = Motion_Wrap(measured[0]);
    if (yaw_direction != s->yaw_input_direction) {
        /* Drop the old target on release or reversal and start from feedback. */
        s->target[0] = yaw_angle;
        PID_Reset(&s->rate[0]);
        s->yaw_input_direction = yaw_direction;
    }
    /* A held displacement can saturate the angle loop without saturating
     * motor torque. Bound the stored drive before the external load lets go. */
    const float yaw_integral_limit = c->rate_pid[0].integral_limit;
    s->rate[0].config.integral_limit = yaw_direction == 0 && c->yaw_hold_integral_limit > 0.0f
        ? fminf(yaw_integral_limit, c->yaw_hold_integral_limit) : yaw_integral_limit;
    const float yaw_rate_limit = fabsf(cmd->yaw) * c->max_rate[0];
    if (yaw_direction != 0) {
        /* Shrink the stored heading lead as the stick returns to center so
         * the rate reference falls before the stick enters the dead zone. */
        const float max_error = fminf(yaw_rate_limit / c->angle_kp[0], MOTION_PI / 2.0f);
        const float step = cmd->yaw * c->yaw_command_rate * dt;
        if (!isfinite(step)) goto stop;
        const float error = Motion_Wrap(s->target[0] - yaw_angle);
        s->target[0] = Motion_Wrap(yaw_angle + Motion_Clamp(error + step, max_error));
        /* As the stick backs off, shed drive torque that would oppose braking.
         * Keep the integral when the joint is still below the requested limit. */
        if (yaw_rate_limit < s->yaw_previous_rate_limit &&
            yaw_direction * rate[0] > yaw_rate_limit && yaw_direction * s->rate[0].i > 0.0f) {
            s->rate[0].i *= yaw_rate_limit / s->yaw_previous_rate_limit;
        }
    }
    s->yaw_previous_rate_limit = yaw_rate_limit;
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
        const bool yaw_hold = i == 0 && yaw_direction == 0;
        const float angle_kp = yaw_hold && c->yaw_hold_angle_kp > 0.0f
            ? c->yaw_hold_angle_kp : c->angle_kp[i];
        const float rate_limit = yaw_hold && c->yaw_hold_max_rate > 0.0f
            ? c->yaw_hold_max_rate : c->max_rate[i];
        const float requested = angle_kp * error;
        if (!isfinite(requested)) goto stop;
        result.rate[i] = Motion_Clamp(requested, rate_limit);
        /* Pitch stiffness reacts immediately to displacement, without the
         * outer rate cap weakening it. Retain velocity damping and integral
         * load compensation, with one shared torque/anti-windup limit. */
        const float hold = i == 1 ? c->pitch_hold_kp * error : 0.0f;
        if (!PID_UpdateWithFeedforward(&s->rate[i], result.rate[i], rate[i],
                                      hold, dt, &result.torque[i])) goto stop;
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
