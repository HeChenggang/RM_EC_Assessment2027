#include "gimbal_drive.h"
#include "motion_math.h"
#include <math.h>
#include <stddef.h>

#define ALL_TX (CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2)
#define FEEDBACK_MS 100U
#define RETRY_MS 20U
#define ARM_TX_WAIT_MS 20U
#define ENABLE_MS 300U
#define WORLD_MS 30U

bool GimbalDrive_Init(GimbalDrive *s, const GimbalDriveConfig *c,
                      CAN_HandleTypeDef *yaw, CAN_HandleTypeDef *pitch, uint32_t now)
{
    if (!s) return false;
    *s = (GimbalDrive){.bus={yaw,pitch}, .last_step=now,
        .last_stop=now-RETRY_MS, .fault=GIMBAL_CONFIG};
    if (!c || !yaw || !pitch) return false;
    s->config = *c;
    s->configured = c->hardware_confirmed && isfinite(c->pitch_zero) &&
        c->pitch_stall_ms > 0 && c->pitch_stall_ms < 0x80000000U &&
        isfinite(c->pitch_stall_travel) && c->pitch_stall_travel > 0 &&
        isfinite(c->pitch_stall_error) && c->pitch_stall_error > c->pitch_stall_travel &&
        (c->yaw_direction == 1 || c->yaw_direction == -1) &&
        (c->pitch_direction == 1 || c->pitch_direction == -1) &&
        (c->control.space[0] == GIMBAL_JOINT || c->control.space[0] == GIMBAL_WORLD) &&
        c->control.space[1] == GIMBAL_JOINT &&
        fabsf(c->pitch_zero+c->control.pitch_min*c->pitch_direction) <= DM4340_P_MAX &&
        fabsf(c->pitch_zero+c->control.pitch_max*c->pitch_direction) <= DM4340_P_MAX &&
        c->control.rate_pid[1].output_limit <= DM4340_T_MAX &&
        Gimbal_Init(&s->control, &c->control);
    if (s->configured) s->fault = GIMBAL_OK;
    return s->configured;
}

static bool empty(GimbalDrive *s)
{
    /* CAN1 also carries the board-link status frames, so only one free
     * mailbox is needed for a single yaw frame. CAN2 still belongs to the
     * pitch sender alone. */
    return s->bus[0] && s->bus[1] &&
        (s->config.pitch_only || HAL_CAN_GetTxMailboxesFreeLevel(s->bus[0]) >= 1) &&
        HAL_CAN_GetTxMailboxesFreeLevel(s->bus[1]) == 3;
}

static void stop(GimbalDrive *s, GimbalDriveFault fault, uint32_t now)
{
    if (fault != GIMBAL_OK && (s->state != GIMBAL_STOPPED || s->last_fault == GIMBAL_OK))
        s->last_fault = fault;
    s->pitch_stall_tracking = false;
    if (s->state != GIMBAL_STOPPED) {
        /* Abort old enable/torque packets before queuing either disable. */
        s->last_stop = now - RETRY_MS;
        s->stop_sent = false;
        s->ready = false;
    }
    s->state = GIMBAL_STOPPED;
    s->fault = fault;
    s->output = (GimbalOutput){0};
    Gimbal_Reset(&s->control);
    if (fault != GIMBAL_OK) s->ready = false;
    if ((uint32_t)(now - s->last_stop) < RETRY_MS) return;
    s->last_stop = now;
    for (unsigned i=0; i<2; ++i) {
        if (!s->bus[i]) { s->tx[i] = HAL_ERROR; continue; }
        if (HAL_CAN_GetTxMailboxesFreeLevel(s->bus[i]) != 3)
            (void)HAL_CAN_AbortTxRequest(s->bus[i], ALL_TX);
        s->tx[i] = i == 0 ? YawMotor_Disable() : PitchMotor_Disable();
    }
    s->stop_sent = (s->config.pitch_only || s->tx[0] == HAL_OK) && s->tx[1] == HAL_OK;
}

void GimbalDrive_SetWorld(GimbalDrive *s, float yaw, float rate, uint32_t now)
{
    if (!s || !isfinite(yaw) || !isfinite(rate)) return;
    s->world_yaw = Motion_Wrap(yaw);
    s->world_rate = rate;
    s->world_ms = now;
    s->world_received = true;
}

void GimbalDrive_Step(GimbalDrive *s, const RobotCommand *cmd, bool link,
                      const YawMotor_Feedback *y, const PitchMotor_Feedback *p, uint32_t now)
{
    if (!s) return;
    uint32_t elapsed = now - s->last_step;
    /* Safety checks run on every call; control output is limited to 200 Hz. */
    if (!s->configured) { stop(s,GIMBAL_CONFIG,now); return; }
    if (!link || !Command_Valid(cmd) || !cmd->remote_valid || cmd->estop) {
        stop(s,GIMBAL_LINK,now); return;
    }
    if (!cmd->arm) {
        stop(s,GIMBAL_OK,now);
        s->ready = Command_Neutral(cmd,0.02f);
        s->last_step = now;
        return;
    }
    if ((cmd->mode == MODE_SPIN &&
         (!s->config.spin_hardware_confirmed || s->config.pitch_only)) ||
        (cmd->mode == MODE_FOLLOW &&
         (!s->config.follow_hardware_confirmed || s->config.pitch_only))) {
        stop(s,GIMBAL_CONFIG,now); return;
    }
    if ((cmd->mode == MODE_SPIN || cmd->mode == MODE_FOLLOW ||
         s->config.control.space[0] == GIMBAL_WORLD) &&
        (!s->world_received || (uint32_t)(now-s->world_ms) >= WORLD_MS)) {
        /* A world-referenced yaw axis is only as good as its attitude sample. */
        stop(s,GIMBAL_FEEDBACK,now); return;
    }
    if (s->state == GIMBAL_STOPPED) {
        if (!s->ready || !Command_Neutral(cmd,0.02f)) {
            s->ready = false;
            stop(s,s->fault,now); return;
        }
        s->ready = false;
        s->last_fault = GIMBAL_OK;
        if (!s->stop_sent) { stop(s,GIMBAL_TX,now); return; }
        s->state = GIMBAL_WAIT_TX;
        s->phase_started = now;
    }
    if (s->state != GIMBAL_ACTIVE && !Command_Neutral(cmd,0.02f)) {
        stop(s,GIMBAL_LINK,now); return;
    }
    if (s->state == GIMBAL_WAIT_TX) {
        /* A queued disable can still be on the bus when the arm request arrives. */
        if ((uint32_t)(now-s->phase_started) >= ARM_TX_WAIT_MS) {
            stop(s,GIMBAL_TX,now); return;
        }
        if (!empty(s)) return;
        s->state = GIMBAL_PREPARING;
        s->last_step = now;
        /* Preload zero MIT torque while disabled. No retained old target. */
        s->tx[0] = s->config.pitch_only ? HAL_OK : YawMotor_SendMIT(0,0,0,0,0);
        s->tx[1] = PitchMotor_SendMIT(0,0,0,0,0);
        if (s->tx[0] != HAL_OK || s->tx[1] != HAL_OK) stop(s,GIMBAL_TX,now);
        return;
    }
    if (s->state != GIMBAL_ACTIVE && (uint32_t)(now-s->phase_started) >= ENABLE_MS) {
        stop(s,GIMBAL_ENABLE_TIMEOUT,now); return;
    }
    if (elapsed < 5U) return;
    s->last_step = now;
    if (elapsed > 20U) { stop(s,GIMBAL_TIMING,now); return; }
    if (s->state == GIMBAL_PREPARING) {
        if (!empty(s)) return;
        s->enable_count[0] = y ? y->rx_count : 0;
        s->enable_count[1] = p ? p->rx_count : 0;
        s->tx[0] = s->config.pitch_only ? HAL_OK : YawMotor_Enable(); s->tx[1] = PitchMotor_Enable();
        if (s->tx[0] != HAL_OK || s->tx[1] != HAL_OK) { stop(s,GIMBAL_TX,now); return; }
        s->state = GIMBAL_ENABLING;
        return;
    }
    bool pitch_fresh = p && p->received && (uint32_t)(now-p->last_rx_ms) < FEEDBACK_MS;
    bool yaw_fresh = y && y->received && y->scaled &&
        (uint32_t)(now-y->last_rx_ms) < FEEDBACK_MS;
    bool fresh = pitch_fresh && (s->config.pitch_only || yaw_fresh);
    bool enabled = fresh && p->motor.state == 1 && p->rx_count != s->enable_count[1] &&
        (s->config.pitch_only || (y->state == 1 && y->rx_count != s->enable_count[0]));
    if (s->state == GIMBAL_ENABLING && !enabled) {
        if (fresh && ((!s->config.pitch_only && y->state > 1) || p->motor.state > 1)) { stop(s,GIMBAL_FEEDBACK,now); return; }
        /* Some motors report only after a command; request with zero torque. */
        if (empty(s)) {
            s->tx[0] = s->config.pitch_only ? HAL_OK : YawMotor_SendMIT(0,0,0,0,0);
            s->tx[1] = PitchMotor_SendMIT(0,0,0,0,0);
            if (s->tx[0] != HAL_OK || s->tx[1] != HAL_OK) stop(s,GIMBAL_TX,now);
        }
        return;
    }
    if (!enabled) { stop(s,GIMBAL_FEEDBACK,now); return; }
    GimbalFeedback f = {.joint={s->config.pitch_only ? 0 : y->pos*s->config.yaw_direction,
        (p->motor.pos-s->config.pitch_zero)*s->config.pitch_direction},
        .joint_rate={s->config.pitch_only ? 0 : y->vel*s->config.yaw_direction,p->motor.vel*s->config.pitch_direction},
        .world={s->world_yaw, 0},
        .world_rate={s->world_rate, 0},
        .joint_fresh=true,
        .world_fresh=s->world_received && (uint32_t)(now-s->world_ms) < WORLD_MS};
    /* User readings are rounded to 0.001 rad; snap only the measurement at
     * endpoints. Targets retain the exact limits and outward torque is blocked. */
    if (f.joint[1] < s->config.control.pitch_min &&
        f.joint[1] >= s->config.control.pitch_min-0.001f)
        f.joint[1] = s->config.control.pitch_min;
    if (f.joint[1] > s->config.control.pitch_max &&
        f.joint[1] <= s->config.control.pitch_max+0.001f)
        f.joint[1] = s->config.control.pitch_max;
    RobotCommand local_command = *cmd;
    if (s->config.pitch_only) local_command.yaw = 0;
    /* Rounding 20 ms to float can exceed the configured max_dt by one ULP. */
    float dt = elapsed * 0.001f;
    if (dt > s->config.control.max_dt) dt = s->config.control.max_dt;
    if (!Gimbal_Step(&s->control,&local_command,&f,true,dt,&s->output)) {
        GimbalDriveFault fault = GIMBAL_LIMIT;
        if (s->control.spin_limit == GIMBAL_SPIN_TIME_LIMIT) fault = GIMBAL_SPIN_TIME;
        if (s->control.spin_limit == GIMBAL_SPIN_TRAVEL_LIMIT) fault = GIMBAL_SPIN_TRAVEL;
        stop(s,fault,now); return;
    }
    s->state = GIMBAL_ACTIVE;
    s->fault = GIMBAL_OK;
    /* A holding load at its target is not a stall. Require significant tracking
     * error AND near-limit torque with no measurable position progress.
     * Position is used instead of the noisy quantized velocity feedback. */
    bool loaded = fabsf(s->output.torque[1]) >= 0.9f*s->config.control.rate_pid[1].output_limit &&
        fabsf(s->output.target[1]-f.joint[1]) >= s->config.pitch_stall_error;
    if (!loaded) s->pitch_stall_tracking = false;
    else if (!s->pitch_stall_tracking ||
             fabsf(f.joint[1]-s->pitch_stall_position) >= s->config.pitch_stall_travel) {
        s->pitch_stall_tracking = true;
        s->pitch_stall_started = now;
        s->pitch_stall_position = f.joint[1];
    } else if ((uint32_t)(now-s->pitch_stall_started) >= s->config.pitch_stall_ms) {
        stop(s,GIMBAL_STALL,now);
        return;
    }
    s->tx[0] = s->config.pitch_only ? HAL_OK :
        YawMotor_SendMIT(0,0,0,0,s->output.torque[0]*s->config.yaw_direction);
    s->tx[1] = PitchMotor_SendMIT(0,0,0,0,s->output.torque[1]*s->config.pitch_direction);
    if (s->tx[0] != HAL_OK || s->tx[1] != HAL_OK) stop(s,GIMBAL_TX,now);
}
