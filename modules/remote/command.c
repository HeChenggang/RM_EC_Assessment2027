#include "command.h"
#include <math.h>
#include <stddef.h>

static bool switches(uint8_t a, uint8_t b, uint8_t c)
{
    return a >= 1 && a <= 3 && b >= 1 && b <= 3 && c >= 1 && c <= 3 &&
        a != b && a != c && b != c;
}
bool Command_Valid(const RobotCommand *c)
{
    return c != NULL && (c->mode == MODE_SEPARATE || c->mode == MODE_FOLLOW || c->mode == MODE_SPIN) &&
        isfinite(c->forward) && fabsf(c->forward) <= 1 &&
        isfinite(c->left) && fabsf(c->left) <= 1 &&
        isfinite(c->yaw) && fabsf(c->yaw) <= 1 &&
        isfinite(c->pitch) && fabsf(c->pitch) <= 1;
}
bool Command_Neutral(const RobotCommand *c, float threshold)
{
    return Command_Valid(c) && isfinite(threshold) && threshold >= 0 && threshold < 1 &&
        fabsf(c->forward) <= threshold && fabsf(c->left) <= threshold &&
        fabsf(c->yaw) <= threshold && fabsf(c->pitch) <= threshold;
}
bool Remote_Map(const RemoteMap *m, const RemoteInput *r, RobotCommand *out)
{
    if (out == NULL) return false;
    *out = (RobotCommand){.estop = true};
    if (m == NULL || r == NULL || !r->fresh || !isfinite(m->deadzone) ||
        m->deadzone < 0 || m->deadzone >= 1 ||
        !switches(m->separate_switch, m->follow_switch, m->spin_switch) ||
        !switches(m->stop_switch, m->disarm_switch, m->enable_switch) ||
        r->mode_switch < 1 || r->mode_switch > 3 || r->arm_switch < 1 || r->arm_switch > 3)
        return false;
    for (unsigned i = 0; i < 5; ++i)
        if (r->channel[i] < -660 || r->channel[i] > 660) return false;
    float axis[4];
    for (unsigned i = 0; i < 4; ++i) {
        if (m->channel[i] >= 5 || (m->direction[i] != 1 && m->direction[i] != -1)) return false;
        const float value = (float)r->channel[m->channel[i]] / 660.0f * m->direction[i];
        const float magnitude = fabsf(value);
        axis[i] = magnitude <= m->deadzone ? 0 :
            copysignf((magnitude - m->deadzone) / (1 - m->deadzone), value);
    }
    out->mode = r->mode_switch == m->separate_switch ? MODE_SEPARATE :
        (r->mode_switch == m->follow_switch ? MODE_FOLLOW : MODE_SPIN);
    out->remote_valid = true;
    out->estop = r->arm_switch == m->stop_switch;
    out->arm = r->arm_switch == m->enable_switch;
    /* Keep stick information while disarmed, so neutral arming can be checked. */
    if (!out->estop) {
        out->forward = axis[0]; out->left = axis[1]; out->yaw = axis[2]; out->pitch = axis[3];
    }
    return true;
}
