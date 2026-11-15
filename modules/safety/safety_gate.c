#include "safety_gate.h"
#include <stddef.h>

bool Safety_Step(SafetyGate *g, const RobotCommand *c, const SafetyInputs *in)
{
    if (g == NULL) return false;
    if (!Command_Valid(c) || in == NULL || !c->remote_valid || c->estop ||
        !in->link_fresh || !in->sensors_fresh || !in->motors_healthy ||
        !in->hardware_configured || in->calibration_failed) {
        *g = (SafetyGate){.state = SAFETY_FAULT};
        return false;
    }
    if (!in->calibrated) {
        *g = (SafetyGate){.state = SAFETY_CALIBRATING};
        return false;
    }
    if (!c->arm) {
        *g = (SafetyGate){SAFETY_DISARMED, Command_Neutral(c, 0.02f)};
        return false;
    }
    if (g->state == SAFETY_ACTIVE) return true;
    if (g->state == SAFETY_DISARMED && g->ready_to_arm && Command_Neutral(c, 0.02f)) {
        *g = (SafetyGate){.state = SAFETY_ACTIVE};
        return true;
    }
    g->ready_to_arm = false;
    return false;
}
