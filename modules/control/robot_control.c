#include "robot_control.h"
#include <stddef.h>

bool Robot_Init(RobotControl *r, const ChassisConfig *c, const GimbalConfig *g)
{
    if (r == NULL) return false;
    *r = (RobotControl){0};
    r->configured = Chassis_Init(&r->chassis, c) && Gimbal_Init(&r->gimbal, g);
    return r->configured;
}
bool Robot_Step(RobotControl *r, const RobotInput *in, RobotOutput *out)
{
    if (out != NULL) *out = (RobotOutput){.state = SAFETY_FAULT};
    if (r == NULL) return false;
    if (!r->configured || in == NULL || out == NULL) {
        r->safety = (SafetyGate){.state = SAFETY_FAULT};
        goto stop;
    }
    if (!Safety_Step(&r->safety, &in->command, &in->safety)) goto stop;
    RobotOutput next = {.state = SAFETY_ACTIVE, .enabled = true};
    if (!Chassis_Step(&r->chassis, &in->command, in->relative_yaw, in->wheel_rpm,
                      in->wheel_fresh, true, in->dt, &next.chassis) ||
        !Gimbal_Step(&r->gimbal, &in->command, &in->gimbal, true, in->dt, &next.gimbal)) {
        r->safety = (SafetyGate){.state = SAFETY_FAULT};
        goto stop;
    }
    *out = next;
    return true;
stop:
    Chassis_Reset(&r->chassis);
    Gimbal_Reset(&r->gimbal);
    if (out != NULL) out->state = r->safety.state;
    return false;
}
