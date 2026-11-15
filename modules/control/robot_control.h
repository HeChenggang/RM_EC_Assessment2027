#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H
#include "safety_gate.h"
#include "chassis_control.h"
#include "gimbal_control.h"

typedef struct {
    ChassisControl chassis;
    GimbalControl gimbal;
    SafetyGate safety;
    bool configured;
} RobotControl;
typedef struct {
    RobotCommand command;
    SafetyInputs safety;
    float relative_yaw, wheel_rpm[4], dt;
    bool wheel_fresh[4];
    GimbalFeedback gimbal;
} RobotInput;
typedef struct {
    ChassisOutput chassis;
    GimbalOutput gimbal;
    SafetyState state;
    bool enabled;
} RobotOutput;

bool Robot_Init(RobotControl *robot, const ChassisConfig *chassis, const GimbalConfig *gimbal);
/* Pure reference coordinator for offline simulation (no HAL, no CAN sends).
 * Any control failure zeroes BOTH outputs, clears controllers and latches stop.
 * Distributed board scheduling/stop delivery still belongs to transport. */
bool Robot_Step(RobotControl *robot, const RobotInput *input, RobotOutput *out);
#endif
