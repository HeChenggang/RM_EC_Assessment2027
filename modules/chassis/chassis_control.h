#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H
#include "command.h"
#include "mecanum.h"
#include "pid.h"
#include <stdint.h>

typedef struct {
    Mecanum_Config geometry;
    PID_Config wheel_pid[4];
    float max_translation, max_rotation, spin_rate, follow_kp;
    float translation_accel, rotation_accel, follow_rotation_accel, max_dt;
} ChassisConfig;
typedef struct {
    ChassisConfig config;
    PID_Controller wheel[4];
    Mecanum_Velocity previous;
    float previous_target_rpm[4];
    bool configured;
} ChassisControl;
typedef struct {
    Mecanum_Velocity velocity;
    Mecanum_Wheels wheels;
    int16_t current[4]; /* raw C620 units, not amperes */
} ChassisOutput;

bool Chassis_Init(ChassisControl *control, const ChassisConfig *config);
void Chassis_Reset(ChassisControl *control);
/* yaw is gimbal-x relative to chassis-x, CCW positive. rpm is signed exactly
 * as M3508 feedback (direction conversion belongs to geometry).
 * enabled must come from Safety_Step. Any stale wheel stops ALL outputs. */
bool Chassis_Step(ChassisControl *control, const RobotCommand *cmd, float yaw,
                  const float rpm[4], const bool fresh[4], bool enabled,
                  float dt, ChassisOutput *out);
#endif
