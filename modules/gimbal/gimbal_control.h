#ifndef GIMBAL_CONTROL_H
#define GIMBAL_CONTROL_H
#include "command.h"
#include "pid.h"
typedef enum { GIMBAL_JOINT = 0, GIMBAL_WORLD = 1 } GimbalSpace;
typedef struct {
    float pitch_min, pitch_max; /* measured calibrated JOINT limits, radians */
    float yaw_command_rate, pitch_command_rate, angle_kp[2], max_dt;
    PID_Config rate_pid[2]; /* output is torque, configured per actual motor */
    float max_rate[2];
    GimbalSpace space[2]; /* yaw,pitch; allows joint/world/mixed control */
    float spin_max_time, spin_max_travel; /* seconds/rad per armed session */
} GimbalConfig;
typedef struct {
    float joint[2], world[2]; /* yaw,pitch radians; joint yaw may be continuous */
    float joint_rate[2], world_rate[2]; /* rad/s, same coordinate as angle */
    bool joint_fresh, world_fresh;
} GimbalFeedback;
typedef struct {
    GimbalConfig config;
    PID_Controller rate[2];
    float target[2];
    int yaw_input_direction;
    bool configured, tracking;
    bool yaw_world;
    float spin_elapsed, yaw_last, yaw_travel;
} GimbalControl;
typedef struct { float target[2], rate[2], torque[2]; } GimbalOutput;
bool Gimbal_Init(GimbalControl *control, const GimbalConfig *config);
void Gimbal_Reset(GimbalControl *control);
/* Aligned yaw/pitch axis model; world angles/rates must be supplied by the
 * installation/kinematic adapter. Body gyro Y/Z alone are NOT Euler rates.
 * Limits are applied in JOINT space even when the pitch target is WORLD.
 * MODE_SPIN always requires world yaw feedback, regardless of space[0].
 * Spin time and net joint travel are bounded per armed session; callers must
 * latch a failed step and require disarm before restarting.
 * At an exact limit, suppress outward torque; outside limits stop both axes. */
bool Gimbal_Step(GimbalControl *control, const RobotCommand *cmd,
                 const GimbalFeedback *feedback, bool enabled, float dt, GimbalOutput *out);
#endif
