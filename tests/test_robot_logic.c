#include "robot_control.h"
#include "motion_math.h"
#include "board_link.h"
#include "chassis_setup.h"
#include <math.h>
#include <stddef.h>
#include <string.h>
#define CHECK(c) do { if (!(c)) return __LINE__; } while (0)
#define NEAR(a,b,e) (fabsf((a)-(b)) < (e))

/* Simulation-only settings, never production motor parameters. */
static ChassisConfig chassis_config(void)
{
    ChassisConfig c = {.geometry = MECANUM_DEFAULT_CONFIG, .max_translation = 2,
        .max_rotation = 3, .spin_rate = 1, .follow_kp = 2,
        .translation_accel = 10, .rotation_accel = 10, .max_dt = 0.02f};
    c.geometry.max_motor_rpm = 5000;
    for (unsigned i = 0; i < 4; ++i) c.wheel_pid[i] = (PID_Config){
        .kp = 1, .ki = 0.5f, .output_limit = 1000, .integral_limit = 200, .max_dt = 0.02f};
    return c;
}
static GimbalConfig gimbal_config(void)
{
    GimbalConfig c = {.pitch_min = -0.4f, .pitch_max = 0.5f,
        .yaw_command_rate = 1, .pitch_command_rate = 1, .angle_kp = {4, 4},
        .max_dt = 0.02f, .max_rate = {2, 2}, .space = {GIMBAL_JOINT, GIMBAL_JOINT}};
    for (unsigned i = 0; i < 2; ++i) c.rate_pid[i] = (PID_Config){
        .kp = 1, .ki = 0.1f, .output_limit = 1, .integral_limit = 0.2f, .max_dt = 0.02f};
    return c;
}
static bool zero_output(const RobotOutput *o)
{
    return !o->enabled && o->chassis.current[0] == 0 && o->chassis.current[1] == 0 &&
        o->chassis.current[2] == 0 && o->chassis.current[3] == 0 &&
        o->gimbal.torque[0] == 0 && o->gimbal.torque[1] == 0;
}
int main(void)
{
    float x, y;
    CHECK(Motion_Rotate(1, 0, MOTION_PI/2, &x, &y) && NEAR(x, 0, 1e-5f) && NEAR(y, 1, 1e-5f));
    CHECK(Motion_Rotate(1, 0, -MOTION_PI/2, &x, &y) && NEAR(y, -1, 1e-5f));
    CHECK(Motion_Rotate(1, 0, MOTION_PI, &x, &y) && NEAR(x, -1, 1e-5f));
    CHECK(!Motion_Rotate(NAN, 0, 0, &x, &y) && x == 0 && y == 0);
    CHECK(!Motion_Rotate(1, 0, 0, &x, &x));
    for (int i = -100; i <= 100; ++i) {
        const float angle = i * 0.13f;
        CHECK(Motion_Rotate(0.3f, -0.7f, angle, &x, &y));
        float xx, yy;
        CHECK(Motion_Rotate(x, y, -angle, &xx, &yy));
        CHECK(NEAR(xx, 0.3f, 1e-5f) && NEAR(yy, -0.7f, 1e-5f));
    }
    AngleTracker angle = {0}; float continuous;
    CHECK(AngleTracker_Update(&angle, 3.13f, 0xfffffff0U, 10, 100, &continuous));
    CHECK(AngleTracker_Update(&angle, -3.13f, 4, 10, 100, &continuous));
    CHECK(continuous > MOTION_PI);
    CHECK(AngleTracker_Update(&angle, 3.13f, 24, 10, 100, &continuous));
    CHECK(NEAR(continuous, 3.13f, 1e-5f));
    CHECK(!AngleTracker_Update(&angle, 0, 25, 10, 100, &continuous) && !angle.initialized);
    CHECK(AngleTracker_Update(&angle, 0, 0, 10, 100, &continuous));
    for (unsigned i = 1; i <= 200; ++i)
        CHECK(AngleTracker_Update(&angle, Motion_Wrap(i*0.1f), i*20, 10, 100, &continuous));
    CHECK(NEAR(continuous, 20, 0.001f));
    CHECK(!AngleTracker_Update(&angle, 1, 4100, 10, 100, &continuous));

    RemoteMap map = {.channel = {3,2,0,1}, .direction = {1,-1,-1,1}, .deadzone = 0.05f,
        .separate_switch = 1, .follow_switch = 3, .spin_switch = 2,
        .stop_switch = 2, .disarm_switch = 3, .enable_switch = 1};
    RemoteInput raw = {.mode_switch = 1, .arm_switch = 1, .fresh = true};
    RobotCommand cmd;
    raw.channel[3] = 660; raw.channel[2] = 660; raw.channel[0] = 10;
    CHECK(Remote_Map(&map, &raw, &cmd) && cmd.forward == 1 && cmd.left == -1 && cmd.yaw == 0);
    raw.arm_switch = 2;
    CHECK(Remote_Map(&map, &raw, &cmd) && cmd.estop && !cmd.arm && cmd.forward == 0);
    raw.arm_switch = 3;
    CHECK(Remote_Map(&map, &raw, &cmd) && !cmd.estop && !Command_Neutral(&cmd, 0.02f));
    raw.channel[4] = 661;
    CHECK(!Remote_Map(&map, &raw, &cmd) && cmd.estop && !cmd.remote_valid);
    raw.channel[4] = 0; raw.mode_switch = 0;
    CHECK(!Remote_Map(&map, &raw, &cmd));
    raw.mode_switch = 3; raw.arm_switch = 1;
    CHECK(Remote_Map(&map, &raw, &cmd) && cmd.mode == MODE_FOLLOW);
    raw.mode_switch = 2;
    CHECK(Remote_Map(&map, &raw, &cmd) && cmd.mode == MODE_SPIN);
    raw.fresh = false;
    CHECK(!Remote_Map(&map, &raw, &cmd));
    raw.fresh = true; map.direction[0] = 0;
    CHECK(!Remote_Map(&map, &raw, &cmd));
    cmd = (RobotCommand){.remote_valid = true, .arm = true};
    SafetyGate gate = {0};
    SafetyInputs safe = {.link_fresh = true, .sensors_fresh = true, .motors_healthy = true,
        .hardware_configured = true};
    CHECK(!Safety_Step(&gate, &cmd, &safe) && gate.state == SAFETY_CALIBRATING);
    safe.calibrated = true;
    CHECK(!Safety_Step(&gate, &cmd, &safe)); /* held ARM */
    cmd.arm = false; cmd.forward = 0.5f;
    CHECK(!Safety_Step(&gate, &cmd, &safe) && !gate.ready_to_arm);
    cmd.forward = 0;
    CHECK(!Safety_Step(&gate, &cmd, &safe) && gate.ready_to_arm);
    cmd.arm = true; cmd.yaw = 0.1f;
    CHECK(!Safety_Step(&gate, &cmd, &safe) && !gate.ready_to_arm);
    cmd.arm = false; cmd.yaw = 0; (void)Safety_Step(&gate, &cmd, &safe);
    cmd.arm = true;
    CHECK(Safety_Step(&gate, &cmd, &safe));
    safe.link_fresh = false;
    CHECK(!Safety_Step(&gate, &cmd, &safe) && gate.state == SAFETY_FAULT);
    safe.link_fresh = true;
    CHECK(!Safety_Step(&gate, &cmd, &safe));

    ChassisConfig cc = chassis_config(); GimbalConfig gc = gimbal_config();
    RobotControl robot;
    CHECK(Robot_Init(&robot, &cc, &gc));
    RobotInput in = {.command = {.remote_valid = true}, .safety = safe,
        .wheel_fresh = {true,true,true,true}, .dt = 0.01f,
        .gimbal = {.joint_fresh = true, .world_fresh = true}};
    RobotOutput out;
    CHECK(!Robot_Step(&robot, &in, &out) && zero_output(&out));
    in.command.arm = true;
    CHECK(Robot_Step(&robot, &in, &out) && out.enabled);
    in.command.forward = 1;
    CHECK(Robot_Step(&robot, &in, &out));
    CHECK(NEAR(out.chassis.velocity.vx, 0.1f, 1e-5f));
    for (unsigned i = 0; i < 30; ++i) CHECK(Robot_Step(&robot, &in, &out));
    CHECK(NEAR(out.chassis.velocity.vx, 2, 1e-5f));
    CHECK(out.chassis.current[0] > 0 && out.chassis.current[0] <= 1000);
    in.command.mode = MODE_FOLLOW; in.relative_yaw = 0.5f;
    for (unsigned i = 0; i < 30; ++i) CHECK(Robot_Step(&robot, &in, &out));
    CHECK(NEAR(out.chassis.velocity.wz, 1, 1e-5f));
    CHECK(NEAR(out.chassis.velocity.vx, 2*cosf(0.5f), 1e-5f));
    in.command.mode = MODE_SPIN; in.relative_yaw = -1;
    CHECK(Robot_Step(&robot, &in, &out) && out.chassis.velocity.wz > 0);
    in.command.mode = MODE_SEPARATE;
    for (unsigned i = 0; i < 30; ++i) CHECK(Robot_Step(&robot, &in, &out));
    CHECK(NEAR(out.chassis.velocity.wz, 0, 1e-5f));
    in.wheel_fresh[2] = false;
    CHECK(!Robot_Step(&robot, &in, &out) && zero_output(&out) && out.state == SAFETY_FAULT);
    CHECK(robot.chassis.wheel[0].i == 0 && !robot.gimbal.tracking);
    in.wheel_fresh[2] = true; in.command = (RobotCommand){.remote_valid = true, .arm = true};
    CHECK(!Robot_Step(&robot, &in, &out) && zero_output(&out));
    in.command.arm = false; (void)Robot_Step(&robot, &in, &out);
    in.command.arm = true;
    CHECK(Robot_Step(&robot, &in, &out));
    in.command.estop = true;
    CHECK(!Robot_Step(&robot, &in, &out) && zero_output(&out));
    in.command.estop = false; in.command.arm = false; in.safety.hardware_configured = false;
    CHECK(!Robot_Step(&robot, &in, &out) && out.state == SAFETY_FAULT);
    in.safety.hardware_configured = true; (void)Robot_Step(&robot, &in, &out);
    in.command.arm = true; in.dt = NAN;
    CHECK(!Robot_Step(&robot, &in, &out) && zero_output(&out));

    GimbalControl gimbal; GimbalOutput go;
    CHECK(Gimbal_Init(&gimbal, &gc));
    GimbalFeedback fb = {.joint = {3.13f, 0.49f}, .joint_fresh = true};
    cmd = (RobotCommand){.remote_valid = true, .arm = true, .yaw = 1, .pitch = 1};
    CHECK(Gimbal_Step(&gimbal, &cmd, &fb, true, 0.02f, &go));
    CHECK(go.target[0] < -3 && go.rate[0] > 0 && NEAR(go.target[1], 0.5f, 1e-5f));
    fb.joint[0] = -3.13f; fb.joint[1] = 0.5f; fb.joint_rate[1] = -1;
    CHECK(Gimbal_Step(&gimbal, &cmd, &fb, true, 0.01f, &go) && go.torque[1] == 0);
    fb.joint[1] = 0.51f;
    CHECK(!Gimbal_Step(&gimbal, &cmd, &fb, true, 0.01f, &go) && go.torque[0] == 0);
    gc.space[1] = GIMBAL_WORLD;
    CHECK(Gimbal_Init(&gimbal, &gc));
    fb = (GimbalFeedback){.joint = {0,0.49f}, .world = {0,0.2f}, .joint_fresh = true, .world_fresh = true};
    CHECK(Gimbal_Step(&gimbal, &cmd, &fb, true, 0.02f, &go));
    CHECK(NEAR(go.target[1], 0.21f, 1e-5f));
    fb.world_fresh = false;
    CHECK(!Gimbal_Step(&gimbal, &cmd, &fb, true, 0.01f, &go));
    cc.wheel_pid[0].output_limit = 20000;
    CHECK(!Chassis_Init(&robot.chassis, &cc));
    gc.pitch_min = gc.pitch_max;
    CHECK(!Gimbal_Init(&gimbal, &gc));

    /* End-to-end: raw neutral disarm -> CAN fragments -> safety -> motion;
     * no later packet -> deterministic zero outputs, held ARM cannot recover. */
    cc = chassis_config(); gc = gimbal_config();
    CHECK(Robot_Init(&robot, &cc, &gc));
    LinkReceiver link; LinkFrame frames[3];
    CHECK(Link_Init(&link, LINK_COMMAND, 10, 100));
    map.direction[0] = 1;
    raw = (RemoteInput){.mode_switch = 1, .arm_switch = 3, .fresh = true};
    CHECK(Remote_Map(&map, &raw, &cmd) && Link_EncodeCommand(&cmd, 1, frames));
    for (unsigned i = 0; i < 3; ++i) (void)Link_Receive(&link, &frames[i], 1);
    in = (RobotInput){.safety = safe, .wheel_fresh = {true,true,true,true}, .dt = 0.01f,
        .gimbal = {.joint_fresh = true}};
    CHECK(Link_GetCommand(&link, 1, &in.command));
    CHECK(!Robot_Step(&robot, &in, &out));
    raw.arm_switch = 1;
    CHECK(Remote_Map(&map, &raw, &cmd) && Link_EncodeCommand(&cmd, 2, frames));
    for (unsigned i = 0; i < 3; ++i) (void)Link_Receive(&link, &frames[i], 2);
    CHECK(Link_GetCommand(&link, 2, &in.command));
    CHECK(Robot_Step(&robot, &in, &out));
    raw.channel[3] = 330;
    CHECK(Remote_Map(&map, &raw, &cmd) && Link_EncodeCommand(&cmd, 3, frames));
    for (unsigned i = 0; i < 3; ++i) (void)Link_Receive(&link, &frames[i], 3);
    CHECK(Link_GetCommand(&link, 3, &in.command));
    CHECK(Robot_Step(&robot, &in, &out) && out.chassis.current[0] > 0);
    CHECK(!Link_GetCommand(&link, 103, &in.command));
    CHECK(!Robot_Step(&robot, &in, &out) && zero_output(&out));
    /* Actual chassis commissioning parameters must drive all four wheels,
     * while retaining limits and immediate stop on missing feedback. */
    CHECK(!CHASSIS_RR_BENCH && !CHASSIS_RR_DIRECT_TEST);
    cc = chassis_config();
    cc.max_translation = CHASSIS_TRANSLATION_LIMIT_MPS;
    cc.geometry.max_motor_rpm = CHASSIS_MAX_ROTOR_RPM;
    const int8_t directions[4] = CHASSIS_WHEEL_DIRECTIONS;
    for (unsigned i = 0; i < 4; ++i) {
        cc.geometry.motor_direction[i] = directions[i];
        cc.wheel_pid[i] = (PID_Config){.kp = CHASSIS_SPEED_KP,
            .output_limit = CHASSIS_CURRENT_LIMIT_RAW, .max_dt = cc.max_dt};
    }
    CHECK(Chassis_Init(&robot.chassis, &cc));
    cmd = (RobotCommand){.remote_valid = true, .arm = true, .forward = 1};
    float stopped_rpm[4] = {0};
    bool fresh[4] = {true, true, true, true};
    ChassisOutput drive;
    for (unsigned step = 0; step < 30; ++step)
        CHECK(Chassis_Step(&robot.chassis, &cmd, 0, stopped_rpm, fresh, true, 0.01f, &drive));
    for (unsigned i = 0; i < 4; ++i) {
        CHECK(directions[i] * drive.current[i] > 0);
        CHECK(fabsf((float)drive.current[i]) <= CHASSIS_CURRENT_LIMIT_RAW);
        CHECK(fabsf(drive.wheels.motor_rpm[i]) <= CHASSIS_MAX_ROTOR_RPM + 0.001f);
    }
    fresh[3] = false;
    CHECK(!Chassis_Step(&robot.chassis, &cmd, 0, stopped_rpm, fresh, true, 0.01f, &drive));
    for (unsigned i = 0; i < 4; ++i) CHECK(drive.current[i] == 0);
    return 0;
}
