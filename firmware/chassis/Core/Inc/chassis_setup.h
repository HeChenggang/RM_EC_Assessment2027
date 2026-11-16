#ifndef CHASSIS_SETUP_H
#define CHASSIS_SETUP_H

/* Initial low-speed commissioning settings, CAN ID order FL, RL, RR, FR.
 * Initial floor-test limits, not a final tuned configuration. */
#define CHASSIS_WHEEL_DIRECTIONS {+1, +1, -1, -1}
#define CHASSIS_MAX_ROTOR_RPM 1600.0f
#define CHASSIS_CURRENT_LIMIT_RAW 2000.0f
#define CHASSIS_SPEED_KP 4.0f
#define CHASSIS_SPEED_KI 3.0f
#define CHASSIS_SPEED_INTEGRAL_LIMIT_RAW 650.0f
#define CHASSIS_TRANSLATION_LIMIT_MPS 0.60f
#define CHASSIS_TRANSLATION_ACCEL_MPS2 1.20f

/* Raised-chassis bench build. Set to 0 before four-wheel operation. */
#define CHASSIS_RR_BENCH 0
#define CHASSIS_RR_BENCH_RPM 100.0f
#define CHASSIS_RR_BENCH_CURRENT_RAW 410.0f
/* Ozone-triggered open-loop pulse. Set to 0 to restore remote PID tuning. */
#define CHASSIS_RR_DIRECT_TEST 0

/* Board-link relative yaw (gimbal status 0x310..0x312). */
/* No physical yaw move can exceed this between two 20 ms status frames. */
#define CHASSIS_YAW_JUMP_RAD 0.3f
/* Follow needs a world-stabilized gimbal yaw. With joint-only yaw control the
 * chassis rotates, the gimbal follows, the joint angle never shrinks and the
 * loop winds up at max_rotation. Keep 0 until the world-frame work is verified. */
#define CHASSIS_FOLLOW_IMU_READY 0

#endif
