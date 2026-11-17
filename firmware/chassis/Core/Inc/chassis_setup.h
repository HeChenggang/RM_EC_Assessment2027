#ifndef CHASSIS_SETUP_H
#define CHASSIS_SETUP_H

/* Initial low-speed commissioning settings, CAN ID order FL, RL, RR, FR.
 * Initial floor-test limits, not a final tuned configuration. */
#define CHASSIS_WHEEL_DIRECTIONS {+1, +1, -1, -1}
#define CHASSIS_MAX_ROTOR_RPM 3000.0f
#define CHASSIS_SPIN_MAX_ROTOR_RPM 4500.0f
/* Loaded strafing saturated at 2000 raw while some wheels stayed near zero. */
#define CHASSIS_CURRENT_LIMIT_RAW 4000.0f
#define CHASSIS_SPEED_KP 4.0f
#define CHASSIS_SPEED_KI 3.0f
#define CHASSIS_SPEED_INTEGRAL_LIMIT_RAW 650.0f
#define CHASSIS_TRANSLATION_LIMIT_MPS 1.80f
#define CHASSIS_TRANSLATION_ACCEL_MPS2 1.20f
/* Conventional spin: chassis rotates under a world-stabilized yaw axis.
 * Translation follows the measured gimbal heading. Upper-board spin time
 * and travel stop limits are currently disabled. */
#define CHASSIS_SPIN_RATE_RAD_S 3.00f
/* Follow-mode rotation cap; the proportional loop saturates here. */
#define CHASSIS_MAX_ROTATION_RAD_S 3.0f

/* Raised-chassis bench build. Set to 0 before four-wheel operation. */
#define CHASSIS_RR_BENCH 0
#define CHASSIS_RR_BENCH_RPM 100.0f
#define CHASSIS_RR_BENCH_CURRENT_RAW 410.0f
/* Ozone-triggered open-loop pulse. Set to 0 to restore remote PID tuning. */
#define CHASSIS_RR_DIRECT_TEST 0

/* Board-link relative yaw (gimbal status 0x310..0x312). */
/* No physical yaw move can exceed this between two 20 ms status frames. */
#define CHASSIS_YAW_JUMP_RAD 0.3f
/* Upper-board status gates follow on calibrated, fresh world yaw. */
#define CHASSIS_FOLLOW_KP 5.5f
#define CHASSIS_FOLLOW_ROTATION_ACCEL_RAD_S2 4.0f

#endif
