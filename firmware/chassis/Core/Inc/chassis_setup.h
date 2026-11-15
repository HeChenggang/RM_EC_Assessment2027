#ifndef CHASSIS_SETUP_H
#define CHASSIS_SETUP_H

/* Initial low-speed commissioning settings, CAN ID order FL, RL, RR, FR.
 * Initial floor-test limits, not a final tuned configuration. */
#define CHASSIS_WHEEL_DIRECTIONS {+1, +1, -1, -1}
#define CHASSIS_MAX_ROTOR_RPM 400.0f
#define CHASSIS_CURRENT_LIMIT_RAW 2000.0f
#define CHASSIS_SPEED_KP 10.0f
#define CHASSIS_TRANSLATION_LIMIT_MPS 0.20f

/* Raised-chassis bench build. Set to 0 before four-wheel operation. */
#define CHASSIS_RR_BENCH 0
#define CHASSIS_RR_BENCH_RPM 100.0f
#define CHASSIS_RR_BENCH_CURRENT_RAW 410.0f
/* Ozone-triggered open-loop pulse. Set to 0 to restore remote PID tuning. */
#define CHASSIS_RR_DIRECT_TEST 0

#endif
