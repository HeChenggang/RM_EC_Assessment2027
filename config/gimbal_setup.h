#ifndef GIMBAL_SETUP_H
#define GIMBAL_SETUP_H
#include "gimbal_drive.h"

/* Fill from the actual motors and measured joint travel, then set to 1.
 * Protocol ranges are NOT mechanical travel or working torque limits.
 * Both motors must be MIT, and Pitch PMAX/VMAX/TMAX must match dm4340_drv.h.
 * Configure each motor's own command-loss timeout before hardware operation. */
#define GIMBAL_HARDWARE_CONFIRMED 1
/* User-authorized low-torque dual-axis test; Yaw CCW direction verified. */
#define GIMBAL_PITCH_ONLY_TEST 0
/* Read from the actual Yaw motor: MIT=1, valid_mask=15 (2026-11-16). */
#define GIMBAL_YAW_PROTOCOL_CONFIRMED 1
#define GIMBAL_YAW_PMAX 3.14159298f
#define GIMBAL_YAW_VMAX 21.0f
#define GIMBAL_YAW_TMAX 7.5f
/* User-measured pitch_feedback.motor.pos, radians (2026-11-15).
 * Raising the barrel decreases raw angle; calibrated joint pitch is up-positive. */
#define GIMBAL_PITCH_ZERO (-0.184f)
#define GIMBAL_PITCH_UPPER_RAW (-1.143f)
/* User positioned the desired lower endpoint; read via J-Link, 2026-11-16. */
#define GIMBAL_PITCH_LOWER_RAW 0.0503824f
#define GIMBAL_PITCH_MIN (GIMBAL_PITCH_ZERO - GIMBAL_PITCH_LOWER_RAW)
#define GIMBAL_PITCH_MAX (GIMBAL_PITCH_ZERO - GIMBAL_PITCH_UPPER_RAW)
#define GIMBAL_YAW_DIRECTION 1
#define GIMBAL_PITCH_DIRECTION (-1)

/* Next commissioning step after measured 1 N m stall. Not hardware-validated. */
#define GIMBAL_PITCH_TORQUE_LIMIT 2.0f
#define GIMBAL_PITCH_SPEED_KP 2.0f
#define GIMBAL_PITCH_SPEED_KI 2.0f
#define GIMBAL_PITCH_INTEGRAL_LIMIT 1.8f
#define GIMBAL_PITCH_STALL_MS 1000U
#define GIMBAL_PITCH_STALL_TRAVEL 0.005f
#define GIMBAL_PITCH_STALL_ERROR 0.05f

/* Small Yaw P-gain step after measured static tracking lag; torque cap unchanged. */
#define GIMBAL_YAW_SPEED_KP 1.2f

/* Conservative initial gains in N m; not final tuned gains. */
static inline GimbalDriveConfig GimbalSetup_Config(void)
{
    return (GimbalDriveConfig){
        .pitch_only=GIMBAL_PITCH_ONLY_TEST,
        .hardware_confirmed=GIMBAL_PITCH_ONLY_TEST || (GIMBAL_HARDWARE_CONFIRMED &&
            GIMBAL_YAW_PMAX > 0 && GIMBAL_YAW_VMAX > 0 && GIMBAL_YAW_TMAX >= 0.3f),
        .pitch_zero=GIMBAL_PITCH_ZERO,
        .pitch_stall_ms=GIMBAL_PITCH_STALL_MS,
        .pitch_stall_travel=GIMBAL_PITCH_STALL_TRAVEL,
        .pitch_stall_error=GIMBAL_PITCH_STALL_ERROR,
        .yaw_direction=GIMBAL_YAW_DIRECTION, .pitch_direction=GIMBAL_PITCH_DIRECTION,
        .control={.pitch_min=GIMBAL_PITCH_MIN, .pitch_max=GIMBAL_PITCH_MAX,
            .yaw_command_rate=0.3f, .pitch_command_rate=0.2f,
            .angle_kp={3,3}, .max_dt=0.02f, .max_rate={0.3f,0.2f},
            .space={GIMBAL_JOINT,GIMBAL_JOINT},
            .rate_pid={{.kp=GIMBAL_YAW_SPEED_KP,.output_limit=0.3f,.max_dt=0.02f},
                       {.kp=GIMBAL_PITCH_SPEED_KP,.ki=GIMBAL_PITCH_SPEED_KI,
                        .integral_limit=GIMBAL_PITCH_INTEGRAL_LIMIT,
                        .output_limit=GIMBAL_PITCH_TORQUE_LIMIT,.max_dt=0.02f}}}};
}
#endif
