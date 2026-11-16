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

/* P0-2 skeleton: hold the yaw axis in the world frame with the onboard IMU
 * instead of servoing the joint angle. Joystick yaw then rotates the world
 * heading while the chassis can turn underneath the gimbal, which is what the
 * chassis follow loop needs. Disabled until the attitude solution is validated:
 * a drifting or frozen world yaw makes the drive disarm (GIMBAL_FEEDBACK). */
#define GIMBAL_YAW_WORLD 0
/* BMI088 sensor axes -> gimbal body axes (x forward, y left, z up). The
 * identity assumes the chip is mounted upright and aligned with the yaw plate;
 * measure the real mounting and replace this matrix before enabling world yaw. */
#define GIMBAL_IMU_SENSOR_TO_BODY {{1.0f,0.0f,0.0f},{0.0f,1.0f,0.0f},{0.0f,0.0f,1.0f}}
/* Attitude fusion tuning; the mounting matrix lives in the gimbal main file
 * because it describes how the BMI088 is bolted to the yaw plate. */
#define GIMBAL_IMU_GRAVITY 9.80665f
#define GIMBAL_IMU_ACCEL_TOLERANCE 0.10f /* fraction of 1 g accepted for correction */
#define GIMBAL_IMU_CORRECTION_GAIN 1.0f  /* rad/s of tilt correction per unit error */
#define GIMBAL_IMU_MAX_DT 0.02f
#define GIMBAL_IMU_MAX_GYRO 35.0f        /* rad/s; BMI088 full scale is 2000 dps */

/* Measured yaw motor position with the gimbal pointing along chassis forward.
 * Measure it on hardware (read yaw_feedback.pos * GIMBAL_YAW_DIRECTION with the
 * gimbal centered), then set the value and GIMBAL_YAW_ZERO_CONFIRMED to 1.
 * Until confirmed the status frame reports calibrated=false and the chassis
 * refuses separate/follow, because a wrong zero skews translation and follow. */
#define GIMBAL_YAW_ZERO 0.0f
#define GIMBAL_YAW_ZERO_CONFIRMED 0

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
            .space={GIMBAL_YAW_WORLD ? GIMBAL_WORLD : GIMBAL_JOINT, GIMBAL_JOINT},
            .rate_pid={{.kp=GIMBAL_YAW_SPEED_KP,.output_limit=0.3f,.max_dt=0.02f},
                       {.kp=GIMBAL_PITCH_SPEED_KP,.ki=GIMBAL_PITCH_SPEED_KI,
                        .integral_limit=GIMBAL_PITCH_INTEGRAL_LIMIT,
                        .output_limit=GIMBAL_PITCH_TORQUE_LIMIT,.max_dt=0.02f}}}};
}
#endif
