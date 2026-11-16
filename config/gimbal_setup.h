#ifndef GIMBAL_SETUP_H
#define GIMBAL_SETUP_H
#include "gimbal_drive.h"

/* Fill from the actual motors and measured joint travel, then set to 1.
 * Protocol ranges are NOT mechanical travel limits. The Pitch working torque
 * cap was intentionally removed and now equals the DM4340 MIT mapping maximum,
 * so DM4340_T_MAX is the only hard torque limit left (the motor's own TMAX
 * setting still applies). Both motors must be MIT, and Pitch PMAX/VMAX/TMAX
 * must match dm4340_drv.h.
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
/* Lower endpoint adjusted around the observed raw position of 0.059107 rad. */
#define GIMBAL_PITCH_LOWER_RAW 0.060f
#define GIMBAL_PITCH_MIN (GIMBAL_PITCH_ZERO - GIMBAL_PITCH_LOWER_RAW)
#define GIMBAL_PITCH_MAX (GIMBAL_PITCH_ZERO - GIMBAL_PITCH_UPPER_RAW)
#define GIMBAL_YAW_DIRECTION 1
#define GIMBAL_PITCH_DIRECTION (-1)

/* Separate retains its configured yaw frame. Follow and spin always use
 * the IMU world heading; spin also bounds duration and joint travel. */
#define GIMBAL_YAW_WORLD 0
/* Manual yaw check with the matrix below: counterclockwise rotation viewed
 * from above gives positive yaw rate and increasing yaw. Static yaw drift
 * with nominal zero gyro bias was about +0.19 degrees over 10.1 s. */
#define GIMBAL_IMU_MOUNT_CONFIRMED 1
#define GIMBAL_SPIN_MAX_TIME_S 4.0f
#define GIMBAL_SPIN_MAX_TRAVEL_RAD 2.0f
/* BMI088 sensor axes -> gimbal body axes (x forward, y left, z up).
 * Use nominal alignment for the current yaw commissioning. */
#define GIMBAL_IMU_SENSOR_TO_BODY {{1.0f,0.0f,0.0f},{0.0f,1.0f,0.0f},{0.0f,0.0f,1.0f}}
/* Attitude fusion tuning. */
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
#define GIMBAL_YAW_ZERO 0.873759449f
#define GIMBAL_YAW_ZERO_CONFIRMED 1

/* User reported that the 2 N m commissioning cap still cannot raise the
 * barrel and asked to remove the torque cap. The rate loop may now command up
 * to the motor MIT mapping maximum; the integral contribution is scaled with
 * it (0.9x) instead of clamping the steady-state holding torque near 2 N m.
 * Joint limits, outward-torque blocking, the loaded-stall stop and the
 * command/feedback watchdogs remain active. The stall detector compares
 * against this same output limit, so it now only triggers near the motor
 * maximum. Hold the emergency stop ready during the first loaded test. */
#define GIMBAL_PITCH_TORQUE_LIMIT DM4340_T_MAX
#define GIMBAL_PITCH_SPEED_KP 2.0f
#define GIMBAL_PITCH_SPEED_KI 2.0f
/* Initial position stiffness, N m/rad; tune with the real loaded mechanism.
 * 0.1 rad displacement adds 2 N m immediately, inside the total torque cap. */
#define GIMBAL_PITCH_HOLD_KP 20.0f
#define GIMBAL_PITCH_INTEGRAL_LIMIT (0.9f * DM4340_T_MAX)
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
        .spin_hardware_confirmed=GIMBAL_IMU_MOUNT_CONFIRMED && GIMBAL_YAW_ZERO_CONFIRMED,
        .follow_hardware_confirmed=GIMBAL_IMU_MOUNT_CONFIRMED && GIMBAL_YAW_ZERO_CONFIRMED,
        .hardware_confirmed=GIMBAL_PITCH_ONLY_TEST || (GIMBAL_HARDWARE_CONFIRMED &&
            GIMBAL_YAW_PMAX > 0 && GIMBAL_YAW_VMAX > 0 && GIMBAL_YAW_TMAX >= 0.3f),
        .pitch_zero=GIMBAL_PITCH_ZERO,
        .pitch_stall_ms=GIMBAL_PITCH_STALL_MS,
        .pitch_stall_travel=GIMBAL_PITCH_STALL_TRAVEL,
        .pitch_stall_error=GIMBAL_PITCH_STALL_ERROR,
        .yaw_direction=GIMBAL_YAW_DIRECTION, .pitch_direction=GIMBAL_PITCH_DIRECTION,
        .control={.pitch_min=GIMBAL_PITCH_MIN, .pitch_max=GIMBAL_PITCH_MAX,
            .spin_max_time=GIMBAL_SPIN_MAX_TIME_S, .spin_max_travel=GIMBAL_SPIN_MAX_TRAVEL_RAD,
            .yaw_command_rate=0.3f, .pitch_command_rate=0.2f,
            .angle_kp={3,3}, .max_dt=0.02f, .max_rate={0.3f,0.2f},
            .pitch_hold_kp=GIMBAL_PITCH_HOLD_KP,
            .space={GIMBAL_YAW_WORLD ? GIMBAL_WORLD : GIMBAL_JOINT, GIMBAL_JOINT},
            .rate_pid={{.kp=GIMBAL_YAW_SPEED_KP,.output_limit=0.3f,.max_dt=0.02f},
                       {.kp=GIMBAL_PITCH_SPEED_KP,.ki=GIMBAL_PITCH_SPEED_KI,
                        .integral_limit=GIMBAL_PITCH_INTEGRAL_LIMIT,
                        .output_limit=GIMBAL_PITCH_TORQUE_LIMIT,.max_dt=0.02f}}}};
}
#endif
