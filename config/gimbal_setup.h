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
/* Dual-axis control; Yaw CCW direction verified. */
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
/* Mechanical lower stop measured at 0.074255 rad (2026-11-17). */
#define GIMBAL_PITCH_LOWER_RAW 0.074f
#define GIMBAL_PITCH_MIN (GIMBAL_PITCH_ZERO - GIMBAL_PITCH_LOWER_RAW)
#define GIMBAL_PITCH_MAX (GIMBAL_PITCH_ZERO - GIMBAL_PITCH_UPPER_RAW)
#define GIMBAL_YAW_DIRECTION 1
#define GIMBAL_PITCH_DIRECTION (-1)

/* Separate retains its configured yaw frame. Follow and spin always use
 * the IMU world heading. */
#define GIMBAL_YAW_WORLD 0
/* Manual yaw check with the matrix below: counterclockwise rotation viewed
 * from above gives positive yaw rate and increasing yaw. Static yaw drift
 * with nominal zero gyro bias was about +0.19 degrees over 10.1 s. */
#define GIMBAL_IMU_MOUNT_CONFIRMED 1
/* Zero disables the corresponding spin-only stop condition. */
#define GIMBAL_SPIN_MAX_TIME_S 0.0f
#define GIMBAL_SPIN_MAX_TRAVEL_RAD 0.0f
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
/* Follow settled 90 degrees CCW from chassis forward at the former zero
 * (0.873759449 rad). Shift the reference clockwise by pi/2. */
#define GIMBAL_YAW_ZERO (-0.697036878f)
#define GIMBAL_YAW_ZERO_CONFIRMED 1

/* User reported that the 2 N m commissioning cap still cannot raise the
 * barrel and asked to remove the torque cap. The rate loop may now command up
 * to the motor MIT mapping maximum; the integral contribution is scaled with
 * it (0.9x) instead of clamping the steady-state holding torque near 2 N m. */
#define GIMBAL_PITCH_TORQUE_LIMIT DM4340_T_MAX
/* Damping gain of the pitch rate loop. Doubled from 2.0 on 2026-11-17: the
 * hold spring (150 N m/rad) only had angle_kp*Kp = 6 N m/(rad/s) of damping
 * and still overshot on the real joint. Much higher values turn encoder
 * differentiation noise into torque ripple. */
#define GIMBAL_PITCH_SPEED_KP 4.0f
#define GIMBAL_PITCH_SPEED_KI 2.0f
/* Static position stiffness, N m/rad; tune with the real loaded mechanism.
 * 0.1 rad displacement adds 15 N m immediately and reaches the 28 N m cap at
 * about 0.18 rad.
 * 200 overshot on the real mechanism, 150 is the last value without reported
 * overshoot; keep the rate-loop damping when raising it again. */
#define GIMBAL_PITCH_HOLD_KP 150.0f
#define GIMBAL_PITCH_INTEGRAL_LIMIT (0.9f * DM4340_T_MAX)
/* Full-stick pitch rate and outer-loop limit, rad/s. At 2.00 rad/s the
 * target leads the measured angle by rate / angle_kp = 0.67 rad while the
 * stick is held, so a release lets the joint glide about 0.2 rad to the
 * frozen target; raise angle_kp[1] if that feels loose on the real mechanism. */
#define GIMBAL_PITCH_MAX_RATE 2.00f

/* Higher rate-loop Kp caused oscillation on hardware. At neutral, use a
 * higher angle-loop gain to correct heading drift after release.
 * The DM-J4310-2EC peak torque is 7 N m and its rating is 3 N m. */
#define GIMBAL_YAW_SPEED_KP 1.20f
#define GIMBAL_YAW_SPEED_KI 3.0f
#define GIMBAL_YAW_HOLD_ANGLE_KP 5.9f
#define GIMBAL_YAW_HOLD_INTEGRAL_LIMIT 0.35f /* N m at neutral */
/* Time constant for estimating chassis rotation from IMU and joint rates. */
#define GIMBAL_YAW_JOINT_RATE_TAU 0.04f
#define GIMBAL_YAW_TORQUE_LIMIT 7.00f
#define GIMBAL_YAW_MAX_RATE 2.00f /* rad/s, full-stick heading rate and outer-loop limit */
#define GIMBAL_YAW_HOLD_MAX_RATE 3.00f /* rad/s, return to the neutral hold target */
/* Allow full torque at maximum rate reference with zero measured speed. */
#define GIMBAL_YAW_INTEGRAL_LIMIT (GIMBAL_YAW_TORQUE_LIMIT - GIMBAL_YAW_SPEED_KP * GIMBAL_YAW_MAX_RATE)

/* Conservative initial gains in N m; not final tuned gains. */
static inline GimbalDriveConfig GimbalSetup_Config(void)
{
    const GimbalDriveConfig config = {
        .control={.pitch_min=GIMBAL_PITCH_MIN, .pitch_max=GIMBAL_PITCH_MAX,
            .yaw_command_rate=GIMBAL_YAW_MAX_RATE, .pitch_command_rate=GIMBAL_PITCH_MAX_RATE,
            .angle_kp={3,3}, .max_dt=0.02f,
            .rate_pid={{.kp=GIMBAL_YAW_SPEED_KP,.ki=GIMBAL_YAW_SPEED_KI,
                        .output_limit=GIMBAL_YAW_TORQUE_LIMIT,
                        .integral_limit=GIMBAL_YAW_INTEGRAL_LIMIT,.max_dt=0.02f},
                       {.kp=GIMBAL_PITCH_SPEED_KP,.ki=GIMBAL_PITCH_SPEED_KI,
                        .output_limit=GIMBAL_PITCH_TORQUE_LIMIT,
                        .integral_limit=GIMBAL_PITCH_INTEGRAL_LIMIT,.max_dt=0.02f}},
            .max_rate={GIMBAL_YAW_MAX_RATE,GIMBAL_PITCH_MAX_RATE},
            .yaw_hold_max_rate=GIMBAL_YAW_HOLD_MAX_RATE,
            .yaw_hold_angle_kp=GIMBAL_YAW_HOLD_ANGLE_KP,
            .yaw_hold_integral_limit=GIMBAL_YAW_HOLD_INTEGRAL_LIMIT,
            .yaw_joint_rate_tau=GIMBAL_YAW_JOINT_RATE_TAU,
            .pitch_hold_kp=GIMBAL_PITCH_HOLD_KP,
            .space={GIMBAL_YAW_WORLD ? GIMBAL_WORLD : GIMBAL_JOINT, GIMBAL_JOINT},
            .spin_max_time=GIMBAL_SPIN_MAX_TIME_S, .spin_max_travel=GIMBAL_SPIN_MAX_TRAVEL_RAD},
        .pitch_zero=GIMBAL_PITCH_ZERO,
        .yaw_direction=GIMBAL_YAW_DIRECTION, .pitch_direction=GIMBAL_PITCH_DIRECTION,
        .hardware_confirmed=GIMBAL_PITCH_ONLY_TEST || (GIMBAL_HARDWARE_CONFIRMED &&
            GIMBAL_YAW_PMAX > 0 && GIMBAL_YAW_VMAX > 0 &&
            GIMBAL_YAW_TMAX >= GIMBAL_YAW_TORQUE_LIMIT),
        .spin_hardware_confirmed=GIMBAL_IMU_MOUNT_CONFIRMED && GIMBAL_YAW_ZERO_CONFIRMED,
        .follow_hardware_confirmed=GIMBAL_IMU_MOUNT_CONFIRMED && GIMBAL_YAW_ZERO_CONFIRMED,
        .pitch_only=GIMBAL_PITCH_ONLY_TEST};
    return config;
}
#endif
