#ifndef GIMBAL_DRIVE_H
#define GIMBAL_DRIVE_H
#include "gimbal_control.h"
#include "pitch_motor.h"
#include "yaw_motor.h"

typedef enum { GIMBAL_STOPPED, GIMBAL_PREPARING, GIMBAL_ENABLING, GIMBAL_ACTIVE } GimbalDriveState;
typedef enum { GIMBAL_OK, GIMBAL_CONFIG, GIMBAL_LINK, GIMBAL_FEEDBACK,
               GIMBAL_LIMIT, GIMBAL_TX, GIMBAL_ENABLE_TIMEOUT, GIMBAL_TIMING,
               GIMBAL_STALL } GimbalDriveFault;
typedef struct {
    GimbalConfig control;
    float pitch_zero;
    int yaw_direction, pitch_direction;
    bool hardware_confirmed;
    bool spin_hardware_confirmed; /* IMU mounting and chassis-relative zero */
    bool pitch_only; /* Commission Pitch without enabling/commanding Yaw. */
    uint32_t pitch_stall_ms;
    float pitch_stall_travel, pitch_stall_error; /* rad; both positive */
} GimbalDriveConfig;
typedef struct {
    GimbalControl control;
    GimbalDriveConfig config;
    CAN_HandleTypeDef *bus[2]; /* exclusively owns TX on these upper-board buses */
    GimbalDriveState state;
    GimbalDriveFault fault;
    GimbalOutput output;
    HAL_StatusTypeDef tx[2];
    uint32_t last_step, phase_started, last_stop, enable_count[2];
    bool configured, ready, stop_sent;
    GimbalDriveFault last_fault; /* Retained across disarm; clears on next arm attempt. */
    bool pitch_stall_tracking;
    uint32_t pitch_stall_started;
    float pitch_stall_position;
    /* Attitude adapter output for a WORLD yaw axis; see GimbalDrive_SetWorld. */
    float world_yaw, world_rate;
    uint32_t world_ms;
    bool world_received;
} GimbalDrive;
/* Drivers must already be initialized with verified MIT protocol ranges.
 * Call from main only; never writes zero position or motor parameters. */
bool GimbalDrive_Init(GimbalDrive *s, const GimbalDriveConfig *config,
                      CAN_HandleTypeDef *yaw_can, CAN_HandleTypeDef *pitch_can, uint32_t now);
void GimbalDrive_Step(GimbalDrive *s, const RobotCommand *cmd, bool link_fresh,
                      const YawMotor_Feedback *yaw, const PitchMotor_Feedback *pitch,
                      uint32_t now);
/* Feed the gimbal heading measured by the onboard attitude solution when the
 * yaw axis is configured as GIMBAL_WORLD or spin may be selected.
 * Call every attitude update; the
 * sample expires after a short timeout and the drive then disarms. */
void GimbalDrive_SetWorld(GimbalDrive *s, float yaw, float yaw_rate, uint32_t now);
#endif
