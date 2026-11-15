#ifndef PITCH_MOTOR_H
#define PITCH_MOTOR_H
#include "dm4340_drv.h"
#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    DM4340_MotorFbPara_t motor;
    uint32_t last_rx_ms;
    uint32_t rx_count;
    bool received;
} PitchMotor_Feedback;

/* Owns CAN2 start, filter bank 14, FIFO1; shared CAN filter split is 14.
 * Initializes reception only, never enables the motor or transmits a command.
 * Mode must match the motor's existing configuration (does not change it). */
HAL_StatusTypeDef PitchMotor_Init(CAN_HandleTypeDef *hcan, uint16_t mode);
void PitchMotor_RxFifo1Callback(CAN_HandleTypeDef *hcan);
bool PitchMotor_GetFeedback(PitchMotor_Feedback *out);
/* Freshness only: a fault/disabled motor can still be online. */
bool PitchMotor_IsOnline(uint32_t timeout_ms);

/* Explicit main-loop operations. No automatic re-enable or stop retry.
 * Caller must enforce remote validity, measured mechanical limits and arming.
 * The MIT range in dm4340_drv.h is protocol scaling, NOT a joint safe range. */
HAL_StatusTypeDef PitchMotor_Enable(void);
HAL_StatusTypeDef PitchMotor_Disable(void);
HAL_StatusTypeDef PitchMotor_SendMIT(float pos, float vel, float kp, float kd, float torque);
HAL_StatusTypeDef PitchMotor_SendPosition(float pos, float speed_limit);
HAL_StatusTypeDef PitchMotor_SendSpeed(float vel);

#ifdef __cplusplus
}
#endif
#endif
