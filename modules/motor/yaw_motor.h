#ifndef YAW_MOTOR_H
#define YAW_MOTOR_H
#include "stm32f4xx_hal.h"
#include <stdbool.h>
/* Symmetric protocol ranges, read from actual PMAX/VMAX/TMAX configuration.
 * These are not mechanical limits or working torque limits. MIT mode only. */
typedef struct { float pmax, vmax, tmax; } YawMotor_Ranges;
typedef struct {
    uint16_t position_raw, velocity_raw, torque_raw;
    uint8_t state, mos_temperature, coil_temperature;
    float pos, vel, torque;
    uint32_t last_rx_ms, rx_count;
    bool received, scaled;
} YawMotor_Feedback;
/* Does not start CAN, change filters, enable motor or send anything.
 * NULL ranges permits raw reception only. Call before RX interrupts start. */
HAL_StatusTypeDef YawMotor_Init(CAN_HandleTypeDef *can, const YawMotor_Ranges *ranges);
bool YawMotor_Receive(const CAN_RxHeaderTypeDef *header, const uint8_t data[8]);
bool YawMotor_GetFeedback(YawMotor_Feedback *out);
bool YawMotor_IsOnline(uint32_t timeout_ms);
/* Read-only 0xCC request; does not enable or command motion. */
HAL_StatusTypeDef YawMotor_RequestFeedback(void);
HAL_StatusTypeDef YawMotor_Enable(void);
HAL_StatusTypeDef YawMotor_Disable(void);
/* Caller enforces arming, limits, fault handling, stop retries and CAN serialization. */
HAL_StatusTypeDef YawMotor_SendMIT(float pos, float vel, float kp, float kd, float torque);
typedef struct {
    uint32_t control_mode, timeout_count, tx_error_count;
    float pmax, vmax, tmax;
    uint8_t valid_mask; /* bits 0..3: mode, PMAX, VMAX, TMAX; complete=15 */
    bool finished;
} YawMotor_Params;
/* Read-only startup scan, three attempts per parameter, 100 ms timeout.
 * Call Service in main loop only while disarmed. No automatic range application. */
void YawMotor_ParamService(bool disarmed);
bool YawMotor_ReceiveParam(const CAN_RxHeaderTypeDef *header, const uint8_t data[8]);
void YawMotor_GetParams(YawMotor_Params *out);
#endif
