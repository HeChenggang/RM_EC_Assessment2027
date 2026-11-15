#include "pitch_motor.h"
#include "gimbal_board.h"
#include <stddef.h>

static CAN_HandleTypeDef *pitch_can;
static DM4340_Motor_t pitch;
static volatile PitchMotor_Feedback feedback;

HAL_StatusTypeDef PitchMotor_Init(CAN_HandleTypeDef *hcan, uint16_t mode)
{
    if (hcan == NULL || hcan->Instance != CAN2 || pitch_can != NULL ||
        dm4340_joint_motor_init(&pitch, GIMBAL_PITCH_TX_ID, mode) != HAL_OK) {
        return HAL_ERROR;
    }
    CAN_FilterTypeDef filter = {0};
    filter.FilterBank = 14;
    filter.SlaveStartFilterBank = 14;
    filter.FilterMode = CAN_FILTERMODE_IDLIST;
    filter.FilterScale = CAN_FILTERSCALE_16BIT;
    filter.FilterIdHigh = GIMBAL_PITCH_RX_ID << 5;
    filter.FilterIdLow = GIMBAL_PITCH_RX_ID << 5;
    filter.FilterMaskIdHigh = GIMBAL_PITCH_RX_ID << 5;
    filter.FilterMaskIdLow = GIMBAL_PITCH_RX_ID << 5;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO1;
    filter.FilterActivation = ENABLE;
    HAL_StatusTypeDef status = HAL_CAN_ConfigFilter(hcan, &filter);
    if (status != HAL_OK) return status;
    status = HAL_CAN_Start(hcan);
    if (status != HAL_OK) return status;
    const PitchMotor_Feedback empty = {0};
    feedback = empty;
    pitch_can = hcan;
    status = HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
    if (status != HAL_OK) {
        pitch_can = NULL;
        (void)HAL_CAN_Stop(hcan);
    }
    return status;
}

void PitchMotor_RxFifo1Callback(CAN_HandleTypeDef *hcan)
{
    if (hcan == NULL || hcan != pitch_can) return;
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO1) != 0U) {
        CAN_RxHeaderTypeDef header = {0};
        uint8_t data[8];
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &header, data) != HAL_OK) break;
        if (header.IDE != CAN_ID_STD || header.RTR != CAN_RTR_DATA ||
            header.StdId != GIMBAL_PITCH_RX_ID || header.DLC != 8U ||
            !dm4340_fbdata(&pitch, data, header.DLC)) continue;
        feedback.motor = pitch.para;
        feedback.last_rx_ms = HAL_GetTick();
        feedback.rx_count++;
        feedback.received = true;
    }
}

bool PitchMotor_GetFeedback(PitchMotor_Feedback *out)
{
    if (out == NULL) return false;
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    *out = feedback;
    __set_PRIMASK(primask);
    return out->received;
}

bool PitchMotor_IsOnline(uint32_t timeout_ms)
{
    PitchMotor_Feedback value;
    return PitchMotor_GetFeedback(&value) &&
        (uint32_t)(HAL_GetTick() - value.last_rx_ms) < timeout_ms;
}

HAL_StatusTypeDef PitchMotor_Enable(void)
{
    return dm4340_enable_motor_mode(pitch_can, GIMBAL_PITCH_TX_ID, pitch.mode);
}

HAL_StatusTypeDef PitchMotor_Disable(void)
{
    return dm4340_disable_motor_mode(pitch_can, GIMBAL_PITCH_TX_ID, pitch.mode);
}

HAL_StatusTypeDef PitchMotor_SendMIT(float pos, float vel, float kp, float kd, float torque)
{
    if (pitch.mode != DM4340_MIT_MODE) return HAL_ERROR;
    return dm4340_mit_ctrl(pitch_can, GIMBAL_PITCH_TX_ID, pos, vel, kp, kd, torque);
}

HAL_StatusTypeDef PitchMotor_SendPosition(float pos, float speed_limit)
{
    if (pitch.mode != DM4340_POS_MODE) return HAL_ERROR;
    return dm4340_pos_speed_ctrl(pitch_can, GIMBAL_PITCH_TX_ID, pos, speed_limit);
}

HAL_StatusTypeDef PitchMotor_SendSpeed(float vel)
{
    if (pitch.mode != DM4340_SPEED_MODE) return HAL_ERROR;
    return dm4340_speed_ctrl(pitch_can, GIMBAL_PITCH_TX_ID, vel);
}
