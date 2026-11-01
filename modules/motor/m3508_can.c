#include "m3508_can.h"

#include <stddef.h>

static CAN_HandleTypeDef *motor_can;
static volatile M3508_Feedback feedback[M3508_COUNT];

static uint16_t read_u16_be(const uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
}

static int16_t read_i16_be(const uint8_t *data)
{
    const uint16_t value = read_u16_be(data);
    return (int16_t)((value & 0x8000U) ? (int32_t)value - 65536 : value);
}

HAL_StatusTypeDef M3508_CAN_Init(CAN_HandleTypeDef *hcan)
{
    if (hcan == NULL || hcan->Instance != CAN2 || motor_can != NULL) {
        return HAL_ERROR;
    }

    /* 16-bit list entries encode STDID at bits 15:5, RTR=0, IDE=0.
     * Accept exactly 0x201..0x204 standard data frames. */
    CAN_FilterTypeDef filter = {0};
    filter.FilterBank = 14;
    filter.FilterMode = CAN_FILTERMODE_IDLIST;
    filter.FilterScale = CAN_FILTERSCALE_16BIT;
    filter.FilterIdHigh = 0x201U << 5;
    filter.FilterIdLow = 0x202U << 5;
    filter.FilterMaskIdHigh = 0x203U << 5;
    filter.FilterMaskIdLow = 0x204U << 5;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO1;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;

    HAL_StatusTypeDef status = HAL_CAN_ConfigFilter(hcan, &filter);
    if (status != HAL_OK) {
        return status;
    }
    status = HAL_CAN_Start(hcan);
    if (status != HAL_OK) {
        return status;
    }

    /* Publish state before enabling receive notifications. */
    for (uint32_t i = 0; i < M3508_COUNT; ++i) {
        const M3508_Feedback empty = {0};
        feedback[i] = empty;
    }
    motor_can = hcan;
    status = HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
    if (status != HAL_OK) {
        motor_can = NULL;
        (void)HAL_CAN_Stop(hcan);
    }
    return status;
}

void M3508_CAN_RxFifo1Callback(CAN_HandleTypeDef *hcan)
{
    if (hcan == NULL || hcan != motor_can) {
        return;
    }
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO1) != 0U) {
        CAN_RxHeaderTypeDef header = {0};
        uint8_t data[8];
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &header, data) != HAL_OK) {
            break;
        }
        if (header.IDE != CAN_ID_STD || header.RTR != CAN_RTR_DATA ||
            header.DLC != 8U || header.StdId < 0x201U || header.StdId > 0x204U) {
            continue;
        }
        const uint16_t encoder = read_u16_be(data);
        if (encoder > 8191U) {
            continue;
        }
        volatile M3508_Feedback *motor = &feedback[header.StdId - 0x201U];
        motor->encoder = encoder;
        motor->speed_rpm = read_i16_be(&data[2]);
        motor->current_raw = read_i16_be(&data[4]);
        motor->temperature_c = data[6];
        motor->last_rx_ms = HAL_GetTick();
        motor->rx_count++;
        motor->received = true;
    }
}

bool M3508_CAN_GetFeedback(uint8_t motor_id, M3508_Feedback *out)
{
    if (out == NULL || motor_id < 1U || motor_id > M3508_COUNT) {
        return false;
    }
    /* Prevent an RX interrupt from mixing two feedback frames in one copy. */
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    *out = feedback[motor_id - 1U];
    __set_PRIMASK(primask);
    return out->received;
}

bool M3508_CAN_IsOnline(uint8_t motor_id, uint32_t timeout_ms)
{
    M3508_Feedback value;
    return M3508_CAN_GetFeedback(motor_id, &value) &&
           (uint32_t)(HAL_GetTick() - value.last_rx_ms) < timeout_ms;
}

HAL_StatusTypeDef M3508_CAN_SendCurrents(const int16_t currents[M3508_COUNT])
{
    if (motor_can == NULL || currents == NULL ||
        HAL_CAN_GetState(motor_can) != HAL_CAN_STATE_LISTENING) {
        return HAL_ERROR;
    }
    /* Do not accumulate old control commands in the three TX mailboxes. */
    if (HAL_CAN_GetTxMailboxesFreeLevel(motor_can) != 3U) {
        return HAL_BUSY;
    }
    CAN_TxHeaderTypeDef header = {0};
    header.StdId = 0x200U;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8U;
    header.TransmitGlobalTime = DISABLE;

    uint8_t data[8];
    for (uint32_t i = 0; i < M3508_COUNT; ++i) {
        int32_t value = currents[i];
        if (value > M3508_CURRENT_MAX) value = M3508_CURRENT_MAX;
        if (value < -M3508_CURRENT_MAX) value = -M3508_CURRENT_MAX;
        const uint16_t raw = (uint16_t)value;
        data[2U * i] = (uint8_t)(raw >> 8);
        data[2U * i + 1U] = (uint8_t)raw;
    }
    uint32_t mailbox;
    return HAL_CAN_AddTxMessage(motor_can, &header, data, &mailbox);
}
