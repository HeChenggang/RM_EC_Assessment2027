/* HAL test doubles: compiled for Cortex-M4 and run by run_motor_tests.py. */
#include "pitch_motor.h"
#include <math.h>
#include <stddef.h>

#define CHECK(condition) do { if (!(condition)) return __LINE__; } while (0)

static uint32_t tick, free_mailboxes = 3, pending;
static CAN_FilterTypeDef configured_filter;
static CAN_TxHeaderTypeDef sent_header;
static uint8_t sent_data[8];
static uint32_t send_count;
static HAL_StatusTypeDef config_result = HAL_OK;
static HAL_StatusTypeDef tx_result = HAL_OK;
static CAN_RxHeaderTypeDef rx_header;
static uint8_t rx_data[8];

uint32_t HAL_GetTick(void) { return tick; }
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *h, const CAN_FilterTypeDef *f)
{ (void)h; configured_filter = *f; return config_result; }
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *h)
{ h->State = HAL_CAN_STATE_LISTENING; return HAL_OK; }
HAL_StatusTypeDef HAL_CAN_Stop(CAN_HandleTypeDef *h)
{ h->State = HAL_CAN_STATE_READY; return HAL_OK; }
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *h, uint32_t mask)
{ (void)h; return mask == CAN_IT_RX_FIFO1_MSG_PENDING ? HAL_OK : HAL_ERROR; }
uint32_t HAL_CAN_GetRxFifoFillLevel(const CAN_HandleTypeDef *h, uint32_t fifo)
{ (void)h; return fifo == CAN_RX_FIFO1 ? pending : 0; }
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *h, uint32_t fifo,
                                     CAN_RxHeaderTypeDef *header, uint8_t data[])
{
    (void)h; (void)fifo;
    *header = rx_header;
    for (unsigned i = 0; i < 8; ++i) data[i] = rx_data[i];
    --pending;
    return HAL_OK;
}
HAL_CAN_StateTypeDef HAL_CAN_GetState(const CAN_HandleTypeDef *h) { return h->State; }
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(const CAN_HandleTypeDef *h)
{ (void)h; return free_mailboxes; }
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *h, const CAN_TxHeaderTypeDef *header,
                                     const uint8_t data[], uint32_t *mailbox)
{
    (void)h;
    sent_header = *header;
    for (unsigned i = 0; i < header->DLC; ++i) sent_data[i] = data[i];
    *mailbox = CAN_TX_MAILBOX0;
    ++send_count;
    return tx_result;
}

int main(void)
{
    static CAN_HandleTypeDef can, other;
    can.Instance = CAN2; other.Instance = CAN1;
    PitchMotor_Feedback f = {0};
    CHECK(PitchMotor_Enable() == HAL_ERROR);
    CHECK(PitchMotor_Init(NULL, 0) == HAL_ERROR);
    CHECK(PitchMotor_Init(&other, 0) == HAL_ERROR);
    CHECK(PitchMotor_Init(&can, 1) == HAL_ERROR);
    config_result = HAL_ERROR;
    CHECK(PitchMotor_Init(&can, 0) == HAL_ERROR);
    config_result = HAL_OK;
    CHECK(PitchMotor_Init(&can, 0) == HAL_OK);
    CHECK(PitchMotor_Init(&can, 0) == HAL_ERROR);
    CHECK(send_count == 0);
    CHECK(configured_filter.FilterBank == 14 && configured_filter.SlaveStartFilterBank == 14);
    CHECK(configured_filter.FilterMode == CAN_FILTERMODE_IDLIST);
    CHECK(configured_filter.FilterFIFOAssignment == CAN_FILTER_FIFO1);
    CHECK(configured_filter.FilterIdHigh == (0x19U << 5));
    CHECK(configured_filter.FilterIdLow == (0x19U << 5));
    CHECK(configured_filter.FilterMaskIdHigh == (0x19U << 5));
    CHECK(configured_filter.FilterMaskIdLow == (0x19U << 5));
    CHECK(!PitchMotor_IsOnline(100));
    CHECK(!PitchMotor_GetFeedback(NULL));
    CHECK(PitchMotor_Enable() == HAL_OK);
    CHECK(sent_header.StdId == 9 && sent_header.DLC == 8);
    CHECK(sent_header.IDE == CAN_ID_STD && sent_header.RTR == CAN_RTR_DATA);
    for (unsigned i = 0; i < 7; ++i) CHECK(sent_data[i] == 0xff);
    CHECK(sent_data[7] == 0xfc);
    CHECK(PitchMotor_Disable() == HAL_OK && sent_data[7] == 0xfd);
    CHECK(PitchMotor_SendMIT(0, 0, 0, 0, 0) == HAL_OK);
    const uint8_t zero[8] = {0x7f, 0xff, 0x7f, 0xf0, 0, 0, 7, 0xff};
    for (unsigned i = 0; i < 8; ++i) CHECK(sent_data[i] == zero[i]);
    CHECK(PitchMotor_SendMIT(DM4340_P_MIN, -10, 0, 0, -28) == HAL_OK);
    for (unsigned i = 0; i < 8; ++i) CHECK(sent_data[i] == 0);
    const uint32_t count = send_count;
    CHECK(PitchMotor_SendMIT(NAN, 0, 0, 0, 0) == HAL_ERROR);
    CHECK(PitchMotor_SendMIT(0, INFINITY, 0, 0, 0) == HAL_ERROR);
    CHECK(PitchMotor_SendMIT(0, 0, -1, 0, 0) == HAL_ERROR);
    CHECK(PitchMotor_SendMIT(0, 0, 0, 6, 0) == HAL_ERROR);
    CHECK(PitchMotor_SendMIT(0, 0, 0, 0, 29) == HAL_ERROR);
    CHECK(PitchMotor_SendMIT(DM4340_P_MAX, 10, 500, 5, 28) == HAL_ERROR);
    CHECK(PitchMotor_SendPosition(0, 1) == HAL_ERROR);
    CHECK(PitchMotor_SendSpeed(0) == HAL_ERROR);
    CHECK(send_count == count);
    free_mailboxes = 2;
    CHECK(PitchMotor_SendMIT(0, 0, 0, 0, 0) == HAL_BUSY);
    CHECK(send_count == count);
    free_mailboxes = 3; tx_result = HAL_ERROR;
    CHECK(PitchMotor_Disable() == HAL_ERROR);
    tx_result = HAL_OK;
    CHECK(dm4340_pos_speed_ctrl(&can, 9, 1.0f, 2.0f) == HAL_OK);
    CHECK(sent_header.StdId == 0x109 && sent_header.DLC == 8);
    const uint8_t pos[8] = {0, 0, 0x80, 0x3f, 0, 0, 0, 0x40};
    for (unsigned i = 0; i < 8; ++i) CHECK(sent_data[i] == pos[i]);
    CHECK(dm4340_speed_ctrl(&can, 9, -2.0f) == HAL_OK);
    CHECK(sent_header.StdId == 0x209 && sent_header.DLC == 4);
    CHECK(sent_data[0] == 0 && sent_data[1] == 0 && sent_data[2] == 0 && sent_data[3] == 0xc0);
    CHECK(dm4340_enable_motor_mode(&can, 9, 0x100) == HAL_OK && sent_header.StdId == 0x109);
    CHECK(dm4340_disable_motor_mode(&can, 9, 0x200) == HAL_OK && sent_header.StdId == 0x209);
    CHECK(dm4340_enable_motor_mode(&can, 9, 0x300) == HAL_ERROR);
    CHECK(dm4340_speed_ctrl(&can, 16, 0) == HAL_ERROR);
    CHECK(dm4340_pos_speed_ctrl(&can, 9, 0, -1) == HAL_ERROR);
    CHECK(dm4340_speed_ctrl(&can, 9, NAN) == HAL_ERROR);
    can.State = HAL_CAN_STATE_READY;
    CHECK(PitchMotor_Enable() == HAL_ERROR);
    can.State = HAL_CAN_STATE_LISTENING;

    rx_header.IDE = CAN_ID_STD; rx_header.RTR = CAN_RTR_DATA;
    rx_header.DLC = 8; rx_header.StdId = 0x19;
    rx_data[0] = 0x19; /* state 1, motor 9 */
    rx_data[1] = 0xff; rx_data[2] = 0xff;
    rx_data[3] = 0; rx_data[4] = 0x0f; rx_data[5] = 0xff;
    rx_data[6] = 42; rx_data[7] = 53;
    tick = 0xfffffff0; pending = 1;
    PitchMotor_RxFifo1Callback(&other);
    CHECK(pending == 1);
    PitchMotor_RxFifo1Callback(&can);
    CHECK(PitchMotor_GetFeedback(&f) && f.rx_count == 1);
    CHECK(f.motor.id == 9 && f.motor.state == 1);
    CHECK(f.motor.pos == DM4340_P_MAX && f.motor.vel == -10 && f.motor.tor == 28);
    CHECK(f.motor.Tmos == 42 && f.motor.Tcoil == 53);
    tick = 0x10;
    CHECK(PitchMotor_IsOnline(33) && !PitchMotor_IsOnline(32));
    for (unsigned bad = 0; bad < 5; ++bad) {
        rx_header.DLC = bad == 0 ? 7 : 8;
        rx_header.IDE = bad == 1 ? CAN_ID_EXT : CAN_ID_STD;
        rx_header.RTR = bad == 2 ? CAN_RTR_REMOTE : CAN_RTR_DATA;
        rx_header.StdId = bad == 3 ? 0x13 : 0x19;
        rx_data[0] = bad == 4 ? 0x18 : 0x19;
        pending = 1; PitchMotor_RxFifo1Callback(&can);
        CHECK(PitchMotor_GetFeedback(&f) && f.rx_count == 1 && f.last_rx_ms == 0xfffffff0);
    }
    __disable_irq();
    CHECK(PitchMotor_GetFeedback(&f) && __get_PRIMASK() == 1);
    __enable_irq();
    DM4340_Motor_t motor;
    CHECK(dm4340_joint_motor_init(NULL, 9, 0) == HAL_ERROR);
    CHECK(dm4340_joint_motor_init(&motor, 9, 0) == HAL_OK);
    CHECK(!dm4340_fbdata(NULL, rx_data, 8));
    CHECK(!dm4340_fbdata(&motor, NULL, 8));
    return 0;
}
