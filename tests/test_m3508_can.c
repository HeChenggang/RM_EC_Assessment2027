/* HAL test doubles: compiled for Cortex-M4 and run by run_motor_tests.py. */
#include "m3508_can.h"
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
    for (unsigned i = 0; i < 8; ++i) sent_data[i] = data[i];
    *mailbox = CAN_TX_MAILBOX0;
    ++send_count;
    return tx_result;
}

int main(void)
{
    static CAN_HandleTypeDef can, other;
    can.Instance = CAN2;
    other.Instance = CAN1;
    M3508_Feedback f = {0};
    const int16_t currents[4] = {1234, -1234, 32767, -32768};
    const uint8_t expected[8] = {0x04, 0xd2, 0xfb, 0x2e, 0x40, 0, 0xc0, 0};
    CHECK(M3508_CAN_SendCurrents(currents) == HAL_ERROR);
    CHECK(M3508_CAN_Init(NULL) == HAL_ERROR);
    CHECK(M3508_CAN_Init(&other) == HAL_ERROR);
    config_result = HAL_ERROR;
    CHECK(M3508_CAN_Init(&can) == HAL_ERROR);
    config_result = HAL_OK;
    CHECK(M3508_CAN_Init(&can) == HAL_OK);
    CHECK(M3508_CAN_Init(&can) == HAL_ERROR);
    CHECK(configured_filter.FilterBank == 14 && configured_filter.SlaveStartFilterBank == 14);
    CHECK(configured_filter.FilterFIFOAssignment == CAN_FILTER_FIFO1);
    CHECK(configured_filter.FilterMode == CAN_FILTERMODE_IDLIST);
    CHECK(configured_filter.FilterIdHigh == (0x201U << 5));
    CHECK(configured_filter.FilterIdLow == (0x202U << 5));
    CHECK(configured_filter.FilterMaskIdHigh == (0x203U << 5));
    CHECK(configured_filter.FilterMaskIdLow == (0x204U << 5));
    CHECK(!M3508_CAN_IsOnline(1, 100));
    CHECK(!M3508_CAN_GetFeedback(0, &f));
    CHECK(!M3508_CAN_GetFeedback(5, &f));
    CHECK(!M3508_CAN_GetFeedback(1, NULL));

    CHECK(M3508_CAN_SendCurrents(currents) == HAL_OK);
    CHECK(sent_header.StdId == 0x200 && sent_header.DLC == 8);
    CHECK(sent_header.IDE == CAN_ID_STD && sent_header.RTR == CAN_RTR_DATA);
    for (unsigned i = 0; i < 8; ++i) CHECK(sent_data[i] == expected[i]);
    free_mailboxes = 2;
    CHECK(M3508_CAN_SendCurrents(currents) == HAL_BUSY && send_count == 1);
    free_mailboxes = 3;
    tx_result = HAL_ERROR;
    CHECK(M3508_CAN_SendCurrents(currents) == HAL_ERROR);
    CHECK(M3508_CAN_SendCurrents(NULL) == HAL_ERROR);

    rx_header.IDE = CAN_ID_STD; rx_header.RTR = CAN_RTR_DATA; rx_header.DLC = 8;
    rx_data[0] = 0x1f; rx_data[1] = 0xff; /* angle 8191 */
    rx_data[2] = 0xfb; rx_data[3] = 0x2e; /* speed -1234 */
    rx_data[4] = 0x80; rx_data[5] = 0;    /* current -32768 */
    rx_data[6] = 65;
    tick = 0xfffffff0U;
    for (uint8_t id = 1; id <= 4; ++id) {
        rx_header.StdId = 0x200U + id; pending = 1;
        M3508_CAN_RxFifo1Callback(&other);
        CHECK(pending == 1);
        M3508_CAN_RxFifo1Callback(&can);
        CHECK(pending == 0 && M3508_CAN_GetFeedback(id, &f));
        CHECK(f.encoder == 8191 && f.speed_rpm == -1234);
        CHECK(f.current_raw == -32768 && f.temperature_c == 65 && f.rx_count == 1);
    }
    tick = 0x10;
    CHECK(M3508_CAN_IsOnline(1, 33));
    CHECK(!M3508_CAN_IsOnline(1, 32));
    CHECK(!M3508_CAN_IsOnline(1, 0));

    /* Malformed frames must not update data or freshness. */
    rx_header.StdId = 0x201;
    for (unsigned bad = 0; bad < 5; ++bad) {
        rx_header.DLC = bad == 0 ? 7 : 8;
        rx_header.IDE = bad == 1 ? CAN_ID_EXT : CAN_ID_STD;
        rx_header.RTR = bad == 2 ? CAN_RTR_REMOTE : CAN_RTR_DATA;
        rx_header.StdId = bad == 3 ? 0x205 : 0x201;
        rx_data[0] = bad == 4 ? 0x20 : 0x1f;
        pending = 1;
        M3508_CAN_RxFifo1Callback(&can);
        CHECK(M3508_CAN_GetFeedback(1, &f) && f.rx_count == 1);
        CHECK(f.last_rx_ms == 0xfffffff0U);
    }
    __disable_irq();
    CHECK(M3508_CAN_GetFeedback(1, &f));
    CHECK(__get_PRIMASK() == 1);
    __enable_irq();
    return 0;
}
