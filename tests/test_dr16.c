#include "dr16.h"
#include <stddef.h>

#define CHECK(x) do { if (!(x)) return __LINE__; } while (0)
static uint32_t tick, starts, aborts;
static uint8_t *dma_buffer;
static uint16_t dma_size, position;
static HAL_StatusTypeDef start_result = HAL_OK, abort_result = HAL_OK;
static HAL_UART_RxEventTypeTypeDef event;
static UART_HandleTypeDef uart, other;
static DMA_HandleTypeDef dma;

uint32_t HAL_GetTick(void) { return tick; }
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *h, uint8_t *b, uint16_t n)
{ (void)h; dma_buffer = b; dma_size = n; ++starts; return start_result; }
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *h)
{ (void)h; ++aborts; return abort_result; }
HAL_UART_RxEventTypeTypeDef HAL_UARTEx_GetRxEventType(UART_HandleTypeDef *h)
{ (void)h; return event; }

static void make_frame(uint8_t b[18], const uint16_t ch[5])
{
    for (unsigned i = 0; i < 18; ++i) b[i] = 0;
    for (unsigned c = 0; c < 4; ++c)
        for (unsigned bit = 0; bit < 11; ++bit)
            if (ch[c] & (1U << bit)) b[(c * 11 + bit) / 8] |= 1U << ((c * 11 + bit) % 8);
    b[5] |= (DR16_SWITCH_DOWN << 6) | (DR16_SWITCH_MIDDLE << 4);
    b[6] = 0xfe; b[7] = 0xff; /* -2 */
    b[8] = 0xff; b[9] = 0x7f; /* 32767 */
    b[10] = 0; b[11] = 0x80;  /* -32768 */
    b[12] = 1; b[14] = 0x81; b[15] = 0x80;
    b[16] = ch[4] & 0xff; b[17] = ch[4] >> 8;
}

/* Emulate circular DMA HT/TC at real write boundaries; neither is a frame. */
static bool feed(const uint8_t *b, unsigned n, bool idle)
{
    bool accepted = false;
    for (unsigned i = 0; i < n; ++i) {
        dma_buffer[position++] = b[i];
        if (position == dma_size / 2 || position == dma_size) {
            event = position == dma_size ? HAL_UART_RXEVENT_TC : HAL_UART_RXEVENT_HT;
            accepted |= DR16_RxEventCallback(&uart, position);
        }
        if (position == dma_size) position = 0;
    }
    if (idle) {
        event = HAL_UART_RXEVENT_IDLE;
        accepted |= DR16_RxEventCallback(&uart, position == 0 ? dma_size : position);
    }
    return accepted;
}

int main(void)
{
    static DR16_State s;
    uint8_t b[18];
    uint16_t ch[5] = {364, 1684, 1024, 1025, 364};
    make_frame(b, ch);
    CHECK(DR16_Decode(b, 18, &s));
    CHECK(s.channel[0] == -660 && s.channel[1] == 660 && s.channel[2] == 0);
    CHECK(s.channel[3] == 1 && s.channel[4] == -660);
    CHECK(s.s1 == DR16_SWITCH_DOWN && s.s2 == DR16_SWITCH_MIDDLE);
    CHECK(s.mouse_x == -2 && s.mouse_y == 32767 && s.mouse_z == -32768);
    CHECK(s.keyboard == 0x8081 && s.mouse_left == 1 && s.mouse_right == 0);
    CHECK(!DR16_Decode(NULL, 18, &s) && !DR16_Decode(b, 17, &s));
    CHECK(!DR16_Decode(b, 18, NULL));
    b[5] &= 0x3f;
    CHECK(!DR16_Decode(b, 18, &s) && s.channel[0] == -660);
    make_frame(b, ch); b[12] = 2;
    CHECK(!DR16_Decode(b, 18, &s));
    for (unsigned i = 0; i < 5; ++i) {
        uint16_t saved = ch[i]; ch[i] = 363; make_frame(b, ch);
        CHECK(!DR16_Decode(b, 18, &s));
        ch[i] = 1685; make_frame(b, ch);
        CHECK(!DR16_Decode(b, 18, &s));
        ch[i] = saved;
    }
    make_frame(b, ch);
    CHECK(!DR16_GetState(&s) && s.channel[1] == 0);
    CHECK(DR16_Init(NULL) == HAL_ERROR && DR16_Init(&uart) == HAL_ERROR);
    uart.Instance = USART3; uart.hdmarx = &dma;
    uart.Init.BaudRate = 100000; uart.Init.WordLength = UART_WORDLENGTH_9B;
    uart.Init.Parity = UART_PARITY_EVEN; uart.Init.StopBits = UART_STOPBITS_1;
    uart.Init.Mode = UART_MODE_RX;
    dma.Init.Mode = DMA_NORMAL;
    CHECK(DR16_Init(&uart) == HAL_ERROR);
    dma.Init.Mode = DMA_CIRCULAR;
    dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    dma.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    start_result = HAL_BUSY;
    CHECK(DR16_Init(&uart) == HAL_BUSY);
    start_result = HAL_OK;
    CHECK(DR16_Init(&uart) == HAL_OK && dma_size == 64);
    CHECK(DR16_Init(&uart) == HAL_ERROR);
    CHECK(!DR16_RxEventCallback(&other, 18));
    CHECK(feed(b, 18, true));
    CHECK(DR16_GetState(&s) && s.valid_frames == 1);
    CHECK(!DR16_RxEventCallback(&uart, 18)); /* duplicate IDLE must not feed watchdog */
    CHECK(DR16_GetState(&s) && s.valid_frames == 1);
    CHECK(!feed(b, 7, false));
    CHECK(feed(b + 7, 11, true)); /* HT in the middle of this frame */
    CHECK(feed(b, 18, true));
    CHECK(feed(b, 18, true)); /* wrap within frame */
    /* 32 frames = 576 bytes: last frame ends exactly at TC/IDLE boundary. */
    for (unsigned i = 4; i < 32; ++i) CHECK(feed(b, 18, true));
    CHECK(position == 0 && DR16_GetState(&s) && s.valid_frames == 32);
    CHECK(!DR16_RxEventCallback(&uart, 64));
    CHECK(starts == 2); /* never restart circular DMA on ordinary events */

    CHECK(!feed(b, 17, true));
    CHECK(!DR16_GetState(&s) && s.channel[0] == 0 && s.s1 == DR16_SWITCH_INVALID);
    CHECK(feed(b, 18, true)); /* next idle boundary restores alignment */
    CHECK(!feed(b, 18, false));
    CHECK(!feed(b, 18, true)); /* oversized/merged burst discarded */
    CHECK(!DR16_GetState(&s));
    b[5] &= 0xcf; CHECK(!feed(b, 18, true)); /* invalid s2 */
    make_frame(b, ch);
    tick = 0xfffffff0U;
    CHECK(feed(b, 18, true));
    tick = 0x53; CHECK(DR16_GetState(&s)); /* elapsed 99 */
    tick = 0x54; CHECK(!DR16_GetState(&s)); /* elapsed 100 */
    DR16_ErrorCallback(&other);
    CHECK(DR16_Service() == HAL_OK && aborts == 0);
    DR16_ErrorCallback(&uart);
    CHECK(!DR16_GetState(&s) && s.uart_errors == 1);
    CHECK(!feed(b, 18, true));
    abort_result = HAL_TIMEOUT;
    CHECK(DR16_Service() == HAL_TIMEOUT);
    abort_result = HAL_OK; start_result = HAL_BUSY;
    CHECK(DR16_Service() == HAL_BUSY && !DR16_GetState(&s));
    start_result = HAL_OK;
    CHECK(DR16_Service() == HAL_OK);
    position = 0;
    CHECK(!DR16_GetState(&s));
    CHECK(feed(b, 18, true) && DR16_GetState(&s));
    CHECK(s.uart_errors == 1 && s.invalid_frames == 3);
    __disable_irq();
    CHECK(DR16_GetState(&s) && __get_PRIMASK() == 1);
    __enable_irq();
    CHECK(!DR16_GetState(NULL));
    CHECK(!DR16_RxEventCallback(&uart, 65) && !DR16_GetState(&s));
    return 0;
}
