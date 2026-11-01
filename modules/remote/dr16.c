#include "dr16.h"
#include <stddef.h>

/* Not in CCM: DMA1 cannot access CCM RAM. Keep HT and TC notifications enabled. */
#define RX_BUFFER_SIZE 64U
static uint8_t rx_buffer[RX_BUFFER_SIZE];
static uint8_t frame[DR16_FRAME_SIZE];
static uint16_t previous_position, frame_length;
static UART_HandleTypeDef *remote_uart;
static volatile bool recovery_needed, valid;
static volatile DR16_State state;

static uint16_t u16_le(const uint8_t *p)
{
    return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

static int16_t i16_le(const uint8_t *p)
{
    uint16_t n = u16_le(p);
    return (int16_t)((n & 0x8000U) ? (int32_t)n - 65536 : n);
}

bool DR16_Decode(const uint8_t *b, uint16_t length, DR16_State *out)
{
    if (b == NULL || out == NULL || length != DR16_FRAME_SIZE) return false;
    DR16_State decoded = {0};
    uint16_t raw[5];
    raw[0] = (b[0] | ((uint16_t)b[1] << 8)) & 0x7ffU;
    raw[1] = ((b[1] >> 3) | ((uint16_t)b[2] << 5)) & 0x7ffU;
    raw[2] = ((b[2] >> 6) | ((uint16_t)b[3] << 2) | ((uint16_t)b[4] << 10)) & 0x7ffU;
    raw[3] = ((b[4] >> 1) | ((uint16_t)b[5] << 7)) & 0x7ffU;
    raw[4] = u16_le(&b[16]);
    for (unsigned i = 0; i < 5; ++i) {
        if (raw[i] < 364U || raw[i] > 1684U) return false;
        decoded.channel[i] = (int16_t)((int32_t)raw[i] - 1024);
    }
    decoded.s1 = (DR16_Switch)((b[5] >> 6) & 3U);
    decoded.s2 = (DR16_Switch)((b[5] >> 4) & 3U);
    if (decoded.s1 == DR16_SWITCH_INVALID || decoded.s2 == DR16_SWITCH_INVALID ||
        b[12] > 1U || b[13] > 1U) return false;
    decoded.mouse_x = i16_le(&b[6]);
    decoded.mouse_y = i16_le(&b[8]);
    decoded.mouse_z = i16_le(&b[10]);
    decoded.mouse_left = b[12];
    decoded.mouse_right = b[13];
    decoded.keyboard = u16_le(&b[14]);
    *out = decoded;
    return true;
}

HAL_StatusTypeDef DR16_Init(UART_HandleTypeDef *huart)
{
    if (remote_uart != NULL || huart == NULL || huart->Instance != USART3 ||
        huart->hdmarx == NULL || huart->hdmarx->Init.Mode != DMA_CIRCULAR ||
        huart->hdmarx->Init.PeriphDataAlignment != DMA_PDATAALIGN_BYTE ||
        huart->hdmarx->Init.MemDataAlignment != DMA_MDATAALIGN_BYTE ||
        huart->Init.BaudRate != 100000U || huart->Init.WordLength != UART_WORDLENGTH_9B ||
        huart->Init.Parity != UART_PARITY_EVEN || huart->Init.StopBits != UART_STOPBITS_1 ||
        (huart->Init.Mode & UART_MODE_RX) == 0U) return HAL_ERROR;
    remote_uart = huart;
    HAL_StatusTypeDef result = HAL_UARTEx_ReceiveToIdle_DMA(huart, rx_buffer, RX_BUFFER_SIZE);
    if (result != HAL_OK) remote_uart = NULL;
    return result;
}

static void append_bytes(uint16_t begin, uint16_t end)
{
    for (uint16_t i = begin; i < end; ++i) {
        if (frame_length < DR16_FRAME_SIZE) frame[frame_length] = rx_buffer[i];
        /* Saturate at 19: oversized bursts are discarded, never scan for a
         * plausible frame in arbitrary bytes (DBUS has no header or CRC). */
        if (frame_length <= DR16_FRAME_SIZE) ++frame_length;
    }
}

bool DR16_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart == NULL || huart != remote_uart || recovery_needed) return false;
    if (size == 0U || size > RX_BUFFER_SIZE) {
        valid = false;
        recovery_needed = true;
        return false;
    }
    /* HAL Size is a write position, not the number of bytes in this event.
     * Keep position=64 at TC so a subsequent IDLE at 64 adds no duplicate bytes. */
    if (size < previous_position) {
        append_bytes(previous_position, RX_BUFFER_SIZE);
        append_bytes(0, size);
    } else {
        append_bytes(previous_position, size);
    }
    previous_position = size;
    if (HAL_UARTEx_GetRxEventType(huart) != HAL_UART_RXEVENT_IDLE) return false;
    if (frame_length == 0U) return false; /* Duplicate IDLE notification. */

    DR16_State decoded;
    bool accepted = DR16_Decode(frame, frame_length, &decoded);
    frame_length = 0;
    if (!accepted) {
        ++state.invalid_frames;
        valid = false;
        return false;
    }
    decoded.last_rx_ms = HAL_GetTick();
    decoded.valid_frames = state.valid_frames + 1U;
    decoded.invalid_frames = state.invalid_frames;
    decoded.uart_errors = state.uart_errors;
    state = decoded;
    valid = true;
    return true;
}

void DR16_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == NULL || huart != remote_uart) return;
    valid = false;
    recovery_needed = true;
    ++state.uart_errors;
}

HAL_StatusTypeDef DR16_Service(void)
{
    if (remote_uart == NULL) return HAL_ERROR;
    if (!recovery_needed) return HAL_OK;
    /* Blocking abort is deliberately outside IRQ context/critical section. */
    HAL_StatusTypeDef result = HAL_UART_AbortReceive(remote_uart);
    if (result != HAL_OK) return result;
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    __HAL_UART_CLEAR_OREFLAG(remote_uart);
    previous_position = 0;
    frame_length = 0;
    valid = false;
    result = HAL_UARTEx_ReceiveToIdle_DMA(remote_uart, rx_buffer, RX_BUFFER_SIZE);
    recovery_needed = result != HAL_OK;
    __set_PRIMASK(primask);
    return result;
}

bool DR16_GetState(DR16_State *out)
{
    if (out == NULL) return false;
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    *out = state;
    bool online = valid && !recovery_needed &&
        (uint32_t)(HAL_GetTick() - state.last_rx_ms) < DR16_TIMEOUT_MS;
    __set_PRIMASK(primask);
    if (!online) {
        for (unsigned i = 0; i < 5; ++i) out->channel[i] = 0;
        out->s1 = out->s2 = DR16_SWITCH_INVALID;
        out->mouse_x = out->mouse_y = out->mouse_z = 0;
        out->mouse_left = out->mouse_right = 0;
        out->keyboard = 0;
    }
    return online;
}
