#ifndef DR16_H
#define DR16_H

#include <stdbool.h>
#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DR16_FRAME_SIZE 18U
#define DR16_TIMEOUT_MS 100U
#define DR16_CHANNEL_RANGE 660

typedef enum {
    DR16_SWITCH_INVALID = 0,
    DR16_SWITCH_UP = 1,
    DR16_SWITCH_DOWN = 2,
    DR16_SWITCH_MIDDLE = 3
} DR16_Switch;

typedef struct {
    /* ch0 right horizontal, ch1 right vertical, ch2 left horizontal,
     * ch3 left vertical, ch4 wheel; centered raw units -660..660. */
    int16_t channel[5];
    DR16_Switch s1; /* byte 5 bits 7:6; verify physical switch on hardware. */
    DR16_Switch s2; /* byte 5 bits 5:4 */
    int16_t mouse_x, mouse_y, mouse_z;
    uint8_t mouse_left, mouse_right;
    uint16_t keyboard; /* W S A D Shift Ctrl Q E R F G Z X C V B, bits 0..15 */
    uint32_t last_rx_ms, valid_frames, invalid_frames, uart_errors;
} DR16_State;

/* USART3 + circular byte DMA. Call once after UART/DMA initialization. */
HAL_StatusTypeDef DR16_Init(UART_HandleTypeDef *huart);
/* Forward HAL callbacks. True only when a NEW valid complete frame is decoded.
 * UART and RX DMA IRQs must have the same preemption priority. */
bool DR16_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size);
void DR16_ErrorCallback(UART_HandleTypeDef *huart);
/* Call from main loop to restart reception after UART/DMA errors. Never feeds IWDG. */
HAL_StatusTypeDef DR16_Service(void);
/* Atomic snapshot; false on invalid/error/timeout, with all control fields zeroed.
 * Diagnostics retained. Always check the return value before using controls. */
bool DR16_GetState(DR16_State *out);
/* Pure decoder: exact 18 bytes, range/switch/button checks; no CRC in DBUS.
 * On failure, output is not modified. Diagnostics initialized to zero on success. */
bool DR16_Decode(const uint8_t *frame, uint16_t length, DR16_State *out);

#ifdef __cplusplus
}
#endif
#endif
