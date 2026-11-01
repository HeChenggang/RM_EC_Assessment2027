#ifndef M3508_CAN_H
#define M3508_CAN_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define M3508_COUNT 4U
#define M3508_CURRENT_MAX 16384

/* IDs 1..4: front-left, rear-left, rear-right, front-right. */
typedef struct {
    uint16_t encoder;          /* Rotor angle, 0..8191; not wheel angle. */
    int16_t speed_rpm;         /* Rotor speed; apply actual reduction externally. */
    int16_t current_raw;
    uint8_t temperature_c;
    bool received;
    uint32_t last_rx_ms;
    uint32_t rx_count;
} M3508_Feedback;

/* Call once after MX_CAN2_Init. Owns CAN2 filter bank 14 and FIFO1.
 * Shared CAN1/CAN2 filter split is 14; other CAN filters must use that split. */
HAL_StatusTypeDef M3508_CAN_Init(CAN_HandleTypeDef *hcan);

/* Call from HAL_CAN_RxFifo1MsgPendingCallback; ignores other CAN instances. */
void M3508_CAN_RxFifo1Callback(CAN_HandleTypeDef *hcan);

/* Atomic snapshot, true only after a valid frame. ID is 1..4. */
bool M3508_CAN_GetFeedback(uint8_t motor_id, M3508_Feedback *out);
bool M3508_CAN_IsOnline(uint8_t motor_id, uint32_t timeout_ms);

/* Main/control-loop context only; serialize calls. Signed raw current units,
 * NOT rpm/amperes. Clamps to +/-16384 (C620 protocol full scale, +/-20 A).
 * HAL_OK means queued, NOT acknowledged. HAL_BUSY means retry on a later tick.
 * Remote validity, arming, current limits and stop retries belong to caller. */
HAL_StatusTypeDef M3508_CAN_SendCurrents(const int16_t currents[M3508_COUNT]);

#ifdef __cplusplus
}
#endif
#endif
