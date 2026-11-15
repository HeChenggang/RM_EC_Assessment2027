#ifndef DM4340_DRV_H_
#define DM4340_DRV_H_
#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DM4340_MIT_MODE 0x000
#define DM4340_POS_MODE 0x100
#define DM4340_SPEED_MODE 0x200

// 映射范围沿用原驱动，须与电机的 PMAX、VMAX、TMAX 配置一致。
#define DM4340_P_MIN -3.141593f
#define DM4340_P_MAX 3.141593f
#define DM4340_V_MIN -10.0f
#define DM4340_V_MAX 10.0f
#define DM4340_KP_MIN 0.0f
#define DM4340_KP_MAX 500.0f
#define DM4340_KD_MIN 0.0f
#define DM4340_KD_MAX 5.0f
#define DM4340_T_MIN -28.0f
#define DM4340_T_MAX 28.0f

typedef struct {
  uint16_t id;
  uint16_t state;
  int p_int;
  int v_int;
  int t_int;

  float pos;
  float vel;
  float tor;

  float Tmos;
  float Tcoil;
} DM4340_MotorFbPara_t;

typedef struct {
  uint16_t mode;
  DM4340_MotorFbPara_t para;
} DM4340_Motor_t;

bool dm4340_fbdata(DM4340_Motor_t *motor, const uint8_t *rx_data,
                          uint32_t data_len);

HAL_StatusTypeDef dm4340_enable_motor_mode(CAN_HandleTypeDef *hcan, uint16_t motor_id,
                              uint16_t mode_id);
HAL_StatusTypeDef dm4340_disable_motor_mode(CAN_HandleTypeDef *hcan, uint16_t motor_id,
                               uint16_t mode_id);

/* Main-loop callers must enforce joint limits, arming and stop retries.
 * HAL_OK means queued, not acknowledged. Non-finite/out-of-range inputs
 * are rejected. Calls on a given CAN bus must be serialized. */
HAL_StatusTypeDef dm4340_mit_ctrl(CAN_HandleTypeDef *hcan, uint16_t motor_id, float pos,
                     float vel, float kp, float kd, float torq);
HAL_StatusTypeDef dm4340_pos_speed_ctrl(CAN_HandleTypeDef *hcan, uint16_t motor_id,
                           float pos, float vel);
HAL_StatusTypeDef dm4340_speed_ctrl(CAN_HandleTypeDef *hcan, uint16_t motor_id,
                       float _vel);

HAL_StatusTypeDef dm4340_joint_motor_init(DM4340_Motor_t *motor, uint16_t id, uint16_t mode);

#ifdef __cplusplus
}
#endif
#endif /* DM4340_DRV_H_ */
