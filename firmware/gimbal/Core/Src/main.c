/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "iwdg.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bmi088.h"
#include "gimbal_board.h"
#include "pitch_motor.h"
#include "yaw_motor.h"
#include "gimbal_setup.h"
#include "board_link.h"
#include "board_link_setup.h"
#include "attitude.h"
#include "motion_math.h"
#include <math.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Inspect IMU status in the debugger during board bring-up. */
volatile HAL_StatusTypeDef imu_init_status = HAL_ERROR;
volatile bool imu_online = false;
BMI088_Sample imu_sample;
BMI088_GyroCalibration imu_calibration;
volatile HAL_StatusTypeDef pitch_init_status = HAL_ERROR;
volatile bool pitch_online = false;
PitchMotor_Feedback pitch_feedback;
volatile HAL_StatusTypeDef yaw_init_status = HAL_ERROR;
volatile uint32_t yaw_rx_count;
volatile uint32_t yaw_last_rx_ms;
volatile uint16_t yaw_position_raw;
volatile uint16_t yaw_velocity_raw;
volatile uint16_t yaw_torque_raw;
volatile uint8_t yaw_motor_state;
volatile bool yaw_online;
YawMotor_Feedback yaw_feedback;
YawMotor_Params yaw_params;
volatile HAL_StatusTypeDef yaw_query_status = HAL_ERROR;
static uint32_t yaw_query_ms;
static LinkReceiver command_rx;
RobotCommand board_command;
volatile bool board_link_online;
volatile uint32_t board_command_count;
static volatile uint32_t watchdog_generation;
static uint32_t watchdog_consumed;
/* Status is split over three frames and sent one per loop pass so the 200 Hz
 * yaw torque frame always has a free CAN1 mailbox. */
static LinkFrame status_frames[3];
static uint8_t status_frames_left;
static uint32_t status_batch_ms;
static uint8_t status_sequence;
volatile uint32_t status_sent_count;
GimbalDrive gimbal_drive;
volatile bool gimbal_configured;
/* World-frame yaw from the onboard IMU, used only when GIMBAL_YAW_WORLD is 1.
 * The attitude is relative (no magnetometer): yaw zero is the heading at the
 * first stationary sample and it drifts, which is acceptable for a hold loop. */
static Attitude attitude;
static AttitudeOutput attitude_out;
static uint32_t attitude_last_ms;
static uint32_t imu_calibration_retry_ms;
volatile bool attitude_ready;
volatile float attitude_world_yaw;
volatile float attitude_world_rate;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static HAL_StatusTypeDef StartYawFeedback(void)
{
  /* Bank 0 on CAN1. Keep the split used by PitchMotor on CAN2. */
  const CAN_FilterTypeDef filter = {
    .FilterBank = 0,
    .FilterMode = CAN_FILTERMODE_IDMASK,
    .FilterScale = CAN_FILTERSCALE_32BIT,
    .FilterIdHigh = GIMBAL_YAW_RX_ID << 5,
    .FilterMaskIdHigh = 0x7FFU << 5,
    .FilterMaskIdLow = CAN_ID_EXT | CAN_RTR_REMOTE,
    .FilterFIFOAssignment = CAN_FILTER_FIFO0,
    .FilterActivation = ENABLE,
    .SlaveStartFilterBank = 14
  };
  HAL_StatusTypeDef status = HAL_CAN_ConfigFilter(&hcan1, &filter);
  if (status != HAL_OK) return status;
  /* A second bank receives exactly the three command fragment IDs. */
  CAN_FilterTypeDef link_filter = {
    .FilterBank = 1, .FilterMode = CAN_FILTERMODE_IDLIST,
    .FilterScale = CAN_FILTERSCALE_16BIT,
    .FilterIdHigh = 0x300U << 5, .FilterIdLow = 0x301U << 5,
    .FilterMaskIdHigh = 0x302U << 5, .FilterMaskIdLow = 0x302U << 5,
    .FilterFIFOAssignment = CAN_FILTER_FIFO0, .FilterActivation = ENABLE,
    .SlaveStartFilterBank = 14
  };
  status = HAL_CAN_ConfigFilter(&hcan1, &link_filter);
  if (status != HAL_OK) return status;
  status = HAL_CAN_Start(&hcan1);
  if (status != HAL_OK) return status;
  status = HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  if (status != HAL_OK) (void)HAL_CAN_Stop(&hcan1);
  return status;
}

/* Publish the measured joint angles to the chassis board. One frame per pass;
 * a batch is dropped and retried with a new sequence if the bus stays busy. */
static void ServiceStatusFrames(uint32_t now)
{
  if (status_frames_left == 0U) {
    if ((uint32_t)(now - status_batch_ms) < BOARD_LINK_STATUS_PERIOD_MS) return;
    const bool motors_ok = yaw_online && pitch_online && yaw_feedback.scaled;
    float yaw = 0.0f;
    if (isfinite(yaw_feedback.pos))
      yaw = Motion_Wrap(yaw_feedback.pos * GIMBAL_YAW_DIRECTION - GIMBAL_YAW_ZERO);
    if (!isfinite(yaw)) yaw = 0.0f;
    float pitch = 0.0f;
    if (isfinite(pitch_feedback.motor.pos))
      pitch = (pitch_feedback.motor.pos - GIMBAL_PITCH_ZERO) * GIMBAL_PITCH_DIRECTION;
    if (!isfinite(pitch)) pitch = 0.0f;
    if (pitch < GIMBAL_PITCH_MIN) pitch = GIMBAL_PITCH_MIN;
    if (pitch > GIMBAL_PITCH_MAX) pitch = GIMBAL_PITCH_MAX;
    const LinkStatus status = {
      .yaw_joint = yaw,
      .pitch_joint = pitch,
      .calibrated = motors_ok && GIMBAL_YAW_ZERO_CONFIRMED,
      .motors_healthy = motors_ok,
      .sensors_fresh = imu_online,
      .fault = gimbal_drive.fault != GIMBAL_OK || !board_link_online ||
        ((board_command.mode == MODE_SPIN || board_command.mode == MODE_FOLLOW) &&
         (!GIMBAL_IMU_MOUNT_CONFIRMED || !GIMBAL_YAW_ZERO_CONFIRMED ||
          !gimbal_drive.world_received ||
          (uint32_t)(now - gimbal_drive.world_ms) >= 30U))
    };
    status_batch_ms = now;
    if (!Link_EncodeStatus(&status, status_sequence++, status_frames)) return;
    status_frames_left = 3;
  }
  /* Keep one mailbox for the yaw torque frame. */
  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) < 2U) {
    if ((uint32_t)(now - status_batch_ms) >= 100U) status_frames_left = 0;
    return;
  }
  const LinkFrame *frame = &status_frames[3U - status_frames_left];
  CAN_TxHeaderTypeDef header = {.StdId = frame->id, .IDE = CAN_ID_STD,
                                .RTR = CAN_RTR_DATA, .DLC = 8};
  uint32_t mailbox;
  if (HAL_CAN_AddTxMessage(&hcan1, &header, frame->data, &mailbox) != HAL_OK) {
    status_frames_left = 0;
    return;
  }
  --status_frames_left;
  ++status_sent_count;
}

/* BMI088 mounting and fusion limits come from config/gimbal_setup.h. */
static const AttitudeConfig attitude_config = {
  .sensor_to_body = GIMBAL_IMU_SENSOR_TO_BODY,
  .gravity = GIMBAL_IMU_GRAVITY,
  .accel_tolerance = GIMBAL_IMU_ACCEL_TOLERANCE,
  .correction_gain = GIMBAL_IMU_CORRECTION_GAIN,
  .max_dt = GIMBAL_IMU_MAX_DT,
  .max_gyro = GIMBAL_IMU_MAX_GYRO,
};

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_TIM6_Init();
  MX_SPI1_Init();
  MX_TIM4_Init();
  MX_IWDG_Init();
  MX_CAN2_Init();
  MX_TIM10_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  /* Hardware confirmation in gimbal_setup.h gates all enable/torque output. */
  pitch_init_status = PitchMotor_Init(&hcan2, DM4340_MIT_MODE);
  (void)Link_Init(&command_rx, LINK_COMMAND, 20U, 100U);
  const YawMotor_Ranges yaw_ranges = {
    GIMBAL_YAW_PMAX, GIMBAL_YAW_VMAX, GIMBAL_YAW_TMAX
  };
  yaw_init_status = YawMotor_Init(&hcan1,
      GIMBAL_YAW_PROTOCOL_CONFIRMED ? &yaw_ranges : NULL);
  if (yaw_init_status == HAL_OK) yaw_init_status = StartYawFeedback();
  imu_init_status = BMI088_Init(&hspi1);
  if (imu_init_status == HAL_OK) (void)Attitude_Init(&attitude, &attitude_config);
  GimbalDriveConfig drive_config = GimbalSetup_Config();
  drive_config.hardware_confirmed = drive_config.hardware_confirmed &&
      pitch_init_status == HAL_OK && yaw_init_status == HAL_OK;
  gimbal_configured = GimbalDrive_Init(&gimbal_drive, &drive_config,
                                      &hcan1, &hcan2, HAL_GetTick());
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* Snapshot decoder state while its CAN ISR cannot modify it. Feed here,
     * not in the ISR, so a stalled main loop still trips the watchdog. */
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    board_link_online = Link_GetCommand(&command_rx, HAL_GetTick(), &board_command);
    const uint32_t generation = watchdog_generation;
    __set_PRIMASK(primask);
    if (generation != watchdog_consumed) {
      watchdog_consumed = generation;
      if (board_link_online && board_command.remote_valid)
        (void)HAL_IWDG_Refresh(&hiwdg);
    }
    (void)PitchMotor_GetFeedback(&pitch_feedback);
    pitch_online = PitchMotor_IsOnline(100U);
    YawMotor_ParamService(board_link_online && board_command.remote_valid && !board_command.arm);
    YawMotor_GetParams(&yaw_params);
    (void)YawMotor_GetFeedback(&yaw_feedback);
    yaw_online = YawMotor_IsOnline(100U);
    GimbalDrive_Step(&gimbal_drive, &board_command, board_link_online,
                     &yaw_feedback, &pitch_feedback, HAL_GetTick());
    ServiceStatusFrames(HAL_GetTick());
    /* Low-rate read-only feedback requests after stop/drive processing.
     * Do not compete with enable/torque traffic while armed. */
    if (board_link_online && board_command.remote_valid && !board_command.arm &&
        yaw_params.finished && (uint32_t)(HAL_GetTick()-yaw_query_ms) >= 100U) {
      yaw_query_ms = HAL_GetTick();
      yaw_query_status = YawMotor_RequestFeedback();
    }
    if (imu_init_status == HAL_OK)
    {
      const bool new_gyro = BMI088_Service();
      imu_online = BMI088_GetSample(&imu_sample);
      BMI088_GetGyroCalibration(&imu_calibration);
      if (!imu_calibration.calibrated) {
        /* Attitude needs a bias-free gyro. Arm a stationary window once per
         * second while disarmed; a window taken while moving is rejected. */
        attitude_ready = false;
        gimbal_drive.world_received = false;
        if (!imu_calibration.running && !board_command.arm &&
            (uint32_t)(HAL_GetTick()-imu_calibration_retry_ms) >= 1000U) {
          imu_calibration_retry_ms = HAL_GetTick();
          (void)BMI088_StartGyroCalibration(BMI088_CALIBRATION_SAMPLES);
        }
      } else if (new_gyro && attitude_last_ms != 0U) {
        const uint32_t elapsed = HAL_GetTick()-attitude_last_ms;
        attitude_last_ms = HAL_GetTick();
        if (elapsed != 0U) {
          const float gyro[3] = {
            imu_sample.gyro_dps[0]*0.01745329252f,
            imu_sample.gyro_dps[1]*0.01745329252f,
            imu_sample.gyro_dps[2]*0.01745329252f};
          if (Attitude_Update(&attitude, imu_sample.accel_mps2, gyro,
                              elapsed*0.001f, &attitude_out)) {
            attitude_ready = true;
            attitude_world_yaw = attitude_out.yaw;
            attitude_world_rate = attitude_out.euler_rate[2];
            if (GIMBAL_YAW_WORLD || GIMBAL_IMU_MOUNT_CONFIRMED)
              GimbalDrive_SetWorld(&gimbal_drive, attitude_out.yaw,
                                   attitude_out.euler_rate[2], HAL_GetTick());
          } else {
            /* A gap or bad sample immediately invalidates world feedback. */
            attitude_ready = false;
            gimbal_drive.world_received = false;
          }
        }
      } else if (new_gyro) {
        attitude_last_ms = HAL_GetTick();
      }
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan != &hcan1) return;
  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) != 0U)
  {
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK) break;
    if (header.IDE == CAN_ID_STD && header.RTR == CAN_RTR_DATA &&
        header.DLC == 8U && header.StdId >= 0x300U && header.StdId <= 0x302U) {
      LinkFrame frame = {.id = (uint16_t)header.StdId, .dlc = 8};
      memcpy(frame.data, data, 8);
      if (Link_Receive(&command_rx, &frame, HAL_GetTick())) {
        ++board_command_count;
        if (command_rx.message.command.remote_valid) ++watchdog_generation;
      }
      continue;
    }
    if (YawMotor_ReceiveParam(&header, data)) continue;
    if (header.IDE != CAN_ID_STD || header.RTR != CAN_RTR_DATA ||
        header.StdId != GIMBAL_YAW_RX_ID || header.DLC != 8U ||
        (data[0] & 0x0FU) != GIMBAL_YAW_TX_ID) continue;
    (void)YawMotor_Receive(&header, data);
    yaw_motor_state = data[0] >> 4;
    yaw_position_raw = (uint16_t)((data[1] << 8) | data[2]);
    yaw_velocity_raw = (uint16_t)((data[3] << 4) | (data[4] >> 4));
    yaw_torque_raw = (uint16_t)(((data[4] & 0x0FU) << 8) | data[5]);
    yaw_last_rx_ms = HAL_GetTick();
    ++yaw_rx_count;
  }
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  PitchMotor_RxFifo1Callback(hcan);
}

/* Motor feedback and incomplete/duplicate commands never feed IWDG. */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Data-ready only sets a flag; the SPI read happens in the main loop. */
  BMI088_ExtiCallback(GPIO_Pin);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
