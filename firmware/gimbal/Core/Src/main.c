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
  status = HAL_CAN_Start(&hcan1);
  if (status != HAL_OK) return status;
  status = HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  if (status != HAL_OK) (void)HAL_CAN_Stop(&hcan1);
  return status;
}

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
  /* Receive only at application level; no motor enable or control frames.
   * Confirm the physical motor's mode before calling any command API. */
  pitch_init_status = PitchMotor_Init(&hcan2, DM4340_MIT_MODE);
  yaw_init_status = StartYawFeedback();
  imu_init_status = BMI088_Init(&hspi1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    (void)PitchMotor_GetFeedback(&pitch_feedback);
    pitch_online = PitchMotor_IsOnline(100U);
    yaw_online = yaw_rx_count != 0U &&
        (uint32_t)(HAL_GetTick() - yaw_last_rx_ms) < 100U;
    if (imu_init_status == HAL_OK)
    {
      (void)BMI088_Service();
      imu_online = BMI088_GetSample(&imu_sample);
      BMI088_GetGyroCalibration(&imu_calibration);
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
    if (header.IDE != CAN_ID_STD || header.RTR != CAN_RTR_DATA ||
        header.StdId != GIMBAL_YAW_RX_ID || header.DLC != 8U ||
        (data[0] & 0x0FU) != GIMBAL_YAW_TX_ID) continue;
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

/* Feed IWDG only after a valid chassis command is decoded. The protocol is
 * not defined yet, so this bring-up image intentionally times out/resets. */
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
