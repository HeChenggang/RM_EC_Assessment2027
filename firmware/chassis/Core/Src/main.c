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
#include "board_link.h"
#include "board_link_setup.h"
#include "motion_math.h"
#include "command.h"
#include "chassis_control.h"
#include "chassis_setup.h"
#include "safety_gate.h"
#include "m3508_can.h"
#include "dr16.h"
#include "bmi088.h"
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
volatile HAL_StatusTypeDef board_link_tx_status = HAL_ERROR;
static uint8_t board_link_sequence;
/* Gimbal status receiver (0x310..0x312). Last value is visible in the debugger. */
static LinkReceiver status_rx;
volatile bool board_link_rx_configured;
volatile uint32_t board_status_count;
volatile float board_relative_yaw;
static float board_yaw_last;
static bool board_yaw_initialized;
/* Visible in the debugger. HAL_OK only means the frame was queued. */
volatile HAL_StatusTypeDef motor_can_tx_status = HAL_OK;
volatile HAL_StatusTypeDef remote_rx_status = HAL_OK;
volatile bool remote_online = false;
DR16_State remote_state;
/* Debugger snapshot. arm is the operator's request, not motor enable. */
volatile RobotCommand remote_command = {.estop = true};
volatile bool chassis_configured;
volatile SafetyState chassis_safety_state;
volatile uint8_t chassis_wheel_fresh_mask;
volatile ChassisOutput chassis_output;
static ChassisControl chassis_control;
static SafetyGate chassis_gate;
volatile uint32_t rr_direct_request;
volatile bool rr_direct_active;
volatile int16_t rr_direct_peak_rpm;
volatile uint32_t rr_direct_pulse_count;
#if CHASSIS_RR_BENCH
static PID_Controller rr_bench_pid;
volatile float rr_bench_kp = 2.0f;
volatile float rr_bench_ki = 0.0f;
volatile float rr_bench_kd = 0.0f;
volatile float rr_bench_measured_rpm;
volatile bool rr_bench_gains_valid;
#endif
static const RemoteMap remote_map = {
  .channel = {3, 2, 0, 1},
  .direction = {1, -1, -1, 1},
  .deadzone = 0.05f,
  .separate_switch = DR16_SWITCH_UP,
  .follow_switch = DR16_SWITCH_MIDDLE,
  .spin_switch = DR16_SWITCH_DOWN,
  .enable_switch = DR16_SWITCH_UP,
  .disarm_switch = DR16_SWITCH_MIDDLE,
  .stop_switch = DR16_SWITCH_DOWN
};
volatile HAL_StatusTypeDef imu_init_status = HAL_ERROR;
volatile bool imu_online = false;
BMI088_Sample imu_sample;
BMI088_GyroCalibration imu_calibration;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* CAN1 is the board link: this board transmits commands 0x300..0x302 and
 * receives gimbal status 0x310..0x312 on FIFO0. */
static HAL_StatusTypeDef StartBoardLink(void)
{
  const CAN_FilterTypeDef status_filter = {
    .FilterBank = 0,
    .FilterMode = CAN_FILTERMODE_IDLIST,
    .FilterScale = CAN_FILTERSCALE_16BIT,
    .FilterIdHigh = 0x310U << 5,
    .FilterIdLow = 0x311U << 5,
    .FilterMaskIdHigh = 0x312U << 5,
    .FilterMaskIdLow = 0x312U << 5,
    .FilterFIFOAssignment = CAN_FILTER_FIFO0,
    .FilterActivation = ENABLE,
    .SlaveStartFilterBank = 14
  };
  HAL_StatusTypeDef status = HAL_CAN_ConfigFilter(&hcan1, &status_filter);
  if (status != HAL_OK) return status;
  status = HAL_CAN_Start(&hcan1);
  if (status != HAL_OK) return status;
  status = HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  if (status != HAL_OK) (void)HAL_CAN_Stop(&hcan1);
  return status;
}

static bool BoardStatus(uint32_t now, LinkStatus *out)
{
  if (!out) return false;
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  const bool fresh = Link_IsFresh(&status_rx, now);
  if (fresh) *out = status_rx.message.status;
  __set_PRIMASK(primask);
  return fresh;
}

/* Joint angle from the gimbal, in the chassis frame. Rejects uncalibrated,
 * faulted and non-finite values, plus steps no physical yaw move can make. */
static bool BoardYaw(const LinkStatus *status, float *relative_yaw)
{
  if (!status || !relative_yaw || !status->calibrated || status->fault ||
      !isfinite(status->yaw_joint)) return false;
  const float yaw = status->yaw_joint;
  if (board_yaw_initialized &&
      fabsf(Motion_Wrap(yaw - board_yaw_last)) > CHASSIS_YAW_JUMP_RAD) {
    board_yaw_initialized = false;
    return false;
  }
  board_yaw_last = yaw;
  board_yaw_initialized = true;
  *relative_yaw = yaw;
  return true;
}

static void SendBoardCommand(const RobotCommand *command)
{
  LinkFrame frames[3];
  /* CAN1 belongs to this sender. Drop/abort a stale batch instead of queuing
   * old control commands behind a disconnected bus. */
  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 3U) {
    (void)HAL_CAN_AbortTxRequest(&hcan1, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
    board_link_tx_status = HAL_BUSY;
    return;
  }
  if (!Link_EncodeCommand(command, board_link_sequence++, frames)) return;
  for (unsigned i = 0; i < 3; ++i) {
    CAN_TxHeaderTypeDef header = {.StdId = frames[i].id, .IDE = CAN_ID_STD,
                                .RTR = CAN_RTR_DATA, .DLC = 8};
    uint32_t mailbox;
    board_link_tx_status = HAL_CAN_AddTxMessage(&hcan1, &header, frames[i].data, &mailbox);
    if (board_link_tx_status != HAL_OK) {
      (void)HAL_CAN_AbortTxRequest(&hcan1, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
      break;
    }
  }
}

static bool InitChassisControl(void)
{
  ChassisConfig config = {
    .geometry = MECANUM_DEFAULT_CONFIG,
    .max_translation = CHASSIS_TRANSLATION_LIMIT_MPS,
    .max_rotation = CHASSIS_MAX_ROTATION_RAD_S,
    .spin_rate = CHASSIS_SPIN_RATE_RAD_S, .follow_kp = 1.0f,
    .translation_accel = CHASSIS_TRANSLATION_ACCEL_MPS2,
    .rotation_accel = 1.0f, .max_dt = 0.025f
  };
  const int8_t direction[M3508_COUNT] = CHASSIS_WHEEL_DIRECTIONS;
  config.geometry.max_motor_rpm = CHASSIS_MAX_ROTOR_RPM;
  for (unsigned i = 0; i < M3508_COUNT; ++i)
  {
    config.geometry.motor_direction[i] = direction[i];
    config.wheel_pid[i] = (PID_Config){
      .kp = CHASSIS_SPEED_KP, .ki = CHASSIS_SPEED_KI,
      .output_limit = CHASSIS_CURRENT_LIMIT_RAW,
      .integral_limit = CHASSIS_SPEED_INTEGRAL_LIMIT_RAW,
      .max_dt = config.max_dt
    };
  }
  return CHASSIS_SPEED_KP > 0.0f && Chassis_Init(&chassis_control, &config);
}

static void ServiceRightRearPulse(bool fresh, float rpm)
{
  enum { pulse_ms = 300, pulse_current = -410 };
  static uint32_t started_ms;
  const uint32_t now = HAL_GetTick();
  const bool debugger_attached = (CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) != 0U;
  const uint32_t request = rr_direct_request;
  if (request != 0U)
  {
    rr_direct_request = 0U;
    if (request == 1U && !rr_direct_active && fresh && debugger_attached &&
        rpm >= -5.0f && rpm <= 5.0f)
    {
      started_ms = now;
      rr_direct_peak_rpm = 0;
      rr_direct_active = true;
      ++rr_direct_pulse_count;
    }
  }
  const bool was_active = rr_direct_active;
  if (!fresh || !debugger_attached || request == 2U || now - started_ms >= pulse_ms)
    rr_direct_active = false;
  if (rr_direct_active)
  {
    const int16_t peak = rr_direct_peak_rpm;
    const int16_t sample = (int16_t)rpm;
    if ((int32_t)sample * sample > (int32_t)peak * peak)
      rr_direct_peak_rpm = sample;
  }

  ChassisOutput output = {0};
  if (rr_direct_active)
    output.current[MECANUM_RR] = pulse_current;
  if (was_active && !rr_direct_active)
    (void)HAL_CAN_AbortTxRequest(&hcan2, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
  motor_can_tx_status = M3508_CAN_SendCurrents(output.current);
  if (rr_direct_active && motor_can_tx_status != HAL_OK)
  {
    rr_direct_active = false;
    output.current[MECANUM_RR] = 0;
    (void)HAL_CAN_AbortTxRequest(&hcan2, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
    (void)M3508_CAN_SendCurrents(output.current);
  }
  chassis_output = output;
  chassis_safety_state = rr_direct_active ? SAFETY_ACTIVE : SAFETY_DISARMED;
  (void)HAL_IWDG_Refresh(&hiwdg);
}

static void ServiceChassisDrive(const RobotCommand *command, float dt)
{
  float rpm[M3508_COUNT] = {0};
  bool fresh[M3508_COUNT] = {false};
  uint8_t mask = 0;
  for (unsigned i = 0; i < M3508_COUNT; ++i)
  {
    M3508_Feedback wheel;
    if (M3508_CAN_GetFeedback((uint8_t)(i + 1), &wheel))
    {
      rpm[i] = wheel.speed_rpm;
      fresh[i] = (uint32_t)(HAL_GetTick() - wheel.last_rx_ms) < 50U;
      if (fresh[i]) mask |= (uint8_t)(1U << i);
    }
  }
  chassis_wheel_fresh_mask = mask;
  if (CHASSIS_RR_DIRECT_TEST)
  {
    ServiceRightRearPulse(fresh[MECANUM_RR], rpm[MECANUM_RR]);
    return;
  }
#if CHASSIS_RR_BENCH
  enum { rr = MECANUM_RR };
  rr_bench_measured_rpm = rpm[rr];
  /* Apply debugger edits only on a valid, neutral standby command. */
  if (command->remote_valid && !command->arm && !command->estop &&
      Command_Neutral(command, 0.02f))
  {
    const PID_Config gains = {
      .kp = rr_bench_kp, .ki = rr_bench_ki, .kd = rr_bench_kd,
      .output_limit = CHASSIS_RR_BENCH_CURRENT_RAW, .integral_limit = 100.0f,
      .derivative_tau = 0.04f, .max_dt = 0.025f
    };
    rr_bench_gains_valid = PID_Init(&rr_bench_pid, &gains);
  }
#endif
  /* Relative yaw comes from the gimbal status frame. Follow additionally
   * needs a world-stabilized gimbal yaw; with joint-only yaw control the
   * chassis would rotate until max_rotation and never converge. */
  const uint32_t now = HAL_GetTick();
  if (chassis_gate.state != SAFETY_ACTIVE) board_yaw_initialized = false;
  LinkStatus status;
  const bool status_ok = BoardStatus(now, &status);
  float relative_yaw = 0.0f;
  const bool yaw_ok = status_ok && BoardYaw(&status, &relative_yaw);
  board_relative_yaw = yaw_ok ? relative_yaw : 0.0f;
  const bool yaw_needed = command->mode == MODE_SEPARATE || command->mode == MODE_FOLLOW;
  const bool follow_ready = command->mode != MODE_FOLLOW || CHASSIS_FOLLOW_IMU_READY;
  SafetyInputs safety = {
    .link_fresh = command->remote_valid,
    /* A stale/faulted board link stops every mode; separate/follow also
     * require a usable relative yaw. */
    .sensors_fresh = status_ok && (!yaw_needed || yaw_ok) && follow_ready,
    .motors_healthy =
#if CHASSIS_RR_BENCH
      fresh[rr],
#else
      mask == (1U << M3508_COUNT) - 1U,
#endif
    .calibrated = true,
    .hardware_configured =
#if CHASSIS_RR_BENCH
      rr_bench_gains_valid
#else
      chassis_configured
#endif
  };
  const bool was_active = chassis_gate.state == SAFETY_ACTIVE;
  const bool allowed = Safety_Step(&chassis_gate, command, &safety);
#if CHASSIS_RR_BENCH
  ChassisOutput output = {0};
  if (allowed && command->forward != 0.0f)
  {
    const int8_t direction[M3508_COUNT] = CHASSIS_WHEEL_DIRECTIONS;
    const float target = direction[rr] * command->forward * CHASSIS_RR_BENCH_RPM;
    float current;
    if (PID_Update(&rr_bench_pid, target, rpm[rr], dt, &current))
    {
      output.wheels.motor_rpm[rr] = target;
      output.current[rr] = (int16_t)current;
    }
    else
      chassis_gate = (SafetyGate){.state = SAFETY_FAULT};
  }
  else
    PID_Reset(&rr_bench_pid);
#else
  ChassisOutput output;
  if (command->forward == 0.0f && command->left == 0.0f)
  {
    chassis_control.previous.vx = 0.0f;
    chassis_control.previous.vy = 0.0f;
  }
  if (!Chassis_Step(&chassis_control, command, yaw_ok ? relative_yaw : 0.0f,
                    rpm, fresh, allowed, dt, &output) && allowed)
    chassis_gate = (SafetyGate){.state = SAFETY_FAULT};
#endif

  bool cancel_pending = was_active && chassis_gate.state != SAFETY_ACTIVE;
#if CHASSIS_RR_BENCH
  cancel_pending |= command->forward == 0.0f && chassis_output.current[rr] != 0;
#endif
  if (cancel_pending)
    (void)HAL_CAN_AbortTxRequest(&hcan2, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
  motor_can_tx_status = M3508_CAN_SendCurrents(output.current);
  if (chassis_gate.state == SAFETY_ACTIVE && motor_can_tx_status != HAL_OK)
  {
    chassis_gate = (SafetyGate){.state = SAFETY_FAULT};
    Chassis_Reset(&chassis_control);
#if CHASSIS_RR_BENCH
    PID_Reset(&rr_bench_pid);
#endif
    (void)HAL_CAN_AbortTxRequest(&hcan2, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
    output = (ChassisOutput){0};
    (void)M3508_CAN_SendCurrents(output.current);
  }
  chassis_output = output;
  chassis_safety_state = chassis_gate.state;
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
  board_link_rx_configured = Link_Init(&status_rx, LINK_STATUS,
                                       BOARD_LINK_ASSEMBLY_MS, BOARD_LINK_FRESH_MS);
  board_link_tx_status = StartBoardLink();
  MX_TIM6_Init();
  MX_SPI1_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  MX_IWDG_Init();
  MX_CAN2_Init();
  MX_TIM10_Init();
  MX_USART6_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  /* Let the IWDG count only while the core runs: halted at a breakpoint it
   * would otherwise reset the board after ~802 ms, which makes single-step
   * debugging impossible. Has no effect when no debugger is attached. */
  __HAL_DBGMCU_FREEZE_IWDG();
  if (M3508_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  chassis_configured = InitChassisControl();
  if (chassis_configured || CHASSIS_RR_BENCH || CHASSIS_RR_DIRECT_TEST)
    __HAL_DBGMCU_UNFREEZE_IWDG();
  if (DR16_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* The IMU is not fatal at boot: a missing or miswired chip must not stop the
   * chassis from being tested, so the loop retries instead of Error_Handler. */
  imu_init_status = BMI088_Init(&hspi1);
  uint32_t last_imu_init_ms = HAL_GetTick();
  uint32_t last_motor_tx_ms = last_imu_init_ms;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    remote_rx_status = DR16_Service();
    const uint32_t now = HAL_GetTick();
    if (!CHASSIS_RR_DIRECT_TEST && !CHASSIS_RR_BENCH && !chassis_configured && imu_init_status != HAL_OK &&
        (uint32_t)(now - last_imu_init_ms) >= 1000U)
    {
      last_imu_init_ms = now;
      imu_init_status = BMI088_Init(&hspi1);
    }
    if (imu_init_status == HAL_OK)
    {
      (void)BMI088_Service();
      imu_online = BMI088_GetSample(&imu_sample);
      BMI088_GetGyroCalibration(&imu_calibration);
    }
    const uint32_t drive_now = HAL_GetTick();
    const uint32_t elapsed = drive_now - last_motor_tx_ms;
    if (elapsed >= 10U)
    {
      last_motor_tx_ms = drive_now;
      remote_online = DR16_GetState(&remote_state);
      const RemoteInput input = {
        .channel = {remote_state.channel[0], remote_state.channel[1],
                    remote_state.channel[2], remote_state.channel[3],
                    remote_state.channel[4]},
        .mode_switch = remote_state.s1,
        .arm_switch = remote_state.s2,
        .fresh = remote_online
      };
      RobotCommand command;
      (void)Remote_Map(&remote_map, &input, &command);
      remote_command = command;
      SendBoardCommand(&command);
      ServiceChassisDrive(&command, (float)elapsed * 0.001f);
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
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (DR16_RxEventCallback(huart, Size))
  {
    /* Only a newly decoded, valid remote frame can refresh the watchdog. */
    (void)HAL_IWDG_Refresh(&hiwdg);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  DR16_ErrorCallback(huart);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan != &hcan1) return;
  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) != 0U)
  {
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK) break;
    if (header.IDE != CAN_ID_STD || header.RTR != CAN_RTR_DATA ||
        header.DLC != 8U || header.StdId < 0x310U || header.StdId > 0x312U) continue;
    LinkFrame frame = {.id = (uint16_t)header.StdId, .dlc = 8};
    memcpy(frame.data, data, 8);
    /* Status never feeds the IWDG; only a valid DR16 frame does that. */
    if (Link_Receive(&status_rx, &frame, HAL_GetTick())) ++board_status_count;
  }
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  M3508_CAN_RxFifo1Callback(hcan);
}

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
