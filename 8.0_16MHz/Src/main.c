/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "robot_config.h"
#include "robot_motion.h"
#include "balance_diag.h"
#include "sts3215.h"
#include "vision_uart.h"
#include "vision_control.h"
#include "sg90_pwm.h"
#include "hc_sr04.h"
#include "obstacle_control.h"
#if ROBOT_ENABLE_TOF
#include "vl53l0x_test.h"
#endif
#include <stdio.h>
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
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart4; /* 视觉专用：PC10=TX、PC11=RX；与USART1/2独立。 */
I2C_HandleTypeDef hi2c2;   /* 新板底部I2C口：PB10=SCL、PB11=SDA。 */
#if ROBOT_USE_EXTERNAL_ADAPTER
UART_HandleTypeDef huart2;
#else
UART_HandleTypeDef huart6;
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_UART4_Init(void);
#if ROBOT_ENABLE_TOF
static void MX_I2C2_Init(void);
#endif
#if ROBOT_USE_EXTERNAL_ADAPTER
static void MX_USART2_UART_Init(void);
#else
static void MX_USART6_UART_Init(void);
#endif
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* 主循环收发队列：每次只在TXE就绪时写一个字节，绝不等待串口。 */
#define CONSOLE_TX_SIZE 2048U
#define KEY_DEBOUNCE_MS 30U
static uint8_t s_console_tx[CONSOLE_TX_SIZE];
static uint16_t s_console_head, s_console_tail;
/* 原理图：K1=PE1、K2=PE0，板载10k上拉，按下为低。 */
static uint8_t s_key_raw[2] = {1U, 1U};
static uint8_t s_key_stable[2] = {1U, 1U};
static uint32_t s_key_changed_ms[2];
/* SG90实机测试：仅静止时允许j启动，避免与图形6动作争用PA11 PWM。 */
static uint8_t s_sg90_test_active;
/* HC-SR04只允许在ROBOT_IDLE下独立调试，避免阻塞测距干扰10ms步态任务。 */
static uint8_t s_hcsr04_test_active;
static uint32_t s_hcsr04_report_due;
#if ROBOT_ENABLE_TOF
static uint8_t s_tof_ready;
static uint32_t s_tof_report_due;
#endif
static void ConsolePrint(const char *text)
{
  uint16_t length = (uint16_t)strlen(text);
  uint16_t free_bytes = (uint16_t)((s_console_tail + CONSOLE_TX_SIZE - s_console_head - 1U) % CONSOLE_TX_SIZE);
  if (length > free_bytes) return; /* 高频查询挤满时丢本行，不阻塞走路。 */
  while (*text) {
    s_console_tx[s_console_head] = (uint8_t)*text++;
    s_console_head = (s_console_head + 1U) % CONSOLE_TX_SIZE;
  }
}
__attribute__((noinline)) static void ConsoleService(void)
{
  if (s_console_tail != s_console_head && __HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE)) {
    huart1.Instance->DR = s_console_tx[s_console_tail];
    s_console_tail = (s_console_tail + 1U) % CONSOLE_TX_SIZE;
  }
}

static void PrintHelp(void)
{
  ConsolePrint("\r\n=== HumanoidRobot 8.0_16MHz / 7.0 UART pins ===\r\n");
  ConsolePrint("s: left-front start pose; finish at left-front landing while moving\r\n");
  ConsolePrint("t: JSON timing trial, stop at step 4 landing\r\n");
  ConsolePrint("g: JSON timing walk, step 4 final frame, then loop steps 3/4\r\n");
  ConsolePrint("r: JSON timing walk (same as g)\r\n");
  ConsolePrint("h: stop at next planned landing; g/r/t resume\r\n");
  ConsolePrint("x: emergency torque OFF; support robot\r\n");
  ConsolePrint("p: scan while idle/holding; ?: help while stopped\r\n");
  ConsolePrint("v: enable vision; i: vision status anytime\r\n");
  ConsolePrint("j: SG90 test ON at PA11/J1, shake 3s, then hold center\r\n");
  ConsolePrint("u: HC-SR04 independent test ON/OFF (IDLE only), PA2 TRIG / PA3 ECHO\r\n");
  ConsolePrint("UART4 PC11 RX: 115200 8N1; vision loss/0 keeps walking; AA56 control; AA55 shapes; AA57 step feedback.\r\n");
  ConsolePrint("g/r: walk with vision shapes; s/t/h/x disables vision; v enables it.\r\n");
  ConsolePrint("KEY1 PE1: same as s; KEY2 PE0: same as g (active low, debounced).\r\n");
  ConsolePrint("No automatic motion. Landing labels are not contact sensing.\r\n");
}

__attribute__((noinline)) static void HandleCommand(uint8_t command)
{
  uint8_t ok = 1U, i;
  char line[128];
  uint16_t positions[ROBOT_SERVO_COUNT];
  RobotState state = Robot_GetState();
  if (command >= 'A' && command <= 'Z') command += 'a' - 'A';
  /* 测试进行时不启动其它动作；x仍可立即回中并关闭总线舵机扭矩。 */
  if (s_sg90_test_active && command != 'i' && command != 'x' && command != '?') {
    ConsolePrint("SG90 TEST BUSY: wait for DONE, or send x to stop\r\n");
    return;
  }
  if (command == 's' || command == 'h' || command == 'x') VisionControl_SetEnabled(0U);
  /* g/r先开启视觉，再处理起步；站姿过渡或起步被拒绝也不漏掉启用。
   * 已开启时不重复初始化，避免清掉正在执行的视觉事务。 */
  if ((command == 'g' || command == 'r') && !VisionControl_IsEnabled())
    VisionControl_SetEnabled(1U);
  /* 行走时g/r/v可开启视觉，i只查询；不重新开始步态、不重置正在执行的事务。 */
  if (state == ROBOT_MARCHING || state == ROBOT_ENTERING_STAND) {
    if (command == 'x') Robot_EmergencyOff();
    else if (command == 's') Robot_StopAtStartPose();
    else if (command == 'h') Robot_StopMarch();
    else if (state == ROBOT_MARCHING && (command == 'g' || command == 'r' || command == 'v')) {
      if (!VisionControl_IsEnabled()) VisionControl_SetEnabled(1U);
    }
    if (command != 'i') return;
  }
  switch (command) {
    case 's': ok = Robot_EnterStand(); break;
    case 'g': ok = VisionControl_StartWalk(); break;
    case 't': VisionControl_SetEnabled(0U); ok = Robot_StartActions(ACTION_REFERENCE, 1U); break;
    case 'r': ok = VisionControl_StartWalk(); break;
    case 'v': VisionControl_SetEnabled(1U); break;
    case 'j':
      SG90_StartShake(HAL_GetTick());
      s_sg90_test_active = SG90_IsActive();
      ConsolePrint(s_sg90_test_active ? "SG90 TEST START: PA11/J1, 3s shake\r\n"
                                      : "SG90 TEST FAILED: PWM did not start\r\n");
      break;
    case 'u':
      if (s_hcsr04_test_active) {
        s_hcsr04_test_active = 0U;
        ConsolePrint("HC-SR04 TEST OFF\r\n");
      } else if (state != ROBOT_IDLE) {
        ok = 0U;
        ConsolePrint("HC-SR04 REFUSED: send x and wait for IDLE first\r\n");
      } else {
        s_hcsr04_test_active = 1U;
        s_hcsr04_report_due = HAL_GetTick();
        ConsolePrint("HC-SR04 TEST ON: PA2 TRIG, PA3 ECHO direct\r\n");
      }
      break;
    case 'i': {
      VisionStatus status;
      VisionAngle angle;
      Vision_GetStatus(&status);
      Vision_GetAngle(&angle);
      sprintf(line, "VISION enabled=%u online=%u seq=%u action=%u shape=%u queue=%u turn_missing=%u\r\n",
              VisionControl_IsEnabled(), status.online, status.sequence, status.action, status.shape,
              status.queued_shapes, VisionControl_TurnUnavailable());
      ConsolePrint(line);
      sprintf(line, "valid=%lu crc=%lu fields=%lu dup=%lu lost_est=%lu uart=%lu overflow=%lu\r\n",
              (unsigned long)status.valid_packets, (unsigned long)status.crc_errors,
              (unsigned long)status.field_errors, (unsigned long)status.duplicate_packets,
              (unsigned long)status.estimated_lost, (unsigned long)status.uart_errors,
              (unsigned long)status.queue_overflows);
      ConsolePrint(line);
      sprintf(line, "AA56 status=%u angle_x10=%d seq=%u; AA57 state=%u request_seq=%u phase=%u\r\n",
              angle.status, (int)angle.deci_degrees, angle.sequence,
              VisionControl_GetFeedbackState(), VisionControl_GetFeedbackSequence(), Robot_GetControlPhase());
      ConsolePrint(line);
      sprintf(line, "shape_active=%u head=%u completed=%lu interrupted=%lu\r\n",
              Robot_GetActiveShape(), SG90_IsActive(),
              (unsigned long)Robot_GetCompletedShapes(), (unsigned long)Robot_GetInterruptedShapes());
      ConsolePrint(line);
      sprintf(line, "SHAPE accepted=%lu last=%u\r\n",
              (unsigned long)VisionControl_GetAcceptedShapes(),
              (unsigned)VisionControl_GetLastAcceptedShape());
      ConsolePrint(line);
      sprintf(line, "ROBOT state=%u frame=%u shape_pending=%u error=%.70s\r\n",
              (unsigned)Robot_GetState(), (unsigned)Robot_GetFrame(),
              (unsigned)Robot_GetPendingShape(), Robot_GetLastError());
      ConsolePrint(line);
      sprintf(line, "OBSTACLE phase=%u distance=%u mm near_confirm=%u\r\n",
              (unsigned)Robot_GetObstaclePhase(),
              (unsigned)ObstacleControl_GetDistance(),
              (unsigned)ObstacleControl_GetNearCount());
      ConsolePrint(line);
      break;
    }
    case 'h': Robot_StopMarch(); break;
    case 'x':
      s_hcsr04_test_active = 0U;
      Robot_EmergencyOff();
      break;
    case 'p':
      ok = Robot_Scan(positions);
      if (ok) for (i = 0U; i < ROBOT_SERVO_COUNT; ++i) {
        sprintf(line, "ID%u=%u\r\n", (unsigned)i+1U, (unsigned)positions[i]);
        ConsolePrint(line);
      }
      break;
    case '?': PrintHelp(); break;
    default: return;
  }
  if (!ok) {
    sprintf(line, "REFUSED: %s\r\n", Robot_GetLastError());
    ConsolePrint(line);
  }
}

/* 只在稳定按下沿触发一次；必须松开并再次按下才会重复执行。 */
static void KeyService(void)
{
  const uint16_t pins[2] = {KEY1_Pin, KEY2_Pin};
  const uint8_t commands[2] = {'s', 'g'};
  uint32_t now = HAL_GetTick();
  uint8_t i;
  for (i = 0U; i < 2U; ++i) {
    uint8_t level = HAL_GPIO_ReadPin(GPIOE, pins[i]) == GPIO_PIN_SET ? 1U : 0U;
    if (level != s_key_raw[i]) {
      s_key_raw[i] = level;
      s_key_changed_ms[i] = now;
    } else if (level != s_key_stable[i] &&
               (uint32_t)(now - s_key_changed_ms[i]) >= KEY_DEBOUNCE_MS) {
      s_key_stable[i] = level;
      if (level == 0U) HandleCommand(commands[i]);
    }
  }
}

/* 只在静止/故障状态变更后输出，不在轨迹中阻塞打印。 */
static void ReportState(void)
{
  static RobotState previous = ROBOT_IDLE;
  RobotState state = Robot_GetState();
  char line[128];
  if (state == previous) return;
  previous = state;
  if (state == ROBOT_STANDING) ConsolePrint("START POSE READY\r\n");
  else if (state == ROBOT_HOLDING) {
    sprintf(line, "HOLD frame=%u; g/r/t resume, supported s returns to stand\r\n", (unsigned)Robot_GetFrame());
    ConsolePrint(line);
  } else if (state == ROBOT_FAULT) {
    sprintf(line, "FAULT frame=%u: %s; x requests torque OFF\r\n", (unsigned)Robot_GetFrame(), Robot_GetLastError());
    ConsolePrint(line);
  } else if (state == ROBOT_IDLE) ConsolePrint("IDLE; torque OFF requested\r\n");
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
  MX_USART1_UART_Init();
  /* 最早串口自检：放在TOF、舵机和机器人初始化之前，便于排除后续模块影响。 */
  {
    static const uint8_t uart_boot_message[] = "\r\nUART OK: HumanoidRobot 8.0_16MHz / HSE 16MHz\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t *)uart_boot_message,
                      (uint16_t)(sizeof(uart_boot_message) - 1U), 100U);
  }
  MX_UART4_Init();
#if ROBOT_ENABLE_TOF
  MX_I2C2_Init();
#endif
#if ROBOT_USE_EXTERNAL_ADAPTER
  MX_USART2_UART_Init();
#else
  MX_USART6_UART_Init();
#endif
  /* USER CODE BEGIN 2 */
#if ROBOT_USE_EXTERNAL_ADAPTER
  STS_Init(&huart2);
#else
  STS_Init(&huart6);
#endif
  if (!SG90_Init()) Error_Handler();
  HC_SR04_Init();
  ObstacleControl_Init();
  Robot_Init();
  VisionControl_Init();
  Vision_Init(&huart4);
  STS_TorqueEnable(STS_BROADCAST_ID, 0U);
  PrintHelp();
  ConsolePrint("READY\r\n");
#if ROBOT_ENABLE_TOF
  s_tof_ready = VL53L0X_TestInit(&hi2c2);
  s_tof_report_due = HAL_GetTick();
  if (s_tof_ready) {
    char tof_line[64];
    sprintf(tof_line, "TOF READY: VL53L0X ID=0x%02X, I2C2 PB10/PB11\r\n", VL53L0X_TestModelId());
    HAL_UART_Transmit(&huart1, (uint8_t *)tof_line, (uint16_t)strlen(tof_line), 100U);
  } else {
    char tof_error[96];
    sprintf(tof_error,
            "TOF INIT ERROR stage=%u: check XSHUT PC0, SDA PB11, SCL PB10\r\n",
            (unsigned)VL53L0X_TestInitStage());
    HAL_UART_Transmit(&huart1, (uint8_t *)tof_error,
                      (uint16_t)strlen(tof_error), 100U);
    s_tof_report_due = HAL_GetTick() + 1000U;
  }
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      uint8_t command;
      if (HAL_UART_Receive(&huart1, &command, 1U, 0U) == HAL_OK) HandleCommand(command);
      KeyService(); /* KEY1=s，KEY2=g；非阻塞消抖。 */
      VisionControl_Task(); /* action/事件分别处理；二进制数据不进入HandleCommand。 */
      ObstacleControl_Task(); /* 非阻塞超声 + 独立AA58横杆角；不改图形识别。 */
      Robot_Task();
      /* 手动j与图形6共用唯一SG90任务：500ms换向、3s回中并保持PWM。 */
      if (!SG90_Task(HAL_GetTick()) && s_sg90_test_active) {
        s_sg90_test_active = 0U;
        ConsolePrint("SG90 TEST DONE: centered, PWM holding\r\n");
      }
      if (s_hcsr04_test_active) {
        if (Robot_GetState() != ROBOT_IDLE) {
          s_hcsr04_test_active = 0U;
          ConsolePrint("HC-SR04 AUTO STOP: robot left IDLE\r\n");
        } else if ((int32_t)(HAL_GetTick() - s_hcsr04_report_due) >= 0) {
          uint32_t pulse_us;
          uint16_t distance_mm;
          HC_SR04_Result result;
          char ultrasonic_line[96];
          s_hcsr04_report_due = HAL_GetTick() + 250U;
          result = HC_SR04_Measure(&pulse_us, &distance_mm);
          if (result == HC_SR04_OK)
            sprintf(ultrasonic_line, "HC-SR04 distance=%u mm echo=%lu us\r\n",
                    (unsigned)distance_mm, (unsigned long)pulse_us);
          else
            sprintf(ultrasonic_line, "HC-SR04 ERROR=%s echo=%lu us\r\n",
                    HC_SR04_ResultText(result), (unsigned long)pulse_us);
          ConsolePrint(ultrasonic_line);
        }
      }
#if ROBOT_ENABLE_TOF
      if (s_tof_ready && (int32_t)(HAL_GetTick() - s_tof_report_due) >= 0) {
        uint16_t distance_mm;
        uint8_t range_status;
        uint8_t tof_result = VL53L0X_TestRead(&distance_mm, &range_status);
        s_tof_report_due = HAL_GetTick() + 200U;
        if (tof_result == 1U) {
          char tof_line[80];
          sprintf(tof_line, "TOF distance=%u mm status=%u GPIO1=%u\r\n",
                  (unsigned)distance_mm, (unsigned)range_status,
                  HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == GPIO_PIN_SET ? 1U : 0U);
          HAL_UART_Transmit(&huart1, (uint8_t *)tof_line,
                            (uint16_t)strlen(tof_line), 100U);
        } else if (tof_result == 0U) {
          static const uint8_t read_error[] = "TOF I2C READ ERROR\r\n";
          HAL_UART_Transmit(&huart1, (uint8_t *)read_error,
                            (uint16_t)(sizeof(read_error) - 1U), 100U);
        }
      } else if (!s_tof_ready && (int32_t)(HAL_GetTick() - s_tof_report_due) >= 0) {
        char tof_error[48];
        s_tof_report_due += 1000U;
        sprintf(tof_error, "TOF INIT ERROR stage=%u\r\n",
                (unsigned)VL53L0X_TestInitStage());
        HAL_UART_Transmit(&huart1, (uint8_t *)tof_error,
                          (uint16_t)strlen(tof_error), 100U);
      }
#endif
      ReportState();
      ConsoleService();
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}

static void MX_UART4_Init(void)
{
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK) Error_Handler();
}

static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();
}

#if ROBOT_ENABLE_TOF
static void MX_I2C2_Init(void)
{
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 50000U; /* 杜邦线测试降低速率，提高抗干扰能力。 */
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0U;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0U;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK) Error_Handler();
}
#endif

#if ROBOT_USE_EXTERNAL_ADAPTER
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 1000000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_8;
  if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
}
#else
static void MX_USART6_UART_Init(void)
{
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 1000000;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_8;
  if (HAL_UART_Init(&huart6) != HAL_OK) Error_Handler();
}
#endif

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
  /* 5.1控制板使用16MHz外部晶振，配置系统时钟为168MHz。 */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* VL53L0X：XSHUT默认拉低，初始化时再拉高；GPIO1为测距中断输入。 */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* 半双工方向控制上电默认关闭收发器，STS_Init后切换到接收状态。 */
  HAL_GPIO_WritePin(GPIOE, SERVO_TX_EN_Pin | SERVO_RX_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = SERVO_TX_EN_Pin | SERVO_RX_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* 板载KEY1/KEY2已有10k上拉；输入不再启用内部上下拉，按下读低。 */
  GPIO_InitStruct.Pin = KEY1_Pin | KEY2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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

#ifdef  USE_FULL_ASSERT
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
