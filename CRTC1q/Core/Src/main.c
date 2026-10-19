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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdarg.h>
#include "pca9685.h"
#include "drv8833.h"
#include "car_ctrl.h"
#include "ps2.h"
#include "ps2_ctrl.h"
#include "arm_ctrl.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* TRYONE 原样搬来的参数（PCA9685 舵机驱动）*/
#define PCA9685_ADDR          (0x40U)     /* A0=A1=A2=000 -> 7-bit 0x40 */
#define PCA9685_FREQ_HZ       ((float)CAR_SERVO_PWM_FREQ_HZ)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t pca9685_ready = 0U;   /* 1 = 初始化成功，可以发命令 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* ---- USART1 调试打印：PA9 = TX，115200 8N1 ----
 * 写法与 TRYONE 工程一致：先 vsnprintf 到栈上缓冲，再阻塞发送。
 * 单条消息不要超过 UART_TX_BUF，超了会被截断。 */
#define UART_TX_BUF  (160U)

static void UART_Printf(const char *fmt, ...)
{
  char buf[UART_TX_BUF];
  va_list ap;
  int len;

  va_start(ap, fmt);
  len = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);

  if (len <= 0)
  {
    return;
  }
  if ((size_t)len >= sizeof(buf))
  {
    len = (int)(sizeof(buf) - 1U);
  }

  (void)HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)len, 100U);
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
  MX_I2C2_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  /* ---- PCA9685 舵机驱动初始化（TRYONE 原样搬来）----
   * I2C2 已由 CubeMX 初始化为 100 kHz，PB10(SCL) / PB11(SDA)。
   * 这里直接对 0x40 发初始化序列：MODE1/MODE2 寄存器 + 50 Hz PWM 频率。
   * 如果板子没接或接线错，pca9685_ready 保持 0，while 循环就不会发命令。 */
  if (PCA9685_Init(&hi2c2, PCA9685_ADDR) == HAL_OK &&
      PCA9685_SetPwmFreq(&hi2c2, PCA9685_ADDR, PCA9685_FREQ_HZ) == HAL_OK &&
      PCA9685_AllOff(&hi2c2, PCA9685_ADDR) == HAL_OK)
  {
    pca9685_ready = 1U;
  }

  /* ---- DRV8833 电机驱动初始化 ----
   * TIM3(左) / TIM4(右) 各 4 路 20 kHz PWM，
   * PA0/PA1 配成下降沿中断（NFAULT 低电平 = 故障）。 */
  DRV8833_Init();

  /* ---- PS2 手柄初始化（GPIO 模拟 SPI，PB13/14/15 + PA8）----
   * PS2_Ctrl_Init() 内部会调用 PS2_Init() 和 PS2_ConfigAnalog()，
   * 把引脚从 CubeMX 的 SPI2 复用功能改成 GPIO，并让手柄进入模拟模式。
   * 摇杆与机械臂按键始终同时生效，无需切换模式。 */
  PS2_Ctrl_Init();

  /* ---- USART1 上电自检快照（PA9=TX / PA10=RX，115200 8N1）---- */
  UART_Printf("\r\n=== CRTC1 boot | DRV8833 %s | PCA9685 %s | PS2 ready ===\r\n",
              DRV8833_VERSION, (pca9685_ready ? "ready" : "NOT FOUND"));

  UART_Printf("[PWM] TIM3 ARR=%u PSC=%u | TIM4 ARR=%u PSC=%u -> 20 kHz, 8 channels started\r\n",
              (unsigned)__HAL_TIM_GET_AUTORELOAD(&htim3), (unsigned)htim3.Instance->PSC,
              (unsigned)__HAL_TIM_GET_AUTORELOAD(&htim4), (unsigned)htim4.Instance->PSC);

  UART_Printf("[FLT] idle levels: PA0/LEFT=%u  PA1/RIGHT=%u  (1=normal, 0=fault or floating)\r\n",
              (unsigned)HAL_GPIO_ReadPin(NFAULT1_GPIO_Port, NFAULT1_Pin),
              (unsigned)HAL_GPIO_ReadPin(NFAULT2_GPIO_Port, NFAULT2_Pin));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* ================= 上层逻辑：PS2 并行控制 =================
     * PS2_Ctrl_Update() 内部流程：
     *   1. PS2_Read() 读手柄
     *   2. 摇杆开车，肩键/叉号同时操作机械臂，SELECT 不参与控制
     *   3. 调抽象命令 Car_Motor* / Car_Servo*
     * 读不到手柄时底盘刹车，舵机保持最后位置；重连后需先松开机械臂按键。 */
    PS2_Ctrl_Update();

    /* ---- USART1 诊断：每 500 ms 打印摇杆、底盘和机械臂指令 ----
     * cnt = nFAULT 下降沿累计次数（真进过几次 EXTI0/EXTI1 中断），
     * pin = 当前引脚电平（1=正常，0=正在报故障或线悬空）。
     * CCR 是真正写进定时器的比较值，有速度时应该是非 0。 */

    {
      static uint32_t d_t0 = 0U;
      uint32_t now = HAL_GetTick();
      const PS2_t *input = PS2_Ctrl_GetInput();
      const arm_ctrl_status_t *arm = Arm_Ctrl_GetStatus();

      if ((now - d_t0) >= 500U)
      {
        d_t0 = now;

        UART_Printf("[PS2] raw LX=%u LY=%u RX=%u ok=%u DZ=%u\r\n",
                    (unsigned)input->LJoy_LR, (unsigned)input->LJoy_UD,
                    (unsigned)input->RJoy_LR, (unsigned)input->ok,
                    (unsigned)PS2_DEADZONE);

        UART_Printf("[DRV] spd=%d,%d,%d,%d | FLT L f=%u n=%u p=%u | R f=%u n=%u p=%u\r\n",
                    DRV8833_GetMotor(DRV8833_MOTOR_LF), DRV8833_GetMotor(DRV8833_MOTOR_LR),
                    DRV8833_GetMotor(DRV8833_MOTOR_RF), DRV8833_GetMotor(DRV8833_MOTOR_RR),
                    (unsigned)DRV8833_GetFault(DRV8833_SIDE_LEFT),
                    (unsigned)DRV8833_GetFaultCount(DRV8833_SIDE_LEFT),
                    (unsigned)HAL_GPIO_ReadPin(NFAULT1_GPIO_Port, NFAULT1_Pin),
                    (unsigned)DRV8833_GetFault(DRV8833_SIDE_RIGHT),
                    (unsigned)DRV8833_GetFaultCount(DRV8833_SIDE_RIGHT),
                    (unsigned)HAL_GPIO_ReadPin(NFAULT2_GPIO_Port, NFAULT2_Pin));

        UART_Printf("[PWM] T3 CCR=%u,%u,%u,%u | T4 CCR=%u,%u,%u,%u\r\n",
                    (unsigned)__HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_1),
                    (unsigned)__HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_2),
                    (unsigned)__HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_3),
                    (unsigned)__HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_4),
                    (unsigned)__HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_1),
                    (unsigned)__HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_2),
                    (unsigned)__HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_3),
                    (unsigned)__HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_4));

        UART_Printf("[ARM] A0x10=%u A1x10=%u A2x10=%u A3x10=%u open=%u ready=%u status=%u\r\n",
                    (unsigned)arm->angle_x10[0],
                    (unsigned)arm->angle_x10[1], (unsigned)arm->angle_x10[2],
                    (unsigned)arm->angle_x10[3],
                    (unsigned)arm->claw_open, (unsigned)arm->ready_mask,
                    (unsigned)arm->last_status);
      }
    }

    HAL_Delay(20U);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 3599;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 3599;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : NFAULT1_Pin NFAULT2_Pin */
  GPIO_InitStruct.Pin = NFAULT1_Pin|NFAULT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_CS_Pin */
  GPIO_InitStruct.Pin = SPI2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI2_CS_GPIO_Port, &GPIO_InitStruct);

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
