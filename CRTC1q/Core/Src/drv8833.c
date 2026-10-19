/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : drv8833.c
  * @brief          : DRV8833 双 H 桥电机驱动实现
  * @version        : nu0.0
  *
  * 与 CubeMX 的分工：
  *   - TIM3/TIM4 的 PWM 初始化（含时钟、复用引脚）由 CubeMX 生成的
  *     MX_TIM3_Init() / MX_TIM4_Init() / HAL_TIM_PWM_MspInit() 负责，
  *     本文件只负责「启动输出 + 运行时调频 + 占空比控制」。
  *   - PA0/PA1 的下降沿中断由本文件负责（CubeMX 里 PA0/PA1 目前是
  *     普通 GPIO_Input，没有配 EXTI，所以不会冲突）。
  *
  * 注意：如果以后在 CubeMX 里把 PA0/PA1 改成 GPIO_EXTI0/EXTI1 并勾上
  *       NVIC 的 EXTI line0/line1，重新生成后 stm32f1xx_it.c 里会出现
  *       EXTI0_IRQHandler / EXTI1_IRQHandler，与本文件重复定义导致链接报错。
  *       那种情况下请删掉本文件里的 drv8833_ExtiInit() /
  *       EXTI0_IRQHandler / EXTI1_IRQHandler 三个部分。
  ******************************************************************************
  */
/* USER CODE END Header */

#include "drv8833.h"

/* CubeMX 在 main.c 中生成的定时器句柄 -------------------------------------*/
extern TIM_HandleTypeDef htim3;   /* 左侧：PA6/PA7/PB0/PB1 */
extern TIM_HandleTypeDef htim4;   /* 右侧：PB6/PB7/PB8/PB9 */

/* 私有定义 -----------------------------------------------------------------*/
/* 故障反馈引脚：正常为高，故障拉低（下降沿触发） */
#define DRV8833_NFAULT_PORT         GPIOA
#define DRV8833_NFAULT_LEFT_PIN     GPIO_PIN_0
#define DRV8833_NFAULT_RIGHT_PIN    GPIO_PIN_1

/* 私有类型 -----------------------------------------------------------------*/
typedef struct
{
  TIM_HandleTypeDef *htim;   /* 该电机所属的定时器 */
  uint32_t in1;              /* 正转通道（PWM） */
  uint32_t in2;              /* 反转通道（PWM） */
  uint8_t  side;             /* DRV8833_SIDE_LEFT / RIGHT */
} drv8833_motor_map_t;

/* 私有变量 -----------------------------------------------------------------*/
/* 电机 -> 通道映射表（顺序与 DRV8833_MOTOR_LF/LR/RF/RR 一致） */
static const drv8833_motor_map_t drv8833_map[DRV8833_MOTOR_NUM] =
{
  { &htim3, TIM_CHANNEL_1, TIM_CHANNEL_2, DRV8833_SIDE_LEFT  },  /* 左前 PA6/PA7 */
  { &htim3, TIM_CHANNEL_3, TIM_CHANNEL_4, DRV8833_SIDE_LEFT  },  /* 左后 PB0/PB1 */
  { &htim4, TIM_CHANNEL_1, TIM_CHANNEL_2, DRV8833_SIDE_RIGHT },  /* 右前 PB6/PB7 */
  { &htim4, TIM_CHANNEL_4, TIM_CHANNEL_3, DRV8833_SIDE_RIGHT },  /* 右后 PB9/PB8（IN1=PB9，方向与其它相反，软件对调） */
};

static int16_t drv8833_speed[DRV8833_MOTOR_NUM];         /* 当前速度缓存 */
/* 使用电机编号 LF/LR/RF/RR 排序，在头文件单独调整每个轮子的正负方向。 */
static const int8_t drv8833_direction[DRV8833_MOTOR_NUM] =
{
  DRV8833_DIR_LF, DRV8833_DIR_LR, DRV8833_DIR_RF, DRV8833_DIR_RR
};
static volatile uint8_t drv8833_fault[2];                /* [0]=左 [1]=右侧 */
static volatile uint16_t drv8833_fault_cnt[2];           /* nFAULT 下降沿计数 */

/* 私有函数 -----------------------------------------------------------------*/
static void     drv8833_SetPwmFreq(TIM_HandleTypeDef *htim);
static uint16_t drv8833_Arr(TIM_HandleTypeDef *htim);
static void     drv8833_ExtiInit(void);

/* 确保 PWM 频率为 20 kHz（与 CubeMX 配置一致）-------------------------------*/
static void drv8833_SetPwmFreq(TIM_HandleTypeDef *htim)
{
  __HAL_TIM_SET_PRESCALER(htim, DRV8833_PWM_PSC);
  __HAL_TIM_SET_AUTORELOAD(htim, DRV8833_PWM_ARR);

  /* PSC 有影子寄存器，产生一次更新事件让它立刻生效 */
  (void)HAL_TIM_GenerateEvent(htim, TIM_EVENTSOURCE_UPDATE);
}

/* 取当前定时器的 ARR（占空比满量程），跟随实际配置，不写死 */
static uint16_t drv8833_Arr(TIM_HandleTypeDef *htim)
{
  return (uint16_t)__HAL_TIM_GET_AUTORELOAD(htim);
}

/* PA0/PA1 下降沿中断 + NVIC */
static void drv8833_ExtiInit(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_AFIO_CLK_ENABLE();

  gpio.Pin  = DRV8833_NFAULT_LEFT_PIN | DRV8833_NFAULT_RIGHT_PIN;   /* PA0 | PA1 */
  gpio.Mode = GPIO_MODE_IT_FALLING;    /* 低电平 = 故障，抓下降沿 */
  gpio.Pull = GPIO_PULLUP;             /* 正常时内部上拉维持高电平 */
  HAL_GPIO_Init(DRV8833_NFAULT_PORT, &gpio);

  HAL_NVIC_SetPriority(EXTI0_IRQn, DRV8833_EXTI_PREEMPT_PRIO, DRV8833_EXTI_SUB_PRIO);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, DRV8833_EXTI_PREEMPT_PRIO, DRV8833_EXTI_SUB_PRIO);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}

/* 中断服务函数 -------------------------------------------------------------*/
void EXTI0_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(DRV8833_NFAULT_LEFT_PIN);
}

void EXTI1_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(DRV8833_NFAULT_RIGHT_PIN);
}

/* 中断回调：记故障 + 立即停掉对应侧 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == DRV8833_NFAULT_LEFT_PIN)
  {
    drv8833_fault_cnt[DRV8833_SIDE_LEFT]++;
    drv8833_fault[DRV8833_SIDE_LEFT] = 1U;
    DRV8833_StopSide(DRV8833_SIDE_LEFT);
  }
  else if (GPIO_Pin == DRV8833_NFAULT_RIGHT_PIN)
  {
    drv8833_fault_cnt[DRV8833_SIDE_RIGHT]++;
    drv8833_fault[DRV8833_SIDE_RIGHT] = 1U;
    DRV8833_StopSide(DRV8833_SIDE_RIGHT);
  }
  else
  {
    /* 其它 EXTI 线，本驱动不处理 */
  }
}

/* 对外接口 -----------------------------------------------------------------*/
void DRV8833_Init(void)
{
  drv8833_fault[DRV8833_SIDE_LEFT]  = 0U;
  drv8833_fault[DRV8833_SIDE_RIGHT] = 0U;
  drv8833_fault_cnt[DRV8833_SIDE_LEFT]  = 0U;
  drv8833_fault_cnt[DRV8833_SIDE_RIGHT] = 0U;

  /* 1) 确保 PWM 频率为 20 kHz（与 CubeMX 的 PSC=0 / ARR=3599 一致） */
  drv8833_SetPwmFreq(&htim3);
  drv8833_SetPwmFreq(&htim4);

  /* 2) 启动 8 路 PWM 输出（CubeMX 只初始化，不启动） */
  (void)HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  (void)HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  (void)HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  (void)HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  (void)HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  (void)HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  (void)HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  (void)HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

  /* 3) 上电先刹车，防止电机乱转 */
  DRV8833_BrakeAll();

  /* 4) 最后开故障中断 */
  drv8833_ExtiInit();
}

void DRV8833_SetMotor(uint8_t motor, int16_t speed)
{
  const drv8833_motor_map_t *m;
  uint32_t arr;
  uint32_t period;
  uint32_t duty;
  uint16_t cmp1;
  uint16_t cmp2;

  if (motor >= DRV8833_MOTOR_NUM)
  {
    return;
  }

  if (speed > DRV8833_SPEED_MAX)
  {
    speed = DRV8833_SPEED_MAX;
  }
  else if (speed < -DRV8833_SPEED_MAX)
  {
    speed = -DRV8833_SPEED_MAX;
  }

  /* 已限幅到 +/-1000，再乘 +/-1，不会发生 int16_t 负数溢出。 */
  speed = (int16_t)(speed * drv8833_direction[motor]);
  m   = &drv8833_map[motor];
  arr = (uint32_t)drv8833_Arr(m->htim);
  period = arr + 1U;

  if (speed == 0)
  {
    /* 刹车：两个输入都拉满 -> IN1=IN2=1 */
    cmp1 = (uint16_t)period;
    cmp2 = (uint16_t)period;
  }
  else
  {
    duty = ((uint32_t)((speed > 0) ? (int32_t)speed : -(int32_t)speed)
            * period) / (uint32_t)DRV8833_SPEED_MAX;

#if DRV8833_PWM_BRAKE_MODE
    /* PWM1、高电平有效：关断时两脚同为 1（制动），驱动时两脚相反。
     * speed=0 -> 两脚恒高；speed=1000 -> 一高一低，驱动时间仍是 100%。
     * duty 用完整周期 ARR+1 计算，不能直接把 duty 写到反相一侧。 */
    if (speed > 0)
    {
      cmp1 = (uint16_t)period;
      cmp2 = (uint16_t)(period - duty);
    }
    else
    {
      cmp1 = (uint16_t)(period - duty);
      cmp2 = (uint16_t)period;
    }
#else
    /* 原来的驱动/滑行模式，留作同车对照。 */
    if (speed > 0)
    {
      cmp1 = (uint16_t)duty;   /* IN1 = PWM，正转 */
      cmp2 = 0U;
    }
    else
    {
      cmp1 = 0U;
      cmp2 = (uint16_t)duty;   /* IN2 = PWM，反转 */
    }
#endif
  }

  __HAL_TIM_SET_COMPARE(m->htim, m->in1, cmp1);
  __HAL_TIM_SET_COMPARE(m->htim, m->in2, cmp2);

  drv8833_speed[motor] = speed;
}

int16_t DRV8833_GetMotor(uint8_t motor)
{
  if (motor >= DRV8833_MOTOR_NUM)
  {
    return 0;
  }

  return drv8833_speed[motor];
}

void DRV8833_BrakeAll(void)
{
  uint8_t i;

  for (i = 0U; i < DRV8833_MOTOR_NUM; i++)
  {
    uint16_t period = (uint16_t)(drv8833_Arr(drv8833_map[i].htim) + 1U);

    __HAL_TIM_SET_COMPARE(drv8833_map[i].htim, drv8833_map[i].in1, period);
    __HAL_TIM_SET_COMPARE(drv8833_map[i].htim, drv8833_map[i].in2, period);
    drv8833_speed[i] = 0;
  }
}

void DRV8833_CoastAll(void)
{
  uint8_t i;

  for (i = 0U; i < DRV8833_MOTOR_NUM; i++)
  {
    __HAL_TIM_SET_COMPARE(drv8833_map[i].htim, drv8833_map[i].in1, 0U);
    __HAL_TIM_SET_COMPARE(drv8833_map[i].htim, drv8833_map[i].in2, 0U);
    drv8833_speed[i] = 0;
  }
}

void DRV8833_StopSide(uint8_t side)
{
  uint8_t i;

  for (i = 0U; i < DRV8833_MOTOR_NUM; i++)
  {
    if (drv8833_map[i].side == side)
    {
      __HAL_TIM_SET_COMPARE(drv8833_map[i].htim, drv8833_map[i].in1, 0U);
      __HAL_TIM_SET_COMPARE(drv8833_map[i].htim, drv8833_map[i].in2, 0U);
      drv8833_speed[i] = 0;
    }
  }
}

uint8_t DRV8833_GetFault(uint8_t side)
{
  if (side > DRV8833_SIDE_RIGHT)
  {
    return 0U;
  }

  return drv8833_fault[side];
}

void DRV8833_ClearFault(uint8_t side)
{
  if (side <= DRV8833_SIDE_RIGHT)
  {
    drv8833_fault[side] = 0U;
  }
}

uint16_t DRV8833_GetFaultCount(uint8_t side)
{
  if (side > DRV8833_SIDE_RIGHT)
  {
    return 0U;
  }

  return drv8833_fault_cnt[side];
}
