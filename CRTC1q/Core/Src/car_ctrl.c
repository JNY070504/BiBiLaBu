/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : car_ctrl.c
  * @brief          : 小车控制层实现（轮子百分比速度 + 舵机角度）
  * @version        : nu0.0
  *
  * 本文件只做"翻译"，不直接碰寄存器：
  *   Car_Motor()       -> 百分比换算成 ±1000       -> DRV8833_SetMotor()
  *   Car_MotorStop()   -> 速度填 0（两脚拉满 = 刹车）-> DRV8833_SetMotor(wheel, 0)
  *   Car_MotorStopAll()-> 四轮急停                 -> DRV8833_BrakeAll()
  *   Car_Servo()       -> 角度换算成脉宽/刻度      -> PCA9685_SetChannelPwm()
  *
  * 所有硬件细节（TIM3/TIM4 通道、nFAULT 中断、I2C 地址、12 位刻度）都留在
  * drv8833.c / pca9685.c 里，本层不重复实现，也不直接写 TIM/I2C 寄存器。
  ******************************************************************************
  */
/* USER CODE END Header */

#include "car_ctrl.h"
#include "drv8833.h"
#include "pca9685.h"

/* main.c 里的全局对象 -------------------------------------------------------
 * hi2c2         : CubeMX 生成的 I2C2 句柄（PB10=SCL / PB11=SDA，100 kHz）
 * pca9685_ready : main 上电自检的结果，1 = PCA9685 应答正常，可以发命令
 * 这两个都是 main.c 里的全局变量，这里只声明不定义，避免重复定义。 */
extern I2C_HandleTypeDef hi2c2;
extern uint8_t pca9685_ready;

/* 私有定义 -----------------------------------------------------------------*/

/* PCA9685 的 7 位 I2C 地址。
 * main() 里 PCA9685_Init() 用的是 0x40（A0=A1=A2 全接地），
 * 这里用 pca9685.h 提供的默认宏，保证和初始化时是同一个地址。 */
#define CAR_SERVO_I2C_ADDR          (PCA9685_DEFAULT_ADDR)

/* 编译期检查：本层假设"4 个轮子"和 drv8833 的电机数一致。
 * 如果以后改了 drv8833.h 里的 DRV8833_MOTOR_NUM，这里会直接编译报错，
 * 避免出现"表对不上但编译还能过、跑起来轮子乱转"的隐蔽 bug。 */
#if (DRV8833_MOTOR_NUM != CAR_WHEEL_NUM)
#error "CAR_WHEEL_NUM 与 DRV8833_MOTOR_NUM 不一致，请同步修改 car_ctrl.h 的轮子编号定义"
#endif

/* 私有函数 ----------------------------------------------------------------*/

/* 百分比速度 -> drv8833 的速度刻度（-1000 ~ +1000，取绝对值）
 *   0%   ->    0
 *   50%  ->  500
 *   100% -> 1000
 * 越界（>100）先限幅，避免上层算错把满速当成 200 发下去。 */
static int16_t Car_SpeedToDrv(uint8_t speed)
{
  if (speed > CAR_SPEED_MAX)
  {
    speed = CAR_SPEED_MAX;
  }

  return (int16_t)(((int32_t)speed * (int32_t)DRV8833_SPEED_MAX) / (int32_t)CAR_SPEED_MAX);
}

/* 微秒 -> PCA9685 的 12 位刻度
 * 50 Hz 时一个周期是 20 ms，被平分成 4096 格（0 ~ 4095），
 * 所以 1 格 ≈ 4.88 us。换算时 +半个周期 是为了四舍五入。
 * 例：1500 us（中位）-> 307 格；500 us -> 102 格；2500 us -> 512 格。
 * 这里必须和 CAR_SERVO_PWM_FREQ_HZ 保持一致，改了频率这个换算就不成立。 */
static uint16_t Car_UsToTicks(uint16_t pulse_us)
{
  return (uint16_t)((((uint32_t)pulse_us * (PCA9685_PWM_MAX + 1U)) + (CAR_SERVO_PERIOD_US / 2U))
                    / CAR_SERVO_PERIOD_US);
}

/* 对外接口实现 -----------------------------------------------------------*/

void Car_Motor(car_wheel_t wheel, car_dir_t dir, uint8_t speed)
{
  int16_t drv_speed;

  /* 越界保护：确保后面不会拿一个非法编号去写定时器通道 */
  if ((uint8_t)wheel >= CAR_WHEEL_NUM)
  {
    return;
  }

  drv_speed = Car_SpeedToDrv(speed);

  if (drv_speed == 0)
  {
    /* 速度 0 = 刹车（IN1=IN2=1），电机立刻停住，不会因为惯性溜车 */
    DRV8833_SetMotor((uint8_t)wheel, 0);
    return;
  }

  /* 方向决定正负号：正 = 前进，负 = 后退。
   * car_wheel_t 的编号与 drv8833 的电机编号一一对应，所以这里直接强转。 */
  DRV8833_SetMotor((uint8_t)wheel,
                   (dir == CAR_DIR_FORWARD) ? drv_speed : (int16_t)(-drv_speed));
}

void Car_MotorAll(car_dir_t dir, uint8_t speed)
{
  uint8_t i;

  /* 四个轮子用同一个方向和速度，四路 PWM 同时改，保证同步起步 */
  for (i = 0U; i < CAR_WHEEL_NUM; i++)
  {
    Car_Motor((car_wheel_t)i, dir, speed);
  }
}

void Car_MotorPermille(car_wheel_t wheel, int16_t speed)
{
  if ((uint8_t)wheel >= CAR_WHEEL_NUM)
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

  DRV8833_SetMotor((uint8_t)wheel, speed);
}

void Car_MotorStop(car_wheel_t wheel)
{
  if ((uint8_t)wheel >= CAR_WHEEL_NUM)
  {
    return;
  }

  /* DRV8833_SetMotor(…, 0) 的内部语义就是"两脚同时拉满 = 刹车" */
  DRV8833_SetMotor((uint8_t)wheel, 0);
}

void Car_MotorStopAll(void)
{
  DRV8833_BrakeAll();
}

HAL_StatusTypeDef Car_Servo(uint8_t id, uint8_t angle)
{
  uint16_t pulse_us;

  if (id >= CAR_SERVO_NUM)
  {
    return HAL_ERROR;                 /* 只有 0 ~ 7 号舵机 */
  }

  if (angle > CAR_SERVO_ANGLE_MAX)
  {
    angle = CAR_SERVO_ANGLE_MAX;      /* 越界限幅，不报错，避免上层算出 200 度就整条命令失败 */
  }

  /* 角度 -> 脉宽：线性插值，0 度 = 500 us，180 度 = 2500 us，中间 90 度正好 1500 us
   * 例：45 度 -> 500 + 45*2000/180 = 500 + 500 = 1000 us */
  pulse_us = (uint16_t)(CAR_SERVO_MIN_US +
                        (((uint32_t)angle * (uint32_t)(CAR_SERVO_MAX_US - CAR_SERVO_MIN_US))
                         / CAR_SERVO_ANGLE_MAX));

  return Car_ServoPulse(id, pulse_us);
}

HAL_StatusTypeDef Car_ServoPulse(uint8_t id, uint16_t pulse_us)
{
  if (id >= CAR_SERVO_NUM)
  {
    return HAL_ERROR;                 /* 编号越界 */
  }

  /* PCA9685 没应答（I2C 没接、地址不对、供电不对）时直接返回错误，
   * 免得在总线上反复等超时，把主循环拖慢。 */
  if (pca9685_ready == 0U)
  {
    return HAL_ERROR;
  }

  if (pulse_us > CAR_SERVO_PERIOD_US)
  {
    return HAL_ERROR;                 /* 超过一个 PWM 周期没有意义 */
  }

  return PCA9685_SetChannelPwm(&hi2c2, CAR_SERVO_I2C_ADDR, id,
                               Car_UsToTicks(pulse_us), CAR_SERVO_INVERTED);
}
