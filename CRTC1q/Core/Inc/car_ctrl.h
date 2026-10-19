/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : car_ctrl.h
  * @brief          : 整车抽象层——把 8 路 PWM 电机 + PCA9685 舵机 收成一组"口语化"命令
  * @version        : nu0.0
  *
  * ============================ 这一层是什么 ============================
  * 这是一个"翻译层 / 抽象层"，它不碰任何寄存器，只做两件事：
  *   ① 把上层说的"口语"翻译成下层驱动能懂的参数：
  *        - 电机：轮子编号 + 方向 + 百分比  →  电机编号 + (-1000 ~ +1000) 刻度
  *        - 舵机：舵机编号 + 角度            →  PCA9685 通道号 + 12 位刻度
  *   ② 做参数范围检查（越界拒绝 / 超限限幅），让上层不必关心底层细节。
  *
  * 三层结构（上层只依赖本层，不要越级调下层）：
  *     上层逻辑（遥控 / 循迹 / 串口命令 / PID）
  *        ↓  只调 Car_Motor() / Car_MotorAll() / Car_MotorStop*() / Car_Servo() / Car_ServoPulse()
  *     car_ctrl.c / car_ctrl.h    ←【本层】面向"整车"：轮子编号、方向、百分比、角度
  *        ↓  内部调 DRV8833_SetMotor() / PCA9685_SetChannelPwm()
  *     drv8833.c / drv8833.h   ← 面向"1 个电机"：TIM3/TIM4 八路 20kHz PWM + PA0/PA1 nFAULT 中断
  *     pca9685.c / pca9685.h   ← 面向"1 个芯片"：I2C2 寄存器读写、脉宽 ↔ 12 位刻度换算
  *
  * ============================ 命令清单 ============================
  * 两类命令，参数一律是"人话"：
  *
  *   电机（4 个轮子，速度单位 = 百分比）
  *     Car_Motor(CAR_WHEEL_RF, CAR_DIR_FORWARD, 100U);   // 右前轮，前进，100%
  *     Car_Motor(CAR_WHEEL_LF, CAR_DIR_BACKWARD, 60U);   // 左前轮，后退，60%
  *     Car_MotorAll(CAR_DIR_FORWARD, 80U);               // 四轮一起，前进，80%
  *     Car_MotorStop(CAR_WHEEL_LR);                      // 左后轮单独刹车
  *     Car_MotorStopAll();                               // 四轮一起刹车（急停）
  *
  *   舵机（8 个，角度单位 = 度，90 度是中位）
  *     Car_Servo(0U, 45U);                               // 0 号舵机，转到 45 度
  *     Car_ServoPulse(0U, 1500U);                        // 0 号舵机，直接给 1500us 脉宽
  *
  * ============================ 逻辑（怎么翻译的） ============================
  *   1) 速度：百分比 0~100 先换算成 DRV8833 的 0~1000 刻度（speed * 1000 / 100），
  *      再按 dir 加正负号（前进 = 正，后退 = 负）；超过 100 按 100 限幅（不报错）。
  *   2) speed = 0 的语义是"刹车"（IN1/IN2 同时拉高，电机快速停住），不是断电滑行。
  *   3) 角度：0~180 度线性映射到 500~2500us 脉宽，再按载波频率换算成 12 位刻度
  *      （50Hz 时 ticks = (us * 4096 + 10000) / 20000）。改载波频率必须同步改 CAR_SERVO_PWM_FREQ_HZ。
  *   4) 越界处理：轮子编号越界 → 静默忽略（不写任何电机）；舵机编号越界 → 返回 HAL_ERROR。
  *      这样上层就算参数传错，也不会误动别的执行器。
  *
  * ============================ 它不负责什么 ============================
  *   - 不初始化硬件：PWM 启动、nFAULT 中断、I2C 与 PCA9685 的 50Hz 设置都在 main() 里做
  *     （DRV8833_Init()、PCA9685_Init()/SetPwmFreq()）。本层只负责"发命令"。
  *   - 不做闭环：不读编码器、不做 PID、不做速度斜坡（渐强渐弱），那些属于再上一层。
  *   - 不管故障：nFAULT 拉低由 drv8833 的中断直接停掉对应侧电机，本层不参与。
  *   - 目前不提供"断电滑行(Coast)"，只有刹车；确需滑行要在 drv8833 层另加接口。
  *
  * 使用前提：
  *   main() 里必须已经调用过 DRV8833_Init() 和 PCA9685_Init()；
  *   否则舵机命令返回 HAL_ERROR（上电日志会打印 "PCA9685 NOT FOUND"），电机命令无效果。
  *
  * 如果以后要做串口命令解析（比如收到 "rightfront forward 100"）：
  *   在解析器里把字符串映射成枚举即可，不需要改本层：
  *     "leftfront"/"leftrear"/"rightfront"/"rightrear"  -> CAR_WHEEL_LF/LR/RF/RR
  *     "forward"/"backward"                             -> CAR_DIR_FORWARD/BACKWARD
  *   然后照样调 Car_Motor() / Car_Servo()。
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __CAR_CTRL_H
#define __CAR_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 版本 ---------------------------------------------------------------------*/
#define CAR_CTRL_VERSION            "nu0.0"

/* ---- 轮子相关 -----------------------------------------------------------*/

/* 轮子数量 */
#define CAR_WHEEL_NUM               (4U)

/* 速度单位：对外一律用"百分比"，0 = 停（刹车），100 = 满速。
 * 内部会换算成 drv8833 的 -1000 ~ +1000 刻度。
 * 这样写上层逻辑时不用记住 -1000 这种刻度值，跟遥控器/上位机发来的
 * "速度 100" 也能直接对上。 */
#define CAR_SPEED_MIN               (0U)
#define CAR_SPEED_MAX               (100U)

/* 轮子编号：顺序与 drv8833.h 里的 DRV8833_MOTOR_LF/LR/RF/RR 完全一致，
 * 所以本层可以直接把 car_wheel_t 强转成 drv8833 的电机编号。
 *   物理引脚对应关系（详见 drv8833.h 顶部说明）：
 *     左前 IN1=PA6(TIM3_CH1)  IN2=PA7(TIM3_CH2)
 *     左后 IN1=PB0(TIM3_CH3)  IN2=PB1(TIM3_CH4)
 *     右前 IN1=PB6(TIM4_CH1)  IN2=PB7(TIM4_CH2)
 *     右后 IN1=PB9(TIM4_CH4)  IN2=PB8(TIM4_CH3) ← 该电机线序相反，已在驱动层软件对调
 * 如果以后要加第 5 个轮子，改这里 + car_ctrl.c 里的换算，并同步改 drv8833 的映射表。 */
typedef enum
{
  CAR_WHEEL_LF = 0,   /* 左前 Left  Front */
  CAR_WHEEL_LR = 1,   /* 左后 Left  Rear  */
  CAR_WHEEL_RF = 2,   /* 右前 Right Front */
  CAR_WHEEL_RR = 3    /* 右后 Right Rear  */
} car_wheel_t;

/* 方向：以"车头朝前"为基准，不是以电机自身为基准。
 * 也就是说不管这个轮子装在左边还是右边、电机线序正接还是反接，
 * CAR_DIR_FORWARD 一定是让车往前走的那个电流方向（反接问题已在驱动层修好）。
 * DRV8833 真值表：正转 = IN1 出 PWM / IN2 恒低，反转 = IN1 恒低 / IN2 出 PWM。 */
typedef enum
{
  CAR_DIR_FORWARD  = 0,   /* 前进 */
  CAR_DIR_BACKWARD = 1    /* 后退 */
} car_dir_t;

/* ---- 舵机相关 -----------------------------------------------------------*/

/* 本车一共 8 路舵机，占 PCA9685 的 0 ~ 7 号通道（PCA9685 共 16 路，够用就说）。
 * 改成 16 也可以，只要把这里改大。 */
#define CAR_SERVO_NUM               (8U)

/* 角度范围（度）：0 ~ 180，90 度为中位。
 * 对应脉宽 500 ~ 2500 us —— SG90 / MG90S 这类标准舵机的常用范围。
 * 如果你的舵机型号不同（例如只接受 1000 ~ 2000 us），只改下面两个宏即可；
 * 也可以用 Car_ServoPulse() 直接按微秒标定，绕开角度换算。 */
#define CAR_SERVO_ANGLE_MIN         (0U)
#define CAR_SERVO_ANGLE_MAX         (180U)
#define CAR_SERVO_MIN_US            (500U)      /* 0 度对应脉宽 */
#define CAR_SERVO_MAX_US            (2500U)     /* CAR_SERVO_ANGLE_MAX 度对应脉宽 */

/* 舵机 PWM 载波频率：50 Hz（周期 20 ms），标准舵机频率。
 * 必须与 main() 里 PCA9685_SetPwmFreq(&hi2c2, ..., PCA9685_FREQ_HZ) 的值一致，
 * 因为 Car_ServoPulse() 的"微秒 -> 12 位刻度"换算是按这个频率算的。
 * 改了频率就必须同步改这里。 */
#define CAR_SERVO_PWM_FREQ_HZ       (50U)
#define CAR_SERVO_PERIOD_US         (1000000U / CAR_SERVO_PWM_FREQ_HZ)

/* 舵机使用高电平脉冲，保持 0。升降反向应在 arm_ctrl.h 中修改
 * ARM_BIG_UP_REVERSED / ARM_SMALL_UP_REVERSED，不能靠改变波形极性实现。 */
#define CAR_SERVO_INVERTED          (0U)

/* ---- 电机（轮子）接口 ---------------------------------------------------*/

/* 设定某个轮子的方向与速度。
 *   wheel : CAR_WHEEL_LF / LR / RF / RR（越界会被直接忽略，不会写坏别的电机）
 *   dir   : CAR_DIR_FORWARD（前进）/ CAR_DIR_BACKWARD（后退）
 *   speed : 0 ~ 100（百分比），超过 100 自动按 100 处理
 * 注意：speed = 0 执行的是"刹车"（两个输入同时拉高，电机快速停住），不是断电滑行。
 *       等价于 Car_MotorStop(wheel)。 */
void Car_Motor(car_wheel_t wheel, car_dir_t dir, uint8_t speed);

/* 精细调速：-1000~+1000，对应反向/正向 0~100% PWM，分辨率 0.1%。
 * 0 仍为刹车；保留现有电机编号、引脚和正反转定义。 */
void Car_MotorPermille(car_wheel_t wheel, int16_t speed);

/* 四个轮子一起设定（做直线前进/后退时用）。 */
void Car_MotorAll(car_dir_t dir, uint8_t speed);

/* 单个轮子刹车（两个输入同时拉高 = 低侧短路制动），比 speed=0 更直观。 */
void Car_MotorStop(car_wheel_t wheel);

/* 四个轮子一起刹车（急停就用这个）。 */
void Car_MotorStopAll(void);

/* ---- 舵机接口 -----------------------------------------------------------*/

/* 按角度控制舵机（日常用这个）。
 *   id    : 0 ~ 7（0 号 ~ 7 号舵机，对应 PCA9685 的 0 ~ 7 通道）
 *   angle : 0 ~ 180（度），超过 180 自动按 180 处理，90 度是中位
 * 返回 HAL_OK = 已发出；HAL_ERROR = 编号越界 / PCA9685 没有初始化成功
 * （后一种情况说明 I2C 没接通，main 上电日志里会打印 "PCA9685 NOT FOUND"）。 */
HAL_StatusTypeDef Car_Servo(uint8_t id, uint8_t angle);

/* 直接按脉宽控制舵机（标定/调试用，比如找这个舵机的真实 0 度和最大角度）。
 *   pulse_us : 高电平时间，单位微秒。标准舵机大约 500 ~ 2500
 * 角度和脉宽的关系是线性的，换算见 Car_Servo() 的实现。 */
HAL_StatusTypeDef Car_ServoPulse(uint8_t id, uint16_t pulse_us);

#ifdef __cplusplus
}
#endif

#endif /* __CAR_CTRL_H */
