/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : drv8833.h
  * @brief          : DRV8833 双 H 桥电机驱动（4 电机 4WD 小车）
  * @version        : nu0.0
  ******************************************************************************
  * 硬件连接（与 CRTC1.ioc 一致）：
  *
  *   左侧 DRV8833  -> TIM3
  *     左前电机  IN1 = PA6  (TIM3_CH1)   IN2 = PA7  (TIM3_CH2)
  *     左后电机  IN1 = PB0  (TIM3_CH3)   IN2 = PB1  (TIM3_CH4)
  *
  *   右侧 DRV8833  -> TIM4
  *     右前电机  IN1 = PB6  (TIM4_CH1)   IN2 = PB7  (TIM4_CH2)
  *     右后电机  IN1 = PB9  (TIM4_CH4)   IN2 = PB8  (TIM4_CH3)
  *               ↑ 该电机实际线序与其它三个相反，在 drv8833.c 的映射表里把
  *                 in1/in2 对调来做软件修正，PB9 才是这一路的"正转"脚。
  *
  *   故障反馈（低电平 = 故障，下降沿触发中断）
  *     PA0 = NFAULT1  (左侧)
  *     PA1 = NFAULT2  (右侧)
  *
  * DRV8833 单路真值表：
  *     IN1=0 IN2=0 -> 滑行(Coast)
  *     IN1=1 IN2=0 -> 正转
  *     IN1=0 IN2=1 -> 反转
  *     IN1=1 IN2=1 -> 刹车(Brake)
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __DRV8833_H
#define __DRV8833_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 版本 ---------------------------------------------------------------------*/

/* 电机数量与编号 -----------------------------------------------------------*/
#define DRV8833_MOTOR_NUM           (4U)

#define DRV8833_MOTOR_LF            (0U)   /* 左前 */
#define DRV8833_MOTOR_LR            (1U)   /* 左后 */
#define DRV8833_MOTOR_RF            (2U)   /* 右前 */
#define DRV8833_MOTOR_RR            (3U)   /* 右后 */

/* 单轮极性校准：+1 保持现有方向，-1 将该轮所有运动指令反向。
 * 只在该轮正负方向接反时修改；整车平移左右反了用 ps2_ctrl.h 的
 * PS2_CTRL_STRAFE_SIGN。右后通道表已有 PB9/PB8 交换，这里 +1 为保留。
 * 2026-10-18 Codex 添加，默认不改变原有轮子方向。 */
#ifndef DRV8833_DIR_LF
#define DRV8833_DIR_LF              (+1)  /* 左前：PA6/PA7 */
#endif
#ifndef DRV8833_DIR_LR
#define DRV8833_DIR_LR              (+1)  /* 左后：PB0/PB1 */
#endif
#ifndef DRV8833_DIR_RF
#define DRV8833_DIR_RF              (+1)  /* 右前：PB6/PB7 */
#endif
#ifndef DRV8833_DIR_RR
#define DRV8833_DIR_RR              (+1)  /* 右后：PB9/PB8 */
#endif
#if ((DRV8833_DIR_LF != 1) && (DRV8833_DIR_LF != -1)) || \
    ((DRV8833_DIR_LR != 1) && (DRV8833_DIR_LR != -1)) || \
    ((DRV8833_DIR_RF != 1) && (DRV8833_DIR_RF != -1)) || \
    ((DRV8833_DIR_RR != 1) && (DRV8833_DIR_RR != -1))
#error "DRV8833_DIR_LF/LR/RF/RR must be +1 or -1"
#endif

/* 左右分组（用于故障处理）---------------------------------------------------*/
#define DRV8833_SIDE_LEFT           (0U)
#define DRV8833_SIDE_RIGHT          (1U)

/* 速度范围：-1000 ~ +1000，0 = 刹车 ---------------------------------------*/
#define DRV8833_SPEED_MAX           (1000)

/* PWM 参数 -----------------------------------------------------------------
 * 定时器时钟 = 72 MHz（HSE 8 MHz x PLL9）。
 *   72 MHz / (PSC+1) / (ARR+1) = 72e6 / 1 / 3600 = 20 kHz
 * 20 kHz 超出人耳听觉范围，电机不会啸叫。
 * 与 CubeMX 配置保持一致（Prescaler = 0，Counter Period = 3599）。 */
#define DRV8833_PWM_PSC             (0U)
#define DRV8833_PWM_ARR             (3599U)

/* PWM 关断阶段：1 = 制动/慢衰减，0 = 原来的滑行/快衰减。
 * 慢衰减按 TI DRV8833 手册表 2 接法实现：正转 IN1 常高、IN2 反相 PWM；
 * 反转交换两脚。这里的 speed 仍表示驱动时间占比，满速和方向保持不变。
 * 不添加起转补偿；实际低速表现仍需在车上测试。 */
#ifndef DRV8833_PWM_BRAKE_MODE
#define DRV8833_PWM_BRAKE_MODE      (1U)
#endif
#if (DRV8833_PWM_BRAKE_MODE != 0U) && (DRV8833_PWM_BRAKE_MODE != 1U)
#error "Invalid DRV8833 PWM brake mode"
#endif
#if DRV8833_PWM_BRAKE_MODE
#define DRV8833_VERSION             "nu0.1-brake"
#else
#define DRV8833_VERSION             "nu0.1-coast"
#endif

/* 中断优先级 -------------------------------------------------------------*/
#define DRV8833_EXTI_PREEMPT_PRIO   (2U)
#define DRV8833_EXTI_SUB_PRIO       (0U)

/* 对外接口 -----------------------------------------------------------------*/

/* 初始化：启动 TIM3/TIM4 八路 PWM（20 kHz）+ PA0/PA1 下降沿中断。
 * 必须在 MX_TIM3_Init()/MX_TIM4_Init() 之后调用，上电调用一次。 */
void DRV8833_Init(void);

/* 设置某个电机的速度和方向。
 *   motor : DRV8833_MOTOR_LF / LR / RF / RR
 *   speed : -1000(全速反转) ~ +1000(全速正转)，0 = 刹车
 * 越界值会被自动限幅。
 * 满速（±1000）时驱动脚输出 100% 恒定高（CCR = ARR+1），另一脚恒定低。 */
void DRV8833_SetMotor(uint8_t motor, int16_t speed);

/* 读取应用 DIR 极性修正后的当前输出指令（-1000 ~ +1000，非实测转速） */
int16_t DRV8833_GetMotor(uint8_t motor);

/* 刹车：所有电机立即停住（IN1=IN2=1，低侧制动） */
void DRV8833_BrakeAll(void);

/* 滑行：所有电机断电自由转动（IN1=IN2=0） */
void DRV8833_CoastAll(void);

/* 只停某一侧（故障保护用）：把该侧 4 个通道全部拉低 */
void DRV8833_StopSide(uint8_t side);

/* 故障标志：1 = 该侧报过故障（低电平触发），需手动清除 */
uint8_t DRV8833_GetFault(uint8_t side);
void DRV8833_ClearFault(uint8_t side);

/* 故障中断计数：该侧 nFAULT 下降沿一共触发过多少次（只增不减）。
 * 用法：读一次记下来，过一会再读，差值 > 0 说明这期间真的进了 EXTI 中断。 */
uint16_t DRV8833_GetFaultCount(uint8_t side);

#ifdef __cplusplus
}
#endif

#endif /* __DRV8833_H */
