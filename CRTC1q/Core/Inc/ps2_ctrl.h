/**
  ******************************************************************************
  * @file    ps2_ctrl.h
  * @brief   PS2 手柄并行控制 —— 读手柄 → 调抽象命令(Car_*)
  *
  * 分层位置：
  *   上层逻辑（本文件）
  *      ↓ 调用 Car_Motor / Car_Servo 等抽象命令
  *   car_ctrl（抽象层）
  *      ↓
  *   drv8833 / pca9685（底层驱动）
  *
  * 摇杆开车，肩键/叉号同时控制机械臂，无需切换模式。
  * SELECT 不参与控制。
  ******************************************************************************
  */

#ifndef __PS2_CTRL_H
#define __PS2_CTRL_H

#include "main.h"
#include "ps2.h"

/* 手柄微调参数，PWM 单位为千分比；不添加固定起转补偿。 */
/* 平移符号：+1 复现 CRTC2z 的实际输出；整车左右平移反了才改成 -1。
 * 不改变前后、转向或单轮电机极性。2026-10-18 Codex 移植。 */
#define PS2_CTRL_STRAFE_SIGN         (+1)
#define PS2_CTRL_PWM_MAX             1000U /* 保留原来的满杆 100% PWM */
#define PS2_CTRL_LOW_INPUT_RANGE     700U  /* 单轴死区外前 70% 输入用于低速细调 */
#define PS2_CTRL_LOW_OUTPUT_RANGE    200U  /* 上述低速段对应 0~20% 输出，后段平顺升至满输出 */
#define PS2_CTRL_TURN_SCALE          1000U /* 保留原来的满杆转向输出 */
#define PS2_CTRL_PWM_RISE_PER_SEC    2500U /* 每秒最多变化 250% PWM，零到满约 0.4 秒 */
#define PS2_CTRL_RAMP_MAX_DT_MS      50U   /* 长时间停顿后禁止一步跳到高速 */

#if (PS2_CTRL_PWM_MAX == 0U) || (PS2_CTRL_PWM_MAX > 1000U)
#error "Invalid PS2 PWM limits"
#endif
#if (PS2_CTRL_TURN_SCALE > 1000U)
#error "Invalid PS2 input scaling"
#endif
#if (PS2_CTRL_STRAFE_SIGN != 1) && (PS2_CTRL_STRAFE_SIGN != -1)
#error "PS2_CTRL_STRAFE_SIGN must be +1 or -1"
#endif
#if (PS2_CTRL_LOW_INPUT_RANGE == 0U) || (PS2_CTRL_LOW_INPUT_RANGE >= 1000U) \
 || (PS2_CTRL_LOW_OUTPUT_RANGE == 0U) || (PS2_CTRL_LOW_OUTPUT_RANGE > PS2_CTRL_LOW_INPUT_RANGE)
#error "Invalid PS2 low-speed input/output ranges"
#endif

/**
  * @brief  初始化 PS2 控制
  *         内部会调用 PS2_Init() 和 PS2_ConfigAnalog()，
  *         清空电机斜坡，初始化机械臂与云台共四路舵机。
  */
void PS2_Ctrl_Init(void);

/**
  * @brief  控制更新函数，主循环里每次调用一次
  *         内部流程：
  *           1. PS2_Read() 读一次手柄
  *           2. 更新机械臂；断连时底盘停车
  *           3. 连接正常时更新底盘，不使用 SELECT 切换模式
  */
void PS2_Ctrl_Update(void);

/* 最近一帧手柄原始值，仅用于主循环串口诊断，不修改返回对象。 */
const PS2_t *PS2_Ctrl_GetInput(void);

#endif /* __PS2_CTRL_H */
