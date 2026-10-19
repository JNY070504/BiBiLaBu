/* Four positional servos on PCA9685. PB10/PB11 remain I2C2 SCL/SDA. */
#ifndef ARM_CTRL_H
#define ARM_CTRL_H

#include "main.h"
#include "ps2.h"

/* 可调参数：通道是板上的 0~7 编号，不是 STM32 引脚编号。
 * 角度均为软件标称角度，装好连杆后再标定真实位置与机械限位。
 * 当前 45~135 度对应默认映射中的 1000~2000 us，只使用中间行程。
 * UP_REVERSED=0：右肩键使角度增加；=1：右肩键使角度减小。
 */
#define ARM_BIG_CHANNEL           0U
#define ARM_BIG_MIN_DEG           0U
#define ARM_BIG_MAX_DEG           180U
#define ARM_BIG_HOME_DEG          90U
#define ARM_BIG_SPEED_DEG_S       80U
#define ARM_BIG_UP_REVERSED       0U

#define ARM_SMALL_CHANNEL         1U
#define ARM_SMALL_MIN_DEG         0U
#define ARM_SMALL_MAX_DEG         180U
#define ARM_SMALL_HOME_DEG        90U
#define ARM_SMALL_SPEED_DEG_S     80U
#define ARM_SMALL_UP_REVERSED     0U

/* “关”对应的实际角度须在安装夹子后标定；上电默认输出 CLOSE_DEG。 */
#define ARM_CLAW_CHANNEL          2U
#define ARM_CLAW_MIN_DEG          0U
#define ARM_CLAW_MAX_DEG          135U
#define ARM_CLAW_CLOSE_DEG        50U
#define ARM_CLAW_OPEN_DEG         90U
#define ARM_CLAW_SPEED_DEG_S      60U

/* 云台：方块左转，圆形右转，按住移动、松开保持。
 * RIGHT_REVERSED=0：圆形键使角度增加；=1：圆形键使角度减小。
 * 0~180 为软件标称行程，MG995 实物端点需在空载时标定。
 */
#define ARM_GIMBAL_CHANNEL        3U
#define ARM_GIMBAL_MIN_DEG        0U
#define ARM_GIMBAL_MAX_DEG        180U
#define ARM_GIMBAL_HOME_DEG       90U
#define ARM_GIMBAL_SPEED_DEG_S    50U
#define ARM_GIMBAL_RIGHT_REVERSED 1U

#define ARM_CTRL_JOINT_NUM        4U

#define ARM_UPDATE_MAX_DT_MS      50U
#define ARM_I2C_RETRY_MS          250U

#if (ARM_BIG_MIN_DEG >= ARM_BIG_MAX_DEG) || (ARM_BIG_MAX_DEG > 180U) || \
    (ARM_BIG_HOME_DEG < ARM_BIG_MIN_DEG) || (ARM_BIG_HOME_DEG > ARM_BIG_MAX_DEG)
#error "Invalid big arm angle limits"
#endif
#if (ARM_SMALL_MIN_DEG >= ARM_SMALL_MAX_DEG) || (ARM_SMALL_MAX_DEG > 180U) || \
    (ARM_SMALL_HOME_DEG < ARM_SMALL_MIN_DEG) || (ARM_SMALL_HOME_DEG > ARM_SMALL_MAX_DEG)
#error "Invalid small arm angle limits"
#endif
#if (ARM_CLAW_MIN_DEG >= ARM_CLAW_MAX_DEG) || (ARM_CLAW_MAX_DEG > 180U) || \
    (ARM_CLAW_CLOSE_DEG < ARM_CLAW_MIN_DEG) || (ARM_CLAW_CLOSE_DEG > ARM_CLAW_MAX_DEG) || \
    (ARM_CLAW_OPEN_DEG < ARM_CLAW_MIN_DEG) || (ARM_CLAW_OPEN_DEG > ARM_CLAW_MAX_DEG) || \
    (ARM_CLAW_CLOSE_DEG == ARM_CLAW_OPEN_DEG)
#error "Invalid claw angle limits"
#endif
#if (ARM_GIMBAL_MIN_DEG >= ARM_GIMBAL_MAX_DEG) || (ARM_GIMBAL_MAX_DEG > 180U) || \
    (ARM_GIMBAL_HOME_DEG < ARM_GIMBAL_MIN_DEG) || (ARM_GIMBAL_HOME_DEG > ARM_GIMBAL_MAX_DEG)
#error "Invalid gimbal angle limits"
#endif
#if (ARM_BIG_UP_REVERSED > 1U) || (ARM_SMALL_UP_REVERSED > 1U) || \
    (ARM_BIG_SPEED_DEG_S == 0U) || (ARM_BIG_SPEED_DEG_S > 180U) || \
    (ARM_SMALL_SPEED_DEG_S == 0U) || (ARM_SMALL_SPEED_DEG_S > 180U) || \
    (ARM_CLAW_SPEED_DEG_S == 0U) || (ARM_CLAW_SPEED_DEG_S > 180U) || \
    (ARM_GIMBAL_RIGHT_REVERSED > 1U) || \
    (ARM_GIMBAL_SPEED_DEG_S == 0U) || (ARM_GIMBAL_SPEED_DEG_S > 180U)
#error "Invalid arm direction or speed"
#endif

/* 顺序：大臂、小臂、夹子、云台。angle_x10 是已成功发送的指令角度乘 10，
 * 并非角度反馈；ready_mask 每一位表示对应关节已成功发过初始指令。
 * claw_open 为开合目标；中途断连/通信错误时会停在当时的指令位置。
 */
typedef struct
{
    uint16_t angle_x10[ARM_CTRL_JOINT_NUM];
    uint8_t ready_mask;
    uint8_t claw_open;
    HAL_StatusTypeDef last_status;
} arm_ctrl_status_t;

void Arm_Ctrl_Init(void);
/* R1/L1 大臂上/下，R2/L2 小臂上/下，CROSS 按下沿切换夹子。
 * SQUARE/CIRCLE 云台左/右，与摇杆开车同时可用。
 * 同组两个方向键同时按下则保持；断连保持输出并等待控制按键松开。
 */
void Arm_Ctrl_Update(const PS2_t *js, uint8_t connected);
const arm_ctrl_status_t *Arm_Ctrl_GetStatus(void);

#endif
