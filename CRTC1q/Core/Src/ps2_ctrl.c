/**
  ******************************************************************************
  * @file    ps2_ctrl.c
  * @brief   PS2 摇杆与机械臂并行控制
  ******************************************************************************
  */

#include "ps2_ctrl.h"
#include "car_ctrl.h"
#include "arm_ctrl.h"

/* ======================= 状态机内部状态 ======================= */
static PS2_t        g_js;
static int16_t      g_motor_pwm[4];
static uint32_t     g_ramp_tick;
static uint32_t     g_ramp_remainder;

/* 回中和断连时清空斜坡，禁止恢复时沿用旧速度。 */
static void reset_motor_ramp(void)
{
    uint8_t i;
    for (i = 0U; i < 4U; i++)
    {
        g_motor_pwm[i] = 0;
    }
    g_ramp_tick = HAL_GetTick();
    g_ramp_remainder = 0U;
}

/* 单轴分段曲线：把大部分摇杆行程分给低速，不叠加固定起转 PWM。
 * 默认前 70% 输入线性覆盖 0~20% 输出，起步附近斜率约 0.286；
 * 后 30% 用二次曲线补足到 100%，连接处的输出和斜率连续（整数取整除外）。
 * 相比整体加大平方/立方项，这样不会同时把起步附近输出压得很小。 */
static int16_t shape_axis(int16_t value)
{
    int32_t mag = (value < 0) ? -(int32_t)value : (int32_t)value;
    const int32_t input_knee = (int32_t)PS2_CTRL_LOW_INPUT_RANGE;
    const int32_t output_knee = (int32_t)PS2_CTRL_LOW_OUTPUT_RANGE;
    int32_t shaped;
    int32_t tail, linear_weight, tail_shaped;

    if (mag > 1000) mag = 1000;
    if (mag <= input_knee)
    {
        shaped = (mag * output_knee + input_knee / 2) / input_knee;
    }
    else
    {
        tail = ((mag - input_knee) * 1000) / (1000 - input_knee);
        /* 匹配前段斜率：output_knee/input_knee，结果为 0~1000。 */
        linear_weight = (output_knee * (1000 - input_knee) * 1000)
                      / (input_knee * (1000 - output_knee));
        tail_shaped = (linear_weight * tail
                      + ((1000 - linear_weight) * tail * tail) / 1000
                      + 500) / 1000;
        shaped = output_knee + ((1000 - output_knee) * tail_shaped + 500) / 1000;
    }
    return (int16_t)((value < 0) ? -shaped : shaped);
}

/* 从零按比例输出，不叠加固定 PWM；满量程仍达到设定的最大输出。 */
static int16_t wheel_to_pwm(int32_t value)
{
    return (int16_t)((value * (int32_t)PS2_CTRL_PWM_MAX) / 1000);
}

static void apply_wheel(car_wheel_t wheel, int16_t target, int32_t step)
{
    int32_t current = g_motor_pwm[(uint8_t)wheel];
    int32_t delta = (int32_t)target - current;

    /* 回零立即刹车；换向先经过一个零输出周期，再平滑反向起步。 */
    if ((target == 0) || ((current > 0) && (target < 0))
                      || ((current < 0) && (target > 0)))
    {
        current = 0;
    }
    else if (delta > step)
    {
        current += step;
    }
    else if (delta < -step)
    {
        current -= step;
    }
    else
    {
        current = target;
    }
    g_motor_pwm[(uint8_t)wheel] = (int16_t)current;
    Car_MotorPermille(wheel, (int16_t)current);
}

/* ======================= 手柄控制电机（麦轮混控） =======================
 * 摇杆映射：
 *   左摇杆 Y（LJoy_UD） → 前后：推上 = 前进（invert=1）
 *   左摇杆 X（LJoy_LR） → 左右平移：推右 = 右移
 *   右摇杆 X（RJoy_LR） → 原地转向：推右 = 右转，与平移指令叠加
 *
 * 平移分配移植自 CRTC2z 的实际下标与 apply_wheel 调用，而非其旧注释：
 *   左前 LF = 前进 + 右移 + 右转
 *   右前 RF = 前进 + 右移 - 右转
 *   左后 LR = 前进 - 右移 + 右转
 *   右后 RR = 前进 - 右移 - 右转
 *
 * 正负号表示传给驱动的指令，不直接等同于实际车轮的前后滚动方向。
 * CRTC2z：wheel[0]/[1]/[2]/[3] 最终传给 LF/RF/LR/RR，
 * 但其公式把 [3] 注释为 RF、[1] 注释为 RR；这里使用轮子枚举消除歧义。
 * +右移时 LF/RF 为正、LR/RR 为负；反向平移整体取反。
 * 前后/转向公式与 CRTC1q 原版相同，低速曲线和轮速斜坡保持原样。
 * ============================================================================ */
static void state_motor(const PS2_t *js)
{
    int16_t forward, strafe, turn;
    int32_t wheel[4];
    int32_t peak = 1000;
    int32_t mag;
    int32_t step;
    uint32_t now, elapsed, budget;
    uint8_t i;

    /* 摇杆原始值 -> -1000~+1000，带死区 */
    forward = shape_axis(PS2_AxisMap(js->LJoy_UD, 1U, PS2_DEADZONE));
    strafe  = shape_axis(PS2_AxisMap(js->LJoy_LR, 0U, PS2_DEADZONE));
    strafe  = (int16_t)(strafe * PS2_CTRL_STRAFE_SIGN);
    turn    = (int16_t)(((int32_t)shape_axis(PS2_AxisMap(js->RJoy_LR, 0U, PS2_DEADZONE))
                      * (int32_t)PS2_CTRL_TURN_SCALE) / 1000);

    if ((forward == 0) && (strafe == 0) && (turn == 0))
    {
        reset_motor_ramp();
        Car_MotorStopAll();
        return;
    }

    /* 2026-10-18 Codex：复现 CRTC2z 已实车可平移的输出分配。
     * 仅左前/左后的 strafe 符号与 CRTC1q 原版不同。
     * 不宣称这是所有麦轮安装方式通用的方向表。 */
    wheel[CAR_WHEEL_LF] = (int32_t)forward + strafe + turn;
    wheel[CAR_WHEEL_RF] = (int32_t)forward + strafe - turn;
    wheel[CAR_WHEEL_LR] = (int32_t)forward - strafe + turn;
    wheel[CAR_WHEEL_RR] = (int32_t)forward - strafe - turn;

    /* 组合指令超量程时四轮等比例缩放，避免逐轮截断造成突变。 */
    for (i = 0U; i < 4U; i++)
    {
        mag = (wheel[i] < 0) ? -wheel[i] : wheel[i];
        if (mag > peak) peak = mag;
    }
    now = HAL_GetTick();
    elapsed = now - g_ramp_tick;
    g_ramp_tick = now;
    if (elapsed > PS2_CTRL_RAMP_MAX_DT_MS) elapsed = PS2_CTRL_RAMP_MAX_DT_MS;
    budget = elapsed * PS2_CTRL_PWM_RISE_PER_SEC + g_ramp_remainder;
    step = (int32_t)(budget / 1000U);
    g_ramp_remainder = budget % 1000U;
    for (i = 0U; i < 4U; i++)
    {
        apply_wheel((car_wheel_t)i, wheel_to_pwm((wheel[i] * 1000) / peak), step);
    }
}

/* ======================= 对外接口 ======================= */

void PS2_Ctrl_Init(void)
{
    PS2_Init();
    PS2_ConfigAnalog();

    reset_motor_ramp();
    Arm_Ctrl_Init();
}

void PS2_Ctrl_Update(void)
{
    uint8_t ok;

    /* 1. 读手柄 */
    ok = PS2_Read(&g_js);

    /* 肩键/叉号与摇杆始终同时生效，SELECT 不再切换模式。 */
    Arm_Ctrl_Update(&g_js, ok);

    if (!ok)
    {
        /* 读不到手柄 → 安全停车 */
        reset_motor_ramp();
        Car_MotorStopAll();
        return;
    }

    state_motor(&g_js);
}

const PS2_t *PS2_Ctrl_GetInput(void)
{
    return &g_js;
}
