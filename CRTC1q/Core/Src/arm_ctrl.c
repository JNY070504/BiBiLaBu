#include "arm_ctrl.h"
#include "car_ctrl.h"

#if (ARM_BIG_CHANNEL >= CAR_SERVO_NUM) || (ARM_SMALL_CHANNEL >= CAR_SERVO_NUM) || \
    (ARM_CLAW_CHANNEL >= CAR_SERVO_NUM) || (ARM_GIMBAL_CHANNEL >= CAR_SERVO_NUM) || \
    (ARM_BIG_CHANNEL == ARM_SMALL_CHANNEL) || (ARM_BIG_CHANNEL == ARM_CLAW_CHANNEL) || \
    (ARM_SMALL_CHANNEL == ARM_CLAW_CHANNEL) || (ARM_GIMBAL_CHANNEL == ARM_BIG_CHANNEL) || \
    (ARM_GIMBAL_CHANNEL == ARM_SMALL_CHANNEL) || (ARM_GIMBAL_CHANNEL == ARM_CLAW_CHANNEL)
#error "Arm channels must be distinct board servo outputs"
#endif
#if (CAR_SERVO_ANGLE_MAX != 180U)
#error "Update arm angle conversion for the changed servo scale"
#endif

typedef struct
{
    uint8_t channel;
    uint16_t min_x10;
    uint16_t max_x10;
    uint16_t home_x10;
    uint16_t speed_x10_s;
} arm_joint_config_t;

static const arm_joint_config_t config[ARM_CTRL_JOINT_NUM] =
{
    {ARM_BIG_CHANNEL, ARM_BIG_MIN_DEG * 10U, ARM_BIG_MAX_DEG * 10U,
     ARM_BIG_HOME_DEG * 10U, ARM_BIG_SPEED_DEG_S * 10U},
    {ARM_SMALL_CHANNEL, ARM_SMALL_MIN_DEG * 10U, ARM_SMALL_MAX_DEG * 10U,
     ARM_SMALL_HOME_DEG * 10U, ARM_SMALL_SPEED_DEG_S * 10U},
    {ARM_CLAW_CHANNEL, ARM_CLAW_MIN_DEG * 10U, ARM_CLAW_MAX_DEG * 10U,
     ARM_CLAW_CLOSE_DEG * 10U, ARM_CLAW_SPEED_DEG_S * 10U},
    {ARM_GIMBAL_CHANNEL, ARM_GIMBAL_MIN_DEG * 10U, ARM_GIMBAL_MAX_DEG * 10U,
     ARM_GIMBAL_HOME_DEG * 10U, ARM_GIMBAL_SPEED_DEG_S * 10U}
};

static arm_ctrl_status_t status;
static uint32_t remainder[ARM_CTRL_JOINT_NUM];
static int8_t previous_direction[ARM_CTRL_JOINT_NUM];
static uint32_t last_tick;
static uint32_t failed_tick;
static uint8_t retry_wait;
static uint8_t wait_release;
static uint8_t cross_previous;
static uint16_t claw_target_x10;

#define ARM_BUTTON_MASK (PS2_BTN_L1 | PS2_BTN_R1 | PS2_BTN_L2 | PS2_BTN_R2 | \
                         PS2_BTN_CROSS | PS2_BTN_SQUARE | PS2_BTN_CIRCLE)

/* 与 Car_Servo() 使用同一标称映射，保留 0.1 度指令精度。
 * PCA9685 的实际脉宽分辨率约 4.88 us，不能等同于真实 0.1 度精度。
 */
static HAL_StatusTypeDef write_angle(uint8_t joint, uint16_t angle_x10)
{
    uint16_t pulse_us;
    pulse_us = (uint16_t)(CAR_SERVO_MIN_US +
        (((uint32_t)angle_x10 * (CAR_SERVO_MAX_US - CAR_SERVO_MIN_US) + 900U) / 1800U));
    return Car_ServoPulse(config[joint].channel, pulse_us);
}

static void freeze_inputs(void)
{
    uint8_t i;
    wait_release = 1U;
    cross_previous = 0U;
    claw_target_x10 = status.angle_x10[2];
    for (i = 0U; i < ARM_CTRL_JOINT_NUM; i++)
    {
        remainder[i] = 0U;
        previous_direction[i] = 0;
    }
}

static void record_failure(HAL_StatusTypeDef result)
{
    status.last_status = result;
    failed_tick = HAL_GetTick();
    retry_wait = 1U;
    last_tick = failed_tick;
    freeze_inputs();
}

/* 只初始化这四路，其他通道保持 main() 初始化时的关闭状态。
 * 一次传输失败后暂停重试，避免三路接连超时拖慢小车主循环。
 */
static uint8_t initialize_outputs(void)
{
    uint8_t i;
    HAL_StatusTypeDef result;
    for (i = 0U; i < ARM_CTRL_JOINT_NUM; i++)
    {
        if ((status.ready_mask & (1U << i)) == 0U)
        {
            result = write_angle(i, status.angle_x10[i]);
            if (result != HAL_OK)
            {
                record_failure(result);
                return 0U;
            }
            status.ready_mask |= (uint8_t)(1U << i);
            status.last_status = HAL_OK;
        }
    }
    return 1U;
}

void Arm_Ctrl_Init(void)
{
    uint8_t i;
    status.ready_mask = 0U;
    status.claw_open = 0U;
    status.last_status = HAL_OK;
    retry_wait = 0U;
    for (i = 0U; i < ARM_CTRL_JOINT_NUM; i++)
        status.angle_x10[i] = config[i].home_x10;
    freeze_inputs();
    (void)initialize_outputs();
    last_tick = HAL_GetTick();
}

void Arm_Ctrl_Update(const PS2_t *js, uint8_t connected)
{
    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - last_tick;
    uint32_t budget;
    uint16_t step;
    uint8_t buttons = 0U;
    uint8_t cross_now;
    uint8_t i;
    int8_t direction[ARM_CTRL_JOINT_NUM] = {0};
    int32_t next;
    HAL_StatusTypeDef result;

    last_tick = now;
    if (elapsed > ARM_UPDATE_MAX_DT_MS) elapsed = ARM_UPDATE_MAX_DT_MS;

    if ((connected == 0U) || (js == 0))
    {
        /* 不关闭 PWM，避免机械臂因断连而失去保持力；不继续未完成的开合。 */
        freeze_inputs();
        return;
    }

    buttons = (uint8_t)(js->btn2 & ARM_BUTTON_MASK);
    if (wait_release != 0U)
    {
        if (buttons == 0U) wait_release = 0U;
        cross_previous = (buttons & PS2_BTN_CROSS) ? 1U : 0U;
        /* 初始化或恢复当帧不执行移动。 */
        elapsed = 0U;
        buttons = 0U;
    }
    else
    {
        direction[0] = (int8_t)(((buttons & PS2_BTN_R1) ? 1 : 0)
                             - ((buttons & PS2_BTN_L1) ? 1 : 0));
        direction[1] = (int8_t)(((buttons & PS2_BTN_R2) ? 1 : 0)
                             - ((buttons & PS2_BTN_L2) ? 1 : 0));
        if (ARM_BIG_UP_REVERSED != 0U) direction[0] = (int8_t)-direction[0];
        if (ARM_SMALL_UP_REVERSED != 0U) direction[1] = (int8_t)-direction[1];

        direction[3] = (int8_t)(((buttons & PS2_BTN_CIRCLE) ? 1 : 0)
                             - ((buttons & PS2_BTN_SQUARE) ? 1 : 0));
        if (ARM_GIMBAL_RIGHT_REVERSED != 0U) direction[3] = (int8_t)-direction[3];

        cross_now = (buttons & PS2_BTN_CROSS) ? 1U : 0U;
        if ((cross_now != 0U) && (cross_previous == 0U))
        {
            status.claw_open ^= 1U;
            claw_target_x10 = (uint16_t)((status.claw_open != 0U)
                ? ARM_CLAW_OPEN_DEG * 10U : ARM_CLAW_CLOSE_DEG * 10U);
        }
        cross_previous = cross_now;
    }

    if (retry_wait != 0U)
    {
        if ((now - failed_tick) < ARM_I2C_RETRY_MS)
        {
            freeze_inputs();
            return;
        }
        retry_wait = 0U;
        elapsed = 0U;
    }
    if (initialize_outputs() == 0U) return;

    if (claw_target_x10 > status.angle_x10[2]) direction[2] = 1;
    else if (claw_target_x10 < status.angle_x10[2]) direction[2] = -1;

    for (i = 0U; i < ARM_CTRL_JOINT_NUM; i++)
    {
        if (direction[i] != previous_direction[i]) remainder[i] = 0U;
        previous_direction[i] = direction[i];
        if (direction[i] == 0)
        {
            remainder[i] = 0U;
            continue;
        }
        budget = elapsed * config[i].speed_x10_s + remainder[i];
        step = (uint16_t)(budget / 1000U);
        remainder[i] = budget % 1000U;
        next = (int32_t)status.angle_x10[i] + (int32_t)direction[i] * step;
        if (next < config[i].min_x10) next = config[i].min_x10;
        if (next > config[i].max_x10) next = config[i].max_x10;
        if ((i == 2U) && (((direction[i] > 0) && (next > claw_target_x10)) ||
                          ((direction[i] < 0) && (next < claw_target_x10))))
            next = claw_target_x10;
        if ((uint16_t)next == status.angle_x10[i]) continue;

        result = write_angle(i, (uint16_t)next);
        status.last_status = result;
        if (result != HAL_OK)
        {
            record_failure(result);
            return;
        }
        status.angle_x10[i] = (uint16_t)next;
    }
}

const arm_ctrl_status_t *Arm_Ctrl_GetStatus(void)
{
    return &status;
}
