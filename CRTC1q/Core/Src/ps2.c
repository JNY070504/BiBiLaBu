/**
  ******************************************************************************
  * @file    ps2.c
  * @brief   PS2 无线手柄驱动（GPIO 模拟 SPI）实现
  ******************************************************************************
  */

#include "ps2.h"

/* ======================= 引脚操作小工具 ======================= */
#define CLK_H()   HAL_GPIO_WritePin(PS2_CLK_PORT, PS2_CLK_PIN, GPIO_PIN_SET)
#define CLK_L()   HAL_GPIO_WritePin(PS2_CLK_PORT, PS2_CLK_PIN, GPIO_PIN_RESET)
#define CMD_H()   HAL_GPIO_WritePin(PS2_CMD_PORT, PS2_CMD_PIN, GPIO_PIN_SET)
#define CMD_L()   HAL_GPIO_WritePin(PS2_CMD_PORT, PS2_CMD_PIN, GPIO_PIN_RESET)
#define CS_H()    HAL_GPIO_WritePin(PS2_CS_PORT, PS2_CS_PIN, GPIO_PIN_SET)
#define CS_L()    HAL_GPIO_WritePin(PS2_CS_PORT, PS2_CS_PIN, GPIO_PIN_RESET)
#define DAT_READ() (HAL_GPIO_ReadPin(PS2_DAT_PORT, PS2_DAT_PIN) == GPIO_PIN_SET)

/* ======================= 微秒级延时（DWT 内核计数器） =======================
 * 用 CPU 的 DWT->CYCCNT 计数，精度 1/72MHz ≈ 14ns，比空循环准得多。
 * 只需要调用一次 PS2_Init()，里面会自动打开。
 * ========================================================================= */
static void ps2_dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  /* 打开跟踪/计数单元 */
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;            /* 启动周期计数器 */
}

static void ps2_delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = (uint32_t)((SystemCoreClock / 1000000U) * us);

    while ((uint32_t)(DWT->CYCCNT - start) < ticks)
    {
        /* 空转等待 */
    }
}

/* ======================= 初始化 ======================= */
void PS2_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* CLK / CMD / CS：推挽输出 */
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;      /* 高速档，跳变更干脆 */

    gpio.Pin = PS2_CLK_PIN;
    HAL_GPIO_Init(PS2_CLK_PORT, &gpio);
    gpio.Pin = PS2_CMD_PIN;
    HAL_GPIO_Init(PS2_CMD_PORT, &gpio);
    gpio.Pin = PS2_CS_PIN;
    HAL_GPIO_Init(PS2_CS_PORT, &gpio);

    /* DAT：输入 + 内部上拉（手柄的数据脚是开漏输出，必须有上拉） */
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    gpio.Pin  = PS2_DAT_PIN;
    HAL_GPIO_Init(PS2_DAT_PORT, &gpio);

    /* 空闲状态：CS=1（不选中），CLK=1，CMD=1 */
    CS_H();
    CLK_H();
    CMD_H();

    ps2_dwt_init();

    HAL_Delay(10);        /* 等手柄接收器电源稳住 */
}

/* ======================= 收发 1 个字节 =======================
 * PS2 的时序（8 位，最低位先发）：
 *   1) 时钟空闲为高；
 *   2) 主机先把命令位摆到 CMD 上；
 *   3) 时钟拉低 —— 手柄在这一刻把数据位摆到 DAT 上；
 *   4) 等半周期后主机读 DAT；
 *   5) 时钟拉高，再等半周期，进入下一位。
 * ============================================================ */
static uint8_t ps2_xfer_byte(uint8_t cmd)
{
    uint8_t i;
    uint8_t res = 0;

    for (i = 0; i < 8; i++)
    {
        /* 摆命令位 */
        if (cmd & (1u << i))
        {
            CMD_H();
        }
        else
        {
            CMD_L();
        }

        /* 时钟拉低，给手柄时间准备数据 */
        CLK_L();
        ps2_delay_us(PS2_HALF_CLK_US);

        /* 读数据位 */
        if (DAT_READ())
        {
            res |= (uint8_t)(1u << i);
        }

        /* 时钟拉高 */
        CLK_H();
        ps2_delay_us(PS2_HALF_CLK_US);
    }

    return res;
}

/* ======================= 收发一整帧（默认 9 字节） ======================= */
static void ps2_xfer_frame(const uint8_t *tx, uint8_t *rx, uint8_t n)
{
    uint8_t i;

    CS_L();                     /* 片选拉低，开始一次通信 */
    ps2_delay_us(2);

    for (i = 0; i < n; i++)
    {
        rx[i] = ps2_xfer_byte(tx[i]);
        ps2_delay_us(16);       /* 字节之间留点空隙 */
    }

    CS_H();                     /* 结束 */
    ps2_delay_us(8);
    CLK_H();
    CMD_H();
}

/* ======================= 读手柄 ======================= */
uint8_t PS2_Read(PS2_t *js)
{
    uint8_t rx[21] = {0};
    uint8_t length = 9;
    uint8_t i;

    CS_L();
    ps2_delay_us(16);

    for (i = 0; i < length; i++)
    {
        uint8_t cmd;
        if (i == 0U)      cmd = 0x01U;
        else if (i == 1U) cmd = 0x42U;
        else              cmd = 0x00U;

        rx[i] = ps2_xfer_byte(cmd);
        ps2_delay_us(20);

        /* 如果第 2 字节返回 0x79，说明是压力模式，需要收 21 字节 */
        if (i == 1U && rx[1] == 0x79U) length = 21U;
    }

    CS_H();
    CLK_H();
    CMD_H();
    ps2_delay_us(16);

    /* 解码 */
    PS2_Frame frame;
    PS2_Decode(rx, length, &frame);

    *js = (PS2_t){0};
    for (i = 0; i < length; i++) js->raw[i] = rx[i];
    js->length  = length;
    js->mode    = frame.mode;
    js->ok      = frame.valid;
    js->btn1    = (uint8_t)frame.buttons;
    js->btn2    = (uint8_t)(frame.buttons >> 8);
    js->RJoy_LR = frame.rx;
    js->RJoy_UD = frame.ry;
    js->LJoy_LR = frame.lx;
    js->LJoy_UD = frame.ly;

    return js->ok;
}

/* ======================= 让手柄进入模拟模式 =======================
 * 手柄有「数字模式(红灯)」和「模拟模式(红灯+绿灯一起亮)」。
 * 只有模拟模式下摇杆才会给出 0~255 的连续值。
 * 这里照抄索尼原厂/PS2X 库的配置命令序列。
 * 如果这条序列对某些接收器无效，也可以手动按手柄上的 MODE/ANALOG 键。
 * 返回值 = 配置完成后的手柄模式 ID（0x73/0x79 表示已经在模拟模式）。
 * ================================================================= */
uint8_t PS2_ConfigAnalog(void)
{
    static const uint8_t enter_config[9] = {0x01, 0x43, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
    static const uint8_t set_analog[9]   = {0x01, 0x44, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
    static const uint8_t exit_config[9]  = {0x01, 0x43, 0x00, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A};
    uint8_t rx[9];
    PS2_t js;

    HAL_Delay(20);

    ps2_xfer_frame(enter_config, rx, 9);   HAL_Delay(5);
    ps2_xfer_frame(set_analog,   rx, 9);   HAL_Delay(5);
    ps2_xfer_frame(exit_config,  rx, 9);   HAL_Delay(5);

    PS2_Read(&js);                          /* 再读一次，看模式对不对 */

    if (js.ok) return js.mode;
    if (js.raw[0] == 0xFF && js.raw[2] == 0x5A && js.mode == 0x41) return 0x41;
    return 0;
}

/* ======================= 摇杆原始值 -> -1000~+1000 =======================
 * raw: 0~255 中间约 128
 * invert: 1 = 数值方向反过来（例如"上"是 0x00，而我们要它输出 +1000）
 * deadzone: 死区，回中时的小抖动直接当 0
 * ====================================================================== */
int16_t PS2_AxisMap(uint8_t raw, uint8_t invert, uint8_t deadzone)
{
    int32_t delta  = (int32_t)raw - 128;
    int32_t span   = (delta < 0) ? 128 : 127;   /* 负方向最大 128，正方向最大 127 */
    int32_t mag    = (delta < 0) ? -delta : delta;
    int32_t mapped;

    if (mag <= deadzone || span <= deadzone) return 0;

    mapped = ((mag - deadzone) * 1000) / (span - deadzone);
    if (mapped > 1000) mapped = 1000;
    if (delta < 0) mapped = -mapped;

    return (int16_t)(invert ? -mapped : mapped);
}
