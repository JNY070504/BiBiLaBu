/**
  ******************************************************************************
  * @file    ps2.h
  * @brief   PS2 无线手柄驱动（用 GPIO 模拟 SPI 时序）
  *
  * 为什么用 GPIO 模拟 SPI 而不用硬件 SPI2：
  *   PS2 手柄的通信虽然长得像 SPI，但它对时序、应答(ACK)比较敏感，
  *   用硬件 SPI 容易读不到数据（特别是国产接收器）。
  *   这里用 4 个普通 GPIO「软件模拟」时钟，频率约 31kHz，稳定可靠，
  *   也方便以后换引脚。
  *
  * CRTC1 板上的引脚分配（与 CubeMX 的 SPI2 引脚一致，但本驱动不用 SPI2 外设）：
  *   CLK -> PB13    （主机 -> 手柄，时钟）
  *   DAT -> PB14    （手柄 -> 主机，数据，输入上拉）
  *   CMD -> PB15    （主机 -> 手柄，命令）
  *   CS  -> PA8     （片选 ATT，低电平有效）
  ******************************************************************************
  */

#ifndef __PS2_H
#define __PS2_H

#include "main.h"
#include "ps2_frame.h"

/* ==========================================================================
 *  引脚定义
 *  优先跟随 CubeMX 标签（如果有的话）；否则用 CRTC1 板的默认走线。
 * ========================================================================== */
#if defined(PS2_CLK_Pin) && defined(PS2_DAT_Pin) && defined(PS2_CMD_Pin) && defined(PS2_CS_Pin)

#define PS2_CLK_PORT      PS2_CLK_GPIO_Port
#define PS2_CLK_PIN       PS2_CLK_Pin
#define PS2_DAT_PORT      PS2_DAT_GPIO_Port
#define PS2_DAT_PIN       PS2_DAT_Pin
#define PS2_CMD_PORT      PS2_CMD_GPIO_Port
#define PS2_CMD_PIN       PS2_CMD_Pin
#define PS2_CS_PORT       PS2_CS_GPIO_Port
#define PS2_CS_PIN        PS2_CS_Pin

#elif defined(SPI2_CS_Pin)

/* CubeMX 里只给 CS 起了标签 SPI2_CS；SCK/MISO/MOSI 走默认引脚 */
#define PS2_CLK_PORT      GPIOB
#define PS2_CLK_PIN       GPIO_PIN_13     /* SPI2_SCK */
#define PS2_DAT_PORT      GPIOB
#define PS2_DAT_PIN       GPIO_PIN_14     /* SPI2_MISO */
#define PS2_CMD_PORT      GPIOB
#define PS2_CMD_PIN       GPIO_PIN_15     /* SPI2_MOSI */
#define PS2_CS_PORT       SPI2_CS_GPIO_Port
#define PS2_CS_PIN        SPI2_CS_Pin     /* PA8 */

#else

#define PS2_CLK_PORT      GPIOB
#define PS2_CLK_PIN       GPIO_PIN_13
#define PS2_DAT_PORT      GPIOB
#define PS2_DAT_PIN       GPIO_PIN_14
#define PS2_CMD_PORT      GPIOB
#define PS2_CMD_PIN       GPIO_PIN_15
#define PS2_CS_PORT       GPIOA
#define PS2_CS_PIN        GPIO_PIN_8

#endif

/* 时钟半周期延时(us)。16us ≈ 31kHz，很慢很稳；想快可以改小到 8 */
#define PS2_HALF_CLK_US   16

/* 摇杆死区（0~127 之间）：摇杆回中时会有几 LSB 的抖动，小于它当 0 处理。
 * 4 对应单侧量程约 3%；若实测回中抖动超过此范围，需要重新标定。 */
#define PS2_DEADZONE      4

/* ------------------------- 手柄数据结构 ------------------------- */

/* btn1（已取反，1 = 按下） */
#define PS2_BTN_SELECT    0x01
#define PS2_BTN_JOYR      0x02
#define PS2_BTN_JOYL      0x04
#define PS2_BTN_START     0x08
#define PS2_BTN_UP        0x10
#define PS2_BTN_RIGHT     0x20
#define PS2_BTN_DOWN      0x40
#define PS2_BTN_LEFT      0x80

/* btn2（已取反，1 = 按下） */
#define PS2_BTN_L2        0x01
#define PS2_BTN_R2        0x02
#define PS2_BTN_L1        0x04
#define PS2_BTN_R1        0x08
#define PS2_BTN_TRIANGLE  0x10   /* △ */
#define PS2_BTN_CIRCLE    0x20   /* ○ */
#define PS2_BTN_CROSS     0x40   /* × */
#define PS2_BTN_SQUARE    0x80   /* □ */

typedef struct
{
    uint8_t mode;        /* 手柄模式：0x41=数字, 0x73/0x79=模拟 */
    uint8_t btn1;        /* 按键1，见 PS2_BTN_xxx */
    uint8_t btn2;        /* 按键2，见 PS2_BTN_xxx */
    uint8_t RJoy_LR;     /* 右摇杆 左右：0x00=左, 0x80≈中, 0xFF=右 */
    uint8_t RJoy_UD;     /* 右摇杆 上下：0x00=上, 0x80≈中, 0xFF=下 */
    uint8_t LJoy_LR;     /* 左摇杆 左右 */
    uint8_t LJoy_UD;     /* 左摇杆 上下 */
    uint8_t ok;          /* 1 = 本次通信成功 */
    uint8_t raw[21];     /* 0x73=9 bytes; 0x79 pressure mode=21 bytes */
    uint8_t length;
} PS2_t;

/* --------------------------- 函数接口 --------------------------- */

/**
  * @brief  初始化 PS2 手柄引脚（CLK/CMD/CS 推挽输出，DAT 输入上拉）
  *         同时启动 DWT 周期计数器用于微秒级延时。
  * @note   会覆盖 CubeMX 对 PB13/PB14/PB15/PA8 的配置（从 AF 改成 GPIO），
  *         所以本驱动不使用 SPI2 外设。
  */
void PS2_Init(void);

/**
  * @brief  读一次手柄数据
  * @param  js  输出：手柄数据
  * @return 1 = 成功，0 = 失败
  */
uint8_t PS2_Read(PS2_t *js);

/**
  * @brief  让手柄进入模拟模式（摇杆才会输出连续值）
  * @return 配置完成后的手柄模式 ID（0x73/0x79 = 模拟模式）
  */
uint8_t PS2_ConfigAnalog(void);

/**
  * @brief  摇杆原始值 -> -1000 ~ +1000
  * @param  raw       摇杆原始值（0~255，中间约 128）
  * @param  invert    1 = 反向（例如"上"是 0x00，想让它输出 +1000）
  * @param  deadzone  死区，回中时的小抖动直接当 0
  * @return -1000 ~ +1000
  */
int16_t PS2_AxisMap(uint8_t raw, uint8_t invert, uint8_t deadzone);

#endif /* __PS2_H */
