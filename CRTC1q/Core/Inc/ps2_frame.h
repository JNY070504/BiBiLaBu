/**
  ******************************************************************************
  * @file    ps2_frame.h
  * @brief   PS2 手柄数据帧结构与解码接口
  *
  * 这一层只做"原始字节 -> 结构体"的解码，不关心引脚、不关心控制逻辑。
  * 解码结果用 PS2_Frame 表示，供上层（ps2_ctrl 状态机）消费。
  ******************************************************************************
  */

#ifndef PS2_FRAME_H
#define PS2_FRAME_H

#include <stdint.h>

/* 解码后的手柄数据帧 */
typedef struct
{
    uint8_t  valid;     /* 1 = 本帧有效（收到 0xFF 头 + 0x5A 标识 + 模拟模式） */
    uint8_t  mode;      /* 手柄模式：0x73=模拟(9字节), 0x79=压力(21字节), 0x41=数字 */
    uint8_t  lx;        /* 左摇杆 X：0x00=左, 0x80≈中, 0xFF=右 */
    uint8_t  ly;        /* 左摇杆 Y：0x00=上, 0x80≈中, 0xFF=下 */
    uint8_t  rx;        /* 右摇杆 X */
    uint8_t  ry;        /* 右摇杆 Y */
    uint16_t buttons;   /* 按键位（已取反，1=按下），低 8 位=btn1，高 8 位=btn2 */
} PS2_Frame;

/* 常用按键位（对应 buttons 的位） */
enum
{
    FRAME_SELECT   = 0x0001U,  /* SELECT */
    FRAME_JOYR     = 0x0002U,  /* 右摇杆按下 */
    FRAME_JOYL     = 0x0004U,  /* 左摇杆按下 */
    FRAME_START    = 0x0008U,  /* START */
    FRAME_UP       = 0x0010U,  /* 方向键上 */
    FRAME_RIGHT    = 0x0020U,  /* 方向键右 */
    FRAME_DOWN     = 0x0040U,  /* 方向键下 */
    FRAME_LEFT     = 0x0080U,  /* 方向键左 */
    FRAME_L2       = 0x0100U,
    FRAME_R2       = 0x0200U,
    FRAME_L1       = 0x0400U,
    FRAME_R1       = 0x0800U,
    FRAME_TRIANGLE = 0x1000U,  /* △ */
    FRAME_CIRCLE   = 0x2000U,  /* ○ */
    FRAME_CROSS    = 0x4000U,  /* × */
    FRAME_SQUARE   = 0x8000U   /* □ */
};

/**
  * @brief  把 PS2 原始字节流解码成 PS2_Frame
  * @param  raw     原始字节数组（PS2_Read 返回的 raw）
  * @param  length  字节数（9 或 21）
  * @param  frame   输出：解码后的帧
  * @return 1=成功，0=失败（帧头不对 / 非模拟模式 / 长度不够）
  */
uint8_t PS2_Decode(const uint8_t *raw, unsigned length, PS2_Frame *frame);

#endif /* PS2_FRAME_H */
