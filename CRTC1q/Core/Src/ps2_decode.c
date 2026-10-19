/**
  ******************************************************************************
  * @file    ps2_decode.c
  * @brief   PS2 手柄数据帧解码实现
  ******************************************************************************
  */

#include "ps2_frame.h"

/**
  * PS2 标准 9 字节返回帧（模拟模式 0x73）：
  *   [0] = 0xFF      （固定）
  *   [1] = 模式 ID   （0x73=模拟，0x79=压力模式，0x41=数字）
  *   [2] = 0x5A      （固定标识）
  *   [3] = 按键字节1（SELECT/JOYR/JOYL/START/UP/RIGHT/DOWN/LEFT，低电平有效）
  *   [4] = 按键字节2（L2/R2/L1/R1/△/○/×/□，低电平有效）
  *   [5] = 右摇杆 X  （0x00~0xFF，0x80≈中）
  *   [6] = 右摇杆 Y
  *   [7] = 左摇杆 X
  *   [8] = 左摇杆 Y
  *
  * 压力模式 0x79 有 21 字节，前 9 字节布局相同，后面 12 字节是按键压力值。
  */
uint8_t PS2_Decode(const uint8_t *r, unsigned length, PS2_Frame *f)
{
    *f = (PS2_Frame){0};
    f->lx = f->ly = f->rx = f->ry = 128U;   /* 默认回中 */

    if (!r || length < 9U) return 0U;

    f->mode = r[1];

    /* 帧头校验：[0]=0xFF、[2]=0x5A，且必须是模拟模式 */
    if (r[0] != 0xFFU || r[2] != 0x5AU) return 0U;
    if (r[1] != 0x73U && r[1] != 0x79U) return 0U;

    /* 压力模式必须收满 21 字节 */
    if (r[1] == 0x79U && length < 21U) return 0U;

    /* 按键字节是低电平有效，取反后 1=按下 */
    f->buttons = (uint16_t)~((uint16_t)r[3] | ((uint16_t)r[4] << 8));

    f->rx = r[5];
    f->ry = r[6];
    f->lx = r[7];
    f->ly = r[8];

    f->valid = 1U;
    return 1U;
}
