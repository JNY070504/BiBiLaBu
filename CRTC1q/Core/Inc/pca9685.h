/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : pca9685.h
  * @brief          : Minimal HAL driver for the NXP PCA9685 16-channel 12-bit
  *                   PWM LED driver (I2C, address 0x40 + A0/A1/A2).
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __PCA9685_H
#define __PCA9685_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* Address ------------------------------------------------------------------*/
#define PCA9685_DEFAULT_ADDR        (0x40U)   /* 7-bit address, A0=A1=A2=0       */
#define PCA9685_ALLCALL_ADDR        (0x70U)   /* 7-bit "All Call" address        */

/* Channel / resolution -----------------------------------------------------*/
#define PCA9685_CH_NUM              (16U)
#define PCA9685_PWM_MAX             (4095U)   /* 12-bit full scale              */

/* Internal oscillator ------------------------------------------------------*/
#define PCA9685_OSC_FREQ_HZ         (25000000UL)

/* Default PWM carrier frequency, useful range is roughly 24 Hz .. 1600 Hz */
#define PCA9685_DEFAULT_PWM_FREQ    (1000.0f)

/* MODE1 register bits -----------------------------------------------------*/
#define PCA9685_MODE1_RESTART       (0x80U)
#define PCA9685_MODE1_EXTCLK        (0x40U)
#define PCA9685_MODE1_AI            (0x20U)
#define PCA9685_MODE1_SLEEP         (0x10U)
#define PCA9685_MODE1_SUBADR        (0x08U)
#define PCA9685_MODE1_ALLCALL       (0x04U)
#define PCA9685_MODE1_INVRT         (0x02U)
#define PCA9685_MODE1_OUTOFT        (0x01U)
#define PCA9685_MODE1_RESET         (0x00U)

/* MODE2 register bits -----------------------------------------------------*/
#define PCA9685_MODE2_OUTDRV        (0x04U)   /* 1 = totem-pole (push-pull)     */
#define PCA9685_MODE2_OPEN_DRAIN    (0x00U)   /* 0 = open-drain                 */

/* MODE2 value used by the driver.
 * Totem-pole (push-pull) is the default so the outputs can also drive a gate
 * or a load high. Clear PCA9685_USE_OUTDRV to get open-drain outputs (lower
 * current, only able to pull the line down). */
#ifndef PCA9685_USE_OUTDRV
#define PCA9685_USE_OUTDRV         (1)
#endif

#if PCA9685_USE_OUTDRV
#define PCA9685_MODE2_VALUE        (PCA9685_MODE2_OUTDRV)
#else
#define PCA9685_MODE2_VALUE        (PCA9685_MODE2_OPEN_DRAIN)
#endif

/* Exported functions ------------------------------------------------------*/
HAL_StatusTypeDef PCA9685_Init(I2C_HandleTypeDef *hi2c, uint16_t dev_addr_7bit);
HAL_StatusTypeDef PCA9685_ProbeBitBang(I2C_HandleTypeDef *hi2c, GPIO_TypeDef *port,
                                       uint16_t scl_pin, uint16_t sda_pin, uint16_t dev_addr_7bit);
HAL_StatusTypeDef PCA9685_Sleep(I2C_HandleTypeDef *hi2c, uint16_t dev_addr_7bit);
HAL_StatusTypeDef PCA9685_WakeUp(I2C_HandleTypeDef *hi2c, uint16_t dev_addr_7bit);
HAL_StatusTypeDef PCA9685_SetPwmFreq(I2C_HandleTypeDef *hi2c, uint16_t dev_addr_7bit, float freq_hz);
HAL_StatusTypeDef PCA9685_SetChannelPwm(I2C_HandleTypeDef *hi2c, uint16_t dev_addr_7bit,
                                        uint8_t channel, uint16_t duty, uint8_t inverted);
HAL_StatusTypeDef PCA9685_SetAllPwm(I2C_HandleTypeDef *hi2c, uint16_t dev_addr_7bit,
                                    uint16_t duty, uint8_t inverted);
HAL_StatusTypeDef PCA9685_AllOff(I2C_HandleTypeDef *hi2c, uint16_t dev_addr_7bit);
HAL_StatusTypeDef PCA9685_AllOn(I2C_HandleTypeDef *hi2c, uint16_t dev_addr_7bit);

#ifdef __cplusplus
}
#endif

#endif /* __PCA9685_H */
