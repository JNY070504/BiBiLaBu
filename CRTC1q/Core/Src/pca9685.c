/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : pca9685.c
  * @brief          : Minimal HAL driver for the NXP PCA9685 16-channel 12-bit
  *                   PWM LED driver.
  *
  *  Every channel register block is 4 bytes:
  *    0x06 + 4*ch + 0 : ON_L   (9 LSB of the ON tick)
  *    0x06 + 4*ch + 1 : ON_H   (full on, ONH bit 4)
  *    0x06 + 4*ch + 2 : OFF_L  (8 LSB of the OFF tick)
  *    0x06 + 4*ch + 3 : OFF_H  (3 MSB of the OFF tick + OFFH bit 4)
  *
  *  A duty cycle of 0     means the output stays OFF.
  *  A duty cycle of 4095  means the output stays ON.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "pca9685.h"

/* USER CODE BEGIN Private defines */
#define PCA9685_REG_MODE1        (0x00U)
#define PCA9685_REG_MODE2        (0x01U)
#define PCA9685_REG_LED0         (0x06U)
#define PCA9685_REG_PRE_SCALE    (0xFEU)

#define PCA9685_LED_REG_BYTES    (4U)
#define PCA9685_LED_REGS_BYTES   (PCA9685_CH_NUM * PCA9685_LED_REG_BYTES)

/* Pins the bit-bang probe can be pointed at, for diagnosing a wrong hookup */
#define BB_HALF_PERIOD_US       5U   /* 5 us + 5 us -> roughly 100 kHz */
/* USER CODE END Private defines */

/* USER CODE BEGIN Private variables */
/* Scratch buffer used to update the 16 channel registers in one I2C transfer */
static uint8_t pca9685_pwm_buf[PCA9685_LED_REGS_BYTES];
/* USER CODE END Private variables */

/* USER CODE BEGIN Private function prototypes */
static HAL_StatusTypeDef pca9685_WriteReg(I2C_HandleTypeDef *hi2c, uint16_t dev_addr_7bit,
                                          uint8_t reg, const uint8_t *data, uint16_t len);
static void bb_DelayInit(void);
static void bb_DelayUs(uint16_t us);
static uint8_t bb_DelaySelfTest(void);
static void bb_Scl(GPIO_TypeDef *port, uint16_t pin, uint8_t release);
static void bb_Sda(GPIO_TypeDef *port, uint16_t pin, uint8_t release);
/* USER CODE END Private function prototypes */

/* USER CODE BEGIN 0 */

static HAL_StatusTypeDef pca9685_WriteReg(I2C_HandleTypeDef *hi2c, uint16_t dev_addr_7bit,
                                          uint8_t reg, const uint8_t *data, uint16_t len)
{
  if ((hi2c == NULL) || (data == NULL) || (len == 0U))
  {
    return HAL_ERROR;
  }

  /* HAL expects the already left-shifted 8-bit address */
  return HAL_I2C_Mem_Write(hi2c, (uint16_t)(dev_addr_7bit << 1), reg,
                           I2C_MEMADD_SIZE_8BIT, (uint8_t *)data, len, 100);
}

HAL_StatusTypeDef PCA9685_Init(I2C_HandleTypeDef *hi2c, uint16_t dev_addr_7bit)
{
  HAL_StatusTypeDef status;
  uint8_t reg = 0x00U;
  uint8_t mode1;
  uint8_t mode2;

  /* The internal oscillator needs some time after power-up */
  HAL_Delay(20);

  /* AI (auto-increment) is mandatory: SetChannelPwm writes 4 bytes and
   * SetAllPwm writes 64 bytes to consecutive registers. Without AI every
   * byte lands in the same register and the multi-byte writes are no-ops. */
  mode1 = PCA9685_MODE1_ALLCALL | PCA9685_MODE1_AI;

  /* MODE1: oscillator on, no sub-address, all-call active, no invert,
   *        outputs pulled low on shutdown, no open-drain override */
  status = pca9685_WriteReg(hi2c, dev_addr_7bit, PCA9685_REG_MODE1, &mode1, 1U);
  if (status != HAL_OK)
  {
    return status;
  }

  mode2 = PCA9685_MODE2_VALUE;

  /* MODE2: totem-pole (or open-drain) outputs */
  status = pca9685_WriteReg(hi2c, dev_addr_7bit, PCA9685_REG_MODE2, &mode2, 1U);
  if (status != HAL_OK)
  {
    return status;
  }

  /* All channels off */
  status = PCA9685_AllOff(hi2c, dev_addr_7bit);
  if (status != HAL_OK)
  {
    return status;
  }

  /* Sanity check: read MODE2 back and make sure the chip answered */
  if (HAL_I2C_Mem_Read(hi2c, (uint16_t)(dev_addr_7bit << 1), PCA9685_REG_MODE2,
                       I2C_MEMADD_SIZE_8BIT, &reg, 1U, 100) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (reg != PCA9685_MODE2_VALUE)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

HAL_StatusTypeDef PCA9685_Sleep(I2C_HandleTypeDef *hi2c, uint16_t dev_addr_7bit)
{
  /* AI must stay set so multi-byte writes keep working after wake-up */
  uint8_t mode1 = PCA9685_MODE1_ALLCALL | PCA9685_MODE1_AI | PCA9685_MODE1_SLEEP;

  return pca9685_WriteReg(hi2c, dev_addr_7bit, PCA9685_REG_MODE1, &mode1, 1U);
}

HAL_StatusTypeDef PCA9685_WakeUp(I2C_HandleTypeDef *hi2c, uint16_t dev_addr_7bit)
{
  /* Keep AI enabled */
  uint8_t mode1 = PCA9685_MODE1_ALLCALL | PCA9685_MODE1_AI;
  HAL_StatusTypeDef status;

  status = pca9685_WriteReg(hi2c, dev_addr_7bit, PCA9685_REG_MODE1, &mode1, 1U);
  if (status != HAL_OK)
  {
    return status;
  }

  /* Clear the SLEEP bit: first a stop, then a start is required by the chip */
  HAL_Delay(1);
  return PCA9685_SetPwmFreq(hi2c, dev_addr_7bit, PCA9685_DEFAULT_PWM_FREQ);
}

/* freq_hz range of the chip is roughly 24 Hz .. 1600 Hz */
HAL_StatusTypeDef PCA9685_SetPwmFreq(I2C_HandleTypeDef *hi2c, uint16_t dev_addr_7bit, float freq_hz)
{
  uint8_t prescale;
  uint8_t mode1;
  float step;

  if ((freq_hz <= 0.0f) || (freq_hz >= PCA9685_OSC_FREQ_HZ))
  {
    return HAL_ERROR;
  }

  /* prescale = round(25 MHz / (4096 * freq)) - 1, then clamp to 0..255 */
  step = (float)PCA9685_OSC_FREQ_HZ / (4096.0f * freq_hz) - 1.0f;
  if (step <= 0.0f)
  {
    prescale = 0U;
  }
  else if (step >= 255.0f)
  {
    prescale = 255U;
  }
  else
  {
    prescale = (uint8_t)(step + 0.5f);
  }

  /* ---- PCA9685 PRE_SCALE write sequence (datasheet mandatory) ----
   * The PRE_SCALE register is writable ONLY while SLEEP = 1. Writing it with
   * the oscillator running is ignored, so the frequency would silently stay
   * at its default (~200 Hz). The correct order is:
   *   1. set SLEEP (oscillator off)
   *   2. write PRE_SCALE
   *   3. clear SLEEP
   *   4. wait >= 500 us for the oscillator to stabilise
   *   5. set RESTART to re-enable the PWM outputs
   * AI is preserved on every MODE1 write so multi-byte LED writes keep working. */

  /* 1. Read current MODE1, then set SLEEP (keep AI and the rest) */
  if (HAL_I2C_Mem_Read(hi2c, (uint16_t)(dev_addr_7bit << 1), PCA9685_REG_MODE1,
                       I2C_MEMADD_SIZE_8BIT, &mode1, 1U, 100) != HAL_OK)
  {
    return HAL_ERROR;
  }
  mode1 |= PCA9685_MODE1_SLEEP;
  if (pca9685_WriteReg(hi2c, dev_addr_7bit, PCA9685_REG_MODE1, &mode1, 1U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* 2. Write PRE_SCALE while SLEEP is active */
  if (pca9685_WriteReg(hi2c, dev_addr_7bit, PCA9685_REG_PRE_SCALE, &prescale, 1U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* 3. Clear SLEEP, keep AI and every other bit untouched */
  mode1 &= (uint8_t)~PCA9685_MODE1_SLEEP;
  if (pca9685_WriteReg(hi2c, dev_addr_7bit, PCA9685_REG_MODE1, &mode1, 1U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* 4. Wait for the internal oscillator to start up (>= 500 us) */
  HAL_Delay(1);

  /* 5. Restart the PWM sequence so the new prescaler takes effect */
  mode1 |= PCA9685_MODE1_RESTART;
  if (pca9685_WriteReg(hi2c, dev_addr_7bit, PCA9685_REG_MODE1, &mode1, 1U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

HAL_StatusTypeDef PCA9685_SetChannelPwm(I2C_HandleTypeDef *hi2c, uint16_t dev_addr_7bit,
                                        uint8_t channel, uint16_t duty, uint8_t inverted)
{
  uint8_t buf[PCA9685_LED_REG_BYTES];
  uint32_t on_tick, off_tick;
  uint8_t reg;

  if (channel >= PCA9685_CH_NUM)
  {
    return HAL_ERROR;
  }

  if (duty > PCA9685_PWM_MAX)
  {
    duty = PCA9685_PWM_MAX;
  }

  if (inverted != 0U)
  {
    /* Active low load: brighten by shortening the OFF part */
    on_tick  = (PCA9685_PWM_MAX - duty);
    off_tick = PCA9685_PWM_MAX;
  }
  else
  {
    /* Active high load: brighten by lengthening the ON part */
    on_tick  = 0U;
    off_tick = duty;
  }

  /* duty == 0 must switch the channel completely off (full_off bit) */
  if ((inverted == 0U) && (duty == 0U))
  {
    on_tick  = (PCA9685_PWM_MAX | 0x1000UL);
    off_tick = (PCA9685_PWM_MAX | 0x1000UL);
  }

  buf[0] = (uint8_t)(on_tick & 0xFFU);
  buf[1] = (uint8_t)((on_tick >> 8) & 0x1FU);   /* full-on flag is bit 4 */
  buf[2] = (uint8_t)(off_tick & 0xFFU);
  buf[3] = (uint8_t)((off_tick >> 8) & 0x1FU);  /* full-off flag is bit 4 */

  reg = (uint8_t)(PCA9685_REG_LED0 + ((uint16_t)channel * PCA9685_LED_REG_BYTES));
  return pca9685_WriteReg(hi2c, dev_addr_7bit, reg, buf, PCA9685_LED_REG_BYTES);
}

/* Sets the same duty on the 16 channels with one single I2C transfer */
HAL_StatusTypeDef PCA9685_SetAllPwm(I2C_HandleTypeDef *hi2c, uint16_t dev_addr_7bit,
                                    uint16_t duty, uint8_t inverted)
{
  uint32_t on_tick, off_tick;
  uint16_t ch;

  if (hi2c == NULL)
  {
    return HAL_ERROR;
  }

  if (duty > PCA9685_PWM_MAX)
  {
    duty = PCA9685_PWM_MAX;
  }

  if (inverted != 0U)
  {
    on_tick  = (PCA9685_PWM_MAX - duty);
    off_tick = PCA9685_PWM_MAX;
  }
  else
  {
    on_tick  = 0U;
    off_tick = duty;
  }

  if ((inverted == 0U) && (duty == 0U))
  {
    on_tick  = (PCA9685_PWM_MAX | 0x1000UL);
    off_tick = (PCA9685_PWM_MAX | 0x1000UL);
  }

  for (ch = 0U; ch < PCA9685_CH_NUM; ch++)
  {
    pca9685_pwm_buf[(ch * PCA9685_LED_REG_BYTES) + 0U] = (uint8_t)(on_tick & 0xFFU);
    pca9685_pwm_buf[(ch * PCA9685_LED_REG_BYTES) + 1U] = (uint8_t)((on_tick >> 8) & 0x1FU);
    pca9685_pwm_buf[(ch * PCA9685_LED_REG_BYTES) + 2U] = (uint8_t)(off_tick & 0xFFU);
    pca9685_pwm_buf[(ch * PCA9685_LED_REG_BYTES) + 3U] = (uint8_t)((off_tick >> 8) & 0x1FU);
  }

  return pca9685_WriteReg(hi2c, dev_addr_7bit, PCA9685_REG_LED0,
                          pca9685_pwm_buf, PCA9685_LED_REGS_BYTES);
}

HAL_StatusTypeDef PCA9685_AllOff(I2C_HandleTypeDef *hi2c, uint16_t dev_addr_7bit)
{
  return PCA9685_SetAllPwm(hi2c, dev_addr_7bit, 0U, 0U);
}

HAL_StatusTypeDef PCA9685_AllOn(I2C_HandleTypeDef *hi2c, uint16_t dev_addr_7bit)
{
  return PCA9685_SetAllPwm(hi2c, dev_addr_7bit, PCA9685_PWM_MAX, 0U);
}

/* ---------------------------------------------------------------------------
 * Diagnostic bit-bang I2C master
 *
 * Talks to the chip on a caller-chosen pin pair without using any I2C
 * peripheral: it only sends START + address + STOP and samples the
 * acknowledge bit. Because the I2C hardware is bypassed, a successful probe
 * proves that the chip and the wiring are fine, which isolates a failure to
 * either the I2C2 peripheral/configuration or the physical connection.
 * ------------------------------------------------------------------------ */

static void bb_DelayInit(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

/* The delay loop below relies on CYCCNT actually counting. On a few clones
 * and under some debug configurations the trace unit stays off, CYCCNT never
 * leaves 0 and the wait would never end, hanging the whole boot. Burning a
 * few hundred cycles proves the counter moves before it is trusted. */
static uint8_t bb_DelaySelfTest(void)
{
  uint32_t before = DWT->CYCCNT;
  uint32_t spin;

  for (spin = 0U; spin < 1000U; spin++)
  {
  }

  return (uint8_t)(DWT->CYCCNT != before);
}

static void bb_DelayUs(uint16_t us)
{
  uint32_t start = DWT->CYCCNT;
  uint32_t ticks = (uint32_t)us * (SystemCoreClock / 1000000UL);
  /* Bounded wait: a dead counter then costs a fixed short delay instead of
   * hanging forever. The cap is generous, a live counter always exits first. */
  uint32_t guard = ticks + 10000UL;

  while ((uint32_t)(DWT->CYCCNT - start) < ticks)
  {
    if (guard-- == 0U)
    {
      break;
    }
  }
}

/* Open drain: writing 1 releases the line, writing 0 pulls it to ground */
static void bb_Scl(GPIO_TypeDef *port, uint16_t pin, uint8_t release)
{
  HAL_GPIO_WritePin(port, pin, (release != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void bb_Sda(GPIO_TypeDef *port, uint16_t pin, uint8_t release)
{
  HAL_GPIO_WritePin(port, pin, (release != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

HAL_StatusTypeDef PCA9685_ProbeBitBang(I2C_HandleTypeDef *hi2c, GPIO_TypeDef *port,
                                       uint16_t scl_pin, uint16_t sda_pin, uint16_t dev_addr_7bit)
{
  GPIO_InitTypeDef gpio = {0};
  uint8_t addr_byte;
  uint8_t i;
  uint8_t ack = 0U;

  bb_DelayInit();

  if (bb_DelaySelfTest() == 0U)
  {
    /* No working cycle counter, so the probe would not be trustworthy.
     * Give the pins back to the peripheral and report a plain failure so
     * the caller can move on to the next diagnosis. */
    if (hi2c != NULL)
    {
      HAL_I2C_Init(hi2c);
    }
    return HAL_ERROR;
  }

  /* Lend the pins to the GPIO peripheral for the duration of the probe */
  if (hi2c != NULL)
  {
    HAL_I2C_DeInit(hi2c);
  }

  gpio.Pin   = (uint16_t)(scl_pin | sda_pin);
  gpio.Mode  = GPIO_MODE_OUTPUT_OD;
  gpio.Pull  = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(port, &gpio);

  /* Bus idle */
  bb_Scl(port, scl_pin, 1U);
  bb_Sda(port, sda_pin, 1U);
  bb_DelayUs(BB_HALF_PERIOD_US);

  /* START: SDA falls while SCL is high */
  bb_Sda(port, sda_pin, 0U);
  bb_DelayUs(BB_HALF_PERIOD_US);
  bb_Scl(port, scl_pin, 0U);
  bb_DelayUs(BB_HALF_PERIOD_US);

  /* Address byte, write direction */
  addr_byte = (uint8_t)(dev_addr_7bit << 1);
  for (i = 0U; i < 8U; i++)
  {
    if ((addr_byte & (uint8_t)(0x80U >> i)) != 0U)
    {
      bb_Sda(port, sda_pin, 1U);
    }
    else
    {
      bb_Sda(port, sda_pin, 0U);
    }
    bb_DelayUs(BB_HALF_PERIOD_US);
    bb_Scl(port, scl_pin, 1U);
    bb_DelayUs(BB_HALF_PERIOD_US);
    bb_Scl(port, scl_pin, 0U);
    bb_DelayUs(BB_HALF_PERIOD_US);
  }

  /* Ninth bit: the slave pulls SDA low to acknowledge */
  bb_Sda(port, sda_pin, 1U);
  bb_DelayUs(BB_HALF_PERIOD_US);
  bb_Scl(port, scl_pin, 1U);
  bb_DelayUs(BB_HALF_PERIOD_US);
  ack = (uint8_t)((port->IDR & sda_pin) == 0U);
  bb_Scl(port, scl_pin, 0U);
  bb_DelayUs(BB_HALF_PERIOD_US);

  /* STOP: SDA rises while SCL is high */
  bb_Sda(port, sda_pin, 0U);
  bb_DelayUs(BB_HALF_PERIOD_US);
  bb_Scl(port, scl_pin, 1U);
  bb_DelayUs(BB_HALF_PERIOD_US);
  bb_Sda(port, sda_pin, 1U);
  bb_DelayUs(BB_HALF_PERIOD_US);

  /* Hand the pins back to the I2C peripheral */
  if (hi2c != NULL)
  {
    HAL_I2C_Init(hi2c);
  }

  return (ack != 0U) ? HAL_OK : HAL_ERROR;
}

/* USER CODE END 0 */

