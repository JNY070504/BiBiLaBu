/* Host-only fake peripherals. Keil does not compile this test directory. */
#ifndef TEST_HOST_HAL_H
#define TEST_HOST_HAL_H
#define __MAIN_H
#include <stdint.h>
typedef enum {HAL_OK=0,HAL_ERROR=1} HAL_StatusTypeDef;
typedef struct {uint32_t arr,psc,ccr[4];} TIM_HandleTypeDef;
typedef struct {uint32_t unused;} GPIO_TypeDef;
extern GPIO_TypeDef fake_gpio_a;
#define GPIOA (&fake_gpio_a)
#define GPIO_PIN_0 1U
#define GPIO_PIN_1 2U
#define GPIO_MODE_IT_FALLING 1U
#define GPIO_PULLUP 1U
#define EXTI0_IRQn 0
#define EXTI1_IRQn 1
typedef struct {uint32_t Pin,Mode,Pull;} GPIO_InitTypeDef;
#define TIM_CHANNEL_1 0U
#define TIM_CHANNEL_2 4U
#define TIM_CHANNEL_3 8U
#define TIM_CHANNEL_4 12U
#define TIM_EVENTSOURCE_UPDATE 1U
#define __HAL_TIM_SET_PRESCALER(t,v) ((t)->psc=(v))
#define __HAL_TIM_SET_AUTORELOAD(t,v) ((t)->arr=(v))
#define __HAL_TIM_GET_AUTORELOAD(t) ((t)->arr)
#define __HAL_TIM_SET_COMPARE(t,c,v) ((t)->ccr[(c)/4U]=(v))
#define __HAL_RCC_GPIOA_CLK_ENABLE() ((void)0)
#define __HAL_RCC_AFIO_CLK_ENABLE() ((void)0)
uint32_t HAL_GetTick(void);
void HAL_GPIO_Init(GPIO_TypeDef *,const GPIO_InitTypeDef *);
void HAL_GPIO_EXTI_IRQHandler(uint16_t);
void HAL_NVIC_SetPriority(int,uint32_t,uint32_t);
void HAL_NVIC_EnableIRQ(int);
HAL_StatusTypeDef HAL_TIM_GenerateEvent(TIM_HandleTypeDef *,uint32_t);
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *,uint32_t);
#endif
