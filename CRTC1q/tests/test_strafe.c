#include "ps2_ctrl.h"
#include "car_ctrl.h"
#include "arm_ctrl.h"
#include "drv8833.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

GPIO_TypeDef fake_gpio_a;
TIM_HandleTypeDef htim3,htim4;
static uint32_t now;
static PS2_t input;
static uint8_t connected=1;
static const int signs[4]={DRV8833_DIR_LF,DRV8833_DIR_LR,DRV8833_DIR_RF,DRV8833_DIR_RR};
uint32_t HAL_GetTick(void) {return now;}
void HAL_GPIO_Init(GPIO_TypeDef *p,const GPIO_InitTypeDef *g) {(void)p;(void)g;}
void HAL_GPIO_EXTI_IRQHandler(uint16_t p) {(void)p;}
void HAL_NVIC_SetPriority(int i,uint32_t p,uint32_t s) {(void)i;(void)p;(void)s;}
void HAL_NVIC_EnableIRQ(int i) {(void)i;}
HAL_StatusTypeDef HAL_TIM_GenerateEvent(TIM_HandleTypeDef *t,uint32_t e) {(void)t;(void)e;return HAL_OK;}
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *t,uint32_t c) {(void)t;(void)c;return HAL_OK;}
void PS2_Init(void) {}
uint8_t PS2_ConfigAnalog(void) {return 0x73;}
uint8_t PS2_Read(PS2_t *p) {*p=input;p->ok=connected;return connected;}
void Arm_Ctrl_Init(void) {}
void Arm_Ctrl_Update(const PS2_t *p,uint8_t ok) {(void)p;(void)ok;}
void Car_MotorPermille(car_wheel_t w,int16_t s) {DRV8833_SetMotor((uint8_t)w,s);}
void Car_Motor(car_wheel_t w,car_dir_t d,uint8_t s)
{DRV8833_SetMotor((uint8_t)w,(int16_t)((d==CAR_DIR_FORWARD ? 1 : -1)*s*10));}
void Car_MotorStopAll(void) {DRV8833_BrakeAll();}

static void set_input(uint8_t x,uint8_t y,uint8_t r)
{ input=(PS2_t){.LJoy_LR=x,.LJoy_UD=y,.RJoy_LR=r,.RJoy_UD=128,.ok=1}; }
static void step(void) {now+=20;PS2_Ctrl_Update();}
static void settle(uint8_t x,uint8_t y,uint8_t r)
{
    DRV8833_BrakeAll();PS2_Ctrl_Init();set_input(x,y,r);
    for(unsigned i=0;i<60;i++) step();
}
static void expect(const int expected[4])
{
    for(unsigned i=0;i<4;i++) assert(DRV8833_GetMotor((uint8_t)i)==expected[i]*signs[i]);
}
static void test_driver_polarity(void)
{
    volatile uint32_t *a[4]={&htim3.ccr[0],&htim3.ccr[2],&htim4.ccr[0],&htim4.ccr[3]};
    volatile uint32_t *b[4]={&htim3.ccr[1],&htim3.ccr[3],&htim4.ccr[1],&htim4.ccr[2]};
    for(unsigned i=0;i<4;i++) {
        DRV8833_SetMotor((uint8_t)i,400);
        assert(DRV8833_GetMotor((uint8_t)i)==400*signs[i]);
        if(signs[i]>0) {assert(*a[i]==3600 && *b[i]==2160);}
        else {assert(*a[i]==2160 && *b[i]==3600);}
        DRV8833_SetMotor((uint8_t)i,0);assert(*a[i]==3600 && *b[i]==3600);
    }
}
int main(int argc,char **argv)
{
    DRV8833_Init();
    if(argc>1 && !strcmp(argv[1],"regression")) {
        /* Pure forward/turn trace must match CRTC1q before modification exactly. */
        PS2_Ctrl_Init();
        for(unsigned i=0;i<12000;i++) {
            set_input(128,(uint8_t)(i*37U),(uint8_t)(i*71U));step();
            for(unsigned w=0;w<4;w++) printf("%d,",DRV8833_GetMotor((uint8_t)w));
            puts("");
        }
        return 0;
    }
    const int forward[4]={1000,1000,1000,1000};
    const int back[4]={-1000,-1000,-1000,-1000};
    const int right[4]={1000,-1000,1000,-1000}; /* Actual CRTC2z applied order LF/LR/RF/RR. */
    const int left[4]={-1000,1000,-1000,1000};
    const int turn[4]={1000,1000,-1000,-1000};
    const int stop[4]={0,0,0,0};
    settle(128,0,128);expect(forward);
    settle(128,255,128);expect(back);
    settle(255,128,128);expect(right);
    settle(0,128,128);expect(left);
    settle(128,128,255);expect(turn);
    /* z's ramp differs; use its executable only to verify settled output pattern. */
    if(argc>1 && !strcmp(argv[1],"reference")) {puts("CRTC2z actual wheel pattern verified");return 0;}
    test_driver_polarity();
    settle(128,128,128);expect(stop);
    set_input(255,128,128);step();
    assert(DRV8833_GetMotor(0)==50*signs[0]); /* Original q 2500/s ramp, 20ms. */
    set_input(128,128,128);step();expect(stop); /* Release brakes immediately. */
    settle(255,128,128);connected=0;step();expect(stop);connected=1;
    set_input(128,128,128);step();
    int last=0;
    for(unsigned x=128;x<=255;x++) {
        settle((uint8_t)x,128,128);
        int v=DRV8833_GetMotor(0)*signs[0];assert(v>=last && v<=1000);last=v;
        assert(DRV8833_GetMotor(1)*signs[1]==-v);
        assert(DRV8833_GetMotor(2)*signs[2]==v);
        assert(DRV8833_GetMotor(3)*signs[3]==-v);
    }
    settle(255,0,128);
    const int diagonal[4]={1000,0,1000,0};expect(diagonal);
    puts("PASS: CRTC2z strafe pattern, forward/back/turn, q ramp, release/disconnect brake, diagonal scaling and per-wheel PWM polarity");
    return 0;
}
