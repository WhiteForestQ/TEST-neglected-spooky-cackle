#include "encoder.h"

/* 速度：编码器脉冲 / 控制周期(10ms)。正数 = 车轮朝前 */
volatile int16_t speed_left  = 0;
volatile int16_t speed_right = 0;

/*
 * 方向校正（必须台架验证）：
 *   Motor_Set(1000, 1000) 让两个轮子都朝前转，然后在 Keil 里看
 *   speed_left / speed_right（或者串口帧里的 SL / SR）。
 *   两个都应该是正数；哪一个读出来是负的，就把对应的宏改成 -1。
 *
 * 这一步不做的话，速度环会变成正反馈：轮子越快 -> 误差越大 -> PWM 越大，
 * 直接满速飞车，而且永远不收敛。
 */
#ifndef ENCODER_L_SIGN
#define ENCODER_L_SIGN  (+1)
#endif
#ifndef ENCODER_R_SIGN
#define ENCODER_R_SIGN  (+1)
#endif

/* 初始化编码器 */
void Encoder_Init(void)
{
    /* 启动 TIM2 编码器接口（右轮，PA15/PB3） */
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    /* 启动 TIM3 编码器接口（左轮，PB4/PB5） */
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
}

/*
 * 更新编码器数据（必须在 10ms 定时器中断里调用）
 *
 * 计数器是 16 位的，反转时读数会变成 65535 这种大数，
 * 强转成 int16_t 就自动变成负数，正反转一起解决了。
 */
void Encoder_Update(void)
{
    /* 1. 左轮 = TIM3 */
    speed_left = (int16_t)((int16_t)__HAL_TIM_GET_COUNTER(&htim3) * ENCODER_L_SIGN);
    __HAL_TIM_SET_COUNTER(&htim3, 0);

    /* 2. 右轮 = TIM2 */
    speed_right = (int16_t)((int16_t)__HAL_TIM_GET_COUNTER(&htim2) * ENCODER_R_SIGN);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
}
