#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"
#include "tim.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 电机 PWM 用 TIM1：
 *   TIM1_CH1 -> PA8  -> 驱动板 PWMA（左电机）
 *   TIM1_CH4 -> PA11 -> 驱动板 PWMB（右电机）
 * //  PSC=71, ARR=49 -> 72MHz / 72 / 50 = 20kHz    49
 * 所以占空比取值 0 ~ 3599。
 *
 * 注意：原来这里写的是 TIM3 + PWM_MAX 3599，但 TIM3 已被用作
 *       左编码器（PB4/PB5，编码器模式），不会输出 PWM。
 */
#define PWM_MAX 3599

void Motor_Init(void);
void Motor_Set(int16_t left, int16_t right);
void Motor_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_H */
