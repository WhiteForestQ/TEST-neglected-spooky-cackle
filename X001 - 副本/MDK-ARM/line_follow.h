#ifndef LINE_FOLLOW_H
#define LINE_FOLLOW_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 灰度循迹控制层（C 接口，内部用 C++ 的 PID 类实现）。
 * main.c 保持纯 C 直接调用，不用改成 main.cpp。
 *
 * 用法：
 *   LineFollow_Init();                 // 初始化（内部会 Motor_Init / Encoder_Init）
 *   TIM4 的 10ms 中断里调：
 *       Encoder_Update();              // 先读编码器
 *       LineFollow_Step();             // 外环：读灰度 -> PID -> 目标速度
 *       Speed_Control();               // 内环：速度 PID -> PWM
 *   while (1) {
 *       LineFollow_SendTelemetry();    // 串口调试帧（内部自带 100ms 节流）
 *   }
 *
 * 注意：控制环放在定时器中断里，主循环再做 OLED 刷屏（一屏 24ms）
 *       就不会影响控制周期了。
 */

/* 控制模式 */
#define LF_MODE_LINE     0u     /* 循迹（外环+内环都工作） */
#define LF_MODE_MANUAL   1u     /* 手动 PWM（台架测试/串口调试） */

/* 目标速度：编码器脉冲 / 控制周期(10ms)，由外环给出、内环使用 */
extern volatile int16_t target_speed_L;
extern volatile int16_t target_speed_R;

void LineFollow_Init(void);

/* 外环：读灰度 -> 位置 PID -> 左右轮目标速度。必须每 10ms 调一次 */
void LineFollow_Step(void);

/* 内环：速度 PID -> PWM。放在 10ms 定时器中断里调 */
void Speed_Control(void);

/*
 * 手动 PWM 模式：直接按 PWM 驱动两个轮子（范围 -PWM_MAX ~ +PWM_MAX）。
 * 500ms 内没有新的手动命令会自动停车，防止串口断开后车一直跑。
 */
void LineFollow_SetManualPwm(int16_t left_pwm, int16_t right_pwm);

/* 重新武装循迹：回到"等线"状态，看到线后自动起步 */
void LineFollow_Start(void);

/* 停车：清零目标、清积分、滑行停车，并回到"等线"状态 */
void LineFollow_Stop(void);

/* ---------------- 只读接口（给调试显示用） ---------------- */

int16_t LineFollow_GetError(void);
int16_t LineFollow_GetCorrection(void);
int16_t LineFollow_GetPwmL(void);
int16_t LineFollow_GetPwmR(void);
uint8_t LineFollow_GetState(void);      /* 0=等线 1=巡线 2=找线 */
uint8_t LineFollow_GetMode(void);       /* LF_MODE_LINE / LF_MODE_MANUAL */
uint8_t LineFollow_GetRaw(void);        /* 最近一次灰度原始值（bit0 最左） */
uint8_t LineFollow_GetSaturated(void);  /* 1 = 速度环长期顶死（多半是 SPEED_FULL_COUNTS 填错） */
uint8_t LineFollow_GetLostCount(void);  /* 连续丢线拍数 */

/* 串口调试帧，非阻塞（中断发送）。可以在主循环里每拍都调，内部按 100ms 节流 */
void LineFollow_SendTelemetry(void);

#ifdef __cplusplus
}
#endif

#endif /* LINE_FOLLOW_H */
