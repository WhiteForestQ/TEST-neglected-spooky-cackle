/**
 * @note 原来这里写的是 #include "pid.h"，但工程里根本没有 pid.h
 *       （头文件实际叫 pid.hpp），一旦把本文件加入 Keil 工程就会立刻报
 *       fatal error: 'pid.h' file not found。此处已修正为 pid.hpp。
 *
 * @note 本文件未加入 MDK 工程（X001.uvprojx 里没有 pid.cpp），
 *       工程实际使用的是 alg_pid.h + PID2.cpp 那一套。
 */
#include "pid.hpp"

void PID_Init(PID* pid, float Kp, float Ki, float Kd, float out_min, float out_max)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->out_min = out_min;
    pid->out_max = out_max;
}

float PID_Update(PID* pid, float error)
{
    float derivative;
    float out;

    pid->integral += error;
    derivative = error - pid->prev_error;

    out = pid->Kp * error
        + pid->Ki * pid->integral
        + pid->Kd * derivative;

    /* 输出限幅 + 简单抗积分饱和 */
    if (out > pid->out_max) {
        out = pid->out_max;
        if (error > 0.0f) pid->integral -= error;
    } else if (out < pid->out_min) {
        out = pid->out_min;
        if (error < 0.0f) pid->integral -= error;
    }

    pid->prev_error = error;
    return out;
}
