#ifndef PID_HPP_
#define PID_HPP_

/**
 * @file  pid.hpp
 * @brief C 风格（struct + 自由函数）的 PID，需要 C99 编译。
 *
 * @warning 本文件与 alg_pid.h 是两套**互不相容**的 PID 实现：
 *          - alg_pid.h  : class PID（C++），工程里实际编译使用的是这一套；
 *          - pid.hpp    : typedef struct PID（本文件）。
 *          两者都定义了名字 PID，**不要**在同一个 .cpp 里同时 include 这两个头文件，
 *          否则会报 “redefinition of 'PID'”。
 *
 *          当前 MDK 工程（X001.uvprojx）只编译 PID2.cpp + alg_pid.h，
 *          本文件与 pid.cpp 未加入工程，保留仅作参考/备用。
 */

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float prev_error;
    float out_min;
    float out_max;
} PID;

void PID_Init(PID* pid, float Kp, float Ki, float Kd, float out_min, float out_max);
float PID_Update(PID* pid, float error);

#endif /* PID_HPP_ */
