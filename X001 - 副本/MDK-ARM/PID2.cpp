#include "alg_pid.h"

/* 构造函数：成员全部清零，防止对象未 Init 就参与运算时出现随机值 */
PID::PID()
{
    Kp = 0.0f;
    Ki = 0.0f;
    Kd = 0.0f;
    Out_Max = 0.0f;
    I_Max = 0.0f;
    Dt = 0.001f;

    Target = 0.0f;
    Now = 0.0f;
    Integral_Error = 0.0f;
    Pre_Error = 0.0f;
    Pre_Now = 0.0f;
    out = 0.0f;
}

void PID::Init(float _Kp, float _Ki, float _Kd, float _Out_Max, float _I_Max, float _Dt)
{
    Kp = _Kp;
    Ki = _Ki;
    Kd = _Kd;
    Out_Max = _Out_Max;
    I_Max = _I_Max;

    /* Dt 必须为正，否则微分项会除零，这里做个保护 */
    Dt = (_Dt > 0.0f) ? _Dt : 0.001f;

    /* 初始化状态清理，防止有脏数据 */
    Integral_Error = 0.0f;
    Pre_Error = 0.0f;
    Pre_Now = 0.0f;
    out = 0.0f;
}

void PID::TIM_Calculate_PeriodElapsedCallback(void)
{
    float error;   /* 当前偏差 e(k) = Target - Now */
    float pout;    /* 比例项输出 Pout = Kp * e(k) */
    float iout;    /* 积分项输出 Iout = Ki * 积分 */
    float dout;    /* 微分项输出 Dout = Kd * de/dt */

    /* 1. 计算当前偏差：期望值减反馈值 */
    error = Target - Now;

    /* 2. 计算比例项输出（响应快，直接与偏差成正比） */
    pout = Kp * error;

    /* 3. 累加积分项，并乘以 Ki 得到积分项输出 Iout */
    Integral_Error += error * Dt;
    iout = Ki * Integral_Error;

    /* 4. 积分限幅（抗积分饱和）：
          当累计的积分输出超过 I_Max 时将其钳位，防止系统超调与振荡。
          I_Max <= 0 表示不限幅（默认值 0）。
          钳位后同步反算 Integral_Error，防止内部积分继续无限累加。 */
    if (I_Max > 0.0f) {
        if (iout > I_Max) {
            iout = I_Max;
            if (Ki != 0.0f) {
                Integral_Error = iout / Ki;
            }
        } else if (iout < -I_Max) {
            iout = -I_Max;
            if (Ki != 0.0f) {
                Integral_Error = iout / Ki;
            }
        }
    }

    /* 5. 计算微分项：用本次偏差与上次偏差之差近似 de/dt */
    dout = Kd * (error - Pre_Error) / Dt;
    /* 如果 D 项噪声太大，可以改成"对测量值微分"并加一阶低通：
       dout = -Kd * (Now - Pre_Now) / Dt;
       dout = 0.7f * Pre_Dout + 0.3f * dout;   (需要再加一个成员存 Pre_Dout) */

    /* 6. 合成最终输出 = 比例 + 积分 + 微分 */
    out = pout + iout + dout;

    /* 7. 总输出限幅：保护下游执行器（如电机、舵机）不被损坏。
          饱和时只撤销"往饱和方向推"的那部分积分，进一步抑制 windup。
          注意这里必须判断 error 的符号：原来是无条件撤销，
          当 error 反向（本来就在把输出往下拉）时会反而让积分变大，
          退出饱和更慢。 */
    if (Out_Max > 0.0f) {
        if (out > Out_Max) {
            out = Out_Max;
            if (error > 0.0f) {
                Integral_Error -= error * Dt;
            }
        } else if (out < -Out_Max) {
            out = -Out_Max;
            if (error < 0.0f) {
                Integral_Error -= error * Dt;
            }
        }
    }

    /* 8. 更新历史状态，保存本次数据供下次计算 */
    Pre_Now = Now;      /* 保存当前过程变量 */
    Pre_Error = error;  /* 必须更新上一次的偏差，否则微分项 D 会失效 */
}
