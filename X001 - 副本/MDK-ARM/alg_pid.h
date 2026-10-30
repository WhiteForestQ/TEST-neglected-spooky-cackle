#ifndef ALG_PID_H
#define ALG_PID_H

/**
 * @file    alg_pid.h
 * @brief   灰度循迹小车使用的 PID 控制器（C++ 类形式）
 *
 * @note    编译器兼容性（重要）：
 *          MDK 工程使用的是 ARM Compiler 5（armcc V5.06），默认按 C++03 标准编译，
 *          不支持 C++11 的“类内成员初始化”（如 float Kp = 0.0f;），
 *          否则会报 12 个 error: #321: data member initializer is not allowed。
 *          因此这里所有成员都不在类内赋初值，统一放到构造函数 PID() 里初始化。
 */
class PID {
public:
    /**
     * @brief  构造函数
     * @details 把所有成员清零。这样即使忘了调用 Init()，也不会用随机值参与运算。
     */
    PID();

    /**
     * @brief  初始化 PID 控制器参数
     * @param  _Kp       比例增益系数
     * @param  _Ki       积分增益系数
     * @param  _Kd       微分增益系数
     * @param  _Out_Max  控制输出限幅值，<= 0 表示不限幅
     * @param  _I_Max    积分输出限幅值，<= 0 表示不限幅（抗积分饱和）
     * @param  _Dt       控制周期，单位秒，必须 > 0
     * @retval void
     */
    void Init(float _Kp, float _Ki, float _Kd,
              float _Out_Max = 0.0f, float _I_Max = 0.0f, float _Dt = 0.001f);

    /**
     * @brief  PID 计算的周期回调函数
     * @details 需要挂在固定周期的定时器中断里调用（周期要与 Init() 的 _Dt 一致）。
     *          每次调用会根据当前 Target、Now 计算并更新控制输出 out。
     */
    void TIM_Calculate_PeriodElapsedCallback(void);

    // 设置目标值
    void SetTarget(float _Target) { Target = _Target; }

    // 设置当前实际值（反馈值）
    void SetNow(float _Now) { Now = _Now; }

    // 强制设置积分项（用于清空积分或初始化）
    void Set_Integral_Error(float _Integral_Error) { Integral_Error = _Integral_Error; }

    // 预置“上一次的偏差”
    // 用途：第一次调用前先把它设成本次偏差，可消除起步瞬间微分项的冲击
    void SetPreError(float _Pre_Error) { Pre_Error = _Pre_Error; }

    // 获取计算输出
    float GetOut(void) { return out; }

    // 获取目标值 / 当前值（调试用）
    float GetTarget(void) { return Target; }
    float GetNow(void) { return Now; }
//		//速度控制
//		void Speed_Control(void);

private:
    // --- 用户参数 ---
    float Kp;
    float Ki;
    float Kd;
    float Out_Max;
    float I_Max;
    float Dt;

    // --- 运行状态 ---
    float Target;          // 目标值
    float Now;             // 当前实际值
    float Integral_Error;  // 累计误差积分
    float Pre_Error;       // 上一次的误差（用于算微分）
    float Pre_Now;         // 上一次的实际值
    float out;             // 最终输出值
};

#endif // ALG_PID_H
