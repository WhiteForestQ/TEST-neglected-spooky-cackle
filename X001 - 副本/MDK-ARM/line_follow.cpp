/*
 * 循迹 + 速度双闭环
 *   外环（10ms，主循环）：灰度位置 PID  ->  左右轮的目标速度
 *   内环（10ms，TIM4 中断）：编码器速度 PID ->  PWM
 *
 * 本版相对上一版改了什么：
 *   1. 控制周期统一成 10ms（TIM4 已从 0.91ms 改成 10ms），
 *      CONTROL_PERIOD_S / CTRL_PERIOD_MS 三处必须一致；
 *   2. 所有速度量都统一成"每 10ms 编码器脉冲数"，并且全部由 SPEED_FULL_COUNTS 推导，
 *      不再写死 1500 这种和实际量纲对不上的数（原来速度环一直饱和 -> 两个轮子满速 -> 不转向）；
 *   3. 加状态机：上电先等线(IDLE) -> 巡线(RUN) -> 丢线找线(LOST)，
 *      不会一上电没放到线上就全速冲出去；
 *   4. 目标速度加斜率限制，起停不再有大的电流冲击；
 *   5. 停车能真正停住（中断里也会检查 s_run）；
 *   6. 加串口调试帧，方便看误差/目标/实际/PWM，调参不用瞎猜。
 */
#include "line_follow.h"
#include "alg_pid.h"
#include "gray_sensor.h"
#include "motor.h"
#include "encoder.h"
#include "usart.h"

/* ============================ 参数区 ============================ */

/*
 * ★ 整个工程只需要实测这一个数：PWM 给满(3599)时，每个 10ms 周期
 *   编码器读到的脉冲数。下面所有速度参数都从它推出来。
 *
 *   测法（把车架空，轮子离地）：
 *     1. 把下面的 LF_OPENLOOP_PWM 改成 3599，编译烧录；
 *     2. 看串口帧里的 SL / SR（或 Keil 里 Watch speed_left / speed_right）；
 *     3. 等数值稳定，记下来填到这里；
 *     4. 把 LF_OPENLOOP_PWM 改回 0。
 *
 *   暂时不测的话，先填一个偏小的值：只会让车跑得慢，不会出事；
 *   填大了就会像以前一样一直饱和（串口帧里 SAT=1 就是在提示这件事）。
 */
#define SPEED_FULL_COUNTS     50

/* 台架开环测速：非 0 时完全跳过循迹和速度环，两轮直接吃这个 PWM。测完改回 0 */
#define LF_OPENLOOP_PWM       0

/* 串口调试帧开关，帧格式见 LineFollow_SendTelemetry() */
#define LF_TELEMETRY          1
#define LF_TELEMETRY_MS       100u

/* ====================== 以下都是推导出来的 ====================== */

#define CTRL_PERIOD_MS        10u
#define CONTROL_PERIOD_S      (CTRL_PERIOD_MS / 1000.0f)

/* 巡线基础速度 = 满速的 50%，剩下 50% 留给转向差速 */
#define BASE_SPEED            ((int16_t)((SPEED_FULL_COUNTS * 50) / 100))

/* 差速上限 = 基础速度的 80%（此时外轮最多到 0.9 倍满速，不会把速度环顶饱和） */
#define CORRECTION_MAX        ((float)BASE_SPEED * 0.8f)

/* 丢线原地找线的转速 = 满速的 40% */
#define SEARCH_SPEED          ((int16_t)((SPEED_FULL_COUNTS * 40) / 100))

/* 目标速度斜率：每 10ms 最多变化满速的 5%（0 -> 满速约 200ms） */
#define TARGET_SLEW           ((int16_t)((SPEED_FULL_COUNTS * 5) / 100) + 1)

/* -------- 速度环（内环）：输入 counts/10ms，输出 PWM --------
 * 被控对象增益 k = SPEED_FULL_COUNTS / PWM_MAX   [counts / PWM]
 * 取 Kp * k ≈ 0.5 作为起始值；换电机/换轮子/换编码器后，这个式子会自动跟着变，
 * 不用再手调数量级。
 */
#define SPEED_KP              ((float)PWM_MAX * 0.5f / (float)SPEED_FULL_COUNTS)
#define SPEED_KI              (SPEED_KP * 10.0f)  /* 积分时间 Ti ≈ 0.1s（原来 0.3 相当于 27s，等于没积分；
                                                     Ti=0.1s 时速度环约 0.7s 跟上，且几乎无超调） */
#define SPEED_KD              0.0f                /* 速度环不用 D，编码器差分会放大噪声 */
#define SPEED_OUT_MAX         ((float)PWM_MAX)
#define SPEED_I_MAX           ((float)PWM_MAX)

/* -------- 循迹环（外环）：输入位置误差，输出差速 --------
 * 误差最大 ±15，希望它刚好对应 CORRECTION_MAX 的差速。
 */
#define LINE_KP               (CORRECTION_MAX / 15.0f)
#define LINE_KI               0.0f
#define LINE_KD               (LINE_KP * 0.02f)   /* 微分时间 Td ≈ 20ms（原来 1.5 相当于 125ms，一个探头翻转就能顶到限幅） */

#define LOST_CNT_MAX          30u    /* 连续丢线 30 拍（300ms）才开始找线 */
#define FIND_CNT_MAX          3u     /* 连续 3 拍看到线才算真的重新找到 */
#define LF_MANUAL_TIMEOUT_MS  500u   /* 手动 PWM 模式：500ms 没有新命令就自动停车 */

/* ============================ 内部状态 ============================ */

typedef enum {
    LF_IDLE = 0,   /* 上电等待起步：看到线之前不动 */
    LF_RUN  = 1,   /* 正常巡线 */
    LF_LOST = 2    /* 丢线，原地打转找线 */
} LfState;

static PID pid_line;  //结构体
static PID pid_speed_L;
static PID pid_speed_R;

/* 外环给内环的目标速度：编码器脉冲 / 10ms */
volatile int16_t target_speed_L = 0;
volatile int16_t target_speed_R = 0;

static LfState  s_state      = LF_IDLE;
static uint8_t  s_lost_cnt   = 0;
static uint16_t s_lost_timer = 0;   /* 丢线搜索计时，单位 10ms */
static uint8_t  s_find_cnt   = 0;
static uint8_t  s_first      = 1;
static uint8_t  s_inited     = 0;
static int16_t  s_error      = 0;
static int16_t  s_correction = 0;
static int16_t  s_last_error = 0;   /* 最后一次看到线时的偏差，决定往哪边找 */

static volatile uint8_t     s_run     = 0;  /* 1 = 允许速度环驱动电机 */
static volatile uint8_t     s_reset_i = 0;  /* 请求速度环清积分（主循环 -> 中断） */
static volatile int16_t     s_pwm_l   = 0;
static volatile int16_t     s_pwm_r   = 0;
static volatile uint8_t     s_sat     = 0;  /* 速度环饱和标志，用于诊断 SPEED_FULL_COUNTS 是否填错 */
static uint16_t             s_sat_cnt = 0;

static volatile uint8_t     s_mode        = LF_MODE_LINE;  /* 0=循迹 1=手动 PWM */
static volatile int16_t     s_manual_l    = 0;
static volatile int16_t     s_manual_r    = 0;
static volatile uint32_t    s_manual_tick = 0;
static volatile uint8_t     s_raw         = 0;   /* 最近一次灰度原始值，给 OLED 显示 */

static int16_t clamp_i16(int32_t v, int16_t lo, int16_t hi)
{
    if (v < (int32_t)lo) {
        return lo;
    }
    if (v > (int32_t)hi) {
        return hi;
    }
    return (int16_t)v;
}

/* 目标速度斜坡：避免阶跃造成的大电流、打滑和齿轮冲击 */
static void ramp_target(volatile int16_t *v, int16_t target)
{
    int32_t cur = (int32_t)*v;
    int32_t d   = (int32_t)target - cur;

    if (d >  (int32_t)TARGET_SLEW) {
        d =  (int32_t)TARGET_SLEW;
    }
    if (d < -(int32_t)TARGET_SLEW) {
        d = -(int32_t)TARGET_SLEW;
    }
    *v = (int16_t)(cur + d);
}

/* ============================ 对外接口 ============================ */

extern "C" void LineFollow_Init(void)
{
    /* 外环：位置 PID，输出差速。Target 由 Step 每次赋成当前误差、Now 恒为 0，
       这样 PID 内部的 error 就等于灰度误差（正 = 线在右边）。 */
    pid_line.Init(LINE_KP, LINE_KI, LINE_KD, CORRECTION_MAX, 0.0f, CONTROL_PERIOD_S);
    pid_line.SetTarget(0.0f);

    /* 内环：左右轮各一个速度 PID */
    pid_speed_L.Init(SPEED_KP, SPEED_KI, SPEED_KD, SPEED_OUT_MAX, SPEED_I_MAX, CONTROL_PERIOD_S);
    pid_speed_R.Init(SPEED_KP, SPEED_KI, SPEED_KD, SPEED_OUT_MAX, SPEED_I_MAX, CONTROL_PERIOD_S);

    s_state      = LF_IDLE;
    s_lost_cnt   = 0;
    s_find_cnt   = 0;
    s_first      = 1;
    s_error      = 0;
    s_correction = 0;
    s_last_error = 0;
    s_run        = 0;
    s_reset_i    = 1;
    s_sat        = 0;
    s_sat_cnt    = 0;
    target_speed_L = 0;
    target_speed_R = 0;
    s_inited     = 1;

    Motor_Init();
    Encoder_Init();
}

extern "C" void LineFollow_Step(void)
{
    uint8_t raw;
    int16_t corr;

    if (!s_inited) {
        return;
    }

#if (LF_OPENLOOP_PWM != 0)
    /* 开环测速模式：只给固定 PWM，完全不跑闭环 */
    s_run = 0;
    Motor_Set(LF_OPENLOOP_PWM, LF_OPENLOOP_PWM);
    return;
#endif

    /* 灰度只采样一次！后面的全白/误差判断都基于这一份 raw，
       避免"这一拍看到线、下一拍又丢了"的抖动。 */
    raw    = Gray_ReadRaw();
    s_raw  = raw;

    if (s_mode == LF_MODE_MANUAL) {
        /* 手动模式：外环停摆（PWM 由 Speed_Control 直接输出），
           但灰度照样采样，OLED 自检页在台架测试时仍然有效 */
        return;
    }

    if (raw != 0u) {
        s_error      = Gray_ErrorFromRaw(raw);
        s_last_error = s_error;
    } else {
        /* 全白：朝最后一次看到线的方向打满，让车往回找 */
        s_error = (s_last_error >= 0) ? 15 : -15;
    }

    switch (s_state) {
    case LF_IDLE:
        /* 上电先等线：连续几拍都看到线才允许起步 */
        if (raw != 0u) {
            if (++s_find_cnt >= FIND_CNT_MAX) {
                s_find_cnt   = 0;
                s_lost_cnt   = 0;
                s_lost_timer = 0;
                s_first      = 1;
                s_reset_i  = 1;
                s_state    = LF_RUN;
                s_run      = 1;
            }
        } else {
            s_find_cnt = 0;
        }
        target_speed_L = 0;
        target_speed_R = 0;
        return;

    case LF_RUN:
        if (raw == 0u) {
            if (s_lost_cnt < 255u) {
                s_lost_cnt++;
            }
            if (s_lost_cnt > LOST_CNT_MAX) {
                s_state      = LF_LOST;
                s_find_cnt   = 0;
                s_lost_timer = 0;
            }
        } else {
            s_lost_cnt = 0;
        }
        break;

    case LF_LOST:
        if (raw != 0u) {
            if (++s_find_cnt >= FIND_CNT_MAX) {
                s_find_cnt   = 0;
                s_lost_cnt   = 0;
                s_lost_timer = 0;
                s_first      = 1;      /* 重新找到线，重置微分，避免 D 项冲击 */
                s_state    = LF_RUN;
            }
        } else {
            s_find_cnt = 0;
        }
        break;

    default:
        s_state = LF_IDLE;
        return;
    }

    if (s_state == LF_LOST) {
        /*
         * 丢线搜索，分两段循环：
         *   前 1s ：倒车回找。断线 / 过弯丢线时线其实就在车身后方，
         *           倒车让前置探头重新扫过它（实测纯原地打转在断线赛道
         *           上永远找不回来 —— 固定差速只会画圆，不是螺旋）。
         *   之后  ：原地转向，朝最后一次看到线的方向扫一圈。
         */
        if (s_lost_timer < 65535u) {
            s_lost_timer++;
        }
        if (s_lost_timer > 200u) {
            s_lost_timer = 1u;      /* 1s 倒车 + 1s 转向，循环 */
        }

        if (s_lost_timer <= 100u) {
            int16_t rev = (int16_t)(-(SEARCH_SPEED / 2));   /* 慢速倒车 */
            ramp_target(&target_speed_L, rev);
            ramp_target(&target_speed_R, rev);
        } else if (s_last_error < 0) {
            ramp_target(&target_speed_L, (int16_t)(-SEARCH_SPEED));
            ramp_target(&target_speed_R, (int16_t)( SEARCH_SPEED));
        } else {
            ramp_target(&target_speed_L, (int16_t)( SEARCH_SPEED));
            ramp_target(&target_speed_R, (int16_t)(-SEARCH_SPEED));
        }
        return;
    }

    /* ---------- 外环：灰度位置 PID -> 差速 ---------- */
    if (s_first) {
        /* 第一拍预置"上次偏差"，消掉起步瞬间 Kd*(e-0)/Dt 的冲击 */
        pid_line.SetPreError((float)s_error);
        s_first = 0;
    }

    pid_line.SetTarget((float)s_error);
    pid_line.SetNow(0.0f);
    pid_line.TIM_Calculate_PeriodElapsedCallback();

    corr = clamp_i16((int32_t)pid_line.GetOut(),
                     (int16_t)(-CORRECTION_MAX),
                     (int16_t)( CORRECTION_MAX));
    s_correction = corr;

    /* ---------- 差速 -> 目标速度（不再直接给电机） ---------- */
    /* 误差为正说明线偏右，要右转 = 左轮快、右轮慢 */
    ramp_target(&target_speed_L, clamp_i16((int32_t)BASE_SPEED + corr,
                                           (int16_t)(-SPEED_FULL_COUNTS),
                                           (int16_t)( SPEED_FULL_COUNTS)));
    ramp_target(&target_speed_R, clamp_i16((int32_t)BASE_SPEED - corr,
                                           (int16_t)(-SPEED_FULL_COUNTS),
                                           (int16_t)( SPEED_FULL_COUNTS)));
}

/* 内环：TIM4 的 10ms 中断里调用，读编码器 -> 速度 PID -> 出 PWM */
extern "C" void Speed_Control(void)
{
#if (LF_OPENLOOP_PWM != 0)
    return;      /* 开环测速时由 LineFollow_Step 直接写 PWM */
#endif

    /* ---------- 手动 PWM 模式（台架 / 串口调试） ---------- */
    if (s_mode == LF_MODE_MANUAL) {
        if ((uint32_t)(HAL_GetTick() - s_manual_tick) > LF_MANUAL_TIMEOUT_MS) {
            /* 上位机没再发命令：自动回循迹并停车，避免车一直跑 */
            s_mode  = LF_MODE_LINE;
            s_run   = 0;
            s_pwm_l = 0;
            s_pwm_r = 0;
            Motor_Stop();
            return;
        }

        s_pwm_l = s_manual_l;
        s_pwm_r = s_manual_r;
        Motor_Set(s_manual_l, s_manual_r);
        return;
    }

    if (!s_run) {
        /* 等待起步 / 已停车：滑行停车，并且下次启动时积分从 0 开始 */
        s_reset_i = 1;
        s_pwm_l   = 0;
        s_pwm_r   = 0;
        s_sat     = 0;
        s_sat_cnt = 0;
        Motor_Stop();
        return;
    }

    if (s_reset_i) {
        pid_speed_L.Set_Integral_Error(0.0f);
        pid_speed_R.Set_Integral_Error(0.0f);
        s_reset_i = 0;
    }

    /* 左轮速度环 */
    pid_speed_L.SetTarget((float)target_speed_L);
    pid_speed_L.SetNow((float)speed_left);
    pid_speed_L.TIM_Calculate_PeriodElapsedCallback();

    /* 右轮速度环 */
    pid_speed_R.SetTarget((float)target_speed_R);
    pid_speed_R.SetNow((float)speed_right);
    pid_speed_R.TIM_Calculate_PeriodElapsedCallback();

    s_pwm_l = (int16_t)pid_speed_L.GetOut();
    s_pwm_r = (int16_t)pid_speed_R.GetOut();
    Motor_Set(s_pwm_l, s_pwm_r);

    /* 诊断：两个轮子都长期顶在最大出力 = SPEED_FULL_COUNTS 填太大（或者编码器没读数） */
    if ((s_pwm_l >= (int16_t)(SPEED_OUT_MAX - 5.0f)) &&
        (s_pwm_r >= (int16_t)(SPEED_OUT_MAX - 5.0f))) {
        if (s_sat_cnt < 1000u) {
            s_sat_cnt++;
        }
    } else {
        s_sat_cnt = 0;
    }
    s_sat = (uint8_t)(s_sat_cnt > 100u);
}

/* ============================ 调试辅助 ============================ */

#if LF_TELEMETRY
static char s_tx_buf[128];

static char *put_i16(char *p, int32_t v)
{
    char    tmp[8];
    int32_t n   = 0;
    uint8_t neg = 0;

    if (v < 0) {
        neg = 1;
        v   = -v;
    }
    if (v == 0) {
        tmp[n++] = '0';
    }
    while (v > 0) {
        tmp[n++] = (char)('0' + (int32_t)(v % 10));
        v /= 10;
    }
    if (neg) {
        *p++ = '-';
    }
    while (n > 0) {
        *p++ = tmp[--n];
    }
    return p;
}

/*
 * 一帧长这样：
 *   E=误差,C=差速,TL/TR=目标速度,SL/SR=实际速度,PL/PR=PWM,ST=状态,SAT=饱和标志
 * 状态：0=等线中 1=巡线 2=找线
 * 用串口助手直接看，或丢给 VOFA+ 都行。
 */
extern "C" void LineFollow_SendTelemetry(void)
{
    static uint32_t last_tick = 0;
    uint32_t        now       = HAL_GetTick();
    char           *p         = s_tx_buf;

    if ((uint32_t)(now - last_tick) < LF_TELEMETRY_MS) {
        return;
    }
    last_tick = now;

    /* 上一帧还没发完就跳过这一帧，绝不阻塞主循环 */
    if (huart1.gState != HAL_UART_STATE_READY) {
        return;
    }

    *p++ = 'E';  *p++ = '=';  p = put_i16(p, s_error);        *p++ = ',';
    *p++ = 'C';  *p++ = '=';  p = put_i16(p, s_correction);   *p++ = ',';
    *p++ = 'T';  *p++ = 'L';  *p++ = '='; p = put_i16(p, target_speed_L); *p++ = ',';
    *p++ = 'T';  *p++ = 'R';  *p++ = '='; p = put_i16(p, target_speed_R); *p++ = ',';
    *p++ = 'S';  *p++ = 'L';  *p++ = '='; p = put_i16(p, speed_left);     *p++ = ',';
    *p++ = 'S';  *p++ = 'R';  *p++ = '='; p = put_i16(p, speed_right);    *p++ = ',';
    *p++ = 'P';  *p++ = 'L';  *p++ = '='; p = put_i16(p, s_pwm_l);        *p++ = ',';
    *p++ = 'P';  *p++ = 'R';  *p++ = '='; p = put_i16(p, s_pwm_r);        *p++ = ',';
    *p++ = 'S';  *p++ = 'T';  *p++ = '='; p = put_i16(p, (int32_t)s_state); *p++ = ',';
    *p++ = 'S';  *p++ = 'A';  *p++ = 'T'; *p++ = '='; p = put_i16(p, (int32_t)s_sat);
    *p++ = '\r'; *p++ = '\n';

    (void)HAL_UART_Transmit_IT(&huart1, (uint8_t *)s_tx_buf, (uint16_t)(p - s_tx_buf));
}
#else
extern "C" void LineFollow_SendTelemetry(void)
{
}
#endif

extern "C" int16_t LineFollow_GetError(void)
{
    return s_error;
}

extern "C" int16_t LineFollow_GetCorrection(void)
{
    return s_correction;
}

extern "C" int16_t LineFollow_GetPwmL(void)
{
    return s_pwm_l;
}

extern "C" int16_t LineFollow_GetPwmR(void)
{
    return s_pwm_r;
}

extern "C" uint8_t LineFollow_GetState(void)
{
    return (uint8_t)s_state;
}

extern "C" uint8_t LineFollow_GetRaw(void)
{
    return s_raw;
}

extern "C" uint8_t LineFollow_GetMode(void)
{
    return s_mode;
}

extern "C" uint8_t LineFollow_GetSaturated(void)
{
    return s_sat;
}

extern "C" uint8_t LineFollow_GetLostCount(void)
{
    return s_lost_cnt;
}

/* ---------------- 手动 PWM：台架测试 / 串口调参用 ---------------- */

extern "C" void LineFollow_SetManualPwm(int16_t left_pwm, int16_t right_pwm)
{
    if (left_pwm > (int16_t)PWM_MAX) {
        left_pwm = (int16_t)PWM_MAX;
    }
    if (left_pwm < (int16_t)(-PWM_MAX)) {
        left_pwm = (int16_t)(-PWM_MAX);
    }
    if (right_pwm > (int16_t)PWM_MAX) {
        right_pwm = (int16_t)PWM_MAX;
    }
    if (right_pwm < (int16_t)(-PWM_MAX)) {
        right_pwm = (int16_t)(-PWM_MAX);
    }

    s_manual_l    = left_pwm;
    s_manual_r    = right_pwm;
    s_manual_tick = HAL_GetTick();
    s_run         = 0;          /* 关掉闭环，别让速度环把手动输出顶掉 */
    s_state       = LF_IDLE;    /* 状态机回等线状态，OLED 上不会同时显示 RUN */
    s_mode        = LF_MODE_MANUAL;
}

extern "C" void LineFollow_Start(void)
{
    /* 重新武装：回到"等线"状态，看到线后自动起步 */
    s_mode         = LF_MODE_LINE;
    s_state        = LF_IDLE;
    s_run          = 0;
    s_lost_cnt     = 0;
    s_find_cnt     = 0;
    s_lost_timer   = 0;
    s_first        = 1;
    s_reset_i      = 1;
    target_speed_L = 0;
    target_speed_R = 0;
}

extern "C" void LineFollow_Stop(void)
{
    s_mode         = LF_MODE_LINE;
    s_state        = LF_IDLE;
    s_run          = 0;
    s_lost_cnt     = 0;
    s_find_cnt     = 0;
    s_lost_timer   = 0;
    s_first        = 1;
    s_reset_i      = 1;
    s_manual_l     = 0;
    s_manual_r     = 0;
    target_speed_L = 0;
    target_speed_R = 0;
    Motor_Stop();
}
