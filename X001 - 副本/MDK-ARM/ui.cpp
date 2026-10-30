/*
 * ui.cpp —— OLED 调试界面
 *
 * 页面：
 *   0 自检页  ：6 路灰度 + 原始值 + 编码器正负号（接线阶段最有用）
 *   1 总览页  ：状态 / 误差 / 差速 / 目标与实际速度 / PWM / 饱和与丢线
 *   2 速度环  ：左轮、右轮的目标、实际、误差、PWM
 *   3 系统页  ：运行时间 / I2C 在线状态 / 串口收字节数 / 模式 / 保护标志
 *
 * 注意：字库是 6x8 ASCII（0x20~0x7E），中文显示不了，超范围字符会变 '?'。
 *       每行 21 个字符，超长部分会被截断。
 */
#include "ui.h"
#include "OledDevice.hpp"
#include "i2c.h"
#include "line_follow.h"
#include "encoder.h"
#include "uart_comm.h"
#include "motor.h"

#include <cstring>

namespace {

const uint32_t kRenderPeriodMs = 100u;   /* 10Hz 刷新 */
const uint32_t kRetryPeriodMs  = 2000u;  /* 掉线重连间隔 */
const uint32_t kDebounceMs     = 20u;

const uint8_t kPageCount = 4u;

/* 屏幕对象：构造函数只保存指针并清显存，不碰 I2C，可以安全地做全局对象 */
OledDevice s_oled(&hi2c1);

uint8_t  s_page        = 0u;
uint8_t  s_key_raw     = 1u;    /* 1 = 松开 */
uint8_t  s_key_stable  = 1u;
uint32_t s_key_tick    = 0u;
uint32_t s_render_tick = 0u;
uint32_t s_retry_tick  = 0u;
bool     s_have_prev   = false;

char s_frame[8][22];
char s_prev[8][22];

/* ---------------- 极简格式化（不用 printf，省 flash） ---------------- */

char *Puts(char *p, const char *s)
{
    while (*s != '\0') {
        *p++ = *s++;
    }
    return p;
}

char *Puti(char *p, int32_t v)
{
    char tmp[12];
    int  n   = 0;
    bool neg = false;

    if (v < 0) {
        neg = true;
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

/* 定宽右对齐，数字跳变时不会左右抖 */
char *PutiW(char *p, int32_t v, uint8_t width)
{
    char    tmp[12];
    int     n = 0;
    int     len;
    int32_t a = (v < 0) ? -v : v;

    do {
        tmp[n++] = (char)('0' + (int32_t)(a % 10));
        a /= 10;
    } while (a > 0);

    len = n + ((v < 0) ? 1 : 0);
    while (len < (int)width) {
        *p++ = ' ';
        len++;
    }
    if (v < 0) {
        *p++ = '-';
    }
    while (n > 0) {
        *p++ = tmp[--n];
    }
    return p;
}

/* 两位补零，用于时间 mm:ss */
char *Puti2(char *p, int32_t v)
{
    if (v < 10) {
        *p++ = '0';
    }
    return Puti(p, v);
}

char *PutHex2(char *p, uint8_t v)
{
    static const char hex[] = "0123456789ABCDEF";
    *p++ = hex[(v >> 4) & 0x0Fu];
    *p++ = hex[v & 0x0Fu];
    return p;
}

const char *StateName(uint8_t st)
{
    if (st == 0u) { return "IDLE"; }
    if (st == 1u) { return "RUN "; }
    if (st == 2u) { return "LOST"; }
    return "??? ";
}

void ClearFrame(void)
{
    for (uint8_t i = 0u; i < 8u; i++) {
        s_frame[i][0] = '\0';
    }
}

/* ---------------- 各页面 ---------------- */

void RenderPage0(void)   /* 自检：接线阶段最有用 */
{
    const uint8_t raw = LineFollow_GetRaw();
    char *p;

    Puts(s_frame[0], "SENSOR SELFTEST");

    p = s_frame[1];
    *p++ = '[';
    for (uint8_t i = 0u; i < 6u; i++) {
        *p++ = (raw & (uint8_t)(1u << i)) ? '1' : '0';
        if (i < 5u) {
            *p++ = ' ';
        }
    }
    *p++ = ']';
    *p   = '\0';

    p = Puts(s_frame[2], "RAW 0x");
    p = PutHex2(p, raw);
    *p = '\0';

    p = Puts(s_frame[3], "ERR");
    p = PutiW(p, LineFollow_GetError(), 5);
    *p = '\0';

    p = Puts(s_frame[4], "ENC L");
    p = PutiW(p, speed_left, 4);
    p = Puts(p, " R");
    p = PutiW(p, speed_right, 4);
    *p = '\0';

    p = Puts(s_frame[5], "STATE ");
    p = Puts(p, StateName(LineFollow_GetState()));
    *p = '\0';

    p = Puts(s_frame[6], "PWM L");
    p = PutiW(p, LineFollow_GetPwmL(), 5);
    p = Puts(p, " R");
    p = PutiW(p, LineFollow_GetPwmR(), 5);
    *p = '\0';

    Puts(s_frame[7], "PAGE 0/4  KEY1>");
}

void RenderPage1(void)   /* 总览：跑车时看 */
{
    char *p;

    p = Puts(s_frame[0], "STATE ");
    p = Puts(p, StateName(LineFollow_GetState()));
    *p = '\0';

    p = Puts(s_frame[1], "E");
    p = PutiW(p, LineFollow_GetError(), 4);
    p = Puts(p, "  C");
    p = PutiW(p, LineFollow_GetCorrection(), 4);
    *p = '\0';

    p = Puts(s_frame[2], "TL");
    p = PutiW(p, target_speed_L, 4);
    p = Puts(p, "  TR");
    p = PutiW(p, target_speed_R, 4);
    *p = '\0';

    p = Puts(s_frame[3], "SL");
    p = PutiW(p, speed_left, 4);
    p = Puts(p, "  SR");
    p = PutiW(p, speed_right, 4);
    *p = '\0';

    p = Puts(s_frame[4], "PL");
    p = PutiW(p, LineFollow_GetPwmL(), 5);
    p = Puts(p, "  PR");
    p = PutiW(p, LineFollow_GetPwmR(), 5);
    *p = '\0';

    p = Puts(s_frame[5], "SAT ");
    p = Puti(p, LineFollow_GetSaturated());
    p = Puts(p, "  LOST ");
    p = Puti(p, LineFollow_GetLostCount());
    *p = '\0';

    p = Puts(s_frame[6], "RAW 0x");
    p = PutHex2(p, LineFollow_GetRaw());
    *p = '\0';

    Puts(s_frame[7], "PAGE 1/4  KEY1>");
}

void RenderPage2(void)   /* 速度环：调 Kp/Ki 时看 */
{
    char *p;

    Puts(s_frame[0], "SPEED LOOP");

    p = Puts(s_frame[1], "TGT L");
    p = PutiW(p, target_speed_L, 4);
    p = Puts(p, " R");
    p = PutiW(p, target_speed_R, 4);
    *p = '\0';

    p = Puts(s_frame[2], "ACT L");
    p = PutiW(p, speed_left, 4);
    p = Puts(p, " R");
    p = PutiW(p, speed_right, 4);
    *p = '\0';

    p = Puts(s_frame[3], "ERR L");
    p = PutiW(p, (int32_t)target_speed_L - (int32_t)speed_left, 4);
    p = Puts(p, " R");
    p = PutiW(p, (int32_t)target_speed_R - (int32_t)speed_right, 4);
    *p = '\0';

    p = Puts(s_frame[4], "PWM L");
    p = PutiW(p, LineFollow_GetPwmL(), 5);
    *p = '\0';

    p = Puts(s_frame[5], "PWM R");
    p = PutiW(p, LineFollow_GetPwmR(), 5);
    *p = '\0';

    p = Puts(s_frame[6], "SAT ");
    p = Puti(p, LineFollow_GetSaturated());
    *p = '\0';

    Puts(s_frame[7], "PAGE 2/4  KEY1>");
}

void RenderPage3(void)   /* 系统 */
{
    const uint32_t up = HAL_GetTick() / 1000u;
    char *p;

    Puts(s_frame[0], "SYSTEM");

    p = Puts(s_frame[1], "UP ");
    p = Puti(p, (int32_t)((up / 3600u) % 100u));
    *p++ = ':';
    p = Puti2(p, (int32_t)((up / 60u) % 60u));
    *p++ = ':';
    p = Puti2(p, (int32_t)(up % 60u));
    *p = '\0';

    p = Puts(s_frame[2], "I2C ");
    p = Puts(p, s_oled.isOnline() ? "ONLINE" : "OFFLINE");
    *p = '\0';

    p = Puts(s_frame[3], "UART RX ");
    p = Puti(p, (int32_t)UARTComm_GetRxBytes());
    *p = '\0';

    p = Puts(s_frame[4], "MODE ");
    p = Puts(p, (LineFollow_GetMode() == 0u) ? "LINE" : "MANUAL");
    *p = '\0';

    p = Puts(s_frame[5], "SAT ");
    p = Puti(p, LineFollow_GetSaturated());
    p = Puts(p, "  LOST ");
    p = Puti(p, LineFollow_GetLostCount());
    *p = '\0';

    p = Puts(s_frame[6], "PWM MAX ");
    p = Puti(p, PWM_MAX);
    *p = '\0';

    Puts(s_frame[7], "PAGE 3/4  KEY1>");
}

void RenderPage(uint8_t page)
{
    ClearFrame();
    switch (page) {
    case 0u: RenderPage0(); break;
    case 1u: RenderPage1(); break;
    case 2u: RenderPage2(); break;
    default: RenderPage3(); break;
    }
}

/* ---------------- 按键 ---------------- */

void ScanKey(uint32_t now)
{
    const uint8_t raw = (HAL_GPIO_ReadPin(GPIOA, KEY1_Pin) == GPIO_PIN_RESET) ? 0u : 1u;

    if (raw != s_key_raw) {
        s_key_raw  = raw;
        s_key_tick = now;
    }

    if ((uint32_t)(now - s_key_tick) >= kDebounceMs) {
        if (s_key_stable != s_key_raw) {
            s_key_stable = s_key_raw;
            if (s_key_stable == 0u) {              /* 按下沿 */
                s_page = (uint8_t)((s_page + 1u) % kPageCount);
            }
        }
    }
}

} /* anonymous namespace */

/* ---------------- 对外接口 ---------------- */

extern "C" void Ui_Init(void)
{
    s_page        = 0u;
    s_key_raw     = 1u;
    s_key_stable  = 1u;
    s_key_tick    = 0u;
    s_render_tick = 0u;
    s_retry_tick  = 0u;
    s_have_prev   = false;
    (void)s_oled.init();
}

extern "C" void Ui_Task(void)
{
    const uint32_t now = HAL_GetTick();

    ScanKey(now);

    if ((uint32_t)(now - s_render_tick) < kRenderPeriodMs) {
        return;
    }
    s_render_tick = now;

    /* 掉线自愈：每 2 秒重试一次 init() */
    if (!s_oled.isOnline()) {
        s_have_prev = false;   /* 重连后必须整屏重画，否则会被内容未变的判断跳过 */
        if ((uint32_t)(now - s_retry_tick) >= kRetryPeriodMs) {
            s_retry_tick = now;
            (void)s_oled.init();
        }
        return;
    }

    RenderPage(s_page);

    /* 内容和上一帧完全一样就跳过，省掉 24ms 的 I2C 传输 */
    if (s_have_prev && (std::memcmp(s_frame, s_prev, sizeof(s_frame)) == 0)) {
        return;
    }
    std::memcpy(s_prev, s_frame, sizeof(s_frame));
    s_have_prev = true;

    s_oled.clear();
    for (uint8_t r = 0u; r < 8u; r++) {
        if (s_frame[r][0] != '\0') {
            s_oled.showString((uint8_t)(r + 1u), 1u, s_frame[r]);
        }
    }
    s_oled.refresh();
}
