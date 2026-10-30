#include "gray_sensor.h"

typedef struct {
    GPIO_TypeDef* port;
    uint16_t      pin;
} GrayPin;

/*
 * 与实际接线一致（对应 CubeMX 里 PA12 / PB8 / PB9 / PC13 / PC14 / PC15）。
 * 注意：PA0~PA5 是 LED/BUZZER/KEY，读不到灰度模块，别改回去。
 */
static const GrayPin gray_pins[GRAY_NUM] = {
    {GPIOA, GPIO_PIN_12},
    {GPIOB, GPIO_PIN_8},
    {GPIOB, GPIO_PIN_9},
    {GPIOC, GPIO_PIN_13},
    {GPIOC, GPIO_PIN_14},
    {GPIOC, GPIO_PIN_15},
};

/* 6 路权重（左负右正，中间对称） */
static const int16_t weights[GRAY_NUM] = {
    -15, -9, -3, 3, 9, 15
};

uint8_t Gray_ReadRaw(void)  //位图代替 6 个变量
{
    uint8_t raw = 0;
    int  i;

    for (i = 0; i < GRAY_NUM; i++) {
        if (HAL_GPIO_ReadPin(gray_pins[i].port, gray_pins[i].pin) == GRAY_BLACK_LEVEL) {
            raw |= (uint8_t)(1u << i);
        }
    }
    return raw;
}

int16_t Gray_ErrorFromRaw(uint8_t raw) 
{
    int32_t sum = 0;
    int cnt = 0;
    int i;

    for (i = 0; i < GRAY_NUM; i++) {
        if (raw & (uint8_t)(1u << i)) {
            sum += weights[i];
            cnt++;
        }
    }

    /*
     * 全白（丢线）这里只返回 0，具体往哪边找由 line_follow 的状态机决定：
     * 它记得最后一次看到线的方向，比在这里猜更靠谱。
     */
    if (cnt == 0) {
        return 0;
    }

    return (int16_t)(sum / cnt);
}

uint8_t Gray_AllWhite(uint8_t raw)
{
    return (uint8_t)(raw == 0u);
}

uint8_t Gray_AllBlack(uint8_t raw)
{
    /* 全黑一般是十字路口 / 粗线，此时加权平均刚好是 0，按"走直"处理 */
    return (uint8_t)(raw == 0x3Fu);
}

/* ---- 兼容旧接口 ---- */

int16_t Gray_GetError(void)
{
    return Gray_ErrorFromRaw(Gray_ReadRaw());
}

uint8_t Gray_IsAllWhite(void)
{
    return Gray_AllWhite(Gray_ReadRaw());
}

uint8_t Gray_IsAllBlack(void)
{
    return Gray_AllBlack(Gray_ReadRaw());
}
