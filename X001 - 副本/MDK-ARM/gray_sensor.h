#ifndef GRAY_SENSOR_H
#define GRAY_SENSOR_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 6 路灰度传感器 */
#define GRAY_NUM 6

/* 黑线对应的电平：模块"黑线输出高电平"用 GPIO_PIN_SET，"黑线输出低电平"改成 GPIO_PIN_RESET */
#define GRAY_BLACK_LEVEL GPIO_PIN_SET

/*
 * 引脚按"车头看过去 从左到右"的顺序排列：
 *   bit0 = 最左 -> PA12
 *   bit1        -> PB8
 *   bit2        -> PB9
 *   bit3        -> PC13
 *   bit4        -> PC14
 *   bit5 = 最右 -> PC15
 *
 * 这几路在 CubeMX 里已配置为 GPIO_Input + 上拉。
 * 如果装车后左右方向反了，把 gray_sensor.c 里的 weights[] 取反即可。
 */

/* 读原始值：bit0~bit5 对应 6 路探头，1 表示检测到黑线 */
uint8_t Gray_ReadRaw(void);

/*
 * 从一份原始值算偏差（推荐用法）。
 * 一个控制周期只读一次 GPIO，后面的判断全用同一份 raw，
 * 避免多次采样不一致导致的"这拍有线、下拍丢线"抖动。
 */
int16_t Gray_ErrorFromRaw(uint8_t raw);     /* 加权偏差：左负右正，-15 ~ +15；全白返回 0 */
uint8_t Gray_AllWhite(uint8_t raw);
uint8_t Gray_AllBlack(uint8_t raw);

/* 兼容旧接口：内部自己重新读一次 GPIO */
int16_t Gray_GetError(void);
uint8_t Gray_IsAllWhite(void);
uint8_t Gray_IsAllBlack(void);

#ifdef __cplusplus
}
#endif

#endif /* GRAY_SENSOR_H */
