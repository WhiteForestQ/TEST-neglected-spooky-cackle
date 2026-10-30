#ifndef UI_H
#define UI_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * OLED 调试界面（OledDevice 之上的一层）
 *
 * 设计原则：
 *   1. 只读：本层只调用 LineFollow_GetXxx / speed_left 这类只读接口，
 *      不参与控制、不写任何控制变量；
 *   2. 异步：控制环在 TIM4 中断里跑，本层在主循环里跑，互不干扰。
 *      刷一屏约 24ms，放在主循环里最多让画面慢一点，不会拖累控制；
 *   3. 省 I2C：10Hz 刷新，并且"这一帧内容和上一帧完全一样就不刷"，
 *      车停着的时候 I2C 总线完全空闲；
 *   4. 自愈：屏幕掉线/热插拔后每 2 秒自动重连，不影响跑车。
 *
 * 用法：
 *   Ui_Init();                 // main 初始化完外设后调一次
 *   while (1) { Ui_Task(); }   // 主循环里每圈调一次（内部自己节流）
 *
 * 翻页：KEY1(PA3) 短按循环切换 4 页。
 */
void Ui_Init(void);
void Ui_Task(void);

#ifdef __cplusplus
}
#endif

#endif /* UI_H */
