#ifndef ENCODER_H
#define ENCODER_H

#include "main.h"
#include "tim.h"

// 加上 extern "C" 保护
#ifdef __cplusplus
extern "C" {
#endif
	
// 声明两个全局变量，用来保存左右轮的速度（脉冲数/10ms）
extern volatile int16_t speed_left;
extern volatile int16_t speed_right;

void Encoder_Init(void);
void Encoder_Update(void); // 这个函数要放在 10ms 定时器中断里调用

// 结束保护
#ifdef __cplusplus
}
#endif
#endif /* ENCODER_H */
