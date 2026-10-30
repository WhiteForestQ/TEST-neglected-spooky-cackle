#include "motor.h"

extern TIM_HandleTypeDef htim1;

/*
 * 方向引脚（接驱动板 AIN1/AIN2、BIN1/BIN2）：
 *   左电机：PB12 = AIN1, PB13 = AIN2
 *   右电机：PB14 = BIN1, PB15 = BIN2
 *
 * 注意：PB0/PB1 是舵机，PB3 是右编码器 TIM2_CH2，PB2 没配置，
 *       都不能当电机方向脚。上面这 4 个脚在 CubeMX 里已配成推挽输出。
 *
 * 真值表（某一通道）：
 *   正转： AIN1 = 1, AIN2 = 0, PWM = duty
 *   反转： AIN1 = 0, AIN2 = 1, PWM = duty
 *   停止： AIN1 = 0, AIN2 = 0, PWM = 0   （驱动板进入滑行态）
 *   刹车： AIN1 = 1, AIN2 = 1, PWM = 0   （本工程未使用，停车用滑行更温和）
 */
#define L_IN1_PORT GPIOB
#define L_IN1_PIN  GPIO_PIN_12
#define L_IN2_PORT GPIOB
#define L_IN2_PIN  GPIO_PIN_13

#define R_IN1_PORT GPIOB
#define R_IN1_PIN  GPIO_PIN_14
#define R_IN2_PORT GPIOB
#define R_IN2_PIN  GPIO_PIN_15

/*
 * 死区补偿：PWM 小于这个值时电机不转（约 5%）。
 * 实测方法：从 0 慢慢加 CCR，记下轮子刚开始转的那个值。
 */
#define MOTOR_DEADZONE 200

static void Motor_SetOne(int16_t duty,
                         uint32_t channel,
                         GPIO_TypeDef* in1_port, uint16_t in1_pin,
                         GPIO_TypeDef* in2_port, uint16_t in2_pin)
{
    /* 限幅，防止越界写 CCR */
    if (duty > PWM_MAX) {
        duty = PWM_MAX;
    }
    if (duty < -PWM_MAX) {
        duty = -PWM_MAX;
    }

    /*
     * 死区补偿：正反两个方向必须都补。
     * 原来只补了反向（正向那段被注释掉了），结果 +1 不动、-1 猛转，
     * 两轮不对称，速度环在 0 附近会一直把车往一边带。
     *
     * 补偿后 duty=0 仍然是 0（滑行），不会自己跑起来。
     */
    if (duty > 0) {
        duty = (int16_t)(MOTOR_DEADZONE +
                         ((int32_t)duty * (PWM_MAX - MOTOR_DEADZONE)) / PWM_MAX);
    } else if (duty < 0) {
        duty = (int16_t)(-MOTOR_DEADZONE +
                         ((int32_t)duty * (PWM_MAX - MOTOR_DEADZONE)) / PWM_MAX);
    }

    /* 决定轮子正反转 */
    if (duty >= 0) {
        HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_SET);
        duty = (int16_t)(-duty);
    }

    /* 写入寄存器 */
    __HAL_TIM_SET_COMPARE(&htim1, channel, (uint32_t)duty);
}

void Motor_Init(void)
{
    /* TIM1 是高级定时器，HAL_TIM_PWM_Start 内部会自动打开 MOE */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    Motor_Stop();
}

void Motor_Set(int16_t left, int16_t right)
{
    Motor_SetOne(left, TIM_CHANNEL_1,
                 L_IN1_PORT, L_IN1_PIN,
                 L_IN2_PORT, L_IN2_PIN);

    Motor_SetOne(right, TIM_CHANNEL_4,
                 R_IN1_PORT, R_IN1_PIN,
                 R_IN2_PORT, R_IN2_PIN);
}

void Motor_Stop(void)
{
    Motor_Set(0, 0);
}
