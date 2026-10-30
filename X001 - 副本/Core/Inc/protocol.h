#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

/*
 * 上位机 <-> 下位机 的调试协议（二进制、带校验）
 *
 * 帧格式：
 *   [0] 0xAA  [1] 0x55  [2] CMD  [3] LEN  [4..] DATA  [last] CHECK
 *   CHECK = XOR(CMD, LEN, 全部 DATA 字节)
 *   LEN 最大 32
 *
 * 应答：
 *   CMD | 0x80，DATA[0] = 状态码
 *
 * 注意：串口链路默认走 USART3（PB10/PB11），不要和 USART1 上的
 *       ASCII 调试帧混在一起；如果只有一个 USB-TTL，见 main.c 里的说明。
 */
#define PROTOCOL_HEADER_1       0xAA
#define PROTOCOL_HEADER_2       0x55
#define PROTOCOL_MAX_DATA_LEN   32U

typedef enum
{
    /* 手动模式：直接把两个轮子按 PWM 驱动（-3599 ~ +3599），用于台架测试。
       DATA: left int16 LE, right int16 LE
       注意：500ms 内没有新命令会自动停车（见 line_follow.cpp）。 */
    CMD_MOTOR_SET_PWM   = 0x01,
    CMD_MOTOR_STOP      = 0x02, /* DATA: none */
    CMD_LINE_START      = 0x03, /* DATA: none，重新武装循迹：等线 -> 自动起步 */
    CMD_LINE_STOP       = 0x04, /* DATA: none */
    CMD_STATUS_QUERY    = 0x20  /* DATA: none */
} ProtocolCommand_t;

typedef enum
{
    PROTO_STATUS_OK             = 0x00,
    PROTO_STATUS_BAD_LENGTH     = 0x01,
    PROTO_STATUS_BAD_ARGUMENT   = 0x02,
    PROTO_STATUS_UNKNOWN_CMD    = 0x03
} ProtocolStatus_t;

#ifdef __cplusplus
extern "C" {
#endif

void Protocol_Init(void);
void Protocol_RxByte(uint8_t byte);
void Protocol_HandleFrame(uint8_t cmd, uint8_t len, const uint8_t *data);
uint8_t Protocol_CalcChecksum(uint8_t cmd, uint8_t len, const uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif
