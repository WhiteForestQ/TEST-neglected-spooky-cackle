#include "protocol.h"
#include "line_follow.h"
#include "motor.h"
#include "encoder.h"
#include "uart_comm.h"

/*
 * 收帧状态机。全部在串口接收中断里跑，只做"组帧 + 分发"，
 * 不做任何阻塞操作（应答用中断发送，见 uart_comm.c）。
 */
typedef enum
{
    RX_WAIT_HEADER_1 = 0,
    RX_WAIT_HEADER_2,
    RX_WAIT_CMD,
    RX_WAIT_LEN,
    RX_WAIT_DATA,
    RX_WAIT_CHECK
} RxState_t;

static RxState_t s_rx_state = RX_WAIT_HEADER_1;
static uint8_t s_cmd;
static uint8_t s_len;
static uint8_t s_index;
static uint8_t s_data[PROTOCOL_MAX_DATA_LEN];

uint8_t Protocol_CalcChecksum(uint8_t cmd, uint8_t len, const uint8_t *data)
{
    uint8_t checksum = (uint8_t)(cmd ^ len);
    uint8_t i;

    for (i = 0; i < len; i++)
        checksum ^= data[i];

    return checksum;
}

void Protocol_Init(void)
{
    s_rx_state = RX_WAIT_HEADER_1;
    s_cmd = 0;
    s_len = 0;
    s_index = 0;
}

void Protocol_RxByte(uint8_t byte)
{
    switch (s_rx_state)
    {
        case RX_WAIT_HEADER_1:
            if (byte == PROTOCOL_HEADER_1)
                s_rx_state = RX_WAIT_HEADER_2;
            break;

        case RX_WAIT_HEADER_2:
            if (byte == PROTOCOL_HEADER_2)
                s_rx_state = RX_WAIT_CMD;
            else if (byte == PROTOCOL_HEADER_1)
                s_rx_state = RX_WAIT_HEADER_2;
            else
                s_rx_state = RX_WAIT_HEADER_1;
            break;

        case RX_WAIT_CMD:
            s_cmd = byte;
            s_rx_state = RX_WAIT_LEN;
            break;

        case RX_WAIT_LEN:
            s_len = byte;
            s_index = 0;
            if (s_len > PROTOCOL_MAX_DATA_LEN)
            {
                s_rx_state = RX_WAIT_HEADER_1;
            }
            else if (s_len == 0)
            {
                s_rx_state = RX_WAIT_CHECK;
            }
            else
            {
                s_rx_state = RX_WAIT_DATA;
            }
            break;

        case RX_WAIT_DATA:
            s_data[s_index++] = byte;
            if (s_index >= s_len)
                s_rx_state = RX_WAIT_CHECK;
            break;

        case RX_WAIT_CHECK:
            if (byte == Protocol_CalcChecksum(s_cmd, s_len, s_data))
                Protocol_HandleFrame(s_cmd, s_len, s_data);

            /* 不管校验对不对都复位；如果这一字节正好是帧头就接着收 */
            s_rx_state = (byte == PROTOCOL_HEADER_1)
                       ? RX_WAIT_HEADER_2
                       : RX_WAIT_HEADER_1;
            break;

        default:
            s_rx_state = RX_WAIT_HEADER_1;
            break;
    }
}

static int16_t ReadInt16LE(const uint8_t *p)
{
    uint16_t raw = (uint16_t)p[0] | ((uint16_t)p[1] << 8);
    return (int16_t)raw;
}

static void PutInt16LE(uint8_t *p, int16_t v)
{
    p[0] = (uint8_t)((uint16_t)v & 0xFFu);
    p[1] = (uint8_t)(((uint16_t)v >> 8) & 0xFFu);
}

static void SendStatus(uint8_t request_cmd, ProtocolStatus_t status)
{
    uint8_t response_data[1];
    response_data[0] = (uint8_t)status;
    (void)UARTComm_SendFrame((uint8_t)(request_cmd | 0x80U), 1U, response_data);
}

/*
 * 这里跑在串口中断里。TIM4 控制中断和串口中断优先级相同（都是 0），
 * 所以两者不会互相打断，直接调用控制层是安全的。
 */
void Protocol_HandleFrame(uint8_t cmd, uint8_t len, const uint8_t *data)
{
    ProtocolStatus_t status = PROTO_STATUS_OK;

    switch (cmd)
    {
        case CMD_MOTOR_SET_PWM:
            if (len != 4U)
            {
                status = PROTO_STATUS_BAD_LENGTH;
                break;
            }
            /* 切到手动模式：直接按 PWM 驱动（带 500ms 超时保护） */
            LineFollow_SetManualPwm(ReadInt16LE(&data[0]), ReadInt16LE(&data[2]));
            break;

        case CMD_MOTOR_STOP:
            if (len != 0U)
            {
                status = PROTO_STATUS_BAD_LENGTH;
                break;
            }
            LineFollow_Stop();
            break;

        case CMD_LINE_START:
            if (len != 0U)
            {
                status = PROTO_STATUS_BAD_LENGTH;
                break;
            }
            LineFollow_Start();
            break;

        case CMD_LINE_STOP:
            if (len != 0U)
            {
                status = PROTO_STATUS_BAD_LENGTH;
                break;
            }
            LineFollow_Stop();
            break;

        case CMD_STATUS_QUERY:
        {
            uint8_t status_data[9];

            if (len != 0U)
            {
                status = PROTO_STATUS_BAD_LENGTH;
                break;
            }

            /* [status, mode, state, error int16, speed_L int16, speed_R int16] */
            status_data[0] = (uint8_t)PROTO_STATUS_OK;
            status_data[1] = LineFollow_GetMode();
            status_data[2] = LineFollow_GetState();
            PutInt16LE(&status_data[3], LineFollow_GetError());
            PutInt16LE(&status_data[5], speed_left);
            PutInt16LE(&status_data[7], speed_right);
            (void)UARTComm_SendFrame((uint8_t)(cmd | 0x80U), 9U, status_data);
            return;
        }

        default:
            status = PROTO_STATUS_UNKNOWN_CMD;
            break;
    }

    SendStatus(cmd, status);
}
