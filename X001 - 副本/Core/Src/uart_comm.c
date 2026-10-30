#include "uart_comm.h"
#include "protocol.h"

static UART_HandleTypeDef *s_uart     = 0;
static uint8_t             s_rx_byte  = 0;
static uint32_t            s_rx_bytes = 0;

/* 中断发送用的静态缓冲：必须活到发送结束，不能用局部数组 */
static uint8_t s_tx_buf[2U + 2U + PROTOCOL_MAX_DATA_LEN + 1U];

void UARTComm_Init(UART_HandleTypeDef *huart)
{
    s_uart = huart;
}

void UARTComm_StartReceive(void)
{
    if (s_uart != 0) {
        (void)HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1U);
    }
}

uint32_t UARTComm_GetRxBytes(void)
{
    return s_rx_bytes;
}

HAL_StatusTypeDef UARTComm_SendFrame(uint8_t cmd, uint8_t len, const uint8_t *data)
{
    uint16_t frame_len;
    uint8_t  i;

    if ((s_uart == 0) || (len > PROTOCOL_MAX_DATA_LEN)) {
        return HAL_ERROR;
    }

    /* 上一帧还没发完就丢弃这一帧，绝不阻塞（本函数可能跑在接收中断里） */
    if (s_uart->gState != HAL_UART_STATE_READY) {
        return HAL_BUSY;
    }

    s_tx_buf[0] = PROTOCOL_HEADER_1;
    s_tx_buf[1] = PROTOCOL_HEADER_2;
    s_tx_buf[2] = cmd;
    s_tx_buf[3] = len;

    for (i = 0; i < len; i++) {
        s_tx_buf[4U + i] = data[i];
    }

    s_tx_buf[4U + len] = Protocol_CalcChecksum(cmd, len, data);
    frame_len = (uint16_t)(5U + len);

    return HAL_UART_Transmit_IT(s_uart, s_tx_buf, frame_len);
}

/* 收到一个字节：计数 + 交给协议组帧 + 立刻重新挂接收 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((s_uart != 0) && (huart->Instance == s_uart->Instance)) {
        s_rx_bytes++;
        Protocol_RxByte(s_rx_byte);
        (void)HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1U);
    }
}

/* 出错（溢出/噪声/帧错误）后重新挂接收，否则串口会永久收不到东西 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if ((s_uart != 0) && (huart->Instance == s_uart->Instance)) {
        (void)HAL_UART_AbortReceive(s_uart);
        (void)HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1U);
    }
}
