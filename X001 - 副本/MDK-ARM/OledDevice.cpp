/* OledDevice.cpp —— [设备层] OLED 显示屏类实现(SSD1306 128x64, I2C) */
#include "OledDevice.hpp"
#include "oled_font.h"
#include <cstring>

namespace {

const uint8_t  kCtrlCmd    = 0x00u;   /* 控制字节: 命令 */
const uint8_t  kCtrlData   = 0x40u;   /* 控制字节: 数据 */
const uint32_t kI2cTimeout = 20u;     /* ms */

/* SSD1306 128x64 初始化序列 */
const uint8_t kInitCmds[] = {
    0xAEu,              /* 关显示                     */
    0x20u, 0x02u,       /* 页寻址模式                 */
    0xB0u,              /* 起始页 0                   */
    0xC8u,              /* COM 反向扫描(屏幕正向)     */
    0x00u, 0x10u,       /* 列地址低/高 4 位 = 0       */
    0x40u,              /* 显示起始行 0               */
    0x81u, 0x7Fu,       /* 对比度                     */
    0xA1u,              /* 段重映射(左右正向)         */
    0xA6u,              /* 正常显示                   */
    0xA8u, 0x3Fu,       /* 复用率 1/64                */
    0xA4u,              /* 显示跟随 RAM               */
    0xD3u, 0x00u,       /* 显示偏移 0                 */
    0xD5u, 0x80u,       /* 时钟分频/振荡频率          */
    0xD9u, 0xF1u,       /* 预充电周期                 */
    0xDAu, 0x12u,       /* COM 硬件配置(128x64 交替)  */
    0xDBu, 0x40u,       /* VCOMH 反压                 */
    0x8Du, 0x14u,       /* 电荷泵使能                 */
    0xAFu               /* 开显示                     */
};

} /* anonymous namespace */

/* ==========================================================================
 * 初始化
 * ========================================================================== */
bool OledDevice::init()
{
    /* SSD1306 上电后需要 >100ms 让内部电荷泵稳定。
       这是一次性的上电等待, 与 500ms 采样节拍无关, 不违反"不得用 delay 定节拍"。 */
    HAL_Delay(120u);

    /* 先探测器件地址, 没插屏就直接返回, 不阻塞主程序 */
    if (HAL_I2C_IsDeviceReady(hi2c_, dev_addr_, 2u, 10u) != HAL_OK) {
        online_ = false;
        return false;
    }

    for (uint32_t i = 0u; i < sizeof(kInitCmds) / sizeof(kInitCmds[0]); i++) {
        if (!writeCmd(kInitCmds[i])) {
            online_ = false;
            return false;
        }
    }

    online_ = true;
    clear();
    refresh();
    return true;
}

/* ==========================================================================
 * 底层 I2C
 * ========================================================================== */
bool OledDevice::writeCmd(uint8_t cmd)
{
    if (HAL_I2C_Mem_Write(hi2c_, dev_addr_, kCtrlCmd, I2C_MEMADD_SIZE_8BIT,
                          &cmd, 1u, kI2cTimeout) != HAL_OK) {
        online_ = false;
        return false;
    }
    return true;
}

bool OledDevice::writeData(const uint8_t* data, uint16_t len)
{
    if ((data == NULL) || (len == 0u)) {
        return true;
    }
    /* 这里直接把显存指针交给 HAL, 不做临时缓冲区拷贝, 也就不需要在
       单片机上使用 new/delete 动态内存 */
    if (HAL_I2C_Mem_Write(hi2c_, dev_addr_, kCtrlData, I2C_MEMADD_SIZE_8BIT,
                          const_cast<uint8_t*>(data), len, kI2cTimeout) != HAL_OK) {
        online_ = false;
        return false;
    }
    return true;
}

/* ==========================================================================
 * 显存与刷屏
 * ========================================================================== */
void OledDevice::clear()
{
    std::memset(buffer_, 0x00, sizeof(buffer_));
}

void OledDevice::refresh()
{
    if (!online_) {
        return;                     /* 屏幕不在线就静默跳过, 不拖慢主循环 */
    }

    for (uint8_t page = 0u; page < kPages; page++) {
        (void)writeCmd(static_cast<uint8_t>(0xB0u + page));
        (void)writeCmd(0x00u);
        (void)writeCmd(0x10u);
        if (!writeData(&buffer_[static_cast<uint32_t>(page) * kWidth], kWidth)) {
            return;
        }
    }
}

/* ==========================================================================
 * 绘制
 * ========================================================================== */
void OledDevice::drawChar(uint8_t page, uint8_t index, char ch)
{
    if (page >= static_cast<uint8_t>(kRows)) {
        return;
    }

    uint8_t code = static_cast<uint8_t>(ch);
    if ((code < OLED_FONT_FIRST_CHAR) || (code > OLED_FONT_LAST_CHAR)) {
        code = static_cast<uint8_t>('?');    /* 字库外字符统一显示 '?' */
    }

    const uint8_t* glyph = OLED_F8x6[code - OLED_FONT_FIRST_CHAR];
    const uint32_t base  = static_cast<uint32_t>(page) * kWidth +
                           static_cast<uint32_t>(index) * kFontW;

    for (uint8_t x = 0u; x < static_cast<uint8_t>(kFontW); x++) {
        const uint32_t idx = base + x;
        if (idx < static_cast<uint32_t>(kBufSize)) {
            buffer_[idx] = glyph[x];
        }
    }
}

void OledDevice::showString(uint8_t row, uint8_t col, const char* str)
{
    if ((str == NULL) || (col == 0u) || (row == 0u)) {
        return;
    }

    /* 对外 row 从 1 开始, 内部 drawChar 的页号从 0 开始 */
    const uint8_t page = static_cast<uint8_t>(row - 1u);
    if (page >= static_cast<uint8_t>(kRows)) {
        return;
    }

    uint8_t index = static_cast<uint8_t>(col - 1u);   /* 转成 0 基列号 */
    while ((*str != '\0') && (index < static_cast<uint8_t>(kCols))) {
        drawChar(page, index, *str);
        str++;
        index++;
    }
}

void OledDevice::showNum(uint8_t row, uint8_t col, uint32_t num, uint8_t len)
{
    char buf[12];

    if (len > (sizeof(buf) - 1u)) {
        len = static_cast<uint8_t>(sizeof(buf) - 1u);
    }

    /* 右对齐零填充: len = 3 时 7 -> "007" */
    for (uint8_t i = len; i > 0u; i--) {
        buf[i - 1u] = static_cast<char>('0' + (num % 10u));
        num /= 10u;
    }
    buf[len] = '\0';

    showString(row, col, buf);
}
