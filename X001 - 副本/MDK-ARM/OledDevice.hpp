/* OledDevice.hpp —— [设备层] OLED 显示屏类(SSD1306 128x64, I2C) */
#ifndef OLED_DEVICE_HPP
#define OLED_DEVICE_HPP

#include "stm32f1xx_hal.h"
#include <cstdint>

class OledDevice {
public:
    /* 用 enum 而不是 static const, 保证常量一定能参与编译期常量折叠 */
    enum {
        kWidth   = 128,
        kHeight  = 64,
        kPages   = 8,                     /* 64 / 8 */
        kFontW   = 6,                     /* 6x8 字库, 见 oled_font.h */
        kFontH   = 8,
        kCols    = kWidth / kFontW,       /* 每行 21 个字符 */
        kRows    = kPages,                /* 共 8 行(每行高 8 像素) */
        kBufSize = kWidth * kHeight / 8   /* 1024 字节显存 */
    };

    explicit OledDevice(I2C_HandleTypeDef* hi2c, uint8_t addr = 0x3Cu)
        : hi2c_(hi2c),
          dev_addr_(static_cast<uint16_t>(addr) << 1),
          online_(false)
    {
        clear();
    }

    /* 发送 SSD1306 初始化序列; 屏幕没插/接线错误时返回 false,
       之后所有显示操作自动跳过, 不影响调光主功能 */
    bool init();
    bool isOnline() const { return online_; }

    /* ---------------- 显存操作(画完要 refresh 才会上屏) ---------------- */
    void clear();

    /* ---------------- 文本显示 ----------------
       row: 1 ~ 8 (第几页), col: 1 ~ 21 (第几个字符) */
    void showString(uint8_t row, uint8_t col, const char* str);
    void showNum(uint8_t row, uint8_t col, uint32_t num, uint8_t len);

    /* 把显存整块刷到屏幕 */
    void refresh();

private:
    /* page: 0 ~ 7 (页号, 对外 row-1); index: 0 ~ 20 (列号, 对外 col-1) */
    void drawChar(uint8_t page, uint8_t index, char ch);
    bool writeCmd(uint8_t cmd);
    bool writeData(const uint8_t* data, uint16_t len);

    I2C_HandleTypeDef* hi2c_;
    uint16_t           dev_addr_;   /* 8 位地址(含读写位) */
    bool               online_;     /* 屏幕是否在线(通信是否成功过) */
    uint8_t            buffer_[kBufSize];
};

#endif /* OLED_DEVICE_HPP */
