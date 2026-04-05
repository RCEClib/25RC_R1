#include "ws2812.h"

// WS2812时序电平定义
// 0码：高电平0.35μs + 低电平0.8μs（对应0xC0）
// 1码：高电平0.7μs + 低电平0.6μs（对应0xF0）
#define WS2812_LowLevel    0xC0       // 0码电平
#define WS2812_HighLevel   0xF0       // 1码电平

/**
 * @brief WS2812 LED颜色控制函数
 * @param r: 红色分量 (0-255)
 * @param g: 绿色分量 (0-255)
 * @param b: 蓝色分量 (0-255)
 * @note WS2812数据传输格式：G-R-B（绿色在前）
 */
void WS2812_Ctrl(uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t txbuf[24];      // 24字节缓冲区，每个颜色分量8位
    uint8_t res = 0;        // 复位信号
    // 将RGB颜色值转换为WS2812的SPI时序数据
    for (int i = 0; i < 8; i++)
    {
        // 绿色分量（G） - 先传输
        txbuf[7-i]  = (((g>>i)&0x01) ? WS2812_HighLevel : WS2812_LowLevel)>>1;
        // 红色分量（R） - 中间传输
        txbuf[15-i] = (((r>>i)&0x01) ? WS2812_HighLevel : WS2812_LowLevel)>>1;
        // 蓝色分量（B） - 最后传输
        txbuf[23-i] = (((b>>i)&0x01) ? WS2812_HighLevel : WS2812_LowLevel)>>1;
    }
    // 发送复位信号（50μs以上的低电平）
    HAL_SPI_Transmit(&WS2812_SPI_UNIT, &res, 0, 0xFFFF);
    while (WS2812_SPI_UNIT.State != HAL_SPI_STATE_READY);
    // 发送颜色数据（24字节）
    HAL_SPI_Transmit(&WS2812_SPI_UNIT, txbuf, 24, 0xFFFF);
    // 发送额外的复位信号确保数据传输完成
    for (int i = 0; i < 100; i++)
    {
        HAL_SPI_Transmit(&WS2812_SPI_UNIT, &res, 1, 0xFFFF);
    }
}