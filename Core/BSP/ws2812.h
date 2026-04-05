#ifndef __WS2812_H__
#define __WS2812_H__
 
 
#include "main.h" 

// 定义WS2812使用的SPI外设（hspi6）
#define WS2812_SPI_UNIT     hspi6
extern SPI_HandleTypeDef WS2812_SPI_UNIT;

// WS2812控制函数声明
void WS2812_Ctrl(uint8_t r, uint8_t g, uint8_t b);
#endif
