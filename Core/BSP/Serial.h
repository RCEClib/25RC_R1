#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdio.h>

// 添加UART10相关函数声明
void UART7_Init(void);
uint8_t UART7_GetRxFlag(void);
uint8_t UART7_GetRxData(void);

void Serial_Init(void);
void Serial_SendByte(uint8_t Byte);
void Serial_SendArray(uint8_t *Array, uint16_t Length);
void Serial_SendString(char *String);
void Serial_SendNumber(uint32_t Number, uint8_t Length);
void Serial_Printf(char *format, ...);

uint8_t Serial_GetRxFlag(void);
uint8_t Serial_GetRxData(void);

#endif
