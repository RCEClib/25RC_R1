#ifndef __AOTO_H
#define __AOTO_H

#include <stdio.h>
#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 固件初始化，注册串口、设置默认调参模式
 */
void Firmware_Init(void);

/**
 * @brief 主循环中调用的任务，处理串口命令（非阻塞）
 */
void Firmware_Loop(void);


#ifdef __cplusplus
}
#endif

#endif