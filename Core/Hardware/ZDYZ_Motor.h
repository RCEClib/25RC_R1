#ifndef MOTOR_API_H
#define MOTOR_API_H

#include <stdint.h>
#include <stdbool.h>
#include "ZDYZ_smd.h"          /* 正点原子官方驱动 */
#include "fd.h"        /* 获取到位状态 */
#define MOTOR_ADDR   1

// 用户协议接收缓冲区
#define MYCAN_RECV_BUF_LEN 256
typedef struct {
    uint8_t buf[MYCAN_RECV_BUF_LEN];
    uint16_t index;
    volatile bool frame_done;
} MyCAN_Frame_t;

extern MyCAN_Frame_t g_mycan_frame;


void ZDYZ_Init(void);

uint8_t can_send_long_msg( FDCAN_HandleTypeDef *hfdcan, uint32_t id,uint32_t type, uint8_t *data, uint16_t len);

void handle_can_rx(FDCAN_HandleTypeDef *hfdcan,uint8_t fifo);

void parse_frame(uint8_t *buf, uint16_t len);

uint8_t calc_checksum(const uint8_t *buf, uint16_t len);

bool motor_is_arrived(void);

uint8_t motor_get_last_error(void);


// 查询电机是否到达目标
bool MyCAN_MotorIsArrived(void);

uint8_t MyCAN_GetLastError(void);






/* 设置细分（内部记录，并发送指令给驱动器）*/
void motor_set_microstep(uint8_t addr, uint16_t step);

/* 绝对位置模式：转到指定角度（0° 为原点，正方向）*/
void motor_go_to_angle(uint8_t addr, float angle_deg, uint16_t speed_rpm, uint8_t acc);

/* 相对位置模式：从当前位置转动指定角度（正数正转，负数反转）*/
void motor_rotate(uint8_t addr, float angle_deg, uint16_t speed_rpm, uint8_t acc);

/* 相对位置模式：转动指定圈数（正数正转，负数反转）*/
void motor_rel_turns(uint8_t addr, float turns, uint16_t speed_rpm, uint8_t acc);


/////绝对位置模式，指定圈数
void motor_abs_turns(uint8_t addr, float turns, uint16_t speed_rpm, uint8_t acc);


// 电机初始化：细分、使能、限位、回零、自动回零、保存参数
void motor_init_auto_zero(uint8_t addr);

/* 等待电机到位（超时返回 false）*/
bool motor_wait_arrived(uint32_t timeout_ms);

#endif

