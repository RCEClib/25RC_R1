
/**
 ****************************************************************************************************
 * @file        smd.h
 * @author      正点原子(ALIENTEK)
 * @version     V1.0
 * @date        2025-05-27
 * @brief       SMD 驱动模块
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 官网: www.yuanzige.com
 * 论坛: www.openedv.com
 * 网址: www.alientek.com
 * 淘宝: openedv.taobao.com
 *
 * 修改记录:
 * V1.0 20250527
 * 初始版本
 *
 ****************************************************************************************************
 */

#ifndef __SMD_H
#define __SMD_H
#include <stdint.h>
#include "stdbool.h"
// #include "./SYSTEM/sys/sys.h"

#define COMM_TYPE               1           /* 0:串口 1:CAN */
#define CAN_EXTID               0x1000      /* CAN扩展ID，固定0x100x */

#define FRAME_HEAD              0xC5        /* 帧头 */
#define FRAME_TAIL              0x5C        /* 帧尾 */

/* 功能命令枚举 */
typedef enum
{
    /* 系统命令(0x00~0x0F) */
    FCT_IDLE                    = 0x00,      /* 空闲指令 */
    FCT_CAL_ENCODER             = 0x01,      /* 校正编码器 */
    FCT_RESTART                 = 0x02,      /* 重启设备 */
    FCT_RESET_FACTORY           = 0x03,      /* 恢复出厂设置 */
    FCT_PARAM_SAVE              = 0x04,      /* 参数保存 */

    /* 读取命令(0x20~0x3F) */
    FCT_READ_SOFT_HARD_VER      = 0x20,      /* 读取软硬件版本 */
    FCT_READ_PSI                = 0x21,      /* 读取相位电流 */
    FCT_READ_PHASE_RES_IND      = 0x22,      /* 读取相位电阻电感 */
    FCT_READ_PHASE_MA           = 0x23,      /* 读取相电流 */
    FCT_READ_VOL                = 0x24,      /* 读取母线电压 */
    FCT_READ_MA_PID             = 0x25,      /* 读取电流环PID参数 */
    FCT_READ_SPEED_PID          = 0x26,      /* 读取速度环PID参数 */
    FCT_READ_POS_PID            = 0x27,      /* 读取位置环PID参数 */
    FCT_READ_TOTAL_PULSE        = 0x28,      /* 读取编码器总脉冲 */
    FCT_READ_ROTATE_SPEED       = 0x29,      /* 读取电机转速 */
    FCT_READ_POS                = 0x2A,      /* 读取电机位置 */
    FCT_READ_POS_ERROR          = 0x2B,      /* 读取位置误差 */
    FCT_READ_MOTOR_STA          = 0x2C,      /* 读取电机状态 */
    FCT_READ_CLOG_FLAG          = 0x2D,      /* 读取堵转标志 */
    FCT_READ_CLOG_CUR           = 0x2E,      /* 读取堵转电流 */
    FCT_READ_ENABLE_STA         = 0x2F,      /* 读取使能状态 */
    FCT_READ_ARRIVED_STA        = 0x30,      /* 读取位置到达状态 */
    FCT_READ_SYS_PARAM          = 0x31,      /* 读取系统参数 */
    FCT_READ_DRIVE_PARAMS       = 0x32,      /* 读取驱动参数 */

    /* 设置命令(0x60~0x7F) */
    FCT_SET_SLAVE_ADD           = 0x60,      /* 设置从机地址 */
    FCT_SET_GROUP_ADD           = 0x61,      /* 设置组地址 */
    FCT_SET_MODE                = 0x62,      /* 设置运行模式 */
    FCT_SET_POS_PID             = 0x63,      /* 设置位置环PID */
    FCT_SET_POS_TORQUE          = 0x64,      /* 设置位置环最大力矩 */
    FCT_SET_STEP                = 0x65,      /* 设置步距角 */
    FCT_SET_MA                  = 0x66,      /* 设置电机额定电流 */
    FCT_SET_UART_BAUD           = 0x67,      /* 设置串口波特率 */
    FCT_SET_CAN_BAUD            = 0x68,      /* 设置CAN波特率 */
    FCT_SET_MODBUS              = 0x69,      /* 设置MODBUS参数 */
    FCT_SET_CLOG_PRO            = 0x6A,      /* 设置堵转保护 */
    FCT_SET_CLOG_CUR            = 0x6B,      /* 设置堵转电流阈值 */
    FCT_SET_CAN_ID              = 0x6C,      /* 设置CAN_ID */
    FCT_SET_DIR_LEVEL           = 0x6D,      /* 设置DIR方向电平 */
    FCT_SET_EN_LEVEL            = 0x6E,      /* 设置EN使能电平 */
    FCT_SET_CMD_ECHO            = 0x6F,      /* 设置命令回传 */
    FCT_SET_KEY_LOCK            = 0x70,      /* 设置按键锁定 */
    FCT_SET_AUTO_NOT_DISPLAY    = 0x71,      /* 设置自动熄屏 */
    FCT_SET_IO_START_LEVEL      = 0x72,      /* 设置IO启动电平 */
    FCT_SET_SPEED_PID           = 0x73,      /* 设置速度环PID */

    /* 回零/限位命令(0x90~0x9F) */
    FCT_ORIGIN_SET_LEFT_POS     = 0x90,      /* 设置左限位位置 */
    FCT_ORIGIN_LIMIT_HOME       = 0x91,      /* 限位回零 */
    FCT_ORIGIN_TRIG             = 0x92,      /* 触发回零 */
    FCT_ORIGIN_BREAK            = 0x93,      /* 中断回零/取消回零 */
    FCT_ORIGIN_READ_PARAMS      = 0x94,      /* 读取回零参数 */
    FCT_ORIGIN_SET_PARAMS       = 0x95,      /* 设置回零参数 */
    FCT_ORIGIN_READ_STA         = 0x96,      /* 读取回零状态 */
    FCT_ORIGIN_AOTO_ZERO        = 0x97,      /* 自动角度归零 */
    FCT_ORIGIN_SET_RIGHT_POS    = 0x98,      /* 设置右限位位置 */
    FCT_ORIGIN_SWITCH           = 0x99,      /* 左右限位开关 */

    /* 运动控制命令(0xE0~0xFF) */
    FCT_OL_SPEED_MODE           = 0xE0,      /* 开环速度模式 */
    FCT_OL_POS_MODE             = 0xE1,      /* 开环位置模式 */
    FCT_OL_POS_REL_MODE         = 0xE2,      /* 开环相对位置模式 */
    FCT_OL_PULSES_MODE          = 0xE3,      /* 开环脉冲模式 */

    FCT_IO_RUN_MODE             = 0xE4,      /* IO控制运行模式 */

    FCT_TORQUE_MODE             = 0xF0,      /* 力矩模式 */
    FCT_SPEED_MODE              = 0xF1,      /* 速度模式 */
    FCT_POS_MODE                = 0xF2,      /* 位置模式 */
    FCT_POS_REL_MODE            = 0xF3,      /* 相对位置模式 */
    FCT_PULSES_MODE             = 0xF4,      /* 脉冲模式 */
    FCT_PULSE_WIDTH_POS_MODE    = 0xF5,      /* 脉冲宽度位置模式 */
    FCT_PULSE_WIDTH_MA_MODE     = 0xF6,      /* 脉冲宽度电流模式 */
    FCT_PULSE_WIDTH_SPEED_MODE  = 0xF7,      /* 脉冲宽度速度模式 */
    FCT_ANGLE_ZERO              = 0xF8,      /* 角度归零 */
    FCT_CLEAR_CLOG_PRO          = 0xF9,      /* 清除堵转保护 */
    FCT_MOTOR_ENABLE            = 0xFA,      /* 电机使能 */
    FCT_CLEAR_STATE             = 0xFB,      /* 清除状态/故障 */
    FCT_STOP_NOW                = 0xFC,      /* 立即停止 */
}FUN_CODE_TYPE;

/**********************************************************
*** 对外函数声明
**********************************************************/
void smd_cal_encoder(uint8_t addr);
void smd_restart(uint8_t addr);
void smd_reset_factory(uint8_t addr);
void smd_param_save(uint8_t addr);

void smd_read_soft_hard_ver(uint8_t addr);
void smd_read_psi(uint8_t addr);
void smd_read_phase_res_ind(uint8_t addr);
void smd_read_phase_ma(uint8_t addr);
void smd_read_vol(uint8_t addr);
void smd_read_ma_pid(uint8_t addr);
void smd_read_speed_pid(uint8_t addr);
void smd_read_pos_pid(uint8_t addr);
void smd_read_tatal_pulse(uint8_t addr);
void smd_read_rotate_speed(uint8_t addr);
void smd_read_pos(uint8_t addr);
void smd_read_pos_error(uint8_t addr);
void smd_read_motor_sta(uint8_t addr);
void smd_read_clog_flag(uint8_t addr);
void smd_read_clog_current(uint8_t addr);
void smd_read_enable_sta(uint8_t addr);
void smd_read_arrived_sta(uint8_t addr);
void smd_read_sys_params(uint8_t addr);
void smd_read_drive_params(uint8_t addr);

void smd_set_slave_add(uint8_t addr, uint8_t new_addr);
void smd_set_group_add(uint8_t addr, uint8_t new_addr);
void smd_set_mode(uint8_t addr, uint8_t mode);
void smd_set_pos_pid(uint8_t addr, uint32_t kp, uint32_t ki, uint32_t kd);
void smd_set_pos_torque(uint8_t addr, int16_t torque);
void smd_set_step(uint8_t addr, uint16_t step);
void smd_set_ma(uint8_t addr, int16_t ma);
void smd_set_uart_baud(uint8_t addr, uint32_t baud);
void smd_set_can_baud(uint8_t addr, uint16_t baud);
void smd_set_modbus(uint8_t addr, uint8_t modbus);
void smd_set_clog_pro(uint8_t addr, uint8_t en);
void smd_set_clog_current(uint8_t addr, int16_t ma);
void smd_set_can_id(uint8_t addr, uint32_t id);
void smd_set_dir_level(uint8_t addr,uint8_t dir);
void smd_set_en_level(uint8_t addr,uint8_t en);
void smd_set_cmd_echo(uint8_t addr,uint8_t echo);
void smd_set_key_lock(uint8_t addr, uint8_t lock);
void smd_set_auto_not_display(uint8_t addr, uint8_t en);
void smd_set_io_start_level(uint8_t addr, uint8_t level);
void smd_set_speed_pid(uint8_t addr, uint32_t kp, uint32_t ki, uint32_t kd);

void smd_origin_set_left_pos(uint8_t addr, int32_t pos);
void smd_origin_homing_by_limit(uint8_t addr, uint8_t limit_enable, uint8_t dir, int32_t speed_rpm, int16_t curr_limit);
void smd_origin_trig(uint8_t addr, uint8_t mode);
void smd_origin_break(uint8_t addr);
void smd_origin_read_params(uint8_t addr);
void smd_origin_set_params(uint8_t addr, uint32_t timout);
void smd_origin_read_sta(uint8_t addr);
void smd_origin_aoto_zero(uint8_t addr, uint8_t flag);
void smd_origin_set_right_pos(uint8_t addr, int32_t pos);
void smd_origin_l_r_switch(uint8_t addr, uint8_t ctrl);

void smd_torque_mode(uint8_t addr, uint8_t dir, uint16_t current);
void smd_speed_mode(uint8_t addr, uint8_t dir, uint8_t acc, float speed);
void smd_pos_mode(uint8_t addr, uint8_t dir, uint8_t acc, uint16_t speed, uint32_t pulses);
void smd_pos_rel_mode(uint8_t addr, uint8_t dir, uint8_t acc, uint16_t speed, uint32_t pulses);
void smd_pulse_mode(uint8_t addr);
void smd_pulse_width_pos_mode(uint8_t addr, uint16_t topw_max, uint16_t topw_min, int32_t top_pos, int32_t down_pos);
void smd_pulse_width_ma_mode(uint8_t addr, uint16_t topw_max, uint16_t topw_min, int32_t top_ma, int32_t down_ma);
void smd_pulse_width_speed_mode(uint8_t addr, uint16_t topw_max, uint16_t topw_min, int32_t top_speed, int32_t down_speed);
void smd_ol_speed_mode(uint8_t addr, uint8_t dir, uint8_t acc, float speed);
void smd_ol_pos_mode(uint8_t addr, uint8_t dir, uint8_t acc, uint16_t speed, uint32_t pulses);
void smd_ol_pos_rel_mode(uint8_t addr, uint8_t dir, uint8_t acc, uint16_t speed, uint32_t pulses);
void smd_ol_pulse_mode(uint8_t addr);
void smd_io_run_ctrl(uint8_t addr, uint8_t dir, uint8_t acc, float speed);

void smd_angle_to_zero(uint8_t addr);
void smd_remove_clog_protect(uint8_t addr);
void smd_motor_enable(uint8_t addr, uint8_t en);
void smd_clear_sta(uint8_t addr);
void smd_stop_now(uint8_t addr);

uint8_t smd_checksum(const uint8_t *data, uint8_t length);

#endif


