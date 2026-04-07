/**
 * ============================================================================
 * firmware.c - LLM PID 自动调参适配层 (STM32H7 + 真实底盘)
 * ============================================================================
 * 功能：
 *   - 通过串口（USART1）接收上位机指令
 *   - 支持切换调参模式（轮速环/舵向速度环/舵向角度环）
 *   - 动态修改底盘对应环路的 PID 参数
 *   - 不干预底盘运动（目标速度仍由遥控器提供）
 *
 * 指令格式：
 *   MODE WHEEL_SPEED      → 切换到 3508 轮速环调参模式
 *   MODE RUDDER_SPEED     → 切换到 6020 速度环（内环）调参模式
 *   MODE RUDDER_ANGLE     → 切换到 6020 角度环（外环）调参模式
 *   SET P:1.5 I:0.2 D:0.05  或  SET 1.5 0.2 0.05  → 更新当前模式的 PID
 *   STATUS                → 打印所有 PID 参数
 *   RESET                 → 重置所有 PID 参数为初始值
 * ============================================================================
 */

#include "aoto.h"
#include "Serial.h"
#include "chassis_rudder.h"
#include "pid.h"
#include <string.h>
#include <stdlib.h>

// 引用底盘全局变量（在 main.c 中定义）
extern Chassis_Rudder_t chassis;

// ---------------------------- 调参模式枚举 ----------------------------
typedef enum {
    TUNE_MODE_WHEEL_SPEED = 0,   // 3508 轮速环（位置式PID）
    TUNE_MODE_RUDDER_SPEED,      // 6020 舵向速度环（内环，增量式）
    TUNE_MODE_RUDDER_ANGLE       // 6020 舵向角度环（外环，增量式）
} TuneMode_t;

static TuneMode_t current_mode = TUNE_MODE_WHEEL_SPEED;   // 默认先调轮速

// ---------------------------- 默认 PID 参数（与 chassis_rudder.c 初始化一致）---
// 轮速环（3508）
#define DEFAULT_WHEEL_KP  2.1f
#define DEFAULT_WHEEL_KI  0.7f
#define DEFAULT_WHEEL_KD  0.1f
// 舵向速度环（6020 内环）
#define DEFAULT_RUDDER_SPEED_KP  9.0f
#define DEFAULT_RUDDER_SPEED_KI  0.5f
#define DEFAULT_RUDDER_SPEED_KD  0.1f
// 舵向角度环（6020 外环）
#define DEFAULT_RUDDER_ANGLE_KP  3.0f
#define DEFAULT_RUDDER_ANGLE_KI  0.4f
#define DEFAULT_RUDDER_ANGLE_KD  0.1f

// ---------------------------- 函数声明 ----------------------------
static void UpdatePidByMode(float Kp, float Ki, float Kd);
static void PrintAllPidParams(void);
static void ResetAllPidParams(void);
static void ProcessSerialCommand(void);

/**
 * @brief 固件初始化
 * @note  串口已在 main.c 中通过 Serial_Init() 初始化，此处只需打印就绪信息
 */
void Firmware_Init(void)
{
    Serial_Printf("# LLM PID Tuner Firmware for STM32H7 Chassis Ready\r\n");
    Serial_Printf("# Current mode: WHEEL_SPEED (3508 wheel speed)\r\n");
    Serial_Printf("# Commands: MODE [WHEEL_SPEED|RUDDER_SPEED|RUDDER_ANGLE]\r\n");
    Serial_Printf("#           SET P:%.2f I:%.2f D:%.2f\r\n",
                  DEFAULT_WHEEL_KP, DEFAULT_WHEEL_KI, DEFAULT_WHEEL_KD);
    Serial_Printf("#           STATUS, RESET\r\n");
}

/**
 * @brief 主循环中调用的任务（非阻塞）
 * @note  必须放在 while(1) 中频繁调用，用于处理串口命令
 */
void Firmware_Loop(void)
{
    ProcessSerialCommand();
}

// ---------------------------- 私有函数实现 ----------------------------
/**
 * @brief 根据当前模式更新 PID 参数
 * @param Kp, Ki, Kd  新的 PID 系数
 */
static void UpdatePidByMode(float Kp, float Ki, float Kd)
{
    // 参数范围保护（与底盘初始化保持一致）
    if (Kp <= 0.0f || Kp > 100.0f) return;
    if (Ki < 0.0f  || Ki > 50.0f)  return;
    if (Kd < 0.0f  || Kd > 50.0f)  return;

    switch (current_mode)
    {
    case TUNE_MODE_WHEEL_SPEED:
        // 修改 4 个 3508 轮速 PID（位置式）
        for (int i = 0; i < 4; i++)
        {
            chassis.wheel_pid[i].Kp = Kp;        // 注意大小写：Kp
            chassis.wheel_pid[i].Ki = Ki;
            chassis.wheel_pid[i].Kd = Kd;
            // 重置积分和上一次误差，防止突变
            chassis.wheel_pid[i].integral = 0.0f;
            chassis.wheel_pid[i].last_error = 0.0f;
        }
        Serial_Printf("# [WHEEL_SPEED] PID updated: P=%.3f I=%.3f D=%.3f\r\n", Kp, Ki, Kd);
        break;

    case TUNE_MODE_RUDDER_SPEED:
        // 修改 4 个 6020 舵向速度环（内环，增量式 PID）
        for (int i = 0; i < 4; i++)
        {
            chassis.rudder_pid[i].inner.Kp = Kp;   // 增量式成员小写 kp, ki, kd
            chassis.rudder_pid[i].inner.Ki = Ki;
            chassis.rudder_pid[i].inner.Kd = Kd;
            // 重置内环状态
            chassis.rudder_pid[i].inner.last_error = 0.0f;
        }
        Serial_Printf("# [RUDDER_SPEED] Inner loop PID updated: P=%.3f I=%.3f D=%.3f\r\n", Kp, Ki, Kd);
        break;

    case TUNE_MODE_RUDDER_ANGLE:
        // 修改 4 个 6020 舵向角度环（外环，增量式 PID）
        for (int i = 0; i < 4; i++)
        {
            chassis.rudder_pid[i].outer.Kp = Kp;
            chassis.rudder_pid[i].outer.Ki = Ki;
            chassis.rudder_pid[i].outer.Kd = Kd;
            chassis.rudder_pid[i].outer.last_error = 0.0f;
        }
        Serial_Printf("# [RUDDER_ANGLE] Outer loop PID updated: P=%.3f I=%.3f D=%.3f\r\n", Kp, Ki, Kd);
        break;
    }
}

/**
 * @brief 打印所有 PID 参数（用于 STATUS 命令）
 */
static void PrintAllPidParams(void)
{
    Serial_Printf("# --- Current PID Parameters ---\r\n");
    Serial_Printf("# Mode: %s\r\n",
        (current_mode == TUNE_MODE_WHEEL_SPEED) ? "WHEEL_SPEED" :
        (current_mode == TUNE_MODE_RUDDER_SPEED) ? "RUDDER_SPEED" : "RUDDER_ANGLE");

    // 轮速环参数（取第一个轮子）
    Serial_Printf("# Wheel Speed (3508): P=%.3f I=%.3f D=%.3f\r\n",
                  chassis.wheel_pid[0].Kp,
                  chassis.wheel_pid[0].Ki,
                  chassis.wheel_pid[0].Kd);
    // 舵向速度环（内环）
    Serial_Printf("# Rudder Speed (6020 inner): P=%.3f I=%.3f D=%.3f\r\n",
                  chassis.rudder_pid[0].inner.Kp,
                  chassis.rudder_pid[0].inner.Ki,
                  chassis.rudder_pid[0].inner.Kd);
    // 舵向角度环（外环）
    Serial_Printf("# Rudder Angle (6020 outer): P=%.3f I=%.3f D=%.3f\r\n",
                  chassis.rudder_pid[0].outer.Kp,
                  chassis.rudder_pid[0].outer.Ki,
                  chassis.rudder_pid[0].outer.Kd);
    Serial_Printf("# -----------------------------\r\n");
}

/**
 * @brief 重置所有 PID 参数为默认值
 */
static void ResetAllPidParams(void)
{
    // 轮速环
    for (int i = 0; i < 4; i++)
    {
        chassis.wheel_pid[i].Kp = DEFAULT_WHEEL_KP;
        chassis.wheel_pid[i].Ki = DEFAULT_WHEEL_KI;
        chassis.wheel_pid[i].Kd = DEFAULT_WHEEL_KD;
        chassis.wheel_pid[i].integral = 0.0f;
        chassis.wheel_pid[i].last_error = 0.0f;
    }
    // 舵向速度环（内环）
    for (int i = 0; i < 4; i++)
    {
        chassis.rudder_pid[i].inner.Kp = DEFAULT_RUDDER_SPEED_KP;
        chassis.rudder_pid[i].inner.Ki = DEFAULT_RUDDER_SPEED_KI;
        chassis.rudder_pid[i].inner.Kd = DEFAULT_RUDDER_SPEED_KD;
        chassis.rudder_pid[i].inner.last_error = 0.0f;
    }
    // 舵向角度环（外环）
    for (int i = 0; i < 4; i++)
    {
        chassis.rudder_pid[i].outer.Kp = DEFAULT_RUDDER_ANGLE_KP;
        chassis.rudder_pid[i].outer.Ki = DEFAULT_RUDDER_ANGLE_KI;
        chassis.rudder_pid[i].outer.Kd = DEFAULT_RUDDER_ANGLE_KD;
        chassis.rudder_pid[i].outer.last_error = 0.0f;
    }
    Serial_Printf("# All PID parameters reset to default\r\n");
}

/**
 * @brief 解析串口命令（基于 Serial_GetRxFlag 和 Serial_GetRxData）
 * @note  完全非阻塞，每收到一行完整命令后处理
 */
static void ProcessSerialCommand(void)
{
    static char cmd_buf[80];
    static uint8_t idx = 0;

    if (Serial_GetRxFlag())
    {
        uint8_t data = Serial_GetRxData();
        char c = (char)data;

        if (c == '\r') return;
        if (c == '\n')
        {
            cmd_buf[idx] = '\0';
            idx = 0;
            if (strlen(cmd_buf) == 0) return;

            // 解析命令
            if (strncmp(cmd_buf, "MODE", 4) == 0)
            {
                char *mode_str = cmd_buf + 4;
                while (*mode_str == ' ') mode_str++;
                if (strcmp(mode_str, "WHEEL_SPEED") == 0)
                    current_mode = TUNE_MODE_WHEEL_SPEED;
                else if (strcmp(mode_str, "RUDDER_SPEED") == 0)
                    current_mode = TUNE_MODE_RUDDER_SPEED;
                else if (strcmp(mode_str, "RUDDER_ANGLE") == 0)
                    current_mode = TUNE_MODE_RUDDER_ANGLE;
                else
                {
                    Serial_Printf("# Invalid mode. Use: WHEEL_SPEED, RUDDER_SPEED, RUDDER_ANGLE\r\n");
                    return;
                }
                Serial_Printf("# Mode switched to: %s\r\n", mode_str);
            }
            else if (strncmp(cmd_buf, "SET", 3) == 0)
            {
                char *params = cmd_buf + 3;
                float kp = 0, ki = 0, kd = 0;
                int parsed = 0;

                if (strchr(params, ':') != NULL)
                {
                    char *p = strstr(params, "P:"); if (p) kp = atof(p+2);
                    char *i = strstr(params, "I:"); if (i) ki = atof(i+2);
                    char *d = strstr(params, "D:"); if (d) kd = atof(d+2);
                    parsed = (p && i && d);
                }
                else
                {
                    parsed = (sscanf(params, "%f %f %f", &kp, &ki, &kd) == 3);
                }
                if (parsed)
                    UpdatePidByMode(kp, ki, kd);
                else
                    Serial_Printf("# SET format error. Usage: SET P:1.0 I:0.2 D:0.05 or SET 1.0 0.2 0.05\r\n");
            }
            else if (strcmp(cmd_buf, "STATUS") == 0)
            {
                PrintAllPidParams();
            }
            else if (strcmp(cmd_buf, "RESET") == 0)
            {
                ResetAllPidParams();
            }
            else
            {
                Serial_Printf("# Unknown command: %s\r\n", cmd_buf);
            }
        }
        else
        {
            if (idx < sizeof(cmd_buf)-1) cmd_buf[idx++] = c;
            else idx = 0;  // 溢出丢弃
        }
    }
}