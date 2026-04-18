#include "dg.h"
#include "fd.h"
#include "motor.h"
#include "pid.h"
#include <math.h>

// 为 ID5 和 ID6 分别定义 PID 控制器
static PID_Controller_Group dg_pid_id5;
static PID_Controller_Group dg_pid_id6;

// 各自的目标总角度（转子角度，已乘减速比）
static float target_total_angle_id5;
static float target_total_angle_id6;

// 2006电机减速比（电机转36圈，轮子转1圈）
#define MOTOR_2006_GEAR_RATIO  36.0f

void dg_Init(void) {
    // 初始化 ID5 的 PID
    PID_Init(&dg_pid_id5.outer, 3.0f, 1.2f, 0.0f, 8000.0f, 600.0f);
    PID_Init(&dg_pid_id5.inner, 1.3f, 0.06f, 0.2f, 10000.0f, 3000.0f);

    // 初始化 ID6 的 PID
    PID_Init(&dg_pid_id6.outer, 0.7f, 0.4f, 0.2f, 3000.0f, 600.0f);
    PID_Init(&dg_pid_id6.inner, 1.3f, 0.06f, 0.2f, 10000.0f, 3000.0f);

    target_total_angle_id5 = 0.0f;
    target_total_angle_id6 = 0.0f;

    // 停止电机
    Motor_SendCurrent_Ex(&hfdcan1, MOTOR_2006_GROUP2, 0, 0, 0, 0);
}

// 设置 ID5 的目标（输出轴圈数 + 角度）
void dg_SetTarget_ID5(int16_t loop, float angle) {
    float norm_angle = fmodf(angle, 360.0f);
    if (norm_angle < 0.0f) norm_angle += 360.0f;
    target_total_angle_id5 = (loop * 360.0f + norm_angle) * MOTOR_2006_GEAR_RATIO;
}

// 设置 ID6 的目标
void dg_SetTarget_ID6(int16_t loop, float angle) {
    float norm_angle = fmodf(angle, 360.0f);
    if (norm_angle < 0.0f) norm_angle += 360.0f;
    target_total_angle_id6 = (loop * 360.0f + norm_angle) * MOTOR_2006_GEAR_RATIO;
}

void dg_Control(void) {
    // 实际总角度（转子角度）
    float actual_total_id5 = motor_feedback[MOTOR_2006_ID5_INDEX].loop * 360.0f
                             + motor_feedback[MOTOR_2006_ID5_INDEX].angle;
    float actual_speed_id5 = motor_feedback[MOTOR_2006_ID5_INDEX].speed;

    float current_id5 = pid_CascadeCalc(&dg_pid_id5,
                                        target_total_angle_id5,
                                        actual_total_id5,
                                        actual_speed_id5);
    if (current_id5 > 10000.0f) current_id5 = 10000.0f;
    if (current_id5 < -10000.0f) current_id5 = -10000.0f;


    float actual_total_id6 = motor_feedback[MOTOR_2006_ID6_INDEX].loop * 360.0f
                             + motor_feedback[MOTOR_2006_ID6_INDEX].angle;
    float actual_speed_id6 = motor_feedback[MOTOR_2006_ID6_INDEX].speed;

    float current_id6 = pid_CascadeCalc(&dg_pid_id6,
                                        target_total_angle_id6,
                                        actual_total_id6,
                                        actual_speed_id6);
    if (current_id6 > 10000.0f) current_id6 = 10000.0f;
    if (current_id6 < -10000.0f) current_id6 = -10000.0f;

    // 一次性发送两个电流到 CAN 组（ID5 对应通道0，ID6 对应通道1）
    Motor_SendCurrent_Ex(&hfdcan1, MOTOR_2006_GROUP2,
                         (int16_t)current_id5,   // 电机1
                         (int16_t)current_id6,   // 电机2
                         0, 0);
}

// 任务函数：mode 0 -> 停止；mode 1 -> ID5转3圈，ID6转4圈
void dg_Task(uint8_t mode) {
    if (mode == 0) {
        dg_SetTarget_ID5(0, 0.0f);
        dg_SetTarget_ID6(0, 0.0f);
    } else if (mode == 1) {
        dg_SetTarget_ID5(3, 90.0f);   // ID5 转3圈，停在90度
        dg_SetTarget_ID6(3, 100.0f);   // ID6 转3圈，停在100度
    } else {
        return;
    }
    dg_Control();
}