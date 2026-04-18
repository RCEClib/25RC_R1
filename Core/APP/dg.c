#include "dg.h"
#include "fd.h"
#include "motor.h"
#include "pid.h"
#include <math.h>

static PID_Controller_Group dg_pid;   // 级联 PID：外环角度环，内环速度环
// 2006电机减速比（电机转36圈，轮子转1圈）
#define MOTOR_2006_GEAR_RATIO  36.0f

float taget_v;


float target_total_angle;      // 目标总角度（度）
float actual_total_angle;      // 实际总角度（度）—— 当前值

void dg_Init(void) {
    //PID_Init(&speed_pid, 1.3f, 0.06f, 0.2f, 10000.0f, 3000.0f);
    // 外环：角度环（输入角度误差，输出目标转速 rpm）
    //PID_Init(&dg_pid.outer, 0.0f, 0.0f, 0.0f, 400.0f, 200.0f);
    PID_Init(&dg_pid.outer, 3.6f, 1.2f, 0.0f, 8000.0f, 1000.0f);
    // 内环：速度环（输入转速误差，输出电流 mA）
    PID_Init(&dg_pid.inner, 1.3f, 0.06f, 0.2f, 10000.0f, 3000.0f);
    target_total_angle = 0.0f;
    Motor_SendCurrent_Ex(&hfdcan1, MOTOR_2006_GROUP2, 0, 0, 0, 0);
}

void dg_SetTarget(int16_t loop, float angle) {
    float norm_angle = fmodf(angle, 360.0f);
    if (norm_angle < 0.0f) norm_angle += 360.0f;
    target_total_angle = loop * 360.0f * MOTOR_2006_GEAR_RATIO + norm_angle;//转子角度*减速比+目标角度
}

void dg_Control(void) {
    // 获取实际总角度（当前值）
    actual_total_angle = motor_feedback[MOTOR_2006_ID5_INDEX].loop * 360.0f
                         + motor_feedback[MOTOR_2006_ID5_INDEX].angle;
    // 获取实际转速（当前值，单位 rpm）
    float actual_speed_rpm = motor_feedback[MOTOR_2006_ID5_INDEX].speed;

    // 级联 PID 计算，返回电流（mA）
    float current = pid_CascadeCalc(&dg_pid, target_total_angle, actual_total_angle, actual_speed_rpm);

    // 电流限幅
    if (current > 10000.0f) current = 10000.0f;
    if (current < -10000.0f) current = -10000.0f;

    // 发送电流指令
    Motor_SendCurrent_Ex(&hfdcan1, MOTOR_2006_GROUP2, (int16_t)current,0, 0, 0);
}

void dg_Task(uint8_t mode) {
    //转轴圈数            //转子角度
    int16_t target_loop; float target_angle;
    if (mode == 0) {
        target_loop = 0;
        target_angle = 0.0f;
    }
    else if (mode == 1) {
        target_loop = 3;
        target_angle = 90.0f;
        dg_SetTarget(target_loop, target_angle);
        dg_Control();
    }
    else return;
    dg_SetTarget(target_loop, target_angle);
    dg_Control();

}