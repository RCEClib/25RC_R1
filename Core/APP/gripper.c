#include "gripper.h"
#include "fd.h"
#include "DJI_Motor.h"
#include "pid.h"
#include <math.h>

#include "tim.h"

// 为 ID5 和 ID6 分别定义 PID 控制器
static PID_Controller_Group dg_pid_id5;
static PID_Controller_Group dg_pid_id6;

float target_total_angle_id5, target_total_angle_id6;      // 目标总角度（度）
float actual_total_angle_id5, actual_total_angle_id6;      // 实际总角度（度）—— 当前值

// 2006电机减速比（电机转36圈，轮子转1圈）
#define MOTOR_2006_GEAR_RATIO  36.0f

/**
 * @brief  夹爪初始化函数
 * @retval None
 */
void gripper_Init(void) {
    //舵机初始化
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

    // 等待电机反馈有效（例如延时100ms，或检查更新标志）
    HAL_Delay(100);  // 确保CAN接收到了至少一帧数据

    // 读取当前实际总角度（转子角度）
    float init_total_id5 = motor_feedback[MOTOR_2006_ID5_INDEX].loop * 360.0f
                           + motor_feedback[MOTOR_2006_ID5_INDEX].angle;
    float init_total_id6 = motor_feedback[MOTOR_2006_ID6_INDEX].loop * 360.0f
                           + motor_feedback[MOTOR_2006_ID6_INDEX].angle;

    // 将目标设置为当前实际值（误差0）
    target_total_angle_id5 = init_total_id5 * MOTOR_2006_GEAR_RATIO;
    target_total_angle_id6 = init_total_id6 * MOTOR_2006_GEAR_RATIO;

    // 初始化 ID5 的 PID
    PID_Init(&dg_pid_id5.outer, 3.0f, 1.2f, 0.0f, 8000.0f, 600.0f);
    PID_Init(&dg_pid_id5.inner, 1.3f, 0.06f, 0.2f, 10000.0f, 3000.0f);

    // 初始化 ID6 的 PID
    PID_Init(&dg_pid_id6.outer, 0.7f, 0.4f, 0.2f, 3000.0f, 600.0f);
    PID_Init(&dg_pid_id6.inner, 1.3f, 0.06f, 0.2f, 10000.0f, 3000.0f);

    target_total_angle_id5 = 0.0f;
    target_total_angle_id6 = 0.0f;

    // 停止电机
    DJI_Motor_SendCurrent_Ex(&hfdcan1, MOTOR_2006_GROUP2, 0, 0, 0, 0);
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

/**
 * @brief  夹爪控制函数
 * @retval None
 */
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
    DJI_Motor_SendCurrent_Ex(&hfdcan1, MOTOR_2006_GROUP2,
                         (int16_t)current_id5,   // 电机1
                         (int16_t)current_id6,   // 电机2
                         0, 0);
}

/**
 * @brief  设置舵机角度
 * @param angle  角度（度）
 * @retval None
 */
void Set_Angle(float angle) {
    /* 限制角度范围 */
    if(angle > 270.0f) angle = 270.0f;
    if(angle < 0.0f)   angle = 0.0f;

    /* angle -> CCR */
    uint32_t ccr = (uint32_t)(500 + (angle / 270.0f) * 2500);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ccr);
}

/**
 * @brief  夹爪任务函数
 * @param mode1  模式1：0-关闭，1-打开
 * @param mode2  模式2：0-关闭，1-打开
 * @param angle  角度（度）
 * @retval None
 */
void gripper_Task(uint8_t mode1, uint8_t mode2, float angle) {
    int valid = 1;
    switch (mode1) {
        case 0:dg_SetTarget_ID5(0, 0.0f);break;
        case 1:dg_SetTarget_ID5(3, 90.0f);break;
        default:valid = 0; break;
    }
    switch (mode2) {
        case 0:dg_SetTarget_ID6(0, 0.0f);
            break;
        case 1:dg_SetTarget_ID6(3, 100.0f);
            break;
        default:valid = 0; break;
    }
    if (valid) dg_Control();
    Set_Angle(-angle *2.7f);
}