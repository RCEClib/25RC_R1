#include "scissor_lift.h"

#include "DJI_Motor.h"
#include "fdcan.h"
#include "pid.h"

//定义 PID 控制器
static PID_Controller_Group sl_pid;

static float target_total_angle;      // 目标总角度（度）

// 3508电机减速比（电机转19圈，输出轴转1圈）
#define MOTOR_3508_GEAR_RATIO  19.0f

/**
 * @brief  夹爪初始化函数
 * @retval None
 */
void sl_Init(void) {
    // 读取当前实际总角度（转子角度）
    float init_total_id5 = motor_feedback[MOTOR_3508_ID5_INDEX].loop * 360.0f
                           + motor_feedback[MOTOR_3508_ID5_INDEX].angle;

    // 将目标设置为当前实际值（误差0）
    target_total_angle = init_total_id5 * MOTOR_3508_GEAR_RATIO;

    // 初始化PID
    PID_Init(&sl_pid.outer, 0.5f, 0.0f, 0.0f, 8000.0f, 600.0f);
    PID_Init(&sl_pid.inner, 1.3f, 0.0f, 0.0f, 10000.0f, 3000.0f);

    target_total_angle = 0.0f;

    // 停止电机
    DJI_Motor_SendCurrent_Ex(&hfdcan1, MOTOR_3508_GROUP2, 0, 0, 0, 0);
}

// 设置 ID5 的目标（输出轴圈数 + 角度）
void sl_SetTarget(int16_t loop, float angle) {
    float norm_angle = fmodf(angle, 360.0f);
    if (norm_angle < 0.0f) norm_angle += 360.0f;
    target_total_angle = (loop * 360.0f + norm_angle) * MOTOR_3508_GEAR_RATIO;
}

/**
 * @brief  剪式升降机构控制函数
 * @retval None
 */
void sl_Control(void) {
    // 实际总角度（转子角度）
    float actual_total_id5 = motor_feedback[MOTOR_3508_ID5_INDEX].loop * 360.0f
                             + motor_feedback[MOTOR_3508_ID5_INDEX].angle;
    float actual_speed_id5 = motor_feedback[MOTOR_3508_ID5_INDEX].speed;

    float current_id5 = pid_CascadeCalc(&sl_pid,
                                        target_total_angle,
                                        actual_total_id5,
                                        actual_speed_id5);
    if (current_id5 > 600.0f) current_id5 = 600.0f;
    if (current_id5 < -600.0f) current_id5 = -600.0f;

    // 一次性发电流
    DJI_Motor_SendCurrent_Ex(&hfdcan1, MOTOR_3508_GROUP2,
                         (int16_t)current_id5,   // 电机1
                         0,
                         0, 0);
}

/**
 * @brief 升降机构任务函数
 * @param mode1  模式1：0-关闭，1-打开
 * @retval None
 */
void scissor_lift_Task(uint8_t mode1) {
    int valid = 1;
    switch (mode1) {
        case 0:sl_SetTarget(0, 100.0f);break;
        case 1:sl_SetTarget(20, 100.0f);break;
        default:valid = 0; break;
    }
    if (valid) sl_Control();
}