#include "scissor_lift.h"

#include "DJI_Motor.h"
#include "fdcan.h"
#include "pid.h"

//定义 PID 控制器
static PID_Controller_Group sl_pid;

static float target_total_angle1;      // 目标总角度（度）
static float target_total_angle2;      // 目标总角度（度）


// 3508电机减速比（电机转19圈，输出轴转1圈）
#define MOTOR_3508_GEAR_RATIO  19.0f

/**
 * @brief  夹爪初始化函数
 * @retval None
 */
void sl_Init(void) {

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);

    // 读取当前实际总角度（转子角度）
    float init_total_id5 = motor_feedback[MOTOR_3508_ID5_INDEX].loop * 360.0f
                           + motor_feedback[MOTOR_3508_ID5_INDEX].angle;
    float init_total_id6 = motor_feedback[MOTOR_3508_ID6_INDEX].loop * 360.0f
                               + motor_feedback[MOTOR_3508_ID6_INDEX].angle;
    // 将目标设置为当前实际值（误差0）
    target_total_angle1 = init_total_id5 * MOTOR_3508_GEAR_RATIO;
    target_total_angle2 = init_total_id6 * MOTOR_3508_GEAR_RATIO;

    // 初始化PID
    PID_Init(&sl_pid.outer, 0.6f, 0.0f, 0.1f, 8000.0f, 3000.0f);
    PID_Init(&sl_pid.inner, 0.2f, 0.06f, 0.06f, 10000.0f, 6000.0f);

    target_total_angle1 = 0.0f;
    target_total_angle2 = 0.0f;

    // 停止电机
    DJI_Motor_SendCurrent_Ex(&hfdcan1, MOTOR_3508_GROUP2, 0, 0, 0, 0);
}

// 设置目标（输出轴圈数 + 角度）
void sl_SetTarget(int16_t loop, float angle) {
    float norm_angle = fmodf(angle, 360.0f);
    if (norm_angle < 0.0f) norm_angle += 360.0f;
    target_total_angle1 = (loop * 360.0f + norm_angle) * MOTOR_3508_GEAR_RATIO;
    target_total_angle2 = (loop * 360.0f + norm_angle) * MOTOR_3508_GEAR_RATIO;
}

/**
 * @brief  剪式升降机构控制函数
 * @retval None
 */
void sl_Control(void) {
    // 实际总角度（转子角度）
    float actual_total_id5 = -(motor_feedback[MOTOR_3508_ID5_INDEX].loop * 360.0f +motor_feedback[MOTOR_3508_ID5_INDEX].angle);
    float actual_speed_id5 = -motor_feedback[MOTOR_3508_ID5_INDEX].speed;

    float current_id5 = pid_CascadeCalc(&sl_pid,
                                            target_total_angle1,
                                            actual_total_id5,
                                            actual_speed_id5);
    if (current_id5 > 16384.0f) current_id5 = 16384.0f;
    if (current_id5 < -16384.0f) current_id5 = -16384.0f;

    float actual_total_id6 = -(motor_feedback[MOTOR_3508_ID6_INDEX].loop * 360.0f +motor_feedback[MOTOR_3508_ID6_INDEX].angle);
    float actual_speed_id6 = -motor_feedback[MOTOR_3508_ID6_INDEX].speed;
    float current_id6 = pid_CascadeCalc(&sl_pid,
                                        target_total_angle2,
                                        actual_total_id6,
                                        actual_speed_id6);
    if (current_id6 > 16384.0f) current_id6 = 16384.0f;
    if (current_id6 < -16384.0f) current_id6 = -16384.0f;

    int16_t current_id5_int = -current_id5;
    int16_t current_id6_int = -current_id6;

    // 一次性发电流（ID5 和 ID6 给相同电流，机械上同步运动）
    DJI_Motor_SendCurrent_Ex(&hfdcan1, MOTOR_3508_GROUP2,
                         current_id5_int,   // 电机1
                         current_id6_int,   // 电机2
                         0, 0);
}

/**
 * @brief 升降机构任务函数
 * @param mode1  模式1：0-关闭，1-打开
 * @retval None
 */
void scissor_lift_Task(int8_t mode1,uint8_t key1) {
    int valid = 1;
    sl_SetTarget(mode1, 0.0f);

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, key1);

    if (valid) sl_Control();
}