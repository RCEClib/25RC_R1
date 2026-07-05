#include "scissor_lift.h"
#include "dm_motor_ctrl.h"
#include "DJI_Motor.h"
#include "fdcan.h"
#include "pid.h"
#include "Serial.h"

//定义 PID 控制器
static PID_Controller_Group sl_pid;

static float target_total_angle1;      // 目标总角度（度）
static float target_total_angle2;      // 目标总角度（度）


// 3508电机减速比（电机转19圈，输出轴转1圈）
#define MOTOR_3508_GEAR_RATIO  19.0f
#define xipan_STEP  0.22f   // 单步最大弧度

/**
 * @brief  夹爪初始化函数
 * @retval None
 */
void sl_Init(void) {

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);          //一级伸缩
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);           //二级伸缩
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);           //气缸1控制
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);           //气缸2控制

    dm_motor_init(Motor1, &hfdcan3, 0x01, mit_mode);/////////////////达妙关节电机

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

    mit_ctrl(motor[Motor1].hcan, &motor[Motor1], motor[Motor1].id, motor[Motor1].para.pos, 0, 50, 2, 0);
}

// 设置目标（输出轴圈数 + 角度）
void sl_SetTarget(int16_t loop, float angle) {
    float norm_angle = fmodf(angle, 360.0f);
    if (norm_angle < 0.0f) norm_angle += 360.0f;
    target_total_angle1 = (loop * 360.0f + norm_angle) * MOTOR_3508_GEAR_RATIO;
    target_total_angle2 = (loop * 360.0f + norm_angle) * MOTOR_3508_GEAR_RATIO;
}

void xipan_SetAngle(float j1)
{
    static uint32_t last_tick = 0;
    uint32_t now = HAL_GetTick();
    if (now - last_tick < 10) return;
    last_tick = now;

    /* 步进限幅：每 5ms 最多移动 ARM_STEP，慢走防抖 */
    float c1 = motor[Motor1].para.pos;

    float d1 = j1 - c1;

    /* 步进 clamp：每轴最多向目标靠近 ARM_STEP，绝不过冲 */
    c1 += fmaxf(-xipan_STEP, fminf(xipan_STEP, d1));

    /* 限位钳位 */
    // if (c1 < J1_MIN) c1 = J1_MIN;  if (c1 > J1_MAX) c1 = J1_MAX;

    mit_ctrl(motor[Motor1].hcan, &motor[Motor1], motor[Motor1].id, c1, 0, 80, 3, 0);
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
void scissor_lift_Task(int8_t loop,float angle_m,uint8_t key0,uint8_t key1,uint8_t key2) {
    int valid = 1;
    static uint8_t jiazhua=1;

    sl_SetTarget(loop, 0);

    //吸盘角度控制                                         没按键检测了直接用S2控制角度
    // if (angle_m != 0) xipan_SetAngle(M_PI);
    // else xipan_SetAngle(0);
     xipan_SetAngle(angle_m);

    //                                                                组装武器
    if (jiazhua && key0 == 1) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);    //气缸1启动
        //这行加上舵机转动
        HAL_Delay(1500);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);    //气缸2启动
        jiazhua = 0;
    }
    //                                                                 释放武器
    else if (key0 ==2) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
    }

    //伸缩取放矿
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, key2);              //一级伸缩
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, key1);               //二级伸缩

    if (valid) sl_Control();
}