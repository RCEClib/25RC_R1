/**
 ******************************************************************************
 * @file    motor.c
 * @brief   电机控制驱动程序 (支持23个电机)
 * @details 提供电机初始化、电流控制、反馈数据接收等功能
 *          支持3508、6020、2006三种电机类型
 *          6020为绝对编码器，3508和2006为增量编码器
 ******************************************************************************
 */

#include "fd.h"
#include "motor.h"

#include <stdio.h>

#include "Serial.h"

/* 宏定义 -------------------------------------------------------------------- */
#define ENCODER_PER_ROUND 8192.0f  // 编码器每圈的脉冲数（8192线）
#define ANGLE_THRESHOLD   270.0f   // 角度穿越阈值（优化为270°）
#define MAX_MOTOR_ID      0x20B    // 最大电机ID

/* 全局变量 ------------------------------------------------------------------ */
Motor_Feedback_t motor_feedback[MOTOR_NUM] = {0};  // 电机反馈数据数组

/**
 * @brief  电机初始化函数
 * @param  无
 * @retval HAL_OK: 初始化成功
 *         HAL_ERROR: 初始化失败
 */
HAL_StatusTypeDef Motor_Init(void) {
    // 初始化所有电机类型
    for (int i = 0; i < MOTOR_NUM; i++) {
        if (i < 8) {
            motor_feedback[i].type = MOTOR_3508;  // 前8个为3508电机（增量编码器）
        } else if (i < 15) {
            motor_feedback[i].type = MOTOR_6020;  // 中间7个为6020电机（绝对编码器）
        } else {
            motor_feedback[i].type = MOTOR_2006;  // 后8个为2006电机（增量编码器）
        }
    }

    // 停止所有电机
    Motor_SendCurrent_3508(MOTOR_3508_GROUP1, 0, 0, 0, 0);
    Motor_SendCurrent_3508(MOTOR_3508_GROUP2, 0, 0, 0, 0);

    Motor_SendCurrent_6020(MOTOR_6020_GROUP1, 0, 0, 0, 0);
    Motor_SendCurrent_6020(MOTOR_6020_GROUP2, 0, 0, 0, 0);

    Motor_SendCurrent_2006(MOTOR_2006_GROUP1, 0, 0, 0, 0);
    Motor_SendCurrent_2006(MOTOR_2006_GROUP2, 0, 0, 0, 0);

    // 初始化FDCAN通信
    if ((FDCAN_Init(&hfdcan1) || FDCAN_Init(&hfdcan2)|| FDCAN_Init(&hfdcan3)) != HAL_OK) {
        return HAL_ERROR;
    }

    printf("Motor init success (23 motors)\n");
    return HAL_OK;
}

/**
 * @brief  发送3508电机控制电流（增量编码器）- 使用CAN1
 * @param  group_id: 电机组ID (0x200)
 * @param  current1-4: 4个电机的目标电流值
 * @retval HAL_OK: 发送成功
 */
HAL_StatusTypeDef Motor_SendCurrent_3508(uint32_t group_id, int16_t current1,
                                         int16_t current2, int16_t current3,
                                         int16_t current4) {
    uint8_t data[8];

    data[0] = current1 >> 8;
    data[1] = current1;
    data[2] = current2 >> 8;
    data[3] = current2;
    data[4] = current3 >> 8;
    data[5] = current3;
    data[6] = current4 >> 8;
    data[7] = current4;

    return FDCAN_Send(&hfdcan1, group_id, FDCAN_STANDARD_ID, data);
}

/**
 * @brief  发送6020电机控制电流（绝对编码器）- 使用CAN2
 * @param  group_id: 电机组ID (0x1FF或0x2FF)
 * @param  current1-4: 4个电机的目标电流值
 * @retval HAL_OK: 发送成功
 */
HAL_StatusTypeDef Motor_SendCurrent_6020(uint32_t group_id, int16_t current1,
                                         int16_t current2, int16_t current3,
                                         int16_t current4) {
    uint8_t data[8];

    data[0] = current1 >> 8;
    data[1] = current1;
    data[2] = current2 >> 8;
    data[3] = current2;
    data[4] = current3 >> 8;
    data[5] = current3;
    data[6] = current4 >> 8;
    data[7] = current4;

    return FDCAN_Send(&hfdcan2, group_id, FDCAN_STANDARD_ID, data);
}

/**
 * @brief  发送2006电机控制电流（增量编码器）- 使用CAN3
 * @param  group_id: 电机组ID (0x200)
 * @param  current1-4: 4个电机的目标电流值
 * @retval HAL_OK: 发送成功
 */
HAL_StatusTypeDef Motor_SendCurrent_2006(uint32_t group_id, int16_t current1,
                                         int16_t current2, int16_t current3,
                                         int16_t current4) {
    uint8_t data[8];

    data[0] = current1 >> 8;
    data[1] = current1;
    data[2] = current2 >> 8;
    data[3] = current2;
    data[4] = current3 >> 8;
    data[5] = current3;
    data[6] = current4 >> 8;
    data[7] = current4;

    // 2006电机使用CAN3控制
    return FDCAN_Send(&hfdcan3, group_id, FDCAN_STANDARD_ID, data);
}

/**
 * @brief  设置电机目标角度和圈数
 * @param  motor_index: 电机索引
 * @param  target_angle: 目标角度 (0~360度)
 * @param  target_loop: 目标圈数
 * @retval 无
 */
void Motor_SetAngleTarget(uint8_t motor_index, float target_angle, int16_t target_loop) {
    if (motor_index < MOTOR_NUM) {
        motor_feedback[motor_index].angle_target = target_angle;
        motor_feedback[motor_index].loop_target = target_loop;
    }
}

/**
 * @brief  接收电机反馈数据
 * @param  RxHeader: FDCAN接收帧头指针
 * @param  RxData: FDCAN接收数据指针
 * @retval 无
 */
void Motor_RecieveFeedback(FDCAN_RxHeaderTypeDef *RxHeader, uint8_t *RxData) {
    uint8_t motor_index = 0;
    uint32_t id = RxHeader->Identifier;

    // 标准ID计算方式，通过优先级判断解决ID冲突
    if (id >= MOTOR_6020_ID1 && id <= MOTOR_6020_ID7) {
        // 6020电机优先：0x205->8, 0x206->9, ..., 0x20B->14
        motor_index = id - MOTOR_6020_ID1 + 8;
    }
    else if (id >= MOTOR_3508_ID1 && id <= MOTOR_3508_ID8) {
        // 3508电机：0x201->0, 0x202->1, ..., 0x208->7
        motor_index = id - MOTOR_3508_ID1;
    }
    else if (id >= MOTOR_2006_ID1 && id <= MOTOR_2006_ID8) {
        // 2006电机：0x201->15, 0x202->16, ..., 0x208->22
        motor_index = id - MOTOR_2006_ID1 + 15;
    }
    else {
        return;  // 未知ID，直接返回
    }

    // 检查索引是否在有效范围内
    if (motor_index < MOTOR_NUM) {
        Motor_Feedback_t *fb = &motor_feedback[motor_index];  // 简化访问

        // 保存上一次的角度值（仅增量编码器需要）
        if (fb->type != MOTOR_6020) {
            fb->last_angle = fb->angle;
        }

        // 解析当前角度
        uint16_t encoder_val = (RxData[0] << 8) | RxData[1];
        // 计算角度
        float calculated_angle = (float)encoder_val / ENCODER_PER_ROUND * 360.0f;
        fb->angle = calculated_angle;

        // 根据电机类型处理角度差值计算
        if (fb->type == MOTOR_6020) {
            // 6020绝对编码器：直接计算角度差值（选择最短路径）
            float angle_diff = fb->angle_target - fb->angle;

            // 优化角度差值计算，处理0度穿越
            if (angle_diff > 180.0f) {
                angle_diff -= 360.0f;
            } else if (angle_diff < -180.0f) {
                angle_diff += 360.0f;
            }
            fb->angle_diff = angle_diff;

            // 绝对编码器不需要圈数计算
            fb->loop = 0;
        } else {
            // 3508和2006增量编码器：需要精确的圈数计算
            float delta_angle = fb->angle - fb->last_angle;

            // 更稳健的0度穿越检测逻辑
            // 使用更保守的阈值（300°）来避免干扰误判
            if (delta_angle < -300.0f) {
                // 正向过零：从接近360°跳到接近0°
                // 例如：350° -> 10°，delta = -340°，应该增加圈数
                fb->loop++;
            } else if (delta_angle > 300.0f) {
                // 反向过零：从接近0°跳到接近360°
                // 例如：10° -> 350°，delta = 340°，应该减少圈数
                fb->loop--;
            }

            // 计算考虑圈数的总角度差值
            float total_target = fb->loop_target * 360.0f + fb->angle_target;
            float total_current = fb->loop * 360.0f + fb->angle;

            // 优化总角度差值（选择最短路径）
            float total_diff = total_target - total_current;
            if (total_diff > 180.0f) {
                total_diff -= 360.0f;
            } else if (total_diff < -180.0f) {
                total_diff += 360.0f;
            }
            fb->angle_diff = total_diff;
        }

        // 解析其他参数
        fb->speed = (int16_t)((RxData[2] << 8) | RxData[3]);
        fb->current = (int16_t)((RxData[4] << 8) | RxData[5]);
        fb->temperature = RxData[6];
    }
}