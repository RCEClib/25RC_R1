/**
 ******************************************************************************
 * @file    motor.c
 * @brief   电机控制驱动程序 (支持23个电机)
 * @details 提供电机初始化、电流控制、反馈数据接收等功能
 *          支持3508、6020、2006三种电机类型
 *          6020为绝对编码器，3508和2006为增量编码器
 ******************************************************************************
 */
#include "motor.h"
#include "fd.h"
#include "Serial.h"
#include <stdio.h>

/* 宏定义 -------------------------------------------------------------------- */
#define ENCODER_PER_ROUND 8192.0f  // 编码器每圈的脉冲数（8192线）
#define ANGLE_THRESHOLD   270.0f   // 角度穿越阈值（优化为270°）

/* 全局变量 ------------------------------------------------------------------ */
Motor_Feedback_t motor_feedback[MOTOR_NUM] = {0};  // 电机反馈数据数组

/* 外部变量声明 -------------------------------------------------------------- */
extern FDCAN_HandleTypeDef hfdcan1;  // 声明外部CAN1句柄
extern FDCAN_HandleTypeDef hfdcan2;  // 声明外部CAN2句柄
extern FDCAN_HandleTypeDef hfdcan3;  // 声明外部CAN3句柄


/**
 * @brief  判断CAN句柄是否已经被初始化,即已经调用了FDCAN_Init
 * @param  hfdcan: FDCAN句柄指针
 * @retval 1: 已初始化, 0: 未初始化
 */
static uint8_t IsCanInitialized(FDCAN_HandleTypeDef *hfdcan) {
    // 如果句柄非空且Instance指针非空，说明已经初始化了该CAN外设
    return (hfdcan != NULL && hfdcan->Instance != NULL);
}

/**
 * @brief  电机初始化函数
 * @param  无
 * @retval HAL_OK: 初始化成功
 * @note   此函数会自动停止所有已激活CAN口上的所有电机组。
 *         必须先初始化所需CAN口（调用FDCAN_Init），再调用本函数。
 *         不会自动初始化FDCAN，也不会操作未初始化的CAN口。
 */
HAL_StatusTypeDef Motor_Init(void) {
    //初始化所有电机的类型（用于接收反馈时的角度计算）
    for (int i = 0; i < MOTOR_NUM; i++) {
        if (i < 8) {
            motor_feedback[i].type = MOTOR_3508;  // 前8个为3508电机（增量编码器）
        } else if (i < 15) {
            motor_feedback[i].type = MOTOR_6020;  // 中间7个为6020电机（绝对编码器）
        } else {
            motor_feedback[i].type = MOTOR_2006;  // 后8个为2006电机（增量编码器）
        }
    }

    //自动停止所有已激活CAN口上的所有电机组
    //所有可能的电机组ID（涵盖3508/6020/2006的所有组）
    uint32_t all_group_ids[] = {0x200, 0x1FF, 0x1FE, 0x2FE};
    int num_groups = sizeof(all_group_ids) / sizeof(all_group_ids[0]);

    // 检查每个CAN口是否已初始化
    FDCAN_HandleTypeDef* can_handles[] = {&hfdcan1, &hfdcan2, &hfdcan3};
    for (int i = 0; i < 3; i++) {
        if (IsCanInitialized(can_handles[i])) {
            // 对该已激活的CAN口，发送所有组ID的0电流（停止电机）
            for (int j = 0; j < num_groups; j++) {
                Motor_SendCurrent_Ex(can_handles[i], all_group_ids[j], 0, 0, 0, 0);
            }
        }
    }
    Serial_Printf("Motor init success (auto-stopped on active CANs)\n");
    return HAL_OK;
}

/**
 * @brief  通用电机电流发送函数（可指定任意CAN口）
 * @param  hfdcan: FDCAN句柄指针（例如 &hfdcan1, &hfdcan2, &hfdcan3）
 * @param  group_id: 电机组ID
 * @param  current1-4: 4个电机的目标电流值
 * @retval HAL_OK: 发送成功，其他: 发送失败
 * @note   此函数为所有发送函数的核心实现，可根据需要调用任意CAN口
 */
HAL_StatusTypeDef Motor_SendCurrent_Ex(FDCAN_HandleTypeDef *hfdcan,
                                        uint32_t group_id,
                                        int16_t current1, int16_t current2,
                                        int16_t current3, int16_t current4) {
    uint8_t data[8];

    // 将4个16位电流值打包成8字节数据
    data[0] = current1 >> 8;
    data[1] = current1;
    data[2] = current2 >> 8;
    data[3] = current2;
    data[4] = current3 >> 8;
    data[5] = current3;
    data[6] = current4 >> 8;
    data[7] = current4;

    // 调用底层FDCAN发送函数
    return FDCAN_Send(hfdcan, group_id, FDCAN_STANDARD_ID, data);
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