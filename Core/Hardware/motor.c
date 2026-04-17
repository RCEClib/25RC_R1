/**
 * @file motor.c
 * @brief 电机控制驱动程序源文件
 * @version 2.0
 * @details 采用配置表查表接收，彻底解决CAN ID冲突，支持多CAN口。
 *          只需修改下面的 motor_configs[] 数组即可适配实际接线。
 *          索引范围：3508(0-7), 6020(8-14), 2006(15-22)
 */

#include "motor.h"
#include "fd.h"          // 包含 hfdcan1/2/3 的声明
#include "Serial.h"      // 调试打印（可根据需要保留或删除）

#include <string.h>

/* =========================== 全局变量定义 =========================== */
Motor_Feedback_t motor_feedback[MOTOR_NUM] = {0};

// ---------- 配置表（重要！请根据实际接线修改）----------
// 格式：{ CAN口编号(1~3), CAN接收ID, 电机类型, 存储索引 }
// 存储索引请使用上面定义的宏，例如 MOTOR_2006_ID5_INDEX 等
// 注意：store_index 必须唯一且小于 MOTOR_NUM
const Motor_Config_t motor_configs[] = {//////////////注释掉实际没有用到的电机
    //3508电机（索引0~7）
    {1, 0x201, MOTOR_3508, MOTOR_3508_ID1_INDEX},
    {1, 0x202, MOTOR_3508, MOTOR_3508_ID2_INDEX},
    {1, 0x203, MOTOR_3508, MOTOR_3508_ID3_INDEX},
    {1, 0x204, MOTOR_3508, MOTOR_3508_ID4_INDEX},
    // {1, 0x205, MOTOR_3508, MOTOR_3508_ID5_INDEX},
    // {1, 0x206, MOTOR_3508, MOTOR_3508_ID6_INDEX},
    // {1, 0x207, MOTOR_3508, MOTOR_3508_ID7_INDEX},
    // {1, 0x208, MOTOR_3508, MOTOR_3508_ID8_INDEX},

    //6020电机（索引8~14）
    {2, 0x205, MOTOR_6020, MOTOR_6020_ID1_INDEX},
    {2, 0x206, MOTOR_6020, MOTOR_6020_ID2_INDEX},
    {2, 0x207, MOTOR_6020, MOTOR_6020_ID3_INDEX},
    {2, 0x208, MOTOR_6020, MOTOR_6020_ID4_INDEX},
    // {3, 0x209, MOTOR_6020, MOTOR_6020_ID5_INDEX},
    // {3, 0x20A, MOTOR_6020, MOTOR_6020_ID6_INDEX},
    // {3, 0x20B, MOTOR_6020, MOTOR_6020_ID7_INDEX},

    // 2006电机（索引15~22）
    //{1, 0x201, MOTOR_2006, MOTOR_2006_ID1_INDEX},
    //{1, 0x202, MOTOR_2006, MOTOR_2006_ID2_INDEX},
    // {1, 0x203, MOTOR_2006, MOTOR_2006_ID3_INDEX},
    // {1, 0x204, MOTOR_2006, MOTOR_2006_ID4_INDEX},
    //{1, 0x205, MOTOR_2006, MOTOR_2006_ID5_INDEX},
    // {1, 0x206, MOTOR_2006, MOTOR_2006_ID6_INDEX},
    // {1, 0x207, MOTOR_2006, MOTOR_2006_ID7_INDEX},
    // {1, 0x208, MOTOR_2006, MOTOR_2006_ID8_INDEX},
};

// 配置表项数（自动计算）
const uint8_t motor_config_count = sizeof(motor_configs) / sizeof(motor_configs[0]);

/* =========================== 内部辅助函数 =========================== */
/**
 * @brief 根据端口号获取对应的CAN句柄
 * @param port 1~3
 * @return CAN句柄指针，若无效则返回NULL
 */
static FDCAN_HandleTypeDef* get_can_handle(uint8_t port) {
    switch (port) {
        case 1: return &hfdcan1;
        case 2: return &hfdcan2;
        case 3: return &hfdcan3;
        default: return NULL;
    }
}

/**
 * @brief 判断CAN句柄是否已初始化
 */
static uint8_t is_can_initialized(FDCAN_HandleTypeDef *hfdcan) {
    return (hfdcan != NULL && hfdcan->Instance != NULL);
}

/* =========================== 电机初始化 =========================== */
HAL_StatusTypeDef Motor_Init(void) {
    // 1. 初始化反馈数组（清零）
    memset(motor_feedback, 0, sizeof(motor_feedback));

    // 2. 根据配置表预设电机类型（可选，接收时还会再设置）
    for (uint8_t i = 0; i < motor_config_count; i++) {
        uint8_t idx = motor_configs[i].store_index;
        if (idx < MOTOR_NUM) {
            motor_feedback[idx].type = motor_configs[i].type;
        }
    }

    // 3. 停止所有已激活CAN口上的所有电机组（安全起见）
    uint32_t all_group_ids[] = {0x200, 0x1FF, 0x1FE, 0x2FE};
    int num_groups = sizeof(all_group_ids) / sizeof(all_group_ids[0]);

    FDCAN_HandleTypeDef* can_handles[] = {&hfdcan1, &hfdcan2, &hfdcan3};
    for (int i = 0; i < 3; i++) {
        if (is_can_initialized(can_handles[i])) {
            for (int j = 0; j < num_groups; j++) {
                Motor_SendCurrent_Ex(can_handles[i], all_group_ids[j], 0, 0, 0, 0);
            }
        }
    }

    Serial_Printf("Motor init success (%d configs)\n", motor_config_count);
    return HAL_OK;
}

/* =========================== 电流发送函数 =========================== */
HAL_StatusTypeDef Motor_SendCurrent_Ex(FDCAN_HandleTypeDef *hfdcan,
                                        uint32_t group_id,
                                        int16_t current1, int16_t current2,
                                        int16_t current3, int16_t current4) {
    uint8_t data[8];
    data[0] = current1 >> 8;
    data[1] = current1;
    data[2] = current2 >> 8;
    data[3] = current2;
    data[4] = current3 >> 8;
    data[5] = current3;
    data[6] = current4 >> 8;
    data[7] = current4;
    return FDCAN_Send(hfdcan, group_id, FDCAN_STANDARD_ID, data);
}

/* =========================== 角度目标设置 =========================== */
void Motor_SetAngleTarget(uint8_t motor_index, float target_angle, int16_t target_loop) {
    if (motor_index >= MOTOR_NUM) return;
    Motor_Feedback_t *fb = &motor_feedback[motor_index];
    fb->angle_target = target_angle;
    fb->loop_target = target_loop;

    // 如果是绝对编码器（6020），直接计算最短路径角度差
    if (fb->type == MOTOR_6020) {
        float diff = target_angle - fb->angle;
        if (diff > 180.0f) diff -= 360.0f;
        else if (diff < -180.0f) diff += 360.0f;
        fb->angle_diff = diff;
    }
    // 增量编码器的角度差可在需要时通过 loop*360+angle 计算
}

/* =========================== 反馈接收函数（核心，查表方式） =========================== */
void Motor_ReceiveFeedback(FDCAN_HandleTypeDef *hfdcan,
                            FDCAN_RxHeaderTypeDef *RxHeader,
                            uint8_t *RxData) {
    uint32_t id = RxHeader->Identifier;

    // 1. 遍历配置表，查找匹配的电机
    for (uint8_t i = 0; i < motor_config_count; i++) {
        if (get_can_handle(motor_configs[i].can_port) == hfdcan &&
            motor_configs[i].can_rx_id == id) {

            uint8_t idx = motor_configs[i].store_index;
            if (idx >= MOTOR_NUM) break; // 索引无效，跳过

            Motor_Feedback_t *fb = &motor_feedback[idx];
            Motor_Type type = motor_configs[i].type;
            fb->type = type;   // 确保类型正确

            // 保存上一次的角度（仅增量编码器需要）
            if (type != MOTOR_6020) {
                fb->last_angle = fb->angle;
            }

            // 2. 解析角度 (编码器值 0~8191 -> 0~360度)
            uint16_t encoder_val = (RxData[0] << 8) | RxData[1];
            fb->angle = (float)encoder_val / ENCODER_PER_ROUND * 360.0f;

            // 3. 解析转速 (rpm) 和电流
            fb->speed = (int16_t)((RxData[2] << 8) | RxData[3]);
            fb->current = (int16_t)((RxData[4] << 8) | RxData[5]);
            fb->temperature = RxData[6];

            // 4. 根据电机类型处理圈数和角度差
            if (type == MOTOR_6020) {
                // 6020 绝对编码器：计算最短路径角度差
                float diff = fb->angle_target - fb->angle;
                if (diff > 180.0f) diff -= 360.0f;
                else if (diff < -180.0f) diff += 360.0f;
                fb->angle_diff = diff;
                fb->loop = 0;   // 6020 不使用圈数
            } else {
                // 3508 / 2006 增量编码器：计算圈数变化
                float delta = fb->angle - fb->last_angle;
                if (delta < -300.0f) {
                    fb->loop++;      // 正向过零
                } else if (delta > 300.0f) {
                    fb->loop--;      // 反向过零
                }
                // 注意：这里不计算 angle_diff，需要时外部通过 loop*360+angle 计算
            }
            break;  // 找到匹配项，退出循环
        }
    }
    // 如果未在配置表中找到匹配，则丢弃该帧（不处理）
}