#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "stm32h7xx_hal.h"


#define MOTOR_NUM 23  // 支持23个电机 (8+7+8)

// 电机类型枚举

typedef enum {
    MOTOR_3508 = 0,  // 增量编码器
    MOTOR_6020 = 1,  // 绝对编码器
    MOTOR_2006 = 2   // 增量编码器
} Motor_Type;

// 电机反馈数据结构

typedef struct {
    float angle;           // 当前角度 (0~360度)
    float last_angle;      // 上一次角度，用于计算圈数（增量编码器用）
    float angle_target;    // 目标角度 (0~360度)
    float angle_diff;      // 角度差值 (自动处理零度穿越)
    int16_t loop;          // 圈数计数（增量编码器用）
    int16_t loop_target;   // 目标圈数
    int16_t speed;         // 转速 (rpm)
    int16_t current;       // 实际电流
    uint8_t temperature;   // 温度 (摄氏度)
    Motor_Type type;       // 电机类型
} Motor_Feedback_t;

// 电机索引宏定义 (简化访问)
#define MOTOR_3508_INDEX(id) ((id) - MOTOR_3508_ID1)
#define MOTOR_6020_INDEX(id) ((id) - MOTOR_6020_ID1 + 8)
#define MOTOR_2006_INDEX(id) ((id) - MOTOR_2006_ID1 + 15)

// 常用电机索引定义
#define MOTOR_3508_ID1_INDEX 0
#define MOTOR_3508_ID2_INDEX 1
#define MOTOR_3508_ID3_INDEX 2
#define MOTOR_3508_ID4_INDEX 3
#define MOTOR_3508_ID5_INDEX 4
#define MOTOR_3508_ID6_INDEX 5
#define MOTOR_3508_ID7_INDEX 6
#define MOTOR_3508_ID8_INDEX 7

#define MOTOR_6020_ID1_INDEX 8
#define MOTOR_6020_ID2_INDEX 9
#define MOTOR_6020_ID3_INDEX 10
#define MOTOR_6020_ID4_INDEX 11
#define MOTOR_6020_ID5_INDEX 12
#define MOTOR_6020_ID6_INDEX 13
#define MOTOR_6020_ID7_INDEX 14

#define MOTOR_2006_ID1_INDEX 15
#define MOTOR_2006_ID2_INDEX 16
#define MOTOR_2006_ID3_INDEX 17
#define MOTOR_2006_ID4_INDEX 18
#define MOTOR_2006_ID5_INDEX 19
#define MOTOR_2006_ID6_INDEX 20
#define MOTOR_2006_ID7_INDEX 21
#define MOTOR_2006_ID8_INDEX 22

extern Motor_Feedback_t motor_feedback[MOTOR_NUM];

// 电机ID枚举 (明确枚举所有ID)
typedef enum {
    // 3508电机组 (8个) - 发送ID
    MOTOR_3508_GROUP1 = 0x200,
    MOTOR_3508_GROUP2 = 0x1FF,

    // 3508电机接收ID (8个)
    MOTOR_3508_ID1 = 0x201,
    MOTOR_3508_ID2 = 0x202,
    MOTOR_3508_ID3 = 0x203,
    MOTOR_3508_ID4 = 0x204,

    MOTOR_3508_ID5 = 0x205,
    MOTOR_3508_ID6 = 0x206,
    MOTOR_3508_ID7 = 0x207,
    MOTOR_3508_ID8 = 0x208,

    // 6020电机组 (7个) - 发送ID
    MOTOR_6020_GROUP1 = 0x1FE,
    MOTOR_6020_GROUP2 = 0x2FE,

    // 6020电机接收ID (7个)
    MOTOR_6020_ID1 = 0x205,
    MOTOR_6020_ID2 = 0x206,
    MOTOR_6020_ID3 = 0x207,
    MOTOR_6020_ID4 = 0x208,
    MOTOR_6020_ID5 = 0x209,
    MOTOR_6020_ID6 = 0x20A,
    MOTOR_6020_ID7 = 0x20B,

    // 2006电机组 (8个) - 发送ID (与3508相同)
    MOTOR_2006_GROUP1 = 0x200,
    MOTOR_2006_GROUP2 = 0x1FF,

    // 2006电机接收ID (8个) - 与3508相同
    MOTOR_2006_ID1 = 0x201,
    MOTOR_2006_ID2 = 0x202,
    MOTOR_2006_ID3 = 0x203,
    MOTOR_2006_ID4 = 0x204,

    MOTOR_2006_ID5 = 0x205,
    MOTOR_2006_ID6 = 0x206,
    MOTOR_2006_ID7 = 0x207,
    MOTOR_2006_ID8 = 0x208
} Motor_ID;

extern Motor_Feedback_t motor_feedback[MOTOR_NUM];

// 函数声明
HAL_StatusTypeDef Motor_Init(void);
HAL_StatusTypeDef Motor_SendCurrent_3508(uint32_t group_id, int16_t current1, int16_t current2,
                                         int16_t current3, int16_t current4);
HAL_StatusTypeDef Motor_SendCurrent_6020(uint32_t group_id, int16_t current1, int16_t current2,
                                         int16_t current3, int16_t current4);
HAL_StatusTypeDef Motor_SendCurrent_2006(uint32_t group_id, int16_t current1, int16_t current2,
                                         int16_t current3, int16_t current4);
void Motor_SetAngleTarget(uint8_t motor_index, float target_angle, int16_t target_loop);
void Motor_RecieveFeedback(FDCAN_RxHeaderTypeDef *RxHeader, uint8_t *RxData);

#endif /* __MOTOR_H__ */