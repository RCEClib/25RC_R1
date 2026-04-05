#ifndef __CHASSIS_RUDDER_H__
#define __CHASSIS_RUDDER_H__
#include <stdint.h>
#include "pid.h"

extern float imu_yaw_rad;
extern float target_speed;
extern float actual_speed;
extern float outspeed;
extern float speed_dps;
extern float actual_angle;
extern float target_angle_deg;

// 速度限制（可调）
// #define MAX_LINEAR_SPEED   1.0f   // 最大线速度 (m/s)，根据底盘性能调整
// #define MAX_ANGULAR_SPEED  2.0f   // 最大角速度 (rad/s)

#define MOTOR1CENTER 211.0f  // 前左舵轮0位
#define MOTOR2CENTER 271.0f  // 前右舵轮0位
#define MOTOR3CENTER 358.0f  // 后左舵轮0位
#define MOTOR4CENTER 330.0f  // 后右舵轮0位

#define RAD2DEG (180.0f / M_PI)    // 弧度 → 角度 (180/PI)
#define DEG2RAD (M_PI / 180.0f)    // 角度 → 弧度 (PI/180)

typedef enum {
    STOP_MODE = 0,        // 停止模式
    NORMAL_MODE = 1,      // 正常模式
} Chassis_Mode;

typedef struct {
    Chassis_Mode Mode;
    float Target_Vx;
    float Target_Vy;
    float Target_Wr;
    Incremental_PID_Controller_Group rudder_pid[4];     // 舵向增量式PID
    PID_Controller wheel_pid[4];                        // 轮向位置式PID
    int16_t rudder_currents[4];                         // 舵向电机目标电流
    int16_t wheel_currents[4];                          // 轮向电机目标电流
    int8_t rudder_direction_calibration[4];             // 6020舵向电机方向校准
    int8_t wheel_direction_calibration[4];              // 3508轮向电机方向校准
} Chassis_Rudder_t;



#ifdef __cplusplus
extern "C"
{
#include "STM32Hardware.h"
#endif
    void Chassis_Rudder_Init(Chassis_Rudder_t *chassis);
    void steering_wheel_solve(float Target_V_x, float Target_V_y, float Target_W);
    void Chassis_Rudder_Control(Chassis_Rudder_t *chassis);
    void Chassis_Rudder_Task(Chassis_Rudder_t *chassis, uint8_t en,
                            Chassis_Mode mode, float vx, float vy, float vw);

#ifdef __cplusplus
}
#endif
#endif
