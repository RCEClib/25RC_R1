#ifndef __CHASSIS_RUDDER_H__
#define __CHASSIS_RUDDER_H__
#include <stdint.h>
#include "pid.h"

extern float target_speed;      // 轮速PID目标值（RPM）
extern float actual_speed;      // 轮速PID反馈值（RPM）
extern float outspeed;          // 舵向PID外环输出（度/秒）
extern float speed_dps;         // 舵电机实际速度（度/秒）
extern float actual_angle;      // 舵电机实际角度（度）
extern float target_angle_deg;  // 舵电机目标角度（度）

// 速度限制（可调）
// #define MAX_LINEAR_SPEED   1.0f   // 最大线速度 (m/s)，根据底盘性能调整
// #define MAX_ANGULAR_SPEED  2.0f   // 最大角速度 (rad/s)

// 顺序：0-左前，1-左后，2-右后，3-右前（逆时针）
#define MOTOR0CENTER 240.0f   // 左前舵轮零点
#define MOTOR1CENTER 91.0f   // 左后舵轮零点
#define MOTOR2CENTER 302.0f   // 右后舵轮零点
#define MOTOR3CENTER 117.0f   // 右前舵轮零点

#define RAD2DEG (180.0f / M_PI)    // 弧度 → 角度 (180/PI)
#define DEG2RAD (M_PI / 180.0f)    // 角度 → 弧度 (PI/180)

typedef enum {
    STOP_MODE = 0,        // 停止模式
    NORMAL_MODE = 1,      // 正常模式
    GYRO_MODE = 2,        // 小陀螺模式
} Chassis_Mode;

typedef struct {
    Chassis_Mode Mode;
    float Target_Vx;          // 目标车体X速度（米/秒，向前为正）
    float Target_Vy;          // 目标车体Y速度（米/秒，向左为正）
    float Target_Wr;          // 目标自转角速度（弧度/秒，逆时针为正）
    float spin_rate;   // 小陀螺自转角速度（弧度/秒，逆时针为正）

    Incremental_PID_Controller_Group rudder_pid[4];   // 舵向级联PID（角度环+速度环）
    PID_Controller wheel_pid[4];                      // 轮向速度PID

    int16_t rudder_currents[4];   // 舵电机目标电流（6020电机）
    int16_t wheel_currents[4];    // 轮电机目标电流（3508电机）

    int8_t rudder_direction_calibration[4];  // 舵电机方向校准（1或-1）
    int8_t wheel_direction_calibration[4];   // 轮电机方向校准（1或-1）
} Chassis_Rudder_t;



// #ifdef __cplusplus
// extern "C"
// {
// #include "STM32Hardware.h"
// #endif
    void Chassis_Rudder_Init(Chassis_Rudder_t *chassis);
    void steering_wheel_solve(float Target_V_x, float Target_V_y, float Target_W);
    void Chassis_Rudder_Control(Chassis_Rudder_t *chassis);
    void Chassis_Rudder_Task(Chassis_Rudder_t *chassis, Chassis_Mode mode, float vx, float vy, float vw);


    extern Chassis_Rudder_t chassis;//调参需要的底盘控制结构体


// #ifdef __cplusplus
// }
//#endif
#endif
