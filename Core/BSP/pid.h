#ifndef PID_H
#define PID_H

#include <stdint.h>
#include <stdbool.h>


// PID控制器结构体
typedef struct {
    float Kp;           // 比例系数
    float Ki;           // 积分系数
    float Kd;           // 微分系数
    float target;       // 目标值
    float actual;       // 实际值
    float error;        // 当前误差
    float last_error;   // 上次误差
    float integral;     // 积分项
    float output;       // 输出值
    float max_output;   // 最大输出限制
    float max_integral; // 最大积分限制
} PID_Controller;

// 增量式PID控制器结构体
typedef struct {
    float Kp;           // 比例系数
    float Ki;           // 积分系数
    float Kd;           // 微分系数
    float target;       // 目标值
    float actual;       // 实际值
    float error;        // 当前误差
    float last_error;   // 上次误差
    float prev_error;   // 上上次误差
    float output;       // 输出值
    float delta_output; // 增量输出
    float max_output;   // 最大输出限制
    float max_delta;    // 最大增量限制
} Incremental_PID_Controller;

// 增量式PID级联控制器组
typedef struct {
    Incremental_PID_Controller outer;  // 外层角度环
    Incremental_PID_Controller inner;  // 内层速度环
    float output;                      // 最终输出
} Incremental_PID_Controller_Group;

// 普通PID级联控制器组
typedef struct {
    PID_Controller outer;  // 外层角度环
    PID_Controller inner;  // 内层速度环
    float output;          // 最终输出
} PID_Controller_Group;

// 普通PID函数声明
void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd,
              float max_output, float max_integral);
float PID_Calculate(PID_Controller *pid, float target, float actual);
float pid_CascadeCalc(PID_Controller_Group *pid, float angle_target,
                      float angle_actual, float motor_speed);

// 增量式PID函数声明
void Incremental_PID_Init(Incremental_PID_Controller *pid, float Kp, float Ki, float Kd,
                         float max_output, float max_delta);
float Incremental_PID_Calculate(Incremental_PID_Controller *pid, float target,
                               float actual, bool is_angle);
float incremental_pid_CascadeCalcWithFeedforward(Incremental_PID_Controller_Group *pid,
                                                  float angle_target, float angle_actual,
                                                  float motor_speed, float speed_feedforward);

void SendPIDDataToPC(float setpoint, float input, float index,           //发送到电脑
                    float error, float p, float i, float d);

void ProcessAICommand(char* command, float* new_kp, float* new_ki, float* new_kd); // 处理AI命令
#endif // PID_H