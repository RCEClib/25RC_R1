#include "pid.h"
#include <math.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "usart.h"




// 普通PID初始化函数
void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd,
              float max_output, float max_integral) {
    pid->Kp           = Kp;
    pid->Ki           = Ki;
    pid->Kd           = Kd;
    pid->max_output   = max_output;
    pid->max_integral = max_integral;
    pid->target       = 0.0f;
    pid->actual       = 0.0f;
    pid->error        = 0.0f;
    pid->last_error   = 0.0f;
    pid->integral     = 0.0f;
    pid->output       = 0.0f;
}

// 普通PID计算函数
float PID_Calculate(PID_Controller *pid, float target, float actual) {

    if (pid == NULL) return 0.0f;  //空指针防护，防止死机

    pid->target = target;
    pid->actual = actual;
    pid->error = pid->target - pid->actual;

    float P = pid->Kp * pid->error;

    pid->integral += pid->error;
    if (pid->integral > pid->max_integral)
        pid->integral = pid->max_integral;
    else if (pid->integral < -pid->max_integral)
        pid->integral = -pid->max_integral;
    float I = pid->Ki * pid->integral;

    float D = pid->Kd * (pid->error - pid->last_error);
    pid->last_error = pid->error;

    pid->output = P + I + D;
    if (pid->output > pid->max_output)
        pid->output = pid->max_output;
    else if (pid->output < -pid->max_output)
        pid->output = -pid->max_output;

    return pid->output;
}

// 普通PID级联计算函数
float pid_CascadeCalc(PID_Controller_Group *pid, float angle_target,
                      float angle_actual, float motor_speed) {

    if (pid == NULL) return 0.0f;  //空指针防护，防止死机

    PID_Calculate(&pid->outer, angle_target, angle_actual);
    PID_Calculate(&pid->inner, pid->outer.output, motor_speed);
    pid->output = pid->inner.output;
    return pid->output;
}

// 增量式PID初始化函数
void Incremental_PID_Init(Incremental_PID_Controller *pid, float Kp, float Ki, float Kd,
                         float max_output, float max_delta) {

    if (pid == NULL) return;  //空指针防护

    pid->Kp           = Kp;
    pid->Ki           = Ki;
    pid->Kd           = Kd;
    pid->max_output   = max_output;
    pid->max_delta    = max_delta;
    pid->target       = 0.0f;
    pid->actual       = 0.0f;
    pid->error        = 0.0f;
    pid->last_error   = 0.0f;
    pid->prev_error   = 0.0f;
    pid->output       = 0.0f;
    pid->delta_output = 0.0f;
}

// 通用增量式PID计算函数
// @param is_angle: true-角度控制（需要归一化），false-速度控制（不需要归一化）
float Incremental_PID_Calculate(Incremental_PID_Controller *pid, float target,
                               float actual, bool is_angle) {

    if (pid == NULL) return 0.0f;  //空指针防护

    pid->target = target;
    pid->actual = actual;

    if (is_angle) {
        float error = target - actual;

        // 强制把误差限制在 [-180, 180] 度之间
        while (error > 180.0f)  error -= 360.0f;
        while (error < -180.0f) error += 360.0f;

        pid->error = error;
    } else {
        pid->error = target - actual;
    }

    // 增量式PID公式: Δu(k) = Kp*(e(k)-e(k-1)) + Ki*e(k) + Kd*(e(k)-2e(k-1)+e(k-2))
    float delta_P = pid->Kp * (pid->error - pid->last_error);
    float delta_I = pid->Ki * pid->error;
    float delta_D = pid->Kd * (pid->error - 2 * pid->last_error + pid->prev_error);

    pid->delta_output = delta_P + delta_I + delta_D;

    // 增量输出限幅
    if (pid->delta_output > pid->max_delta)
        pid->delta_output = pid->max_delta;
    else if (pid->delta_output < -pid->max_delta)
        pid->delta_output = -pid->max_delta;

    // 更新总输出
    pid->output += pid->delta_output;

    // 总输出限幅
    if (pid->output > pid->max_output)
        pid->output = pid->max_output;
    else if (pid->output < -pid->max_output)
        pid->output = -pid->max_output;

    // 更新误差历史
    pid->prev_error = pid->last_error;
    pid->last_error = pid->error;

    return pid->output;
}

// 增量式PID级联计算函数
// @param speed_feedforward: 外部传入的目标速度（前馈项）
float incremental_pid_CascadeCalcWithFeedforward(Incremental_PID_Controller_Group *pid,
                                                  float angle_target, float angle_actual,
                                                  float motor_speed, float speed_feedforward) {

    if (pid == NULL) return 0.0f;  //空指针防护

    // 外层角度环计算（需要角度归一化）
    Incremental_PID_Calculate(&pid->outer, angle_target, angle_actual, true);

    // 融合外部目标速度（前馈）与外环输出
    float inner_target = pid->outer.output + speed_feedforward;

    if (inner_target > pid->inner.max_output)
        inner_target = pid->inner.max_output;
    if (inner_target < -pid->inner.max_output)
        inner_target = -pid->inner.max_output;

    // 内层速度环计算（不需要角度归一化）
    Incremental_PID_Calculate(&pid->inner, inner_target, motor_speed, false);

    // 最终输出为内层输出
    pid->output = pid->inner.output;

    return pid->output;
}

void SendPIDDataToPC(float setpoint, float input, float index,           //发送到电脑
                    float error, float p, float i, float d) {
    uint32_t timestamp = HAL_GetTick();
    char buffer[128];

    sprintf(buffer, "%lu,%.2f,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f\n",
            timestamp, setpoint, input, index, error, p, i, d);

    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

void ProcessAICommand(char* command, float* new_kp, float* new_ki, float* new_kd) {

    if (strncmp(command, "PID,", 4) == 0) /* 检查命令是否以 "PID," 开头 */{
        sscanf(command, "PID,%f,%f,%f", new_kp, new_ki, new_kd); /* 解析命令参数 */
        printf("AI调参完成: P=%.4f, I=%.4f, D=%.4f\n", *new_kp, *new_ki, *new_kd);
    } else {
        *new_kp = *new_ki = *new_kd = 0.0f;
    }
}

// // 调用示例
// float kp, ki, kd;
// ProcessAICommand("PID,2.5,0.3,0.15", &kp, &ki, &kd);
// 现在 kp=2.5, ki=0.3, kd=0.15


