//
// Created by zaiyad0 on 2026/4/10.
//
#include "main.h"
#include <stdio.h>
#include "usart.h"
#include "stdint.h"
#include "string.h"
#include "PID.h"
#include "motor.h"

#define RX_BUFFER_SIZE 256
uint8_t rx[RX_BUFFER_SIZE];      // 改名：rx_buffer_pid -> rx
uint8_t rx_pid = 0;


// uint8_t rx_buffer_pid[256];
// uint8_t rx_index_pid = 0;

PID_Controller my_pid;
float current_cmd = 0.0f;


float new_kp, new_ki, new_kd;
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

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        // 检查是否收到换行符（结束标志）
        // if (rx_buffer_pid[rx_index_pid - 1] == '\n' || rx_buffer_pid[rx_index_pid - 1] == '\r') {
        // rx_buffer_pid[rx_index_pid - 1] = '\0';  // 替换换行符为结束符

        // 解析AI命令
        ProcessAICommand((char*)1, &new_kp, &new_ki, &new_kd);//进入回调函数 先进行数据解析

        // 更新PID参数
        my_pid.Kp = new_kp;
        my_pid.Ki = new_ki;
        my_pid.Kd = new_kd;

        PID_Init(&my_pid, new_kp, new_ki, new_kd, 1000, 1000);  //将值赋值给pid控制器
        // 计算PID
        current_cmd = PID_Calculate(&my_pid, 1000,
                                    motor_feedback[MOTOR_3508_ID1_INDEX].speed);  //计算pid输出电流

        Motor_SendCurrent_3508(MOTOR_3508_GROUP1, current_cmd, current_cmd, current_cmd, current_cmd);

        // 发送调试数据到PC
        SendPIDDataToPC(1000,//目标速度
            motor_feedback[MOTOR_3508_ID1_INDEX].speed,//反馈速度
            current_cmd,
            1000 - motor_feedback[MOTOR_3508_ID1_INDEX].speed,//误差：1000输入减去反馈速度
            my_pid.Kp, my_pid.Ki, my_pid.Kd);//定义pid数值

        // 继续接收下一个字节
        HAL_UART_Receive_IT(&huart1, &rx[rx_pid++], 1);
    }
}