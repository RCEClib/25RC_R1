///radio master

#include "elrs.h"
#include <stdio.h>
#include <serial.h>
#include "string.h"
#include "usart.h"
#include "pid.h"
#include "chassis_rudder.h"
#include "main.h"
#include "stm32h7xx_hal.h"
#include "motor.h"

//uint8_t rx_buff[BUFF_SIZE];
__attribute__((section(".ram_d1"))) uint8_t rx_buff[BUFF_SIZE];

uint8_t rx_buffer_pid[256];
uint8_t rx_index_pid = 0;

PID_Controller my_pid;
float current_cmd = 0.0f;


float new_kp, new_ki, new_kd;

remoter_t remoter;

HAL_StatusTypeDef ELRS_Init(void) {
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart10, rx_buff, BUFF_SIZE) != HAL_OK) {
        return HAL_ERROR;
    }
    printf("ERLS init success!\n");
    HAL_UART_Receive_IT(&huart1, &rx_buffer_pid[rx_index_pid], 1);//重启接收
    return HAL_OK;
}

float float_Map(float input_value, float input_min, float input_max,
                float output_min, float output_max) {
    float output_value;
    if (input_value < input_min) {
        output_value = output_min;
    } else if (input_value > input_max) {
        output_value = output_max;
    } else {
        output_value = output_min + (input_value - input_min) *
                                        (output_max - output_min) /
                                        (input_max - input_min);
    }
    return output_value;
}

float float_Map_with_median(float input_value, float input_min, float input_max,
                            float median, float output_min, float output_max) {
    float output_median = (output_max - output_min) / 2 + output_min;
    if (input_min >= input_max || output_min >= output_max ||
        median <= input_min || median >= input_max) {
        return output_min;
    }

    if (input_value < median) {
        return float_Map(input_value, input_min, median, output_min,
                         output_median);
    } else {
        return float_Map(input_value, median, input_max, output_median,
                         output_max);
    }
}

void sbus_frame_parse(remoter_t *remoter, uint8_t *buf) {
    if ((buf[0] != CRSF_ADDR) || (buf[2] != CRSF_TYPE)) return;

    remoter->rc.ch[0] =
        ((uint16_t)buf[3] >> 0 | ((uint16_t)buf[4] << 8)) & 0x07FF;
    remoter->rc.ch[1] =
        ((uint16_t)buf[4] >> 3 | ((uint16_t)buf[5] << 5)) & 0x07FF;
    remoter->rc.ch[2] = ((uint16_t)buf[5] >> 6 | ((uint16_t)buf[6] << 2) |
                         ((uint16_t)buf[7] << 10)) &
                        0x07FF;
    remoter->rc.ch[3] =
        ((uint16_t)buf[7] >> 1 | ((uint16_t)buf[8] << 7)) & 0x07FF;
    remoter->rc.ch[4] =
        ((uint16_t)buf[8] >> 4 | ((uint16_t)buf[9] << 4)) & 0x07FF;
    remoter->rc.ch[5] = ((uint16_t)buf[9] >> 7 | ((uint16_t)buf[10] << 1) |
                         ((uint16_t)buf[11] << 9)) &
                        0x07FF;
    remoter->rc.ch[6] =
        ((uint16_t)buf[11] >> 2 | ((uint16_t)buf[12] << 6)) & 0x07FF;
    remoter->rc.ch[7] =
        ((uint16_t)buf[12] >> 5 | ((uint16_t)buf[13] << 3)) & 0x07FF;
    remoter->rc.ch[8] =
        ((uint16_t)buf[14] >> 0 | ((uint16_t)buf[15] << 8)) & 0x07FF;
    remoter->rc.ch[9] =
        ((uint16_t)buf[15] >> 3 | ((uint16_t)buf[16] << 5)) & 0x07FF;
    remoter->rc.ch[10] = ((uint16_t)buf[16] >> 6 | ((uint16_t)buf[17] << 2) |
                          ((uint16_t)buf[18] << 10)) &
                         0x07FF;
    remoter->rc.ch[11] =
        ((uint16_t)buf[18] >> 1 | ((uint16_t)buf[19] << 7)) & 0x07FF;
    remoter->rc.ch[12] =
        ((uint16_t)buf[19] >> 4 | ((uint16_t)buf[20] << 4)) & 0x07FF;
    remoter->rc.ch[13] = ((uint16_t)buf[20] >> 7 | ((uint16_t)buf[21] << 1) |
                          ((uint16_t)buf[22] << 9)) &
                         0x07FF;
    remoter->rc.ch[14] =
        ((uint16_t)buf[22] >> 2 | ((uint16_t)buf[23] << 6)) & 0x07FF;
    remoter->rc.ch[15] =
        ((uint16_t)buf[23] >> 5 | ((uint16_t)buf[24] << 3)) & 0x07FF;

    remoter->joy.l_y =
        float_Map_with_median(remoter->rc.ch[3], 174, 1811, 992, -100, 100);
    remoter->joy.l_x =
        float_Map_with_median(remoter->rc.ch[2], 174, 1811, 992, -100, 100);
    remoter->joy.r_y =
        float_Map_with_median(remoter->rc.ch[0], 174, 1811, 992, -100, 100);
    remoter->joy.r_x =
        float_Map_with_median(remoter->rc.ch[1], 174, 1811, 992, -100, 100);

    remoter->key.SA = remoter->rc.ch[5] == 997 ? 1 : (remoter->rc.ch[5] == 1792 ? 2 : 0);
    remoter->key.SB =remoter->rc.ch[6] == 997 ? 1 : (remoter->rc.ch[6] == 1792 ? 2 : 0);

    remoter->key.SC =remoter->rc.ch[7] == 997 ? 1 : (remoter->rc.ch[7] == 1792 ? 2 : 0);

    remoter->key.SD = remoter->rc.ch[8] == 997 ? 1 : (remoter->rc.ch[8] == 1792 ? 2 : 0);

    remoter->key.SE = remoter->rc.ch[4] > 1000 ? 1 : 0;
    remoter->key.SF = remoter->rc.ch[9] > 1000 ? 1 : 0;

    remoter->var.S1 =
        float_Map_with_median(remoter->rc.ch[10], 191, 1792, 992, -100, 100);
    remoter->var.S2 =
        float_Map_with_median(remoter->rc.ch[11], 191, 1792, 992, -100, 100);
}


// void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
//     if (huart->Instance == USART10) {
//         if (Size <= BUFF_SIZE) {
//             HAL_UARTEx_ReceiveToIdle_DMA(&huart10, rx_buff, BUFF_SIZE);
//             sbus_frame_parse(&remoter, rx_buff);
//         } else {
//             HAL_UARTEx_ReceiveToIdle_DMA(&huart10, rx_buff, BUFF_SIZE);
//             memset(rx_buff, 0, BUFF_SIZE);
//         }
//     }
// }


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART10) {

        if (Size > 0 && Size <= BUFF_SIZE) {
            sbus_frame_parse(&remoter, rx_buff);      // 先解析
        }
        HAL_UARTEx_ReceiveToIdle_DMA(&huart10, rx_buff, BUFF_SIZE);  // 再重启
    }

//     if (huart->Instance == USART1) {                     //处理ai数据部分
//
//         current_cmd = PID_Calculate(&my_pid, remoter.joy.l_y,motor_feedback[MOTOR_3508_ID1_INDEX].speed);
//         if (received_bufferpid[buffer_index_pid] == '\n') {
//             received_bufferpid[buffer_index_pid] = '\0';  // 替换换行符为结束符
//
//
//             ProcessAICommand((char*)received_bufferpid, &new_kp, &new_ki, &new_kd);//接收后处理ai数据
//
//             for (int i = 0; i < 4; i++) {
//                 my_pid.Kp = new_kp;
//                 my_pid.Ki = new_ki;
//                 my_pid.Kd = new_kd;       //赋值pid
//             }
// //数据格式：遥控器输入,实际值（编码器返回）,pid计算输出,误差,P,I,D\n
//             SendPIDDataToPC(10
//                 , motor_feedback[MOTOR_3508_ID1_INDEX].speed, current_cmd, remoter.joy.l_y - motor_feedback[MOTOR_3508_ID1_INDEX].speed,my_pid.Kp,my_pid.Ki,my_pid.Kd);
//         }
//             if (buffer_index_pid >= sizeof(received_bufferpid)) {
//                 buffer_index_pid = 0;  // 防止溢出
//             }
//             HAL_UART_Receive_IT(&huart1, &received_bufferpid[buffer_index_pid], 1);//重启接收
//     }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        // 检查是否收到换行符（结束标志）
        // if (rx_buffer_pid[rx_index_pid - 1] == '\n' || rx_buffer_pid[rx_index_pid - 1] == '\r') {
            // rx_buffer_pid[rx_index_pid - 1] = '\0';  // 替换换行符为结束符

            // 解析AI命令
            ProcessAICommand((char*)rx_buffer_pid, &new_kp, &new_ki, &new_kd);

            // 更新PID参数
            my_pid.Kp = new_kp;
            my_pid.Ki = new_ki;
            my_pid.Kd = new_kd;

            // 计算PID
            current_cmd = PID_Calculate(&my_pid, remoter.joy.l_y,
                                        motor_feedback[MOTOR_3508_ID1_INDEX].speed);

            // 发送调试数据到PC
            SendPIDDataToPC(HAL_GetTick(),
                motor_feedback[MOTOR_3508_ID1_INDEX].speed,
                current_cmd,
                remoter.joy.l_y - motor_feedback[MOTOR_3508_ID1_INDEX].speed,
                my_pid.Kp, my_pid.Ki, my_pid.Kd);

            // 重置索引，准备接收下一帧
            rx_index_pid = 0;
        // }

        // 检查缓冲区溢出
        if (rx_index_pid >= BUFF_SIZE) {
            rx_index_pid = 0;  // 溢出则丢弃当前帧，重新开始
        }

        // 继续接收下一个字节
        HAL_UART_Receive_IT(&huart1, &rx_buffer_pid[rx_index_pid++], 1);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART10) {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart10, rx_buff, BUFF_SIZE);
        memset(rx_buff, 0, BUFF_SIZE);
    }
}
