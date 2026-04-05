///radio master

#include "elrs.h"
#include <stdio.h>
#include <serial.h>
#include "string.h"
#include "usart.h"

//uint8_t rx_buff[BUFF_SIZE];
__attribute__((section(".ram_d1"))) uint8_t rx_buff[BUFF_SIZE];

remoter_t remoter;

HAL_StatusTypeDef ELRS_Init(void) {
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart10, rx_buff, BUFF_SIZE) != HAL_OK) {
        return HAL_ERROR;
    }
    printf("ERLS init success!\n");
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
}




void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART10) {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart10, rx_buff, BUFF_SIZE);
        memset(rx_buff, 0, BUFF_SIZE);
    }
}
