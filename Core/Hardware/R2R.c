#include "elrs.h"
#include "stm32h7xx.h"
#include "Serial.h"

// 0或1作为字符串发送，加换行方便查看
static char sf_tx_buf[4];
static uint8_t last_sf_state = 0;
// 接收缓冲区
static uint8_t rx_data[1];

void R2R_Init() {
    last_sf_state = remoter.key.SF;
    HAL_UART_Receive_IT(&huart7, rx_data, 1);
}

// 在主循环中周期调用，检测到 SF 变化就通过 UART7 发送字符串
void R2R_Update() {
    uint8_t cur_sf = remoter.key.SF;
    if (cur_sf != last_sf_state) {
        last_sf_state = cur_sf;
        sf_tx_buf[0] = '0' + cur_sf;
        sf_tx_buf[1] = '\r';
        sf_tx_buf[2] = '\n';
        HAL_UART_Transmit_IT(&huart7, (uint8_t *)sf_tx_buf, 3);
    }
}

// UART7 接收中断回调——收到1字节回显并继续等待接收
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART7)
    {
        // HAL_UART_Transmit_IT(&huart7, rx_data, 1);
        HAL_UART_Receive_IT(&huart7, rx_data, 1);

        if (rx_data[0] == '1') {
         Serial_Printf("R2R: 1\n");
        }
    }
}
