/**
 ******************************************************************************
 * @file    MyCAN.c
 * @brief   FDCAN通信驱动程序
 * @details 提供FDCAN初始化、发送、接收和中断回调函数
 ******************************************************************************
 */

#include "fd.h"

#include <stdio.h>

/* 外部函数声明 -------------------------------------------------------------- */
extern void Motor_RecieveFeedback(FDCAN_RxHeaderTypeDef *RxHeader,
                                  uint8_t *RxData);

/**
 * @brief  FDCAN初始化函数
 * @param  hfdcan: FDCAN句柄指针 (&hfdcan1 或 &hfdcan2)
 * @retval HAL_OK: 初始化成功
 *         HAL_ERROR: 初始化失败
 * @note   配置FDCAN滤波器、全局滤波器、启动FDCAN并激活接收中断
 */
HAL_StatusTypeDef FDCAN_Init(FDCAN_HandleTypeDef *hfdcan) {
    /* 配置FDCAN滤波器 */
    FDCAN_FilterTypeDef sFilterConfig = {
        .IdType      = FDCAN_STANDARD_ID,    // 标准ID（11位）
        .FilterIndex = 0,                     // 滤波器索引0
        .FilterType  = FDCAN_FILTER_RANGE     // 范围滤波模式
    };

    // 根据不同的FDCAN实例配置不同的FIFO
    if (hfdcan == &hfdcan1) {
        sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;  // 接收到FIFO0
        sFilterConfig.FilterID1    = 0x000;  // 滤波器ID范围起始值
        sFilterConfig.FilterID2    = 0x7FF;  // 滤波器ID范围结束值（接收所有标准ID）
    } else if (hfdcan == &hfdcan2) {
        sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;  // 接收到FIFO1
        sFilterConfig.FilterID1    = 0x000;  // 滤波器ID范围起始值
        sFilterConfig.FilterID2    = 0x7FF;  // 滤波器ID范围结束值（接收所有标准ID）
    }else if (hfdcan == &hfdcan3) {
        sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;  // 接收到FIFO1
        sFilterConfig.FilterID1    = 0x000;  // 滤波器ID范围起始值
        sFilterConfig.FilterID2    = 0x7FF;  // 滤波器ID范围结束值（接收所有标准ID）
    }

    // 配置滤波器
    if (HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig) != HAL_OK) {
        return HAL_ERROR;
    }

    // 配置全局滤波器：拒绝不匹配的标准ID、扩展ID和远程帧
    if (HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE,
                                     FDCAN_REJECT_REMOTE) != HAL_OK) {
        return HAL_ERROR;
    }

    // 启动FDCAN
    if (HAL_FDCAN_Start(hfdcan) != HAL_OK) {
        return HAL_ERROR;
    }

    // 激活接收中断
    if (hfdcan == &hfdcan1) {
        // 激活FDCAN1的FIFO0新消息中断
        if (HAL_FDCAN_ActivateNotification(
                hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
            return HAL_ERROR;
        }
    } else if (hfdcan == &hfdcan2) {
        // 激活FDCAN2的FIFO1新消息中断
        if (HAL_FDCAN_ActivateNotification(
                hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK) {
            return HAL_ERROR;
        }
    }else if (hfdcan == &hfdcan3) {
        // 激活FDCAN3的FIFO1新消息中断
        if (HAL_FDCAN_ActivateNotification(
                hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK) {
            return HAL_ERROR;
        }
    }
    return HAL_OK;
}

/**
 * @brief  FDCAN发送数据函数
 * @param  hfdcan: FDCAN句柄指针
 * @param  id: CAN消息ID
 * @param  idtype: ID类型 (FDCAN_STANDARD_ID 或 FDCAN_EXTENDED_ID)
 * @param  data: 要发送的数据指针（8字节）
 * @retval HAL_OK: 发送成功
 *         HAL_ERROR: 发送失败
 * @note   使用经典CAN格式，数据长度固定为8字节
 */
HAL_StatusTypeDef FDCAN_Send(FDCAN_HandleTypeDef *hfdcan, uint32_t id,
                             uint32_t idtype, uint8_t *data) {
    /* 配置发送帧头 */
    FDCAN_TxHeaderTypeDef txHeader = {
        .Identifier         = id,                   // CAN消息ID
        .IdType             = idtype,               // ID类型（标准/扩展）
        .TxFrameType        = FDCAN_DATA_FRAME,     // 数据帧
        .DataLength         = FDCAN_DLC_BYTES_8,    // 数据长度8字节
        .BitRateSwitch      = FDCAN_BRS_OFF,        // 不使用位速率切换
        .FDFormat           = FDCAN_CLASSIC_CAN,    // 经典CAN格式
        .TxEventFifoControl = FDCAN_NO_TX_EVENTS,   // 不存储发送事件
        .MessageMarker      = 0                     // 消息标记为0
    };

    // 将消息添加到发送FIFO队列
    return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &txHeader, data);
}

/**
 * @brief  FDCAN接收数据函数
 * @param  hfdcan: FDCAN句柄指针
 * @param  RxHeader: 接收帧头指针，用于存储接收到的帧头信息
 * @param  RxData: 接收数据缓冲区指针，用于存储接收到的数据
 * @retval HAL_OK: 接收成功
 *         HAL_ERROR: 接收失败或FIFO为空
 * @note   此函数为轮询方式接收，通常在中断回调中使用
 */
HAL_StatusTypeDef FDCAN_Receive(FDCAN_HandleTypeDef *hfdcan,
                                FDCAN_RxHeaderTypeDef *RxHeader,
                                uint8_t *RxData) {
    if (hfdcan == &hfdcan1) {
        // 检查FDCAN1的FIFO0是否有数据
        if (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) == 0) {
            return HAL_ERROR;  // FIFO为空
        }
        // 从FIFO0读取消息
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, RxHeader, RxData) !=
            HAL_OK) {
            return HAL_ERROR;
        }
    } else if (hfdcan == &hfdcan2) {
        // 检查FDCAN2的FIFO1是否有数据
        if (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO1) == 0) {
            return HAL_ERROR;  // FIFO为空
        }
        // 从FIFO1读取消息
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, RxHeader, RxData) !=
            HAL_OK) {
            return HAL_ERROR;
        }
    }else if (hfdcan == &hfdcan3) {
        // 检查FDCAN3的FIFO0是否有数据
        if (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) == 0) {
            return HAL_ERROR;  // FIFO为空
        }
        // 从FIFO0读取消息
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, RxHeader, RxData) !=
            HAL_OK) {
            return HAL_ERROR;
            }
    }

    return HAL_OK;
}

/**
 * @brief  FDCAN FIFO0接收中断回调函数
 * @param  hfdcan: FDCAN句柄指针
 * @param  RxFifo0ITs: 触发中断的标志位
 * @retval 无
 * @note   当FIFO0接收到新消息时，此函数会被自动调用
 *         FDCAN1使用FIFO0接收电机反馈数据
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo0ITs) {
    // 检查是否是FIFO0新消息中断
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
        FDCAN_RxHeaderTypeDef RxHeader;  // 接收帧头
        uint8_t RxData[8];               // 接收数据缓冲区


        // 从FIFO0读取消息
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) ==
            HAL_OK) {
            // 如果是FDCAN1，调用电机反馈处理函数
            if (hfdcan == &hfdcan1 || hfdcan == &hfdcan3) {
                Motor_RecieveFeedback(&RxHeader, RxData);
            }
        }
    }
}

/**
 * @brief  FDCAN FIFO1接收中断回调函数
 * @param  hfdcan: FDCAN句柄指针
 * @param  RxFifo1ITs: 触发中断的标志位
 * @retval 无
 * @note   当FIFO1接收到新消息时，此函数会被自动调用
 *         FDCAN2使用FIFO1接收数据（预留接口）
 */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo1ITs) {
    // 检查是否是FIFO1新消息中断
    if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET) {
        FDCAN_RxHeaderTypeDef RxHeader;  // 接收帧头
        uint8_t RxData[8];               // 接收数据缓冲区

        // 从FIFO1读取消息
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &RxHeader, RxData) ==
            HAL_OK) {
            // 如果是FDCAN2，可以在这里添加处理逻辑
            if (hfdcan == &hfdcan2) {
                Motor_RecieveFeedback(&RxHeader, RxData);
                // 预留接口：可以添加FDCAN2的数据处理
                // 示例：打印接收到的数据（已注释）
                // printf("fdcan2 callback\nID:%x Data:%x %x %x %x %x %x %x %x\n",
                //        RxHeader.Identifier, RxData[0], RxData[1], RxData[2],
                //        RxData[3], RxData[4], RxData[5], RxData[6], RxData[7]);
            }
        }
    }
}