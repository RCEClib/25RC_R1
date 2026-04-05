#ifndef __MyCAN_H__
#define __MyCAN_H__

#include "stm32h7xx_hal.h"
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

typedef struct {
    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t FDCAN_Data[8];
} FDCAN_RxTypeDef;

HAL_StatusTypeDef FDCAN_Init(FDCAN_HandleTypeDef *hfdcan);
HAL_StatusTypeDef FDCAN_Send(FDCAN_HandleTypeDef *hfdcan, uint32_t id,
                             uint32_t idtype, uint8_t *data);
HAL_StatusTypeDef FDCAN_Receive(FDCAN_HandleTypeDef *hfdcan,
                                FDCAN_RxHeaderTypeDef *RxHeader,
                                uint8_t *RxData);

#endif /* __MyCAN_H__ */
