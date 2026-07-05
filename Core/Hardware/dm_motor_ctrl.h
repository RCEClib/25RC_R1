#ifndef __DM_MOTOR_CTRL_H__
#define __DM_MOTOR_CTRL_H__
#include "main.h"
#include "dm_motor_drv.h"

extern motor_t motor[num];

void dm_motor_init(motor_num id, FDCAN_HandleTypeDef *hcan, uint16_t can_id,mode_e mode);

void read_all_motor_data(motor_t *motor);
void receive_motor_data(motor_t *motor, uint8_t *data);

#endif /* __DM_MOTOR_CTRL_H__ */

