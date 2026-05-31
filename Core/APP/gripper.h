#ifndef RC_R2_DG_H
#define RC_R2_DG_H
#include <stdint.h>

void gripper_Init(void);
void dg_SetTarget_ID5(int16_t loop, float angle);
void dg_SetTarget_ID6(int16_t loop, float angle);
void dg_Control(void);
void gripper_Task(uint8_t mode1, uint8_t mode2, float angle);

#endif //RC_R2_DG_H