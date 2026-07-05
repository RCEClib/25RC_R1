#ifndef RC_R1_SCISSOR_LIFT_H
#define RC_R1_SCISSOR_LIFT_H
#include <stdint.h>

void sl_Init(void);
void sl_SetTarget(int16_t loop, float angle);
void sl_Control(void);
void scissor_lift_Task(int8_t loop,float angle_m,uint8_t key0,uint8_t key1,uint8_t key2);

#endif //RC_R1_SCISSOR_LIFT_H
