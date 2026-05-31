#ifndef RC_R1_SCISSOR_LIFT_H
#define RC_R1_SCISSOR_LIFT_H
#include <stdint.h>

void sl_Init(void);
void sl_SetTarget(int16_t loop, float angle);
void sl_Control(void);
void scissor_lift_Task(uint8_t mode1);

#endif //RC_R1_SCISSOR_LIFT_H
