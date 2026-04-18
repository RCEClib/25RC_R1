#ifndef RC_R2_DG_H
#define RC_R2_DG_H
#include <stdint.h>

void dg_Init(void);
void dg_Task(uint8_t mode);

void dg_SetTarget_ID5(int16_t loop, float angle);
void dg_SetTarget_ID6(int16_t loop, float angle);
void dg_Control(void);

#endif //RC_R2_DG_H