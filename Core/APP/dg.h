#ifndef RC_R2_DG_H
#define RC_R2_DG_H
#include <stdint.h>

extern float target_total_angle;
extern float actual_total_angle;

extern float taget_v;

void dg_Init(void);
void dg_Task(uint8_t mode);

#endif //RC_R2_DG_H