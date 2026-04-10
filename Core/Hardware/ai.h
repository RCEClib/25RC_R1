//
// Created by zaiyad0 on 2026/4/10.
//

#ifndef RC_R1_AI_H
#define RC_R1_AI_H

void SendPIDDataToPC(float setpoint, float input, float index,           //发送到电脑
                    float error, float p, float i, float d);

void ProcessAICommand(char* command, float* new_kp, float* new_ki, float* new_kd);//ai数据处理函数


#endif //RC_R1_AI_H