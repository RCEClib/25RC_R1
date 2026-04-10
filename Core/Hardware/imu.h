#ifndef __IMU_H__
#define __IMU_H__

typedef struct {
    float roll;
    float pitch;
    float yaw;
    float gyro[3];
    float accel[3];
    float temp;
    uint8_t TempKey;
} IMU_Data;

HAL_StatusTypeDef IMU_Init(void);
void IMU_Task(uint8_t temp_key);

extern IMU_Data imu_data;
extern float yaw;

#endif /* __IMU_H__ */
