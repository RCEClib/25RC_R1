#include <math.h>

#include "BMI088driver.h"
#include "imu.h"

#include <stdio.h>

#include "chassis_rudder.h"
#include "tim.h"
#include "usart.h"

IMU_Data imu_data;

static float alpha = 0.98f;
static float dt    = 0.001f;

#define DES_TEMP 40.0f
#define KP 100.f
#define KI 50.f
#define KD 10.f
#define MAX_OUT 500

float out    = 0;
float err    = 0;
float err_l  = 0;
float err_ll = 0;

HAL_StatusTypeDef IMU_Init(void) {
    imu_data.roll    = 0;
    imu_data.pitch   = 0;
    imu_data.yaw     = 0;
    imu_data.TempKey = 0;
    if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4) != HAL_OK ||
        BMI088_init() != BMI088_NO_ERROR) {
        return HAL_ERROR;
    }
    printf("IMU init success!\n");
    return HAL_OK;
}

void IMU_Calculate(IMU_Data* imu_data) {
    float accel_x   = imu_data->accel[0];
    float accel_y   = imu_data->accel[1];
    float accel_z   = imu_data->accel[2];
    float gyro_x    = imu_data->gyro[0];
    float gyro_y    = imu_data->gyro[1];
    float gyro_z    = imu_data->gyro[2];

    float acc_pitch = atan2f(accel_y, accel_z);
    float acc_roll = atan2f(-accel_x, sqrtf(accel_y * accel_y + accel_z * accel_z));
    float gyro_pitch = gyro_x  * dt;
    float gyro_roll  = gyro_y  * dt;
    float gyro_yaw   = gyro_z  * dt;
    imu_data->pitch =
        alpha * (imu_data->pitch + gyro_pitch) + (1 - alpha) * acc_pitch;
    imu_data->roll =
        alpha * (imu_data->roll + gyro_roll) + (1 - alpha) * acc_roll;

     // 弧度
    imu_data->yaw += gyro_yaw;

    // 归一化到 [0, 2PI)
    if (imu_data->yaw >= 2*M_PI) imu_data->yaw -= 2*M_PI;
    if (imu_data->yaw < 0.0f)    imu_data->yaw += 2*M_PI;


}

void IMU_TempCtrl(float temp) {
    err_ll = err_l;
    err_l  = err;
    err    = DES_TEMP - temp;
    out    = KP * err + KI * (err + err_l + err_ll) + KD * (err - err_l);
    if (out > MAX_OUT) out = MAX_OUT;
    if (out < 0) out = 0.0f;
    if (imu_data.TempKey == 1) out = 0.0f;
    htim3.Instance->CCR4 = (uint16_t)out;
}

void IMU_Task(uint8_t temp_key) {
    imu_data.TempKey = temp_key;
    BMI088_read(imu_data.gyro, imu_data.accel, &imu_data.temp);// 读取原始数据
    IMU_TempCtrl(imu_data.temp);// 温度控制
    IMU_Calculate(&imu_data);// 姿态解算
}
