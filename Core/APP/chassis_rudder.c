#include "chassis_rudder.h"
#include <math.h>
#include <stdlib.h>

#include "elrs.h"
#include "fdcan.h"
#include "imu.h"
#include "motor.h"
#include "pid.h"
#include "Serial.h"

// ============================ 全局变量定义 ============================
float theta[4];               // 四个舵轮的目标角度（弧度）
float wheel_omega_radps[4];   // 四个驱动轮的目标角速度（弧度/秒）

// 调试变量
float target_speed;
float actual_speed;
float outspeed;
float speed_dps;
float feed_forward;
float actual_angle;
float target_angle_deg;
volatile float yaw_rad;

// 底盘几何参数（根据实际轮子和底盘尺寸调整）
const float wheel_R = 0.1f;      // 轮子半径（米）
const float chassis_R = 0.385f;  // 底盘旋转半径（米），车中心到轮子的水平距离

// 舵电机零点偏移（度），顺序：左前、左后、右后、右前
const float zero_offset[4] = {MOTOR0CENTER, MOTOR1CENTER,
                              MOTOR2CENTER, MOTOR3CENTER};

// 3508电机减速比（电机转19圈，轮子转1圈）
#define MOTOR_3508_GEAR_RATIO 19.0f

// 安全保护：最大允许电流（根据电机和驱动器实际限制调整）
#define MAX_RUDDER_CURRENT 25000   // 舵电机最大电流（6020电机）
#define MAX_WHEEL_CURRENT  16384   // 轮电机最大电流（3508电机）

// ============================ 辅助函数 ============================
/**
 * @brief 角度归一化到 [-180, 180] 度
 */
static float NormalizeAngleDeg(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

/**
 * @brief 弧度归一化到 [-π, π]
 */
static float NormalizeAngleRad(float angle) {
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

/**
 * @brief 检查方向校准系数有效性，非法值返回1
 */
static int8_t CheckDirectionCalibration(int8_t val) {
    if (val == 1 || val == -1) return val;
    return 1;   // 默认为正向
}

// ============================ 初始化函数 ============================
void Chassis_Rudder_Init(Chassis_Rudder_t *chassis) {
    chassis->Mode = NORMAL_MODE;
    chassis->Target_Vx = 0.0f;
    chassis->Target_Vy = 0.0f;
    chassis->Target_Wr = 0.0f;
    chassis->spin_rate = 0.0f;
    chassis->feed_forward = 60.0f;

    for (int i = 0; i < 4; i++) {
        chassis->rudder_currents[i] = 0;
        chassis->wheel_currents[i] = 0;
    }

    // 舵轮方向校准（根据实际接线调整，1=正向，-1=反向）
    chassis->rudder_direction_calibration[0] =  1;  // 左前
    chassis->rudder_direction_calibration[1] =  1;  // 左后
    chassis->rudder_direction_calibration[2] =  1;  // 右后
    chassis->rudder_direction_calibration[3] =  1;  // 右前

    // 轮向电机方向校准（根据实际接线调整）
    chassis->wheel_direction_calibration[0] =  1;   // 左前
    chassis->wheel_direction_calibration[1] =  1;   // 左后
    chassis->wheel_direction_calibration[2] =  -1;   // 右后
    chassis->wheel_direction_calibration[3] =  -1;   // 右前

    // 验证并修正校准系数
    for (int i = 0; i < 4; i++) {
        chassis->rudder_direction_calibration[i] = CheckDirectionCalibration(chassis->rudder_direction_calibration[i]);
        chassis->wheel_direction_calibration[i] = CheckDirectionCalibration(chassis->wheel_direction_calibration[i]);
    }

    // 初始化舵向PID（增量式级联PID）
    for (int i = 0; i < 4; i++) {
        // 外层：角度环（输入：度，输出：度/秒）
        Incremental_PID_Init(&chassis->rudder_pid[i].outer, 12.4f, 0.0f, 0.0f, 3000.0f, 3600.0f);
        // 内层：速度环（输入：度/秒，输出：电流）
        Incremental_PID_Init(&chassis->rudder_pid[i].inner, 12.0f, 0.25f, 1.7f, 10000.0f, 30000.0f);
        // 初始化轮向PID（位置式速度环，输入：RPM，输出：电流）
        PID_Init(&chassis->wheel_pid[i], 2.1f, 0.7f, 0.1f, 16384.0f, 1000.0f);
    }

    // 发送零电流，使电机处于待机状态
    Motor_SendCurrent_Ex(&hfdcan2,MOTOR_6020_GROUP1, 0, 0, 0, 0);
    Motor_SendCurrent_Ex(&hfdcan1,MOTOR_3508_GROUP1, 0, 0, 0, 0);
}

// ============================ 运动学解算（核心） ============================
/**
 * @brief 舵轮运动学解算（车体坐标系，使用数组方式）
 * @param vx_body  车体前进速度（米/秒，向前为正）
 * @param vy_body  车体横向速度（米/秒，向左为正）
 * @param omega    车体自转角速度（弧度/秒，逆时针为正）
 * @note 输出 theta[4]（弧度，相对零点偏移）和 wheel_omega_radps[4]（弧度/秒）
 *       轮子编号：0左前, 1左后, 2右后, 3右前（逆时针）
 *       公式：vwx = vx_body - ω * ly,  vwy = vy_body + ω * lx
 *       其中 (lx, ly) 是轮心在车体坐标系中的坐标（带符号）
 */
void steering_wheel_solve(float vx_body, float vy_body, float omega) {
    // 几何常数
    const float r = chassis_R;          // 底盘中心到轮子的水平距离（米）
    const float d = 0.70710678f * r;    // √2/2 * r（米），即轮心坐标绝对值

    // 轮心坐标系数（带符号），顺序：0左前, 1左后, 2右后, 3右前
    // lx: 前后方向坐标（前正后负），ly: 左右方向坐标（左正右负）
    const float lx[4] = {  d, -d, -d,  d};   // 左前(+d), 左后(-d), 右后(-d), 右前(+d)
    const float ly[4] = {  d,  d, -d, -d};   // 左前(+d), 左后(+d), 右后(-d), 右前(-d)

    // 计算每个轮子的轮心合速度分量（车体坐标系）
    float vwx[4], vwy[4];
    for (int i = 0; i < 4; i++) {
        vwx[i] = vx_body + omega * ly[i];   // 轮心X方向速度
        vwy[i] = vy_body + omega * lx[i];   // 轮心Y方向速度
    }

    // 计算每个轮子的目标舵角（弧度）和目标轮速（弧度/秒）
    float target_angle_rad[4];
    float target_speed_radps[4];
    for (int i = 0; i < 4; i++) {
        target_angle_rad[i] = atan2f(vwy[i], vwx[i]);
        target_speed_radps[i] = hypotf(vwx[i], vwy[i]) / wheel_R;
    }

    // 获取当前实际舵角（相对零点，归一化到 [-π, π]）
    float actual_angle_rad[4];
    for (int i = 0; i < 4; i++) {
        float angle_deg = motor_feedback[MOTOR_6020_ID1_INDEX + i].angle; // 原始角度（度）
        angle_deg -= zero_offset[i];        // 减去零点偏移，得到相对角度（度）
        angle_deg = NormalizeAngleDeg(angle_deg);   // 归一化到 [-180,180]
        actual_angle_rad[i] = angle_deg * DEG2RAD; // 转弧度
    }

    // 劣弧优化（智能转向）并输出最终值
    for (int i = 0; i < 4; i++) {
        float diff = target_angle_rad[i] - actual_angle_rad[i];
        diff = NormalizeAngleRad(diff);   // 差值归一化到 [-π, π]

        float out_angle, out_speed;
        if (fabsf(diff) <= M_PI_2) {   // 角度差 ≤ 90°，直接转
            out_angle = target_angle_rad[i];
            out_speed = target_speed_radps[i];
        } else {                        // 角度差 > 90°，转互补方向（±π），轮速取反
            out_angle = target_angle_rad[i] + (diff > 0 ? -M_PI : M_PI);
            out_speed = -target_speed_radps[i];
        }

        // 最终输出（弧度）
        theta[i] = out_angle;
        wheel_omega_radps[i] = out_speed;
    }
}

// ============================ 舵轮控制（PID + 电流发送） ============================
void Chassis_Rudder_Control(Chassis_Rudder_t *chassis) {
    // ---------- 舵向控制（级联PID） ----------
    for (int i = 0; i < 4; i++) {
        // 目标角度：theta[i] 是相对零点偏移的弧度 → 转度 + 零点偏移
        target_angle_deg = theta[i] * RAD2DEG;
        target_angle_deg = NormalizeAngleDeg(target_angle_deg);
        target_angle_deg += zero_offset[i];

        // 处理角度跳变（确保目标与当前角度差不超过180°）
        float angle_diff = target_angle_deg - motor_feedback[MOTOR_6020_ID1_INDEX + i].angle;
        if (angle_diff > 180.0f) target_angle_deg -= 360.0f;
        else if (angle_diff < -180.0f) target_angle_deg += 360.0f;

        // 获取实际角度和速度
        actual_angle = motor_feedback[MOTOR_6020_ID1_INDEX + i].angle;
        speed_dps = motor_feedback[MOTOR_6020_ID1_INDEX + i].speed * 6.0f; // RPM → 度/秒（6020电机直驱，一圈360度，RPM*6=度/秒）

        // 级联PID计算目标电流
        int16_t raw_current = (int16_t)incremental_pid_CascadeCalcWithFeedforward(
                                            &chassis->rudder_pid[i],
                                            target_angle_deg, actual_angle, speed_dps, feed_forward);
        // 输出限幅保护（防止PID计算溢出或异常）
        if (raw_current > MAX_RUDDER_CURRENT) raw_current = MAX_RUDDER_CURRENT;
        if (raw_current < -MAX_RUDDER_CURRENT) raw_current = -MAX_RUDDER_CURRENT;

        chassis->rudder_currents[i] = raw_current;
        outspeed = chassis->rudder_pid[i].outer.output;   // 调试用

        // 方向校准（乘以 ±1）
        chassis->rudder_currents[i] *= chassis->rudder_direction_calibration[i];
    }

    // ---------- 轮向控制（速度PID） ----------
    for (int i = 0; i < 4; i++) {
        // 反馈值乘以方向系数
        actual_speed = motor_feedback[MOTOR_3508_ID1_INDEX + i].speed * chassis->wheel_direction_calibration[i];

        // 轮子角速度 ω_wheel (rad/s) → 轮子转速 n_wheel = ω_wheel * 60 / (2π) (RPM)
        // 电机转速 = n_wheel * 减速比
        target_speed = wheel_omega_radps[i] * (30.0f / M_PI) * MOTOR_3508_GEAR_RATIO;

        // PID计算目标电流
        int16_t raw_current = (int16_t)PID_Calculate(&chassis->wheel_pid[i],
                                                     target_speed, actual_speed);
        // 输出限幅保护
        if (raw_current > MAX_WHEEL_CURRENT) raw_current = MAX_WHEEL_CURRENT;
        if (raw_current < -MAX_WHEEL_CURRENT) raw_current = -MAX_WHEEL_CURRENT;

        chassis->wheel_currents[i] = raw_current;

        // 方向校准（乘以 ±1）
        chassis->wheel_currents[i] *= chassis->wheel_direction_calibration[i];
    }

    // 发送电流指令
    Motor_SendCurrent_Ex(&hfdcan2,MOTOR_6020_GROUP1,
            chassis->rudder_currents[0], chassis->rudder_currents[1],
            chassis->rudder_currents[2], chassis->rudder_currents[3]);
    Motor_SendCurrent_Ex(&hfdcan1,MOTOR_3508_GROUP1,
            chassis->wheel_currents[0], chassis->wheel_currents[1],
            chassis->wheel_currents[2], chassis->wheel_currents[3]);
}

// ============================ 底盘任务函数 ============================
/**
 * @brief 舵轮底盘任务函数（在主线循环中周期性调用）
 * @param chassis 底盘控制结构体指针
 * @param mode    工作模式（NORMAL_MODE / STOP_MODE / GYRO_MODE）
 * @param vx      遥控器X轴（-100~100），对应车体前进速度
 * @param vy      遥控器Y轴（-100~100），对应车体左向速度
 * @param vw      遥控器Z轴（-100~100），对应自转角速度
 * @note 输入范围 -100~100，内部会转换为实际速度（米/秒，弧度/秒）
 */
void Chassis_Rudder_Task(Chassis_Rudder_t *chassis, Chassis_Mode mode,
                          float vx, float vy, float vw) {
    if (chassis == NULL) return;

    // 死区处理（摇杆小信号归零）
    const float deadzone = 1.0f;   // 小于1%的信号视为0
    if (fabsf(vx) < deadzone) vx = 0.0f;
    if (fabsf(vy) < deadzone) vy = 0.0f;
    if (fabsf(vw) < deadzone) vw = 0.0f;

    // 速度缩放：摇杆-100~100 → 实际速度
    float max_linear_speed  = 3.8f;   // 最大线速度 1.5 m/s
    float max_angular_speed = 6.0f;   // 最大角速度 5 rad/s

    if (mode == GYRO_MODE) chassis->spin_rate = 7.0f;// 小陀螺模式下的固定转角速度 10 rad/s
    else chassis->spin_rate = 0.0f;                  // 正常模式下，不固定转角速度


    // 世界坐标系下的期望平移速度（单位：m/s）
    float vx_world = vx / 100.0f * max_linear_speed;   // 向正北的速度分量
    float vy_world = vy / 100.0f * max_linear_speed;   // 向正西的速度分量
    float omega_user = vw / 100.0f * max_angular_speed;// 遥控器给出的角速度（单位：rad/s）


    //小陀螺模式：固定自转速度 + 遥控器额外角速度
    // chassis->spin_rate: 底盘持续自转的固定角速度（rad/s），由外部开关设置（例如 1.0）
    //                     如果不开启小陀螺，则 spin_rate = 0
    // omega_user:         遥控器拨杆给出的附加角速度，例如向右拨杆会让自转更快
    // 两者相加得到最终的总角速度
    float omega_total = omega_user + chassis->spin_rate;

    // 读取IMU当前车头朝向（与正北的夹角）
    yaw_rad = imu_data.yaw;   //返回弧度值 [0, 2π]

    // 世界坐标系 → 车体坐标系（核心变换）
    // 无论车头朝向哪里，推摇杆的前就是正北，左就是正西
    float cos_yaw = cosf(yaw_rad);
    float sin_yaw = sinf(yaw_rad);

    float vx_body =  vx_world * cos_yaw - vy_world * sin_yaw;
    float vy_body =  vx_world * sin_yaw + vy_world * cos_yaw;


    // 模式切换处理
    static Chassis_Mode last_mode = STOP_MODE;
    if (last_mode != mode) {
        last_mode = mode;
        chassis->Mode = mode;
        if (mode == STOP_MODE) {
            chassis->Target_Vx = 0.0f;
            chassis->Target_Vy = 0.0f;
            chassis->Target_Wr = 0.0f;
        }
    }

    if (mode) {
        switch (chassis->Mode) {
            case NORMAL_MODE:
            case GYRO_MODE:
                chassis->Target_Vx = vx_body;
                chassis->Target_Vy = vy_body;
                chassis->Target_Wr = omega_total;
                // 运动学解算（车体坐标系）
                steering_wheel_solve(chassis->Target_Vx, chassis->Target_Vy, chassis->Target_Wr);
                // PID控制
                Chassis_Rudder_Control(chassis);
                break;
            case STOP_MODE:
            default:
                Motor_SendCurrent_Ex(&hfdcan2,MOTOR_6020_GROUP1, 0, 0, 0, 0);
                Motor_SendCurrent_Ex(&hfdcan1,MOTOR_3508_GROUP1, 0, 0, 0, 0);
                break;
        }
    } else {
        // 未使能，发送零电流
        Motor_SendCurrent_Ex(&hfdcan2,MOTOR_6020_GROUP1, 0, 0, 0, 0);
        Motor_SendCurrent_Ex(&hfdcan1,MOTOR_3508_GROUP1, 0, 0, 0, 0);
    }
}