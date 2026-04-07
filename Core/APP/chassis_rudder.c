#include "chassis_rudder.h"
#include <math.h>
#include <stdlib.h>

#include "elrs.h"
#include "motor.h"
#include "pid.h"
#include "Serial.h"

// ============================ 全局变量定义 ============================

float theta[4];    // 四个舵轮的目标角度（弧度），供舵向PID使用
float wheel_omega_radps[4];   // 四个驱动轮的目标角速度（rad/s），供轮速PID使用

// 以下变量用于PID控制过程中的临时计算（原样保留）
float target_speed;      // 轮速PID的目标值（RPM）
float actual_speed;      // 轮速PID的反馈值（RPM）
float outspeed;          // 舵向PID外环输出（度/秒），调试用
float speed_dps;         // 舵电机实际速度（度/秒）
float actual_angle;      // 舵电机实际角度（度）
float target_angle_deg;  // 舵电机目标角度（度）


/**
 * @brief IMU航向角（弧度），车头相对于全局X轴的角度。
 * @note 此变量由外部 IMU_Task 函数更新，单位必须为弧度。
 *       原代码中此变量名为 Theat，含义为 theta（航向角），保持不变以兼容。
 */
float imu_yaw_rad = 0.0f;

// 物理参数（根据实际轮子和底盘尺寸调整）
const float wheel_R = 0.1f;      // 轮子半径（米）
const float chassis_R = 0.385f;  // 底盘旋转半径（米），车中心到轮子的水平距离

// 舵电机零点偏移（度），四个轮子顺序：前左、前右、后左、后右
const float zero_offset[4] = {MOTOR1CENTER, MOTOR2CENTER,
                                 MOTOR3CENTER, MOTOR4CENTER};




void Chassis_Rudder_Init(Chassis_Rudder_t *chassis) {
    // 初始化舵轮
    chassis->Mode      = NORMAL_MODE;
    chassis->Target_Vx = 0;
    chassis->Target_Vy = 0;
    chassis->Target_Wr = 0;
    for (int i = 0; i < 4; i++) {
        chassis->rudder_currents[i] = 0;
        chassis->wheel_currents[i] = 0;
    }
    // 舵轮方向校准数组（根据实际接线调整）
    chassis->rudder_direction_calibration[0] = 1;
    chassis->rudder_direction_calibration[1] = 1;
    chassis->rudder_direction_calibration[2] = 1;
    chassis->rudder_direction_calibration[3] = 1;

    // 设置轮向电机方向校准
    chassis->wheel_direction_calibration[0] = 1;
    chassis->wheel_direction_calibration[1] = 1;
    chassis->wheel_direction_calibration[2] = 1;
    chassis->wheel_direction_calibration[3] = 1;

    for (int i = 0; i < 4; i++) {
        //舵电机（转向）：增量式 级联PID（角度环 + 速度环）
        //外层：角度环
        Incremental_PID_Init(&chassis->rudder_pid[i].outer,3.0f,0.4f,0.1f,3000.0f,3600.0f);
        // 内层：速度环
        Incremental_PID_Init(&chassis->rudder_pid[i].inner,9.0f,0.5f,0.1f,10000.0f,30000.0f);

        //轮电机（驱动）：普通PID（速度环）
        PID_Init(&chassis->wheel_pid[i],2.1f,0.7f,0.1f,16384.0f,1000.0f);
    }

    Motor_SendCurrent_6020(MOTOR_6020_GROUP1,0, 0,0, 0);
    Motor_SendCurrent_3508(MOTOR_3508_GROUP1,0, 0,0, 0);
}

/**
 * @brief 角度归一化到 [-180, 180] 度
 */
static float NormalizeAngleDeg(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

/**
 * @brief 角度归一化到 [-π, π] 弧度
 */
static float __attribute__((unused)) NormalizeAngleRad(float angle) {
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

/**
 * @brief 解算出每个舵轮的运动参数（基于矩阵运动学 + 智能转向）
 * @param vx_global  全局X方向速度（米/秒），正前方
 * @param vy_global  全局Y方向速度（米/秒），正左方
 * @param vw_global  全局旋转角速度（弧度/秒），逆时针为正
 * @note 输出全局变量 theta[4]（弧度）和 wheel_omega_radps[4]（rad/s）
 *       使用 imu_yaw_rad（IMU航向角，弧度）将全局速度转换到车体坐标系。
 *       运动学公式采用矩阵形式：
 *         [v_wx]   [1   0   -d * Ly_sign]   [vx_body]
 *         [v_wy] = [0   1    d * Lx_sign] * [vy_body]
 *                                            [vw     ]
 *       其中 d = sqrt(2)/2 * chassis_R，Lx_sign/Ly_sign 为轮心符号。
 *       智能转向：若目标舵角与当前舵角差值 ≤90° 则直接转，否则转互补方向并反转轮速。
 */
void steering_wheel_solve(float vx_global, float vy_global, float vw_global)
{
// ---------- 坐标变换：世界坐标系 → 车体坐标系 ----------
// 读取IMU偏航角（车头朝向）的余弦、正弦值
float cy = cosf(imu_yaw_rad);
float sy = sinf(imu_yaw_rad);

// 全局速度（地图方向）→ 车体自身速度（前后左右方向）
float vx_body = vx_global * cy + vy_global * sy;   // 车体前进方向速度
float vy_body = -vx_global * sy + vy_global * cy;  // 车体横向速度
float vw = vw_global;                              // 自转角速度（无需变换）

// ----------  底盘几何参数 ----------
float r = chassis_R;        // 底盘中心到每个轮子的旋转半径
float s = wheel_R;          // 驱动轮半径（用于计算转速）
// d = 底盘中心到轮心的偏移量 = r * √2/2
float d = 0.70710678f * r;

// 4个轮子的位置符号（前左0、前右1、后左2、后右3）
// Lx：前后方向符号，Ly：左右方向符号
const float Lx[4] = { 1,  1, -1, -1};
const float Ly[4] = { 1, -1,  1, -1};

// ---------- 读取当前4个舵轮的实际角度 ----------
float actual[4];  // 存储每个舵轮当前的实际角度（归一化到 [-180,180]）
for(int i=0; i<4; i++){
    // 电机反馈角度 - 零偏 = 真实舵角
    actual[i] = motor_feedback[MOTOR_6020_ID1_INDEX + i].angle - zero_offset[i];
    // 归一化角度，防止超过范围
    actual[i] = NormalizeAngleDeg(actual[i]);
}

// ---------- 逐个轮子解算（核心运动学 + 劣弧优化） ----------
for(int i=0; i<4; i++){
    // 计算轮心的线速度（车体速度 + 自转速度合成）
    float vwx = vx_body - vw * d * Ly[i];  // 轮心 X 方向速度
    float vwy = vy_body + vw * d * Lx[i];  // 轮心 Y 方向速度


    float target, speed;
    float out_angle, out_speed;// 最终输出的角度、转速

    // 零速保护：合速度极小，保持当前舵角，轮速为0
    if (fabsf(vwx) < 1e-6f && fabsf(vwy) < 1e-6f) {
        out_angle = 0.0f;               // 目标角度0°（车头方向）
        out_speed = 0.0f;               // 轮子不转
    }
    else {
    // 计算轮子目标角度（弧度 → 角度）
    // atan2(y,x) 得到轮子指向的弧度，转成角度
    target = atan2f(vwy, vwx) * RAD2DEG;
    target = NormalizeAngleDeg(target);  // 归一化到 [-180,180]

    // 计算轮子转速（线速度 → 角速度）
    // 轮心合速度 / 轮子半径 = 轮子需要的转速
    speed = hypotf(vwx, vwy) / s;

    //劣弧优化（舵轮核心！只转最小角度）
    // 计算目标角度与当前角度的差值
    float diff = target - actual[i];
    diff = NormalizeAngleDeg(diff);  // 把差值限制在 [-180,180]

    // 判断：角度差 ≤90 度 → 直接转向目标角度
    if(fabsf(diff) <= 90){
        out_angle = target;
        out_speed = speed;
    }
    // 角度差 >90 度 → 转反方向（省距离），同时转速反向
    else{
        out_angle = target + (diff > 0 ? -180 : 180);
        out_speed = -speed;
    }

    // 最终角度归一化，确保合法
    out_angle = NormalizeAngleDeg(out_angle);
    }
    // 输出结果给电机控制
    theta[i] = out_angle * DEG2RAD;          // 角度 → 弧度（给舵向PID）
    wheel_omega_radps[i] = out_speed;        // 最终驱动轮转速
    }
}

/**
 * @brief 舵轮控制函数
 * @param chassis: 舵轮控制结构体指针
 * @note 执行舵向和轮向的PID控制计算
 */
void Chassis_Rudder_Control(Chassis_Rudder_t *chassis) {
                             // ---------- 舵向控制（角度环+速度环） ----------
   for (int i = 0; i < 4; i++) {
        // 1. 计算目标角度（弧度→度，加零偏）
        float target_deg = theta[i] * RAD2DEG;          // 已劣弧优化
        target_deg += zero_offset[i];                   // 加零点偏移（度）

        // 2. 获取实际反馈角度（0~360度）
        float actual_deg = motor_feedback[MOTOR_6020_ID1_INDEX + i].angle;

       float angle_diff = target_angle_deg - actual_angle;
       if (angle_diff > 180.0f) {
           target_angle_deg -= 360.0f;
       } else if (angle_diff < -180.0f) {
           target_angle_deg += 360.0f;
       }

       float diff = target_angle_deg - actual_angle;
       if (fabsf(diff) > 90.0f) {
           if (diff > 0) {
               target_angle_deg -= 180.0f;
           } else {
               target_angle_deg += 180.0f;
           }

           speed_dps = motor_feedback[MOTOR_6020_ID1_INDEX + i].speed * 6.0f;
       }

        // 3. 速度反馈：RPM → 度/秒（6020 电机速比通常 6 度/RPM）
        float speed_dps = motor_feedback[MOTOR_6020_ID1_INDEX + i].speed * 6.0f;

        // 4. 级联 PID 计算电流
        chassis->rudder_currents[i] = (int16_t)incremental_pid_CascadeCalcWithFeedforward(
                                            &chassis->rudder_pid[i],
                                            target_deg,          // 修正后的目标角度
                                            actual_deg,           // 实际角度
                                            speed_dps,
                                            0);

        // 5. 方向校准
        chassis->rudder_currents[i] *= chassis->rudder_direction_calibration[i];
    }

    // ============================ 轮向控制循环 ============================
    for (int i = 0; i < 4; i++) {
        float actual_speed = motor_feedback[MOTOR_3508_ID1_INDEX + i].speed;  // RPM
        float target_speed = wheel_omega_radps[i] * 30.0f / M_PI;             // rad/s → RPM
        chassis->wheel_currents[i] = (int16_t)PID_Calculate(&chassis->wheel_pid[i],
                                                            target_speed, actual_speed);
        chassis->wheel_currents[i] *= chassis->wheel_direction_calibration[i];
    }

    // ============================ 发送电流指令 ============================
    Motor_SendCurrent_6020(MOTOR_6020_GROUP1,
            chassis->rudder_currents[0],
            chassis->rudder_currents[1],
            chassis->rudder_currents[2],
            chassis->rudder_currents[3]);
    Motor_SendCurrent_3508(MOTOR_3508_GROUP1,
            chassis->wheel_currents[0],
            chassis->wheel_currents[1],
            chassis->wheel_currents[2],
            chassis->wheel_currents[3]);
}


/**
 * @brief 舵轮底盘任务函数
 * @param chassis 舵轮控制结构体指针
 * @param en      使能标志（1=使能，0=禁用）
 * @param mode    工作模式（VELOCITY_MODE / STOP_MODE）
 * @param vx      全局X方向速度（米/秒）
 * @param vy      全局Y方向速度（米/秒）
 * @param vw      全局旋转角速度（弧度/秒）
 */
void Chassis_Rudder_Task(Chassis_Rudder_t *chassis, uint8_t en,
                        Chassis_Mode mode, float vx, float vy, float vw) {
    // 检查参数有效性
    if (chassis == NULL) {
        return;
    }


    // 死区处理（摇杆不动时设为0）
    if (fabs(vx) < 5.0f) vx = 0;
    if (fabs(vy) < 5.0f) vy = 0;
    if (fabs(vw) < 5.0f) vw = 0;

    // 速度缩放（摇杆 -100~100 → 真实速度）
    float max_linear_speed  = 1.0f;   // 最大线速度 1 m/s
    float max_angular_speed = 3.0f;   // 最大角速度 3 rad/s

    float v_x = vx / 100.0f * max_linear_speed;
    float v_y = vy / 100.0f * max_linear_speed;
    float v_w = vw / 100.0f * max_angular_speed;

    // 模式切换处理
    static Chassis_Mode last_mode = STOP_MODE;
    if (last_mode != mode) {
        last_mode = mode;
        chassis->Mode = mode;

        // 模式切换时的特殊处理
        if (mode == STOP_MODE) {
            // 停止模式：设置目标速度为0
            chassis->Target_Vx = 0.0f;
            chassis->Target_Vy = 0.0f;
            chassis->Target_Wr = 0.0f;
        }
    }

    // 控制逻辑
    if (en) {
        switch (chassis->Mode) {
            case NORMAL_MODE:

                // 正常模式：设置目标速度并执行控制
                chassis->Target_Vx = v_x;
                chassis->Target_Vy = v_y;
                chassis->Target_Wr = v_w;

                 //计算舵轮运动参数
                steering_wheel_solve(chassis->Target_Vx, chassis->Target_Vy, chassis->Target_Wr);
                 //执行舵轮控制
                Chassis_Rudder_Control(chassis);
                break;

            case STOP_MODE:
            default:
                // 停止模式：发送零电流
                Motor_SendCurrent_6020(MOTOR_6020_GROUP1, 0, 0, 0, 0);
                Motor_SendCurrent_3508(MOTOR_3508_GROUP1, 0, 0, 0, 0);
                break;
        }
    } else {
        // 禁用状态：发送零电流
        Motor_SendCurrent_6020(MOTOR_6020_GROUP1, 0, 0, 0, 0);
        Motor_SendCurrent_3508(MOTOR_3508_GROUP1, 0, 0, 0, 0);
    }
}
