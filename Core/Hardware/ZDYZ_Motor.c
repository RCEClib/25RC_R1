
#include "ZDYZ_Motor.h"

#include <string.h>

#include "tim.h"


// /* ========== 电机状态数据 ========== */
// static bool last_arrived = false;
// static uint8_t last_error = 0;
// MyCAN_Frame_t g_mycan_frame = {0};


/* 电机参数：200 步/圈（1.8° 步进电机）*/
#define MOTOR_STEPS_PER_REV   200

//当前细分8
static uint16_t microstep = 8;


/* 角度转脉冲数（内部函数）*////通信位置模式设置了1圈51200的编码器值
 uint32_t angle_to_pulses(float angle_deg)
{
    // 驱动器位置单位：一圈 = 51200
    return (uint32_t)(angle_deg / 360.0f * 51200.0f);
}



/* 设置细分，并发送指令给驱动器*/
void motor_set_microstep(uint8_t addr, uint16_t step)
{
    microstep = step;
    smd_set_step(addr, step);
}

/* 绝对位置模式：转到指定角度 */
void motor_go_to_angle(uint8_t addr, float angle_deg, uint16_t speed_rpm, uint8_t acc)
{
    uint8_t dir = (angle_deg >= 0) ? 0 : 1;   /* 0正转，1反转 */
    uint32_t pulses = angle_to_pulses(angle_deg >= 0 ? angle_deg : -angle_deg);

    smd_pos_mode(addr, dir, acc, speed_rpm, pulses);
}

/* 相对位置模式：转动指定角度 */
void motor_rotate(uint8_t addr, float angle_deg, uint16_t speed_rpm, uint8_t acc)
{
    uint8_t dir = (angle_deg >= 0) ? 0 : 1;   /* 0正转，1反转 */
    uint32_t pulses = angle_to_pulses(angle_deg >= 0 ? angle_deg : -angle_deg);

    smd_pos_rel_mode(addr, dir, acc, speed_rpm, pulses);
}



/* 相对位置模式：转动指定圈数 */
void motor_rel_turns(uint8_t addr, float turns, uint16_t speed_rpm, uint8_t acc)
{
    motor_rotate(addr, turns * 360.0f, speed_rpm, acc);
}


/* 绝对位置模式，指定圈数 */
void motor_abs_turns(uint8_t addr, float turns, uint16_t speed_rpm, uint8_t acc)
{
    motor_go_to_angle(addr,  turns * 360.0f, speed_rpm, acc);

}



/* 等待电机到位读取到位状态）*/
bool motor_wait_arrived(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    while (HAL_GetTick() - start < timeout_ms) {
        /* 发送“读取到位状态”命令（功能码 0x30）*/
        smd_read_arrived_sta(1);
        /* 等待 50ms 让 CAN 中断处理完成并更新 last_arrived 变量 */
        HAL_Delay(50);
        if (motor_is_arrived()) {
            return true;
        }
    }
    return false;
}





// 电机初始化：细分、使能、限位、回零、自动回零、保存参数
void motor_init_auto_zero(uint8_t addr)
{
    // 1. 基础设置
    motor_set_microstep(addr, 8);    // 细分8
   // HAL_Delay(100);
    smd_motor_enable(addr, 0);      // 电机使能
    // HAL_Delay(100);

    // 2. 设置限位：0 ~ 10圈
    smd_origin_l_r_switch(addr, 1);        // 开启限位
    smd_origin_set_left_pos(addr, 0);      // 左限位 0圈
    smd_origin_set_right_pos(addr, 512000);// 右限位 10圈
    HAL_Delay(1);

    // 4. 上电自动回零
    smd_origin_aoto_zero(addr, 1);
    HAL_Delay(1);
    // 3. 多圈回零
    smd_origin_trig(addr, 2);
    HAL_Delay(1);


    // 5. 保存上面所有参数
    smd_param_save(addr);
    HAL_Delay(1);

    // 6. 清零位置
    smd_angle_to_zero(addr);
    HAL_Delay(1);
}


/////////////////

/* ========== 电机状态数据 ========== */
static bool last_arrived = false;
static uint8_t last_error = 0;
MyCAN_Frame_t g_mycan_frame = {0};

/**
 * @brief  初始化正点原子CAN接收
 * @retval 无
 */
void ZDYZ_Init(void)
{
    //FDCAN_Init(&hfdcan2);//前面已经
    HAL_TIM_Base_Start_IT(&htim6); // 启动超时定时器
    __HAL_TIM_DISABLE(&htim6);
    g_mycan_frame.index = 0;
    g_mycan_frame.frame_done = false;
}

// /* 计算校验和 */
//  uint8_t calc_checksum(uint8_t *buf, uint16_t len)
// {
//     uint8_t sum = 0;
//     for (uint16_t i = 0; i < len; i++) sum += buf[i];
//     return sum;
// }

/* 计算校验和 */
uint8_t calc_checksum( const uint8_t *buf, uint16_t len)
 {
     uint8_t sum = 0;
     for (uint16_t i = 0; i < len; i++) sum += buf[i];
     return sum;
 }



/**
 * @brief  长子节CAN接收数据分包
 * @retval 无
 */
void handle_can_rx(FDCAN_HandleTypeDef *hfdcan,uint8_t fifo )
 {//debug_printf("handle_can_rx enter\r\n");
     uint8_t rx_data[8];
     FDCAN_RxHeaderTypeDef rx_header;

     HAL_FDCAN_GetRxMessage(hfdcan, fifo, &rx_header, rx_data);


     uint32_t dlc = rx_header.DataLength;
     uint8_t len = 0;
     switch (dlc) {
         case FDCAN_DLC_BYTES_0: len = 0; break;
         case FDCAN_DLC_BYTES_1: len = 1; break;
         case FDCAN_DLC_BYTES_2: len = 2; break;
         case FDCAN_DLC_BYTES_3: len = 3; break;
         case FDCAN_DLC_BYTES_4: len = 4; break;
         case FDCAN_DLC_BYTES_5: len = 5; break;
         case FDCAN_DLC_BYTES_6: len = 6; break;
         case FDCAN_DLC_BYTES_7: len = 7; break;
         case FDCAN_DLC_BYTES_8: len = 8; break;
         default: return;
     }
     if (len == 0) return;

     parse_frame(rx_data, len);

     if (g_mycan_frame.index == 0) {
         __HAL_TIM_ENABLE(&htim6);
     }
     if (g_mycan_frame.index + len <= MYCAN_RECV_BUF_LEN) {
         for (uint8_t i = 0; i < len; i++) {
             g_mycan_frame.buf[g_mycan_frame.index++] = rx_data[i];
         }
         __HAL_TIM_SET_COUNTER(&htim6, 0);
     } else {
         g_mycan_frame.index = 0;
     }
 }




/**
 * @note   适配官方发送长消息的函数
 * @brief  发送长消息（8字节以上）
 * @param  id: CAN ID
 * @param  data: 数据指针
 * @param  len: 数据长度
 * @retval 0: 发送成功
 *         1: 发送失败
 */
uint8_t can_send_long_msg( FDCAN_HandleTypeDef *hfdcan, uint32_t id,uint32_t type, uint8_t *data, uint16_t len) {
     uint16_t offset = 0;
     while (offset < len) {
         uint8_t send_len = (len - offset >= 8) ? 8 : (len - offset);

         uint8_t data8[8] = {0};
         memcpy(data8, data + offset, send_len);

         if (FDCAN_Send_Var(hfdcan, id, FDCAN_EXTENDED_ID, data8, send_len) != 0) {
             return 1;
         }
         offset += send_len;
     }
     return 0;

 }

// ===================== 解析 =====================
 void parse_frame(uint8_t *buf, uint16_t len)
{
    if (len < 6) return;
    if (buf[0] != 0xC5 || buf[len-1] != 0x5C) return;
    if (calc_checksum(buf, len-2) != buf[len-2]) return;

    uint8_t func = buf[2];
    uint8_t err  = buf[3];
    last_error = err;

    if (func == 0x30 && len >= 7) {
        last_arrived = (buf[4] == 1);
    }
}

/**
 * @brief  定时器周期中断回调函数
 * @param htim: 定时器句柄指针
 * @retval 无
 */
// ===================== 定时器超时 =====================
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6 && g_mycan_frame.index > 0) {
        //返回调试信息
        // SERIAL_FRAME frame;
        // serial_frame_process(g_mycan_frame.buf, g_mycan_frame.index, &frame);
        // parse_frame(g_mycan_frame.buf, g_mycan_frame.index);

        g_mycan_frame.frame_done = true;
        __HAL_TIM_DISABLE(htim);
        g_mycan_frame.index = 0;
    }
}





bool MyCAN_MotorIsArrived(void)
{
    return last_arrived;
}

uint8_t MyCAN_GetLastError(void)
{
    return last_error;
}