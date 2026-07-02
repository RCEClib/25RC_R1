#include "bsp_fdcan.h"

// [MOD] 新增头文件：用于处理大疆电机和达妙电机反馈
#include "DJI_Motor.h"           // 大疆电机处理函数 Motor_ReceiveFeedback
#include "dm_motor_drv.h"    // 达妙电机反馈解析 dm_motor_fbdata, receive_motor_data
#include "dm_motor_ctrl.h"   // 达妙电机全局数组 motor[]

/**
 * @brief       CAN外设初始化（启动并激活中断）
 * @param       void
 * @retval      void
 * @details     使能FDCAN，配置滤波器，启动通知及错误中断
 * @note
 */
void bsp_can_init(FDCAN_HandleTypeDef *hfdcan)
{
    can_filter_init(hfdcan);                     // 先配置接收过滤器（决定哪些ID能通过）

    HAL_FDCAN_Start(hfdcan);             // 启动FDCAN外设（进入正常模式）

    // 激活多种FDCAN中断：RX FIFO0水位线、TX完成、TX FIFO空、总线关闭、仲裁/数据协议错误、错误被动、错误警告
    // 注意：第二个参数中 FDCAN_IT_RX_FIFO0_WATERMARK 重复写了两次（不影响结果），第三个参数是掩码（0x00000F00）此处未实际使用
    HAL_FDCAN_ActivateNotification(hfdcan,
                                   0 | FDCAN_IT_RX_FIFO0_WATERMARK | FDCAN_IT_RX_FIFO0_WATERMARK
                                       | FDCAN_IT_TX_COMPLETE | FDCAN_IT_TX_FIFO_EMPTY | FDCAN_IT_BUS_OFF
                                       | FDCAN_IT_ARB_PROTOCOL_ERROR | FDCAN_IT_DATA_PROTOCOL_ERROR
                                       | FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING,
                                   0x00000F00);
}

/**
 * @brief       CAN接收过滤器初始化
 * @param       void
 * @retval      void
 * @details     设置一个掩码模式的标准ID过滤器，所有消息都放入FIFO0，并配置全局过滤策略拒绝未匹配帧
 * @note        当前 FilterID1 和 FilterID2 均为 0，这意味着：
 *              掩码模式下（FDCAN_FILTER_MASK）：FilterID1 = ID，FilterID2 = 掩码。
 *              此处 ID=0, 掩码=0 → 所有标准ID都被接收（相当于不过滤）。
 *              之后通过全局过滤设置拒绝扩展帧和远程帧。
 */
void can_filter_init(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_FilterTypeDef fdcan_filter;

    fdcan_filter.IdType = FDCAN_STANDARD_ID;          // 只过滤标准ID（11位），不处理扩展ID
    fdcan_filter.FilterIndex = 0;                     // 使用第0个过滤器单元（共若干个）
    fdcan_filter.FilterType = FDCAN_FILTER_MASK;      // 掩码模式（ID1为期望ID，ID2为掩码位）
    fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // 匹配成功的帧放到 RX FIFO 0 中
    fdcan_filter.FilterID1 = 0x00;                    // 期望的ID（当掩码为0时无效）
    fdcan_filter.FilterID2 = 0x00;                    // 掩码：0表示不关心任何位 → 所有ID都匹配

    HAL_FDCAN_ConfigFilter(hfdcan, &fdcan_filter);  // 配置过滤器0

    // 全局过滤器设置：
    // 拒绝所有未匹配的扩展帧和标准帧（因为上面只是对标准ID设置了过滤器，但扩展ID根本没匹配机会）
    // 同时拒绝远程帧（RTR）
    HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

    // 设置RX FIFO0的水位线为1：当FIFO0中收到1条消息时就产生水位线中断
    HAL_FDCAN_ConfigFifoWatermark(hfdcan, FDCAN_CFG_RX_FIFO0, 1);
    // 可选：RX FIFO1的水位线也设置（当前注释未使用）
    // HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO1, 1);
    // HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_TX_COMPLETE, FDCAN_TX_BUFFER0);
}

/**
 * @brief       设置FDCAN的波特率（仲裁域和数据域），可配置为经典CAN或CAN FD带BRS
 * @param       hfdcan：FDCAN句柄（含外设句柄和初始化结构体）
 * @param       mode：模式选择，CAN_CLASS（经典CAN）或 CAN_FD_BRS（FD模式，数据域BRS使能）
 * @param       baud：波特率枚举值（如 CAN_BR_125K 等）
 * @retval      void
 * @details     根据所选模式分别配置仲裁域和数据域的预分频器、时间段1/2、同步跳转宽度。
 *              内部使用系统时钟120MHz（假设）进行计算。
 *              采样点计算：(1+seg1)/(1+seg1+seg2)，约75%~87.5%。
 */
void bsp_fdcan_set_baud(hcan_t *hfdcan, uint8_t mode, uint8_t baud)
{
    uint32_t nom_brp=0, nom_seg1=0, nom_seg2=0, nom_sjw=0; // 仲裁域参数
    uint32_t dat_brp=0, dat_seg1=0, dat_seg2=0, dat_sjw=0; // 数据域参数

    /* 经典CAN模式：仲裁域使用指定波特率，数据域默认为1M（不启用BRS） */
    if(mode == CAN_CLASS)
    {
        switch (baud)
        {
            // 参数计算：实际波特率 = 120MHz / (brp*(1+seg1+seg2))
            case CAN_BR_125K: nom_brp=6 ; nom_seg1=139; nom_seg2=20; nom_sjw=20; break; // 精确125K，采样点87.5%
            case CAN_BR_200K: nom_brp=3 ; nom_seg1=174; nom_seg2=25; nom_sjw=25; break; // 精确200K，采样点87.5%
            case CAN_BR_250K: nom_brp=3 ; nom_seg1=139; nom_seg2=20; nom_sjw=20; break; // 精确250K，采样点87.5%
            case CAN_BR_500K: nom_brp=1 ; nom_seg1=209; nom_seg2=30; nom_sjw=30; break; // 精确500K，采样点87.5%
            case CAN_BR_1M:   nom_brp=1 ; nom_seg1=89 ; nom_seg2=30; nom_sjw=30; break; // 精确1M，采样点75%
            default:          nom_brp=3 ; nom_seg1=29 ; nom_seg2=10; nom_sjw=10; break;
        }
        dat_brp=1 ; dat_seg1=29; dat_seg2=10; dat_sjw=10;   // 数据域固定1M（实际经典CAN不会用到数据域，但占位）
        hfdcan->Init.FrameFormat = FDCAN_FRAME_CLASSIC;     // 帧格式：经典CAN
    }

    /* CAN FD with BRS模式：仲裁域固定1M，数据域使用指定高速波特率 */
    if(mode == CAN_FD_BRS)
    {
        switch (baud)
        {
            case CAN_BR_2M:   dat_brp=1 ; dat_seg1=44; dat_seg2=15; dat_sjw=15; break; // 精确2.0M，采样点75%
            case CAN_BR_2M5:  dat_brp=1 ; dat_seg1=47; dat_seg2=12; dat_sjw=12; break; // 精确2.5M，采样点80%
            case CAN_BR_3M2:  dat_brp=1 ; dat_seg1=28; dat_seg2=9;  dat_sjw=9;  break; // 实际3.158M，采样点76.3%
            case CAN_BR_4M:   dat_brp=1 ; dat_seg1=23; dat_seg2=6;  dat_sjw=6;  break; // 精确4.0M，采样点80%
            case CAN_BR_5M:   dat_brp=1 ; dat_seg1=20; dat_seg2=3;  dat_sjw=3;  break; // 精确5.0M，采样点87.5%
            default:          dat_brp=1 ; dat_seg1=44; dat_seg2=15; dat_sjw=15; break;
        }
        nom_brp=1 ; nom_seg1=89 ; nom_seg2=30; nom_sjw=30; // 仲裁域固定1M
        hfdcan->Init.FrameFormat = FDCAN_FRAME_FD_BRS;      // 帧格式：CAN FD 且 BRS使能
    }

    // 先反初始化，再写新参数，最后初始化
    HAL_FDCAN_DeInit(hfdcan);

    hfdcan->Init.NominalPrescaler   = nom_brp;   // 仲裁域预分频器（分频值 = 80MHz / brp）
    hfdcan->Init.NominalTimeSeg1    = nom_seg1;  // 相位段1（Tq数）
    hfdcan->Init.NominalTimeSeg2    = nom_seg2;  // 相位段2（Tq数）
    hfdcan->Init.NominalSyncJumpWidth = nom_sjw; // 同步跳转宽度（Tq数）

    hfdcan->Init.DataPrescaler      = dat_brp;   // 数据域预分频器
    hfdcan->Init.DataTimeSeg1       = dat_seg1;  // 数据域相位段1
    hfdcan->Init.DataTimeSeg2       = dat_seg2;  // 数据域相位段2
    hfdcan->Init.DataSyncJumpWidth  = dat_sjw;   // 数据域同步跳转宽度

    HAL_FDCAN_Init(hfdcan);
}

/**
 * @brief       FDCAN发送数据（自动适配经典CAN或CAN FD）
 * @param       hfdcan：FDCAN句柄
 * @param       id：CAN标识符（标准ID，11位）
 * @param       data：待发送数据缓冲区指针
 * @param       len：数据长度（字节，经典CAN最大8，CAN FD支持8/12/16/20/24/32/48/64）
 * @retval      0 - 成功，1 - 失败
 * @details     根据 hfdcan->Init.FrameFormat 自动选择帧格式和位速率切换。
 *              若为经典CAN模式，发送标准CAN帧（BRS关闭，FD格式关闭）；
 *              若为CAN FD模式，发送CAN FD帧（BRS开启，FD格式开启）。
 *              该函数非阻塞，若硬件发送队列满则返回失败。
 */
uint8_t fdcanx_send_data(hcan_t *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
{
    FDCAN_TxHeaderTypeDef pTxHeader;

    // ---------- 公共帧头配置 ----------
    pTxHeader.Identifier = id;                    // CAN标识符
    pTxHeader.IdType = FDCAN_STANDARD_ID;         // 标准帧（11位ID）
    pTxHeader.TxFrameType = FDCAN_DATA_FRAME;     // 数据帧（非远程帧）
    pTxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE; // 主动错误指示
    pTxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    pTxHeader.MessageMarker = 0;

    // ---------- 根据控制器当前工作模式配置帧格式 ----------
    if (hfdcan->Init.FrameFormat == FDCAN_FRAME_CLASSIC)
    {
        // ********** 经典 CAN 模式 **********
        pTxHeader.BitRateSwitch = FDCAN_BRS_OFF;     // 关闭位速率切换
        pTxHeader.FDFormat = FDCAN_CLASSIC_CAN;      // 使用经典 CAN 帧格式
        if (len > 8) return 1;                       // 经典 CAN 最大 8 字节
        pTxHeader.DataLength = len;                  // DLC 直接等于字节数（0~8）
    }
    else   // FDCAN_FRAME_FD_BRS 或其它 FD 模式
    {
        // ********** CAN FD 模式（带 BRS）**********
        pTxHeader.BitRateSwitch = FDCAN_BRS_ON;      // 使能位速率切换（数据域高速）
        pTxHeader.FDFormat = FDCAN_FD_CAN;           // 使用 CAN FD 帧格式

        // 根据数据长度设置对应的 DLC
        if (len <= 8)
            pTxHeader.DataLength = len;
        else if (len == 12)
            pTxHeader.DataLength = FDCAN_DLC_BYTES_12;
        else if (len == 16)
            pTxHeader.DataLength = FDCAN_DLC_BYTES_16;
        else if (len == 20)
            pTxHeader.DataLength = FDCAN_DLC_BYTES_20;
        else if (len == 24)
            pTxHeader.DataLength = FDCAN_DLC_BYTES_24;
        else if (len == 32)
            pTxHeader.DataLength = FDCAN_DLC_BYTES_32;
        else if (len == 48)
            pTxHeader.DataLength = FDCAN_DLC_BYTES_48;
        else if (len == 64)
            pTxHeader.DataLength = FDCAN_DLC_BYTES_64;
        else
            return 1;   // 不支持的数据长度
    }

    // ---------- 发送（非阻塞）----------
    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &pTxHeader, data) != HAL_OK)
        return 1;   // 发送失败（队列满或参数错误）
    return 0;       // 发送成功
}

/**
 * @brief       FDCAN接收数据（从RX FIFO0读取）
 * @param       hfdcan：FDCAN句柄
 * @param       rec_id：用于返回接收到的CAN ID
 * @param       buf：存储接收数据的缓冲区
 * @retval      接收到的数据长度（字节），0表示无新消息
 * @details     从FIFO0中取出一条消息，解析长度并返回。注意该函数不阻塞，若无消息则返回0。
 */
uint8_t fdcanx_receive(hcan_t *hfdcan, uint16_t *rec_id, uint8_t *buf)
{
    FDCAN_RxHeaderTypeDef pRxHeader;
    uint8_t len;

    // 从RX FIFO0获取一条消息（如果FIFO为空则HAL_FDCAN_GetRxMessage返回非HAL_OK）
    if(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &pRxHeader, buf) == HAL_OK)
    {
        *rec_id = pRxHeader.Identifier;           // 返回ID

        // 根据DLC转换为实际字节数
        if(pRxHeader.DataLength <= FDCAN_DLC_BYTES_8)
            len = pRxHeader.DataLength;
        else if(pRxHeader.DataLength == FDCAN_DLC_BYTES_12)
            len = 12;
        else if(pRxHeader.DataLength == FDCAN_DLC_BYTES_16)
            len = 16;
        else if(pRxHeader.DataLength == FDCAN_DLC_BYTES_20)
            len = 20;
        else if(pRxHeader.DataLength == FDCAN_DLC_BYTES_24)
            len = 24;
        else if(pRxHeader.DataLength == FDCAN_DLC_BYTES_32)
            len = 32;
        else if(pRxHeader.DataLength == FDCAN_DLC_BYTES_48)
            len = 48;
        else if(pRxHeader.DataLength == FDCAN_DLC_BYTES_64)
            len = 64;

        return len;    // 返回实际数据长度
    }
    return 0;          // 无新消息
}


/**
 * @brief       FDCAN1 接收FIFO0中断回调（由HAL库调用）
 * @param       hfdcan：触发中断的FDCAN句柄
 * @param       RxFifo0ITs：具体中断标志
 * @retval      void
 * @details     在此函数中直接处理大疆电机和达妙电机反馈（同级，无优先级）
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
 // 检查是否是RX FIFO0水位线中断（我们设置的水位线为1，即每收到1帧就触发）
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_WATERMARK) == 0)
        return;   // 不是预期的中断，直接返回

    uint16_t rec_id;          // 接收到的CAN ID
    uint8_t rx_data[64];      // 数据缓冲区（最大64字节，兼容CAN FD）
    uint8_t len;              // 实际接收到的数据长度（字节）

    // ----- 循环读取FIFO0中所有待处理的消息（直到硬件FIFO为空）-----
    // fdcanx_receive 每次从FIFO0取出一条消息，若无消息则返回0，循环结束
    while ((len = fdcanx_receive(hfdcan, &rec_id, rx_data)) != 0)
    {
        /* ====================处理大疆电机反馈 ==================== */
        // 大疆电机反馈使用的标准ID范围：
        if (rec_id >= 0x201 && rec_id <= 0x20B)
        {
            // 大疆电机反馈固定为8字节，若收到的数据少于8字节则忽略（实际不会发生）
            if (len >= 8)
            {
                // 构造一个虚拟的 RxHeader，因为 Motor_ReceiveFeedback 需要该结构体
                FDCAN_RxHeaderTypeDef rx_header;
                rx_header.Identifier = rec_id;
                rx_header.IdType     = FDCAN_STANDARD_ID;
                rx_header.DataLength = FDCAN_DLC_BYTES_8;   // 大疆协议固定8字节
                // 调用大疆电机反馈解析函数（定义在 DJI_Motor.c 中）
                DJI_Motor_ReceiveFeedback(hfdcan, &rx_header, rx_data);
            }
        }

        /* ====================处理达妙电机反馈 ==================== */
        // 遍历全局达妙电机数组 motor[]，寻找 ID 和 CAN 句柄都匹配的电机
        for (int i = 0; i < num; i++)
        {
            // motor[i].id    : 该电机的CAN反馈ID（通常为0x01~0x0F）
            // motor[i].hcan  : 该电机所连接的CAN口句柄
            if (motor[i].id == rec_id && motor[i].hcan == hfdcan)
            {
                // 解析反馈数据（位置、速度、扭矩、温度等）
                dm_motor_fbdata(&motor[i], rx_data);
                // 如果当前处于读取寄存器模式，则继续解析寄存器数据
                receive_motor_data(&motor[i], rx_data);
                break;   // 找到匹配的电机后退出循环，避免重复处理
            }
        }
        // 注：如果消息ID既不属于大疆范围，也不在 motor[] 中，则直接丢弃
    }
    // 循环结束后，FIFO0中的所有消息均已处理完毕，硬件可继续接收新消息
}

/**
 * @brief       FDCAN错误状态回调（总线关闭、错误被动、错误警告等）
 * @param       hfdcan：FDCAN句柄
 * @param       ErrorStatusITs：触发的中断标志组合
 * @retval      void
 */
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
    // 总线关闭错误：清除 INIT 位，让CAN控制器自动恢复通信
    if(ErrorStatusITs & FDCAN_IR_BO)
    {
        CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
    }

    // 错误被动中断：原打算重新初始化，但被注释
    if(ErrorStatusITs & FDCAN_IR_EP)
    {
        // MX_FDCAN1_Init();
        // bsp_can_init();
    }
}
