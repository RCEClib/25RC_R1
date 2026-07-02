#ifndef __DM_MOTOR_DRV_H__
#define __DM_MOTOR_DRV_H__
#include "main.h"
#include "fdcan.h"
#include "bsp_fdcan.h"

#define MIT_MODE 			0x000
#define POS_MODE			0x100
#define SPD_MODE			0x200
#define PSI_MODE		  	0x300

#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f

typedef enum
{
    Motor1,
    Motor2,
    Motor3,
    Motor4,
    Motor5,
    Motor6,
	Motor7,
	Motor8,
	Motor9,
	Motor10,
    num
} motor_num;

typedef enum
{
	mit_mode = 1,
	pos_mode = 2,
	spd_mode = 3,
	psi_mode = 4
} mode_e;

typedef enum {
    RID_UV_VALUE=0,    // ��ѹ����ֵ
    RID_KT_VALUE=1,    // Ť��ϵ��
    RID_OT_VALUE=2,    // ���±���ֵ
    RID_OC_VALUE=3,    // ��������ֵ
    RID_ACC		=4,    // ���ٶ�
    RID_DEC		=5,    // ���ٶ�
    RID_MAX_SPD	=6,    // ����ٶ�
    RID_MST_ID	=7,    // ����ID
    RID_ESC_ID	=8,    // ����ID
    RID_TIMEOUT	=9,    // ��ʱ����ʱ��
    RID_CMODE	=10,   // ����ģʽ
    RID_DAMP	=11,   // ���ճ��ϵ��
    RID_INERTIA =12,   // ���ת������
    RID_HW_VER	=13,   // ����
    RID_SW_VER	=14,   // �����汾��
    RID_SN		=15,   // ����
    RID_NPP		=16,   // ���������
    RID_RS		=17,   // ����
    RID_LS		=18,   // ���
    RID_FLUX	=19,   // ����
    RID_GR		=20,   // ���ּ��ٱ�
    RID_PMAX	=21,   // λ��ӳ�䷶Χ
    RID_VMAX	=22,   // �ٶ�ӳ�䷶Χ
    RID_TMAX	=23,   // Ť��ӳ�䷶Χ
    RID_I_BW	=24,   // ���������ƴ���
    RID_KP_ASR	=25,   // �ٶȻ�Kp
    RID_KI_ASR	=26,   // �ٶȻ�Ki
    RID_KP_APR	=27,   // λ�û�Kp
    RID_KI_APR	=28,   // λ�û�Ki
    RID_OV_VALUE=29,   // ��ѹ����ֵ
    RID_GREF	=30,   // ��������Ч��
    RID_DETA	=31,   // �ٶȻ�����ϵ��
    RID_V_BW	=32,   // �ٶȻ��˲�����
    RID_IQ_CL	=33,   // ��������ǿϵ��
    RID_VL_CL	=34,   // �ٶȻ���ǿϵ��
    RID_CAN_BR	=35,   // CAN�����ʴ���
    RID_SUB_VER	=36,   // �Ӱ汾��
    RID_U_OFF	=50,   // u��ƫ��
    RID_V_OFF	=51,   // v��ƫ��
    RID_K1		=52,   // ��������1
    RID_K2		=53,   // ��������2
    RID_M_OFF	=54,   // �Ƕ�ƫ��
    RID_DIR		=55,   // ����
    RID_P_M		=80,   // ���λ��
    RID_X_OUT	=81    // �����λ��
} rid_e;




// �������
typedef struct
{
	uint8_t read_flag;
	uint8_t write_flag;
	uint8_t save_flag;
	
    float UV_Value;		// ��ѹ����ֵ
    float KT_Value;		// Ť��ϵ��
    float OT_Value;		// ���±���ֵ
    float OC_Value;		// ��������ֵ
    float ACC;			// ���ٶ�
    float DEC;			// ���ٶ�
    float MAX_SPD;		// ����ٶ�
    uint32_t MST_ID;	// ����ID
    uint32_t ESC_ID;	// ����ID
    uint32_t TIMEOUT;	// ��ʱ����ʱ��
    uint32_t cmode;		// ����ģʽ
    float  	 Damp;		// ���ճ��ϵ��
    float    Inertia;	// ���ת������
    uint32_t hw_ver;	// ����
    uint32_t sw_ver;	// �����汾��
    uint32_t SN;		// ����
    uint32_t NPP;		// ���������
    float    Rs;		// ����
    float    Ls;		// ���
    float    Flux;		// ����
    float    Gr;		// ���ּ��ٱ�
    float    PMAX;		// λ��ӳ�䷶Χ
    float    VMAX;		// �ٶ�ӳ�䷶Χ
    float    TMAX;		// Ť��ӳ�䷶Χ
    float    I_BW;		// ���������ƴ���
    float    KP_ASR;	// �ٶȻ�Kp
    float    KI_ASR;	// �ٶȻ�Ki
    float    KP_APR;	// λ�û�Kp
    float    KI_APR;	// λ�û�Ki
    float    OV_Value;	// ��ѹ����ֵ
    float    GREF;		// ��������Ч��
    float    Deta;		// �ٶȻ�����ϵ��
    float 	 V_BW;		// �ٶȻ��˲�����
    float 	 IQ_cl;		// ��������ǿϵ��
    float    VL_cl;		// �ٶȻ���ǿϵ��
    uint32_t can_br;	// CAN�����ʴ���
    uint32_t sub_ver;	// �Ӱ汾��
	float 	 u_off;		// u��ƫ��
	float	 v_off;		// v��ƫ��
	float	 k1;		// ��������1
	float 	 k2;		// ��������2
	float 	 m_off;		// �Ƕ�ƫ��
	float  	 dir;		// ����
	float	 p_m;		// ���λ��
	float	 x_out;		// �����λ��
} esc_inf_t;

// ����ش���Ϣ�ṹ��
typedef struct
{
    int id;
    int state;
    int p_int;
    int v_int;
    int t_int;
    int kp_int;
    int kd_int;
    float pos;
    float vel;
    float tor;
    float Kp;
    float Kd;
    float Tmos;
    float Tcoil;
} motor_fbpara_t;

// ����������ýṹ��
typedef struct
{
    uint8_t mode;
    float pos_set;
    float vel_set;
    float tor_set;
	float cur_set;
    float kp_set;
    float kd_set;
} motor_ctrl_t;

typedef struct
{
    uint16_t id;
	uint16_t mst_id;
    motor_fbpara_t para;
    motor_ctrl_t ctrl;
	esc_inf_t tmp;
    FDCAN_HandleTypeDef *hcan;   // ������ָ��õ��ʹ�õ� CAN ���
} motor_t;



float uint_to_float(int x_int, float x_min, float x_max, int bits);
int float_to_uint(float x_float, float x_min, float x_max, int bits);
void dm_motor_ctrl_send(motor_t *motor);
void dm_motor_enable(motor_t *motor);
void dm_motor_disable(motor_t *motor);
void dm_motor_clear_para(motor_t *motor);
void dm_motor_clear_err(motor_t *motor);
void dm_motor_fbdata(motor_t *motor, uint8_t *rx_data);

void enable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id);
void disable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id);

void mit_ctrl(hcan_t* hcan, motor_t *motor, uint16_t motor_id, float pos, float vel,float kp, float kd, float tor);
void pos_ctrl(hcan_t* hcan, uint16_t motor_id, float pos, float vel);
void spd_ctrl(hcan_t* hcan, uint16_t motor_id, float vel);
void psi_ctrl(hcan_t* hcan, uint16_t motor_id, float pos, float vel, float cur);
	
void save_pos_zero(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id);
void clear_err(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id);

void read_motor_data(hcan_t* hfdcan, uint16_t id, uint8_t rid);
void read_motor_ctrl_fbdata(hcan_t* hfdcan,uint16_t id);
void write_motor_data(hcan_t* hfdcan,uint16_t id, uint8_t rid, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3);
void save_motor_data(hcan_t* hfdcan,uint16_t id, uint8_t rid);

#endif /* __DM_MOTOR_DRV_H__ */

