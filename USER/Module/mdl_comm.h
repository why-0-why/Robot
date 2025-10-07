#ifndef MDL_COMM_H
#define MDL_COMM_H

/* 包含头文件 ----------------------------------------------------------------*/
//#include "fifo.h"// 数据类型及操作函数,Chassis/Gimbal中也有调用
#include "linux_list.h" // 数据类型
#include "Motor.h"
#include "alg_pid.h"
#include "mdl_Console.h"
#include "mdl_IMU.h"

//底盘数据结构
typedef enum
{
    CHASSIS_RELAX = 0,          //安全模式
    CHASSIS_STOP,               //底盘停止
    CHASSIS_FOLLOW_GIMBAL,      //底盘跟随云台
    CHASSIS_SEPARATE_GIMBAL,    //底盘云台分离
    CHASSIS_SPIN,                //底盘旋转
    CHASSIS_ISLAND,
} ChassisCtrlMode_e;


typedef struct
{
    float wheel_perimeter; /* the perimeter(mm) of wheel */
    float wheeltrack;      /* wheel track distance(mm) */
    float wheelbase;       /* wheelbase distance(mm) */
    float rotate_x_offset; /* rotate offset(mm) relative to the x-axis of the chassis center */
    float rotate_y_offset; /* rotate offset(mm) relative to the y-axis of the chassis center */
} MechanicalStructure_t;

typedef struct
{
    MotorInfo_t*    motor_info;
    pid_t           pid;
    float            given_speed;
    int16_t         current_set;
} ChassisMotor_t;

typedef struct
{
    Console_t*    console;
    IMU_Data_t*   imu;                      //底盘陀螺仪指针
    CAN_Object_t* chassis_can;              //

    ChassisCtrlMode_e       ctrl_mode;             //底盘控制状态
    MechanicalStructure_t   structure;
    ChassisMotor_t          chassis_motor[4];
    pid_t                   chassis_follow_pid;        //follow angle PID.
    pid_t                  super_power_limit_pid;

    float vx;                      //
    float vy;                      //
    float vw;                      //
    float wheel_rpm[4];

    float Super_Power_ratio;         //超电充电比
    float Chassis_super_power;
    uint8_t	SuperPower_State;

    float gimbal_yaw_ecd_angle;
    float chassis_yaw;   //the yaw angle calculated by gyro sensor and gimbal motor.
    float chassis_pitch; //the pitch angle calculated by gyro sensor and gimbal motor.
    float chassis_roll;  //the roll angle calculated by gyro sensor and gimbal motor.

}	ChassisHandle_t;

typedef union
{
    uint8_t u[8];
    uint16_t f;
    uint8_t super_state;
    uint16_t f2;
    double ff;
}FormatTrans_t;

/*                             云台数据包                               */

typedef enum
{
    GIMBAL_RELAX = 0,
    GIMBAL_INIT,
    GIMBAL_GYRO,
    GIMBAL_RELATIVE,
    GIMBAL_NORMAL,
    GIMBAL_VISION,
    GIMBAL_ZARO,
} GimbalCtrlMode_e;


// TODO:放到comm组件里
#pragma pack(push, 1)
typedef struct
{
    uint8_t data_head;          // 默认 0xAA
    uint8_t pitch[4];
    uint8_t yaw[4];
    uint8_t palstance[4];
    uint8_t can_shoot[2];
    uint8_t enemy_move_state[2];
    uint8_t state[2];
    uint8_t data_tail;
} Comm_VisionInfo_t;

typedef struct
{
    uint8_t data_head;
    uint8_t enemy_color;
    float   yaw_relative_angle;
    float   pitch_relative_angle;
    uint8_t robot_level;
} Comm_RobotInfo_t;
#pragma pack(pop)

typedef enum
{
    RAW_VALUE_MODE = 0,
    GYRO_MODE,
    ENCONDE_MODE,
} GimbalMotorMode_e;

typedef struct
{
    float            relative_angle; /* unit: degree */
    float            gyro_angle;
    float            palstance;      /* uint: degree/s */
} GimbalSensor_t;

typedef struct
{
    pid_t           outer_pid;
    pid_t           inter_pid;
    float            angle_ref;
    float            angle_fdb;
    float            speed_ref;
    float            speed_fdb;
} Gimbal_PID_t;

typedef struct
{
    MotorInfo_t*    motor_info;
    uint16_t        offset_ecd;
    float            ecd_ratio;
    float            max_relative_angle;
    float            min_relative_angle;

    GimbalMotorMode_e mode;
    GimbalMotorMode_e last_mode;
    float            given_value;         //输入期望值
    float            last_given_value;    //上一次输入期望值

    GimbalSensor_t  sensor;             //传感器数据（包含编码器相对角度、陀螺仪角度、角速度）
    Gimbal_PID_t    pid;

    int16_t         current_set;
} GimbalMotor_t;

typedef struct
{
    Console_t*      console;
    IMU_Data_t*     imu;              //
    CAN_Object_t*   gimbal_can;    //

    Comm_RobotInfo_t*    vision_tx_data;
    Comm_VisionInfo_t*   vision_rx_data;

    Gimbal_CMD_e last_cmd;
    GimbalCtrlMode_e ctrl_mode;  //
    GimbalCtrlMode_e last_ctrl_mode;  //
    GimbalCtrlMode_e last_moment_ctrl_mode;  //

    GimbalMotor_t   yaw_motor;
    GimbalMotor_t   pitch_motor;

    float       infrared;
} GimbalHandle_t;



typedef enum
{
    AIM_NO = 0,
    AIM_RIGHT,
} VisionAim_e;

typedef enum
{
    RELAX = 0,
    FOLLOW,
    FIRST_AIMING,
    COMPLETE_AIMING,
} AimContorl_e;

typedef struct
{
    float    pitch;
    float    yaw;
    float    palstance;
    float    accelerated_speed;
    float    last_pitch;
    float    last_yaw;
    float    last_palstance;
    float    last_accelerated_speed;
    float    rpm;
    int      can_shoot;
    int      switch_armor;
    int      last_switch_armor;
    int      enemy_move_state;
    int      state;     //VisionState_e  0x01->Comm_Successed
    VisionAim_e yaw_success;
    VisionAim_e pitch_success;
} VisionDatabase_t;

typedef struct
{
    AimContorl_e aim_mode;
    uint32_t pid_SAtime;
    uint32_t aiming_time;
    uint32_t stay_time;
    uint32_t systeam_time;
    float tol_angle ;                      //人为规定消抖角度范围(-tol_angle,tol_angle)
    uint32_t tol_time ;                    //the aiming-done continueous time that user set 人为规定
    uint32_t Sstart_time,Astart_time;      //消抖持续时间,单次自瞄持续时间
    int first_aim ;                        //初入消抖范围标志
    int aim_flag ;                         //新的单次自瞄标志
    float Ap_parm;
    float Sp_parm;
    int enable_paramSA;
} AutoAim_t;

typedef struct
{
    float angle[2];  //目标角度
    float gv[2];     //角速度
    float acc[2];    //角加速度
    float out;
    uint32_t times;
} AimCalcData_t;

typedef struct
{
    float yaw_now;
    float yaw_last;
    float pitch_now;
    float pitch_last;
    float distance;
} AutoAimError_t;

typedef struct
{
    float yaw_lpc;   //移动预测系数   38；35；30
    float pitch_lpc;
    float auto_err_yaw;   //角度误差
    float auto_err_pitch;
    float kalman_filter_delay;   //预测开启延时
    float kalman_filter_yaw_speed_min;
    float kalman_filter_pitch_speed_min;
    float kalman_filter_yaw_speed_max;
    float kalman_filter_pitch_speed_max;
    float kalman_filter_yaw_amplification;   //预测增幅    130；125；115；135
    float kalman_filter_pitch_amplification;
} AutoAimCoefficient_t;

typedef struct
{
    uint32_t delay_cnt;
    uint32_t freq;
    uint32_t last_time;
    float last_position;
    float speed;
    float last_speed;
    float processed_speed;
} Speed_Calc_Data_t;

typedef struct
{
    uint32_t auto_aim_shoot_l;
    uint32_t auto_aim_shoot_r;
    uint32_t auto_aim_shoot_stop;
} Aim_Shootdelay_t;
/*                             传递消息包                               */
typedef struct
{
    uint8_t  sof;
    uint16_t data_length;
    uint8_t  seq;
    uint8_t  crc8;
} FrameHeader_t;

typedef enum
{
    STEP_HEADER_SOF  = 0,
    STEP_LENGTH_LOW  = 1,
    STEP_LENGTH_HIGH = 2,
    STEP_FRAME_SEQ   = 3,
    STEP_HEADER_CRC8 = 4,
    STEP_DATA_CRC16  = 5,
} UnpackStep_e;

typedef void (*UnpackHookFunc_t)(uint16_t cmd_id, uint8_t *data, uint16_t len);
typedef void (*TransmitHookFunc_t)(uint8_t *data, uint16_t len);
typedef uint16_t (*GetPortFreeLength)(void);

#define PROTOCOL_FRAME_MAX_SIZE 128

typedef struct
{
    list_t          list;
    fifo_s_t        fifo;
    uint8_t         header_sof;
    uint16_t        data_len;
    uint8_t         protocol_packet[PROTOCOL_FRAME_MAX_SIZE];
    UnpackStep_e    unpack_step;
    uint16_t        index;
    UnpackHookFunc_t func;
} ReceiveHandle_t;

typedef struct
{
    list_t          list;
    fifo_s_t        fifo;
    uint8_t         seq;
    uint8_t         protocol_packet[PROTOCOL_FRAME_MAX_SIZE];
    TransmitHookFunc_t func;
    GetPortFreeLength get_free_func;
} TransmitHandle_t;

#pragma pack(push, 1)
typedef struct
{
    ChassisCtrlMode_e mode;
} Comm_ChassisInfo_t;

typedef struct
{
    GimbalCtrlMode_e mode;
    float pitch_ecd_angle;
    float yaw_ecd_angle;
    float pitch_gyro_angle;
    float yaw_gyro_angle;
    float pitch_rate;
    float yaw_rate;
    float infrared;
    int   can_shoot;
} Comm_GimbalInfo_t;
#pragma pack(pop)


/* 宏定义 --------------------------------------------------------------------*/
#define PROTOCOL_HEADER_SIZE            sizeof(FrameHeader_t)
#define PROTOCOL_CMD_SIZE               2
#define PROTOCOL_CRC16_SIZE             2
#define HEADER_CRC_LEN                  (PROTOCOL_HEADER_SIZE + PROTOCOL_CRC16_SIZE)
#define HEADER_CRC_CMDID_LEN            (PROTOCOL_HEADER_SIZE + PROTOCOL_CRC16_SIZE + sizeof(uint16_t))
#define HEADER_CMDID_LEN                (PROTOCOL_HEADER_SIZE + sizeof(uint16_t))

/* 扩展变量 ------------------------------------------------------------------*/
extern Comm_GimbalInfo_t gimbal_info;
extern Comm_VisionInfo_t vision_info;
extern Comm_RobotInfo_t robot_info;

/* 函数声明 ------------------------------------------------------------------*/
void Comm_ReceiveInit(ReceiveHandle_t* p_handle, uint8_t header_sof, uint8_t *rx_fifo_buffer, uint16_t rx_fifo_size, UnpackHookFunc_t func);
void Comm_ReceiveData(ReceiveHandle_t* p_handle, uint8_t *p_data, uint16_t len);
void Comm_ReceiveDataHandler(void);
void Comm_TransmitInit(TransmitHandle_t* p_handle, uint8_t *tx_fifo_buffer, uint16_t tx_fifo_size, TransmitHookFunc_t func);
void Comm_TransmitData(TransmitHandle_t* p_handle, uint8_t header_sof, uint16_t cmd_id, uint8_t *p_data, uint16_t len);
void Comm_TransmitData_Vision(TransmitHandle_t* p_handle, uint8_t* p_data, uint16_t len);
void Comm_TransmitDataHandler(void);

extern Comm_GimbalInfo_t* GimbalInfo_Pointer(void);
extern Comm_VisionInfo_t* VisionInfo_Pointer(void);
extern Comm_RobotInfo_t* RobotInfo_Pointer(void);

/*------------------------------------板间通讯-----------------------------*/
    
typedef enum
{
    RC_DATA_CMD_ID           = 0x0001,
    CHASSIS_INFO_CMD_ID      = 0x0002,
    GIMBAL_INFO_CMD_ID       = 0x0003,
    CAP_INFO_CMD_ID          = 0x026,//0x211张泓、0x026浙纺23赛季
    LONG_CAP_INFO_CMD_ID     = 0x211,
} USER_CMD_ID_e;

#pragma pack(pop)
/* 宏定义 --------------------------------------------------------------------*/
#define USER_PROTOCOL_HEADER_SOF     0xAA
/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/
void UserProtocol_ParseHandler(uint16_t cmd_id, uint8_t *data, uint16_t len);
Comm_ChassisInfo_t* ChassisInfo_Pointer(void);
Comm_GimbalInfo_t* GimbalInfo_Pointer(void);

extern Comm_ChassisInfo_t chassis_info;
extern Comm_GimbalInfo_t gimbal_info;
/*------------------------------------视觉通讯-----------------------------*/
/* 类型定义 ------------------------------------------------------------------*/
typedef enum
{
    VISION_DATA_CMD_ID          = 0x0001,
    ROBOT_DATA_CMD_ID           = 0x0002,
} VISION_CMD_ID_e;

typedef enum
{
    VISION_TRACK_LOSS       = 0x0001,
    VISION_TRACK            = 0x01,
    VISION_TARGET_CHANGE    = 0x0003
} VisionState_e;

typedef enum
{
    Hero        = 0x0001,
    Engineer    = 0x0002,
    Infantry    = 0x0004,
    Sentey      = 0x0008,
    AllRobot    = Hero|Engineer|Infantry|Sentey,
    Windmill    = 0x1000
} DetectTarget_e;

typedef enum
{
    Gray = 0,
    Red = 1,
    Blue = 2,
    AllColor = Red|Blue
} EnemyColor_e;


/* 宏定义 --------------------------------------------------------------------*/
#define VISION_PROTOCOL_HEADER_SOF     0x55
#define VISION_DATA_FIFO_SIZE          (256u)
/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/
void VisionProtocol_ParseHandler(uint16_t cmd_id, uint8_t* data, uint16_t len);
Comm_VisionInfo_t* VisionInfo_Pointer(void);
Comm_RobotInfo_t* RobotInfo_Pointer(void);

int TwoBytesToInt (uint8_t byte[2]);
float FourBytesToFloat (uint8_t byte[4]);

#endif /* MDL_COMM_H */