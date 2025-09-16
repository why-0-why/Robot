#ifndef MDL_COMM_H
#define MDL_COMM_H

/* 包含头文件 ----------------------------------------------------------------*/
#include "fifo.h"
#include "linux_list.h"
#include "mdl_Chassis.h"
#include "mdl_Gimbal.h"
#include "alg_crc.h"


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

#endif /* MDL_COMM_H */