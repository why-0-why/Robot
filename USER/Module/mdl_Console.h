//
// Created by ZJH on 25-8-6.
//

#ifndef MDL_CONSOLE_H
#define MDL_CONSOLE_H
#include "alg_ramp.h"
#include "dvc_dt7.h"

typedef enum
{
    PREPARE_MODE = 0,       //初始化
    NORMAL_MODE,            //正常运行模式
    SAFETY_MODE,              //安全模式（停止运动）
} CtrlMode_e;
typedef enum
{
    GIMBAL_RELEASE_CMD = 0,
    GIMBAL_INIT_CMD,
    GIMBAL_GYRO_CMD,
    GIMBAL_RELATIVE_CMD,
    GIMBAL_NORMAL_CMD,
    GIMBAL_VISION_CMD,          //视觉
} Gimbal_CMD_e;

typedef enum
{
    CHASSIS_RELEASE_CMD = 0,
    CHASSIS_STOP_CMD,               //底盘停止
    CHASSIS_FOLLOW_GIMBAL_CMD,      //底盘跟随云台
    CHASSIS_SEPARATE_GIMBAL_CMD,    //底盘云台分离
    CHASSIS_SPIN_CMD,               //底盘旋转
    CHASSIS_ISLAND_CMD,             //上岛
    CHASSIS_BEHIND_CMD,             //转头
    CHASSIS_SUPERPOWER_CMD,         //超电


} Chassis_CMD_e;

typedef enum
{
    SHOOT_RELEASE_CMD = 0,
    SHOOT_START_CMD,
    SHOOT_STOP_CMD,
      SHOOT_INIT_CMD,
} Shoot_CMD_e;

typedef enum
{
    STOP_FIRE_CMD = 0, //停止射击
    ONE_FIRE_CMD,      //单发模式
    RAPID_FIRE_CMD,    //无限制连发模式
} ShootFire_CMD_e;

typedef enum
{
    ISLAND_RELAX_CMD  = 0,
    ISLAND_START_CMD,
    ISLAND_OP_CMD
} Island_CMD_e;

typedef enum
{
    AIR_OPEN = 0,
    AIR_CLOSE,
} Air_CMD_e;

typedef enum
{
    Power_IN	=	0,
    Power_Out	=	1,
}Power_cmd;

typedef enum
{
    Heat_limit_on=	0,
    Heat_limit_off	=	1,
}Heat_limit_state;

typedef enum
{
    MAGAZINE_INIT_CMD = 0,
    MAGAZINE_OFF_CMD,
    MAGAZINE_ON_CMD,
} Magazine_CMD_e;

typedef enum
{
    SUPERCAP_OFF_CMD=0,
      SUPERCAP_ON_CMD,
  }SuperCap_CMD_e;

typedef enum
{
    ON = 0,
    OFF,
}flag_e;

typedef enum
{
    flag_positive,
    flag_reverse,
}gimbal_motor_flag_e;

typedef enum
{
    LEFT=0,
    RIGHT,
}gimbal_motor_direction;

typedef struct
{
    RC_Info_t* rc;
    CtrlMode_e ctrl_mode;
    CtrlMode_e last_ctrl_mode;
    Gimbal_CMD_e gimbal_cmd;
    Chassis_CMD_e chassis_cmd;
    Shoot_CMD_e shoot_cmd;
    Magazine_CMD_e magazine_cmd;
    SuperCap_CMD_e supercap_cmd;
    Island_CMD_e island_cmd;
    flag_e shift_flag;
    
    gimbal_motor_flag_e Reset_yaw;
    gimbal_motor_flag_e Reset_pitch;
    gimbal_motor_direction  gimbal_motor_direction_cmd;

    struct
    {
        float vx;
        float vy;
        float vw;
    } chassis;

    struct
    {
        float pitch_v;
        float yaw_v;
        ramp_v0_t pitch_ramp;
        ramp_v0_t yaw_ramp;
    } gimbal;

    struct
    {
        ShootFire_CMD_e fire_cmd;
        ShootFire_CMD_e last_fire_cmd;
        ramp_v0_t trigger_ramp;
    } shoot;
		
    float Yaw_comps;
    float Pitch_comps;
    float spin90_flag;
    Power_cmd	SuperPower_cmd;
    uint8_t clipped_flag;
    uint8_t r3_flag;
    uint8_t Keyboard_flag;
    Heat_limit_state heat_limit;
    uint8_t auto_aim_flag;
    uint8_t r_vision_flag;
    uint8_t q_vision_flag;
    int16_t friction_wheel_speed_buff;
    int8_t can_shoot_cmd;//自动开火命令
    int8_t shoot_flag;
    uint8_t revolve90;
		
} Console_t;

extern RC_Info_t last_rc;
extern RC_Switch_t wheel_switch;
extern RC_Switch_t rc_switch2;

extern Console_t console;
extern ramp_v0_t front_back_ramp;
extern ramp_v0_t left_right_ramp;
extern ramp_v0_t shift_front_ramp;
extern ramp_v0_t shift_left_ramp;

void RemoteControlWheelAction(void);
void Other_Operation(void);
void RemoteControl_Operation(void);
void Keyboard_Operation(void);

Console_t* Console_Pointer(void);
#endif //MDL_CONSOLE_H
