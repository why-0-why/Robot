//
// Created by ZJH on 25-8-6.
//

#ifndef ROBOT_INFO_H
#define ROBOT_INFO_H

/******************************************************************************
 *                                                              机械安装参数                                                                      *
 ******************************************************************************/
#define WHEEL_RADIUS                (76)    //轮子半径(mm)
#define WHEEL_PERIMETER             (478)   //轮子周长(mm)
#define WHEELTRACK                  (489)   //轮距(mm)
#define WHEELBASE                   (460)   //轴距(mm)
#define GIMBAL_X_OFFSET             (0)     //云台相对底盘中心X轴偏移
#define GIMBAL_Y_OFFSET             (0)     //云台相对底盘中心Y轴偏移
#define PITCH_REDUCTION_RATIO       (1.0f)  //pitch减速比
#define YAW_REDUCTION_RATIO         (1.0f)  //yaw减速比
#define PITCH_MOTO_POSITIVE_DIR     (1.0f) //pitch电机安装方向
#define YAW_MOTO_POSITIVE_DIR       (1.0f)  //yaw电机安装方向

/******************************************************************************
 *                                                                   移动控制                                                                         *
 ******************************************************************************/
#define MAX_CHASSIS_VX_SPEED        (4000.0f)
#define MAX_CHASSIS_VY_SPEED        (4000.0f)
#define MAX_CHASSIS_VW_SPEED        (300.0f)
/*-----------------↓ 遥控 ↓-----------------*/
#define RC_CHASSIS_MAX_SPEED_X      MAX_CHASSIS_VX_SPEED     //X轴方向最大速度(mm/s)
#define RC_CHASSIS_MAX_SPEED_Y      MAX_CHASSIS_VX_SPEED     //Y轴方向最大速度(mm/s)
#define RC_CHASSIS_MAX_SPEED_R      MAX_CHASSIS_VW_SPEED     //旋转最大速度(deg/s)
#define RC_GIMBAL_MOVE_RATIO_PIT    0.0005f       //pitch移动比例
#define RC_GIMBAL_MOVE_RATIO_YAW    0.0005f       //yaw移动比例
/*---------------↓ 鼠标键盘 ↓---------------*/
#define KB_CHASSIS_MAX_SPEED_X      MAX_CHASSIS_VX_SPEED     //X轴方向最大速度
#define KB_CHASSIS_MAX_SPEED_Y      MAX_CHASSIS_VX_SPEED     //Y轴方向最大速度
#define KB_CHASSIS_MAX_SPEED_R      MAX_CHASSIS_VW_SPEED      //旋转最大速度
#define KB_GIMBAL_MOVE_RATIO_PIT    0.005f       //pitch移动比例
#define KB_GIMBAL_MOVE_RATIO_YAW    0.01f        //yaw移动比例
#define KB_SENSITIVITY_MULTIPLE     5            //鼠标灵敏度修改倍数(被除数)
#define KB_SENSITIVITY_MULTIPLE_QUICK     2.5            //鼠标灵敏度修改倍数(被除数)

#define VS_GIMBAL_MOVE_RATIO_YAW    0.15f         //yaw移动比例
#define VS_GIMBAL_MOVE_RATIO_PIT    0.15f         //pitch移动比例

#define CHASSIS_ACCEL_TIME      1500  //ms
#define ROTATE_ACCEL_TIME       3000  //ms
#define CHASSIS_SHIFT_ACCEL_TIME      2161  //ms            底盘加速度时间


/******************************************************************************
 *                              CAN通讯ID配置                                                                    *
 ******************************************************************************/
/*---------------↓ 底盘电机ID ↓---------------*/
#define CHASSIS_MOTOR_CONTROL_STD_ID    MOTOR_1TO4_CONTROL_STD_ID
#define CHASSIS_MOTOR_LF_MESSAGE_ID     MOTOR_1_FEEDBACK_ID     //底盘左前电机
#define CHASSIS_MOTOR_RF_MESSAGE_ID     MOTOR_2_FEEDBACK_ID     //底盘右前电机
#define CHASSIS_MOTOR_LB_MESSAGE_ID     MOTOR_3_FEEDBACK_ID     //底盘左后电机
#define CHASSIS_MOTOR_RB_MESSAGE_ID     MOTOR_4_FEEDBACK_ID     //底盘右后电机
/*---------------↓ 云台电机ID ↓---------------*/
#define GIMBAL_MOTOR_CONTROL_STD_ID     MOTOR_5TO8_CONTROL_STD_ID
#define GIMBAL_MOTOR_YAW_MESSAGE_ID     MOTOR_6_FEEDBACK_ID     //Yaw云台电机
#define GIMBAL_MOTOR_PITCH_MESSAGE_ID   MOTOR_5_FEEDBACK_ID     //Pitch云台电机
/*---------------↓ 射击电机ID ↓---------------*/
#define SHOOT_MOTOR_CONTROL_STD_ID      MOTOR_1TO4_CONTROL_STD_ID
#define FRICTION_WHEEL_1_MESSAGE_ID     MOTOR_1_FEEDBACK_ID
#define FRICTION_WHEEL_2_MESSAGE_ID     MOTOR_2_FEEDBACK_ID
#define PLUCK_MOTOR_MESSAGE_ID          MOTOR_5_FEEDBACK_ID
#define COVER_MOTOR_MESSAGE_ID          MOTOR_6_FEEDBACK_ID
/*---------------↓ 超电ID ↓---------------*/
#define	Super_Power_Message_ID						0x026
#define LONGSuper_Power_Message_ID        0x211
/******************************************************************************
 *                                                     云台主控与底盘主控交互ID                            *
 ******************************************************************************/
#define GIMBAL_DATA_STD_ID              (0x600)
#define CHASSIS_DATA_STD_ID             (0x500)
#define GIMBAL_TX_DATA_STD_ID           GIMBAL_DATA_STD_ID
#define GIMBAL_RX_DATA_STD_ID           CHASSIS_DATA_STD_ID
#define CHASSIS_TX_DATA_STD_ID          CHASSIS_DATA_STD_ID
#define CHASSIS_RX_DATA_STD_ID          GIMBAL_DATA_STD_ID
#define GIMBAL_CHASSIS_DATA_FIFO_SIZE   (1024u)
#define REFEREE_DATA_STD_ID             (0x700)



#endif //ROBOT_INFO_H
