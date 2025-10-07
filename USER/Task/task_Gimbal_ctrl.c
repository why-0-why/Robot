#include "task_Gimbal_ctrl.h"
#include "cmsis_os.h"

#include "alg_crc.h"//CRC校验函数
#include "alg_AHRS.h"//转换单位函数

#include "drv_timer.h"// 获取时间函数

#include "dvc_Referee_system.h"// 裁判系统函数
#include "task_Detect.h"// 离线检测函数
#include "task_SoftwareTimer_Check.h"//
#include <user_lib.h>//数学函数
#include "robot_info.h"//机器人参数
#include "mdl_comm.h"//通讯数据类型函数

GimbalHandle_t gimbal_handle;

/* 任务 */
osThreadId GimbalTaskHandle;

#if INCLUDE_uxTaskGetStackHighWaterMark
static uint32_t gimbal_task_stack = 0;
#endif



void GimbalTask(void const *argument)
{
    for(;;)
    {
        GimbalSensorUpdata();
		VisionDataUpdate(); 	//处理视觉数据
        GimbalCtrlModeSwitch();
        GimbalRampInit();
        GimbalRampUpdate();
        switch (gimbal_handle.ctrl_mode)
        {
            case GIMBAL_INIT:
            {
                GimbalInitMode();           //云台初始化
            }break;

            case GIMBAL_GYRO:
            {
                GimbalGyroAngleMode();     //云台陀螺仪模式
            }break;

            case GIMBAL_RELATIVE:           //云台分离模式
            {
                GimbalRelativeAngleMode();
            }break;

            case GIMBAL_NORMAL:             //云台跟随模式
            {
                GimbalNormalMode();
            }break;
            case GIMBAL_VISION:         //云台视觉跟踪模式
            {
                GimbalVisionAimMode();
            }break;
            default:
                break;
        }

//				 GimbalVisionAimMode();
        GimbalMotorControl(&gimbal_handle.yaw_motor);
        GimbalMotorControl(&gimbal_handle.pitch_motor);

        if (gimbal_handle.ctrl_mode == GIMBAL_RELAX)
        {
            pid_clear(&gimbal_handle.yaw_motor.pid.outer_pid);
            pid_clear(&gimbal_handle.yaw_motor.pid.inter_pid);
            pid_clear(&gimbal_handle.pitch_motor.pid.outer_pid);
            pid_clear(&gimbal_handle.pitch_motor.pid.inter_pid);
            gimbal_handle.yaw_motor.current_set = 0;
            gimbal_handle.pitch_motor.current_set = 0;
            lock_motor(gimbal_handle.gimbal_can,CAN_ID);

        }

        GimbalMotorSendCurrent((int16_t)YAW_MOTO_POSITIVE_DIR * gimbal_handle.yaw_motor.current_set,
                               (int16_t)PITCH_MOTO_POSITIVE_DIR * gimbal_handle.pitch_motor.current_set);
//        GimbalMotorSendCurrent((int16_t)YAW_MOTO_POSITIVE_DIR * gimbal_handle.yaw_motor.current_set,
//                               (int16_t)PITCH_MOTO_POSITIVE_DIR * gimbal_handle.pitch_motor.current_set);

				float pitch_pos = gimbal_handle.pitch_motor.motor_info->given_value;
        float pitch_vel=calculate_derivative(gimbal_handle.pitch_motor.motor_info->given_value,BSP_GetTime_ms());
        if(gimbal_handle.ctrl_mode == GIMBAL_INIT)  //初始化模式时对pitch进行限速
				{
					VAL_LIMIT(pitch_vel,0,0.02);
					VAL_LIMIT(pitch_pos,gimbal_dmmotor_pitch.pos-0.05,gimbal_dmmotor_pitch.pos+0.05);
				}
        GimbalMotorSendPos(pitch_pos,
                           pitch_vel);//后续需要添加达妙信息结构体
        osDelay(GIMBAL_CTRL_TASK_PERIOD);

#if INCLUDE_uxTaskGetStackHighWaterMark
        gimbal_task_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

void GimbalTaskInit(void)
{
    ramp_v0_init(&yaw_ramp, BACK_CENTER_TIME/GIMBAL_CTRL_TASK_PERIOD);
    ramp_v0_init(&pitch_ramp, BACK_CENTER_TIME/GIMBAL_CTRL_TASK_PERIOD);
	gimbal_handle.last_ctrl_mode = GIMBAL_ZARO;											  //无意义，仅用于代指未上电时的上一控制模式
//	  ESO_init();

    osThreadDef(gimbal_task, GimbalTask, osPriorityNormal, 0, 256);
    GimbalTaskHandle = osThreadCreate(osThread(gimbal_task), NULL);
}

/* 私有变量 ------------------------------------------------------------------*/
GimbalHandle_t gimbal_handle; //云台处理结构体数值

extern Console_t console;

static TransmitHandle_t gimbal_tx_handle;
static uint8_t gimbal_tx_fifo_buffer[GIMBAL_CHASSIS_DATA_FIFO_SIZE];
static ReceiveHandle_t gimbal_rx_handle;
static uint8_t gimbal_rx_fifo_buffer[GIMBAL_CHASSIS_DATA_FIFO_SIZE];

static TransmitHandle_t vision_tx_handle;                          //视觉发送处理结构体
static uint8_t vision_tx_fifo_buffer[VISION_DATA_FIFO_SIZE];
static ReceiveHandle_t vision_rx_handle;
static uint8_t vision_rx_fifo_buffer[VISION_DATA_FIFO_SIZE];

static ReceiveHandle_t referee_rx_handle;
static uint8_t referee_rx_fifo_buffer[REFEREE_SYSTEM_FIFO_SIZE];

/* 扩展变量 ------------------------------------------------------------------*/
uint8_t rtt[256];
extern pid_t yaw_vision_pid;
extern pid_t pitch_vision_pid;
extern float attitude_yaw_initial;
extern AutoAim_t yaw_aim;
extern VisionDatabase_t vision_data;
/* 私有函数原形 --------------------------------------------------------------*/
static void CAN1_UploadDataHook(uint8_t *data, uint16_t len);
static int32_t GimbalInfoUploadCallback(void *argc);
static void DBUS_ReceiveCallback(uint8_t* data, uint16_t len);
static void COM1_ReceiveCallback(uint8_t* data, uint16_t len);
static void COM2_ReceiveCallback(uint8_t* data, uint16_t len);
static void CAN1_ReceiveCallback(uint32_t std_id, uint8_t *data, uint32_t dlc);
static void CAN2_ReceiveCallback(uint32_t std_id, uint8_t *data, uint32_t dlc);
static void Vision_UploadDataHook(uint8_t *data, uint16_t len);
static void Vision_UploadDataHook(uint8_t *data, uint16_t len);
static int32_t Vision_RobotInfoUploadCallback(void *argc);
static void Aim_init(AutoAim_t* Aim);
/* 函数体 --------------------------------------------------------------------*/
void GimbalAppConfig(void)
{
    gimbal_handle.console     = Console_Pointer();                   //云台控制数据更新
    gimbal_handle.imu  = IMU_GetDataPointer();                       //陀螺仪数据更新
    gimbal_handle.gimbal_can  = &can1_obj;                           //云台CAN1通道
    gimbal_handle.ctrl_mode = GIMBAL_INIT;                            //云台控制模式为初始化

    DJMotor_yaw_Init(&gimbal_djmotor_yaw);
//        DJMotor_yaw_Init(&gimbal_djmotor_pitch);不启用大疆电机作为yaw
    DMMotor_Init(&gimbal_dmmotor_pitch);//启用达妙电机作为pitch

    gimbal_handle.yaw_motor.motor_info = GimbalMotorYaw_Pointer(); //大疆Y轴电机信息更新
    gimbal_handle.pitch_motor.motor_info = GimbalMotorPitch_Pointer();//P轴电机信息更新


    attitude_yaw_initial = gimbal_handle.imu->attitude.yaw;
    Aim_init(&yaw_aim);
    pid_init(&gimbal_handle.yaw_motor.pid.outer_pid, POSITION_PID, 2000.0f, 0.0f,                //Y轴外环PID
             23.0f, 0.0f, 40.0f);  //30    /*40,0.0,3.5*///30 0 5.5 30 0.15 15
    pid_init(&gimbal_handle.yaw_motor.pid.inter_pid, POSITION_PID, GM6020_MOTOR_MAX_CURRENT, 3000.0f,      //Y轴内环PID
             200.0f, 0.00f, 200.0f);  //60    /*60,0.1,12*/ //75 0.03 35  70 0.2 30         //P调大越硬
    pid_init(&gimbal_handle.pitch_motor.pid.outer_pid, POSITION_PID, 20000.0f, 0.0f,                       //P轴外环PID
             50.0f, 0.0f, 10.0f);          /*40,0,98有摄像头*/   /*40,0,90无摄像头*/     /*60,0,10 */  //10 80
    pid_init(&gimbal_handle.pitch_motor.pid.inter_pid, POSITION_PID, GM6020_MOTOR_MAX_CURRENT, 6000.0f,    //P轴内环PID
             200.0f, 0.0f, 100.0f);         /*38,0.1,0有摄像头*/    /*38,0.1,0无摄像头*/    /*90,0.1,30*/     //75 0.0 0.0  //30 10

    pid_init(&yaw_vision_pid,POSITION_PID, 200.0f, 200.0f,0.019, 0, 0.1); 	//d0.4
    pid_init(&pitch_vision_pid, POSITION_PID, 200.0f, 200.0f,0.0025, 0, 0.1);


 //                     离线事件                                 离线返回码                            叫声次数
    /*--------------------event------------------------|-------enable-------|-offline time-|-beep_times-*/           //OfflineHandle离线处理
    OfflineHandle_Init(OFFLINE_GIMBAL_PITCH,            OFFLINE_ERROR_LEVEL,       100,         5);        //云台P轴
    OfflineHandle_Init(OFFLINE_GIMBAL_YAW,              OFFLINE_ERROR_LEVEL,       100,         4);        //云台Y轴
    OfflineHandle_Init(OFFLINE_FRICTION_WHEEL_MOTOR1,   OFFLINE_ERROR_LEVEL,       100,         1);        //一声，左摩擦轮
    OfflineHandle_Init(OFFLINE_FRICTION_WHEEL_MOTOR2,   OFFLINE_ERROR_LEVEL,       100,         2);        //两声，右摩擦轮
    // OfflineHandle_Init(OFFLINE_PLUCK_MOTOR,             OFFLINE_ERROR_LEVEL,       100,         0);        //拨弹轮
    // OfflineHandle_Init(OFFLINE_COVER_MOTOR,             OFFLINE_ERROR_LEVEL,       100,         0);        //
    // OfflineHandle_Init(OFFLINE_REFEREE_SYSTEM,          OFFLINE_WARNING_LEVEL,     100,         0);  //
    // OfflineHandle_Init(OFFLINE_CHASSIS_INFO,            OFFLINE_WARNING_LEVEL,     100,         0);  //
		//OfflineHandle_Init(OFFLINE_INFRARED_INFO,           OFFLINE_WARNING_LEVEL,     100,         0);
    OfflineHandle_Init(OFFLINE_DBUS,                    OFFLINE_WARNING_LEVEL,     100,         0);  //
    // OfflineHandle_Init(OFFLINE_VISION_INFO,             OFFLINE_WARNING_LEVEL,     500,         0);
    // OfflineHandle_Init(OFFLINE_KEYBOARDINFO,            OFFLINE_WARNING_LEVEL,     100,         6);        //图传链路键鼠数据

    Comm_TransmitInit(&gimbal_tx_handle, gimbal_tx_fifo_buffer, GIMBAL_CHASSIS_DATA_FIFO_SIZE, CAN1_UploadDataHook);
    Comm_ReceiveInit(&gimbal_rx_handle, USER_PROTOCOL_HEADER_SOF, gimbal_rx_fifo_buffer, GIMBAL_CHASSIS_DATA_FIFO_SIZE, UserProtocol_ParseHandler);
    SoftwareTimerRegister(GimbalInfoUploadCallback, (void*)NULL, GIMBAL_UPLOAD_TIMER_PERIOD);

    Comm_TransmitInit(&vision_tx_handle, vision_tx_fifo_buffer, VISION_DATA_FIFO_SIZE, Vision_UploadDataHook);                                     //视觉发送：视觉发送地址，视觉接受信息缓冲区，视觉数据信息大小，视觉数据上载。
    Comm_ReceiveInit(&vision_rx_handle, VISION_PROTOCOL_HEADER_SOF, vision_rx_fifo_buffer, VISION_DATA_FIFO_SIZE, VisionProtocol_ParseHandler);    //视觉接收: 视觉接受地址，视觉协议帧头起始，视觉接受信息缓冲区，视觉数据信息大小，视觉协议、
    SoftwareTimerRegister(Vision_RobotInfoUploadCallback, (void*)NULL, 5);                                            //软时钟寄存器：视觉信息回调

    Comm_ReceiveInit(&referee_rx_handle, REFEREE_SYSTEM_HEADER_SOF, referee_rx_fifo_buffer, REFEREE_SYSTEM_FIFO_SIZE, RefereeSystem_ParseHandler);

    BSP_UART_SetRxCallback(&dbus_obj, DBUS_ReceiveCallback);
    BSP_UART_SetRxCallback(&com1_obj, COM1_ReceiveCallback);
    BSP_UART_SetRxCallback(&com2_obj, COM2_ReceiveCallback);
    BSP_CAN_SetRxCallback(&can1_obj, CAN1_ReceiveCallback);
    BSP_CAN_SetRxCallback(&can2_obj, CAN2_ReceiveCallback);
}

static void CAN1_UploadDataHook(uint8_t *data, uint16_t len)
{
    BSP_CAN_TransmitData(&can1_obj, GIMBAL_TX_DATA_STD_ID, data, len);
}

static int32_t GimbalInfoUploadCallback(void *argc)
{
    Comm_GimbalInfo_t* info = GimbalInfo_Pointer();
    info->mode = gimbal_handle.ctrl_mode;
    info->pitch_ecd_angle   = gimbal_handle.pitch_motor.sensor.relative_angle;
    info->yaw_ecd_angle     = gimbal_handle.yaw_motor.sensor.relative_angle;
    info->pitch_gyro_angle  = gimbal_handle.pitch_motor.sensor.gyro_angle;
    info->yaw_gyro_angle    = gimbal_handle.yaw_motor.sensor.gyro_angle;
    info->pitch_rate        = gimbal_handle.pitch_motor.sensor.palstance;
    info->can_shoot         = vision_data.can_shoot;
 //   info->yaw_rate          = gimbal_handle.yaw_motor.sensor.palstance;
	  info->infrared          = gimbal_handle.infrared;
    Comm_TransmitData(&gimbal_tx_handle, USER_PROTOCOL_HEADER_SOF, GIMBAL_INFO_CMD_ID, (uint8_t*)info, sizeof(Comm_GimbalInfo_t));
    return 0;
}

static void DBUS_ReceiveCallback(uint8_t* data, uint16_t len)
{
    RC_DataParser(RC_GetDataPointer(), data, len);
    Comm_TransmitData(&gimbal_tx_handle, USER_PROTOCOL_HEADER_SOF, RC_DATA_CMD_ID, data, len);
    OfflineHandle_TimeUpdate(OFFLINE_DBUS);
}

static void Vision_UploadDataHook(uint8_t *data, uint16_t len)    //视觉上载数据
{
    BSP_UART_TransmitData(&com1_obj, data, len);                  //异步发送
}

static int32_t Vision_RobotInfoUploadCallback(void *argc)
{
    Comm_RobotInfo_t* info = RobotInfo_Pointer();
    uint16_t robot_id = RefereeSystem_GetRobotID();
    Console_t* console_info = Console_Pointer();
    info->data_head = 0xAA;  //
    if (robot_id > 100)     //ID大于100是蓝方  应该打红方；
    {
       info->enemy_color = Red;
    }
    else if (robot_id >= 1)
    {
        info->enemy_color = Blue;
    }
    else
    {
       info->enemy_color = AllColor;  //
    }

      info->yaw_relative_angle = (gimbal_handle.imu->attitude.yaw) * ANGLE_TO_RAD; //
      info->pitch_relative_angle = (gimbal_handle.imu->attitude.pitch) * ANGLE_TO_RAD;  //
//		  info->yaw_palstance = gimbal_handle.yaw_motor.sensor.palstance;
		  info->robot_level = RefereeSystem_RobotState_Pointer()->robot_level;
      //info->bullet_speed = RefereeSystem_SpeedData_Pointer()->initial_speed;
		append_crc16_check_sum((uint8_t*)info, sizeof(Comm_RobotInfo_t) + 2);  //添加CRC16校验位
//    Comm_TransmitData_Vision(&vision_tx_handle, (uint8_t*)info, sizeof(Comm_RobotInfo_t));
	  HAL_UART_Transmit_DMA(&huart6,(uint8_t*)info,sizeof(Comm_RobotInfo_t) + 2);
}

static void COM1_ReceiveCallback(uint8_t* data, uint16_t len)//
{
    Comm_ReceiveData(&vision_rx_handle, data, len);
}

static void COM2_ReceiveCallback(uint8_t* data, uint16_t len) {
    if (data == NULL || len < sizeof(uint16_t)) {
        // 处理错误: 数据为空或长度不足以包含命令ID
        return;
    }

    // 假设命令ID在数据开始的两个字节
    uint16_t cmd_id = (uint16_t)(data[0] | data[1] << 8);

    // 从data+2开始传递实际数据，因为前两个字节用于cmd_id，len-2为实际数据长度
    RefereeSystem_ParseHandler(cmd_id, data + 2, len - 2);
    OfflineHandle_TimeUpdate(OFFLINE_KEYBOARDINFO);
}

static void CAN1_ReceiveCallback(uint32_t std_id, uint8_t *data, uint32_t dlc)
{
    switch (std_id)
    {

        //yaw电机数据
        case GIMBAL_MOTOR_YAW_MESSAGE_ID://采用大疆GM6020的CAN协议
        {
            Motor_DataParse(gimbal_handle.yaw_motor.motor_info, DJ_MOTOR, data, &gimbal_djmotor_yaw,NULL);
            //添加电机总接口信息更新
            OfflineHandle_TimeUpdate(OFFLINE_GIMBAL_YAW);
        }break;
       case Master_ID: //采用达妙DM4310的CAN协议
       {
           Motor_DataParse(gimbal_handle.pitch_motor.motor_info, DM_MOTOR, data, NULL,&gimbal_dmmotor_pitch);
            //添加电机总接口信息更新
           OfflineHandle_TimeUpdate(OFFLINE_GIMBAL_PITCH);
       }break;
        //pitch电机数据
//        case GIMBAL_MOTOR_PITCH_MESSAGE_ID:
//        {
//            Motor_DataParse(gimbal_handle.pitch_motor.motor_info, DJ_MOTOR, data, &gimbal_djmotor_pitch,NULL);
//            //添加电机总接口信息更新
//            OfflineHandle_TimeUpdate(OFFLINE_GIMBAL_PITCH);
//        }break;
        //底盘数据
        case GIMBAL_RX_DATA_STD_ID:
        {
            Comm_ReceiveData(&gimbal_rx_handle, data, dlc);
            OfflineHandle_TimeUpdate(OFFLINE_CHASSIS_INFO);
        }break;
        //裁判系统数据
        case REFEREE_DATA_STD_ID:
        {
            Comm_ReceiveData(&referee_rx_handle, data, dlc);
            OfflineHandle_TimeUpdate(OFFLINE_REFEREE_SYSTEM);
        }break;
        default:
            break;
    }
}

static void CAN2_ReceiveCallback(uint32_t std_id, uint8_t *data, uint32_t dlc)
{
    switch (std_id)
    {
        case FRICTION_WHEEL_1_MESSAGE_ID:
        {
            Motor_DataParse(FrictionWheelMotor_1_Pointer(), DJ_MOTOR, data, &friction_wheel_djmotor[0],NULL);
            OfflineHandle_TimeUpdate(OFFLINE_FRICTION_WHEEL_MOTOR1);
        }break;
        case FRICTION_WHEEL_2_MESSAGE_ID:
        {
            Motor_DataParse(FrictionWheelMotor_2_Pointer(), DJ_MOTOR, data, &friction_wheel_djmotor[1],NULL);
            OfflineHandle_TimeUpdate(OFFLINE_FRICTION_WHEEL_MOTOR2);
        }break;
        default:
            break;
    }
}

static void Aim_init(AutoAim_t* Aim)
{
	Aim->pid_SAtime=0;
  Aim->aiming_time=0;
  Aim->stay_time=0;
  Aim->systeam_time=0;
  Aim->tol_angle = 1.0f;  //人为规定消抖角度范围(-tol_angle,tol_angle)
  Aim->tol_time = 700;//the aiming-done continueous time that user set 人为规定
  Aim->Sstart_time=Aim->Astart_time=0;//消抖持续时间,单次自瞄持续时间
  Aim->first_aim = 0;//初入消抖范围标志
  Aim->aim_flag = 0; //新的单次自瞄标志
	Aim->Ap_parm = 3500;//Sp_parm = 5000;//user set
  Aim->Sp_parm = 0.03;
  Aim->enable_paramSA= 1;
}