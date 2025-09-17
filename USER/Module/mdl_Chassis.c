#include "mdl_Chassis.h"
#include "mdl_comm.h"
#include "stm32f407xx.h"
#include "cmsis_os.h"
#include "arm_math.h"




ChassisHandle_t chassis_handle;

#define MAX_WHEEL_RPM   M3508_MAX_RPM

#define POWER_LIMIT         80.0f
#define WARNING_POWER       40.0f
#define WARNING_POWER_BUFF  50.0f
#define MAX_TOTAL_CURRENT_LIMIT         64000.0f    //16000 * 4,
#define BUFFER_TOTAL_CURRENT_LIMIT      16000.0f
#define POWER_TOTAL_CURRENT_LIMIT       20000.0f

void ChassisSensorUpdata(void)
{
    Comm_GimbalInfo_t* gimbal_info = GimbalInfo_Pointer();
    chassis_handle.gimbal_yaw_ecd_angle = gimbal_info->yaw_ecd_angle;
    chassis_handle.chassis_pitch = chassis_handle.imu->attitude.pitch - gimbal_info->pitch_gyro_angle;
    chassis_handle.chassis_roll = chassis_handle.imu->attitude.roll;
    chassis_handle.chassis_yaw = chassis_handle.imu->attitude.yaw - gimbal_info->yaw_gyro_angle;
}

void ChassisCtrlModeSwitch(void)
{
    if (chassis_handle.console->chassis_cmd == CHASSIS_RELEASE_CMD)
    {
        chassis_handle.ctrl_mode = CHASSIS_RELAX;
    }
    else if (chassis_handle.console->chassis_cmd == CHASSIS_STOP_CMD)
    {
        chassis_handle.ctrl_mode = CHASSIS_STOP;
    }
    else if (chassis_handle.console->chassis_cmd == CHASSIS_FOLLOW_GIMBAL_CMD)
    {
        chassis_handle.ctrl_mode = CHASSIS_FOLLOW_GIMBAL;
    }
    else if (chassis_handle.console->chassis_cmd == CHASSIS_SEPARATE_GIMBAL_CMD)
    {
        chassis_handle.ctrl_mode = CHASSIS_SEPARATE_GIMBAL;
    }
    else if (chassis_handle.console->chassis_cmd == CHASSIS_SPIN_CMD)
    {
        chassis_handle.ctrl_mode = CHASSIS_SPIN;
    }
    else if (chassis_handle.console->chassis_cmd == CHASSIS_ISLAND_CMD)
    {
        chassis_handle.ctrl_mode = CHASSIS_ISLAND;
    }
}

void ChassisMotorSendCurrent(int16_t motor1_cur, int16_t motor2_cur, int16_t motor3_cur, int16_t motor4_cur)
{
    Motor_SendMessage(chassis_handle.chassis_can, MOTOR_1TO4_CONTROL_STD_ID, motor1_cur, motor2_cur, motor3_cur, motor4_cur);
}



void ChassisStopMode(void)
{
    chassis_handle.vx = 0;
    chassis_handle.vy = 0;
    chassis_handle.vw = 0;
}

void ChassisFollowGimbalMode(void)
{
    chassis_handle.vx = chassis_handle.console->chassis.vx;
    chassis_handle.vy = chassis_handle.console->chassis.vy;
    chassis_handle.vw = pid_calc(&chassis_handle.chassis_follow_pid,
                                 -chassis_handle.gimbal_yaw_ecd_angle,
                                 0);
}

void ChassisSeparateGimbalMode(void)
{
    chassis_handle.vx = chassis_handle.console->chassis.vx;
    chassis_handle.vw = -chassis_handle.console->chassis.vw;
}

void ChassisSpinMode(void)
{
	if(chassis_handle.console->supercap_cmd == SUPERCAP_OFF_CMD)
	{
    chassis_handle.vx = chassis_handle.console->chassis.vx;
    chassis_handle.vy = chassis_handle.console->chassis.vy;
    chassis_handle.vw = 110;
		if(RefereeSystem_RobotState_Pointer()->chassis_power_limit<=55)
		{
			chassis_handle.vw = 100;
		}
		else if(RefereeSystem_RobotState_Pointer()->chassis_power_limit<=60)
		{
			chassis_handle.vw = 110;
		}
		else if(RefereeSystem_RobotState_Pointer()->chassis_power_limit<=65)
		{
			chassis_handle.vw = 120;
		}
		else if(RefereeSystem_RobotState_Pointer()->chassis_power_limit<=70)
		{
			chassis_handle.vw = 130;
		}
		else if(RefereeSystem_RobotState_Pointer()->chassis_power_limit<=75)
		{
			chassis_handle.vw = 140;
		}
		else if(RefereeSystem_RobotState_Pointer()->chassis_power_limit<=80)
		{
			chassis_handle.vw = 150;
		}
		else if(RefereeSystem_RobotState_Pointer()->chassis_power_limit<=85)
		{
			chassis_handle.vw = 160;
		}
		else if(RefereeSystem_RobotState_Pointer()->chassis_power_limit<=90)
		{
			chassis_handle.vw = 170;
		}
		else if(RefereeSystem_RobotState_Pointer()->chassis_power_limit<=100)
		{
			chassis_handle.vw = 190;
		}
		else if(RefereeSystem_RobotState_Pointer()->chassis_power_limit<=120)
		{
			chassis_handle.vw = 230;
		}
		//100 55w
		//110 60w
		//120 65w
		//130 70w
		//140 75w
		//150 80w
		//160 85w
		//170 90w
		//190 100w
		//230 120w
	}
	if(chassis_handle.console->supercap_cmd == SUPERCAP_ON_CMD)
	{
		chassis_handle.vx = chassis_handle.console->chassis.vx;
    chassis_handle.vy = chassis_handle.console->chassis.vy;
    chassis_handle.vw = 500;
	}
}

void Chassis_MoveTransform(ChassisHandle_t* chassis_handle, float* chassis_vx, float* chassis_vy)
{
    static float sin_yaw = 0.0f, cos_yaw = 0.0f;

    sin_yaw = arm_sin_f32(chassis_handle->gimbal_yaw_ecd_angle / RADIAN_COEF);
    cos_yaw = arm_cos_f32(chassis_handle->gimbal_yaw_ecd_angle / RADIAN_COEF);

    *chassis_vx = cos_yaw * chassis_handle->vx + sin_yaw * chassis_handle->vy;
    *chassis_vy =-sin_yaw * chassis_handle->vx + cos_yaw * chassis_handle->vy;

}

void Mecanum_Calculate(ChassisHandle_t* chassis_handle, float chassis_vx, float chassis_vy, float chassis_vw)
{
    static float rotate_ratio_fr;
    static float rotate_ratio_fl;
    static float rotate_ratio_bl;
    static float rotate_ratio_br;
    static float wheel_rpm_ratio;

    rotate_ratio_fl = ((chassis_handle->structure.wheelbase + chassis_handle->structure.wheeltrack)/2.0f \
            - chassis_handle->structure.rotate_x_offset - chassis_handle->structure.rotate_y_offset)/RADIAN_COEF;
    rotate_ratio_fr = ((chassis_handle->structure.wheelbase + chassis_handle->structure.wheeltrack)/2.0f \
            - chassis_handle->structure.rotate_x_offset + chassis_handle->structure.rotate_y_offset)/RADIAN_COEF;
    rotate_ratio_bl = ((chassis_handle->structure.wheelbase + chassis_handle->structure.wheeltrack)/2.0f \
            + chassis_handle->structure.rotate_x_offset - chassis_handle->structure.rotate_y_offset)/RADIAN_COEF;
    rotate_ratio_br = ((chassis_handle->structure.wheelbase + chassis_handle->structure.wheeltrack)/2.0f \
            + chassis_handle->structure.rotate_x_offset + chassis_handle->structure.rotate_y_offset)/RADIAN_COEF;

    wheel_rpm_ratio = 60.0f/(chassis_handle->structure.wheel_perimeter * M3508_REDUCTION_RATIO);

    VAL_LIMIT(chassis_vx, -MAX_CHASSIS_VX_SPEED, MAX_CHASSIS_VX_SPEED);  //mm/s
    VAL_LIMIT(chassis_vy, -MAX_CHASSIS_VY_SPEED, MAX_CHASSIS_VY_SPEED);  //mm/s
    VAL_LIMIT(chassis_vw, -MAX_CHASSIS_VW_SPEED, MAX_CHASSIS_VW_SPEED);  //deg/s

    float wheel_rpm[4];
    float max = 0;

    wheel_rpm[0] = ( chassis_vx + chassis_vy - chassis_vw * rotate_ratio_fl) * wheel_rpm_ratio;
    wheel_rpm[1] = (-chassis_vx + chassis_vy - chassis_vw * rotate_ratio_fr) * wheel_rpm_ratio;
    wheel_rpm[2] = ( chassis_vx - chassis_vy - chassis_vw * rotate_ratio_bl) * wheel_rpm_ratio;
    wheel_rpm[3] = (-chassis_vx - chassis_vy - chassis_vw * rotate_ratio_br) * wheel_rpm_ratio;

    //find max item
    for (uint8_t i = 0; i < 4; i++)
    {
        if (fabs(wheel_rpm[i]) > max)
        {
            max = fabs(wheel_rpm[i]);
        }
    }

    //equal proportion
    if (max > MAX_WHEEL_RPM)
    {
        float rate = MAX_WHEEL_RPM / max;
        for (uint8_t i = 0; i < 4; i++)
        {
            wheel_rpm[i] *= rate;
        }
    }
    memcpy(chassis_handle->wheel_rpm, wheel_rpm, 4 * sizeof(float));
}

void Chassis_ControlCalc(ChassisHandle_t* chassis_handle)
{
    static float chassis_vx = 0.0f, chassis_vy = 0.0f;

    Chassis_MoveTransform(chassis_handle, &chassis_vx, &chassis_vy);
    Mecanum_Calculate(chassis_handle, chassis_vx, chassis_vy, chassis_handle->vw);
}

void Chassis_LimitPower(ChassisHandle_t* chassis_handle)
{
    float total_current_limit = 0.0f;
    float total_current = 0.0f;
    float chassis_power = 0.0f;
    float chassis_power_buffer = 0.0f;
    float max_chassis_power = 0.0f;
    uint8_t robot_id = RefereeSystem_GetRobotID();

    if (robot_id == 0 || CheckDeviceIsOffline(OFFLINE_REFEREE_SYSTEM))
    {
        total_current_limit = 30000;
    }
    else if (robot_id == RED_ENGINEER || robot_id == BLUE_ENGINEER)
    {
        total_current_limit = MAX_TOTAL_CURRENT_LIMIT;
    }
    else
    {
        chassis_power = RefereeSystem_PowerHeatData_Pointer()->chassis_power;            //读取裁判系统的反馈的实时功率
       chassis_power_buffer = RefereeSystem_PowerHeatData_Pointer()->chassis_power_buffer;    //功率缓冲
        // chassis_power_buffer = 0;    //功率缓冲
			max_chassis_power = RefereeSystem_RobotState_Pointer()->chassis_power_limit;           //功率限制上限
        if(chassis_power_buffer < WARNING_POWER_BUFF)
        {
            float power_scale;
            if(chassis_power_buffer > 5.0f)
            {
                //scale down WARNING_POWER_BUFF
                power_scale = chassis_power_buffer / WARNING_POWER_BUFF;//功率缓冲剩50W时此系数为1
            }
            else
            {
                //only left 10% of WARNING_POWER_BUFF
                power_scale = 5.0f / WARNING_POWER_BUFF;
            }
            //scale down
            total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT * power_scale;//缓冲低于50W时功率缓冲
        }
        else
        {
            //power > WARNING_POWER
            if(chassis_power > WARNING_POWER)
            {

                float power_scale;
                //power < 80w
                if(chassis_power < max_chassis_power)
                {
                    //scale down
                    power_scale = (max_chassis_power - chassis_power) / (max_chassis_power - WARNING_POWER);

                }
                //power > 80w
                else
                {
                    power_scale = 0.0f;
                }

                total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT * power_scale;
            }
            //power < WARNING_POWER
            else
            {
                total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT;
            }
        }
    }

    //calculate the original motor current set
    for(uint8_t i = 0; i < 4; i++)
    {
        total_current += fabs(chassis_handle->chassis_motor[i].current_set);
    }


    if(total_current > total_current_limit)
    {
        float current_scale = total_current_limit / total_current;
        chassis_handle->chassis_motor[0].current_set *= current_scale;
        chassis_handle->chassis_motor[1].current_set *= current_scale;
        chassis_handle->chassis_motor[2].current_set *= current_scale;
        chassis_handle->chassis_motor[3].current_set *= current_scale;
    }
}

/*--------------------------------------初始化----------------------------------------*/
ADRC adrc_3508[4];
ChassisHandle_t chassis_handle;
static TransmitHandle_t chassis_tx_handle;
static uint8_t chassis_tx_fifo_buffer[GIMBAL_CHASSIS_DATA_FIFO_SIZE];
static ReceiveHandle_t chassis_rx_handle;
static uint8_t chassis_rx_fifo_buffer[GIMBAL_CHASSIS_DATA_FIFO_SIZE];
static TransmitHandle_t referee_tx_handle;
static uint8_t referee_tx_fifo_buffer[REFEREE_SYSTEM_FIFO_SIZE];
static ReceiveHandle_t referee_rx_handle;
static uint8_t referee_rx_fifo_buffer[REFEREE_SYSTEM_FIFO_SIZE];



static void CAN1_UploadDataHook(uint8_t *data, uint16_t len)
{
    BSP_CAN_TransmitData(&can1_obj, CHASSIS_TX_DATA_STD_ID, data, len);
}

static void CAN1_RefereeDataHook(uint8_t *data, uint16_t len)
{
    BSP_CAN_TransmitData(&can1_obj, REFEREE_DATA_STD_ID, data, len);
}

static int32_t Transmit_RefereeData(void *argc)        //裁判系统数据
{
    if (CheckDeviceIsOffline(OFFLINE_REFEREE_SYSTEM))
        return 0;
		ext_game_robot_state_t* robot_state = RefereeSystem_RobotState_Pointer();
		Comm_TransmitData(&referee_tx_handle, REFEREE_SYSTEM_HEADER_SOF, GAME_ROBOT_STATE_CMD_ID, (uint8_t*)robot_state, sizeof(ext_game_robot_state_t));
		ext_power_heat_data_t* power_heat_data = RefereeSystem_PowerHeatData_Pointer();
		Comm_TransmitData(&referee_tx_handle, REFEREE_SYSTEM_HEADER_SOF, POWER_HEAT_DATA_CMD_ID, (uint8_t*)power_heat_data, sizeof(ext_power_heat_data_t));
    return 0;
}

static void DBUS_ReceiveCallback(uint8_t* data, uint16_t len)
{
    RC_DataParser(RC_GetDataPointer(), data, len);
    Comm_TransmitData(&chassis_tx_handle, USER_PROTOCOL_HEADER_SOF, RC_DATA_CMD_ID, data, len);
    OfflineHandle_TimeUpdate(OFFLINE_DBUS);
}

static void COM1_ReceiveCallback(uint8_t* data, uint16_t len)
{
    Comm_ReceiveData(&referee_rx_handle, data, len);
    OfflineHandle_TimeUpdate(OFFLINE_REFEREE_SYSTEM);
}

static void COM2_ReceiveCallback(uint8_t* data, uint16_t len)
{

}

static void CAN1_ReceiveCallback(uint32_t std_id, uint8_t *data, uint32_t dlc)
{
    switch (std_id)
    {
        case CHASSIS_RX_DATA_STD_ID:
        {
            Comm_ReceiveData(&chassis_rx_handle, data, dlc);
        }break;
	        case Super_Power_Message_ID:
        {
            CAP_PowerParser(ZFCAP_GetDataPointer(), data, dlc);//浙纺
        }break;
				case LONGSuper_Power_Message_ID:
				{
						LONG_CAP_PowerParser(&cap_info,data,dlc);//龙JG
				}break;
//				 case LONGSuper_Power_Message_ID:
//        {
//            Comm_ReceiveData(&chassis_rx_handle, data, dlc);
//        }break;
        default:
            break;
    }
}

static void CAN2_ReceiveCallback(uint32_t std_id, uint8_t *data, uint32_t dlc)
{
    switch (std_id)
    {
        case CHASSIS_MOTOR_LF_MESSAGE_ID:
        case CHASSIS_MOTOR_RF_MESSAGE_ID:
        case CHASSIS_MOTOR_LB_MESSAGE_ID:
        case CHASSIS_MOTOR_RB_MESSAGE_ID:
        {
            uint8_t i = std_id - CHASSIS_MOTOR_LF_MESSAGE_ID;
			Motor_DataParse(chassis_handle.chassis_motor[i].motor_info, DJ_MOTOR, data, &chassis_djmotor[i],NULL);
            OfflineHandle_TimeUpdate(OFFLINE_CHASSIS_MOTOR1+i);
        }break;
        case PLUCK_MOTOR_MESSAGE_ID:
        {
			Motor_DataParse(PluckMotor_Pointer(), DJ_MOTOR, data, &pluck_djmotor,NULL);
            OfflineHandle_TimeUpdate(OFFLINE_PLUCK_MOTOR);
        }break;
        case COVER_MOTOR_MESSAGE_ID:
        {
			Motor_DataParse(MagazineMotor_Pointer(), DJ_MOTOR, data, &pluck_djmotor,NULL);
            OfflineHandle_TimeUpdate(OFFLINE_COVER_MOTOR);
        }break;
//        case SENSOR_MESSAGE_ID:
//        {
//            island_handle.sensor_f2_state  =  data[0];
//            island_handle.sensor_switch1_state  =  data[1];
//            island_handle.sensor_b_state  =  data[2];
//            island_handle.sensor_f1_state=data[3];
//        }break;

        default:
            break;
    }
}

void ChassisAppConfig(void)
{
    chassis_handle.console      = Console_Pointer();
    chassis_handle.imu          = IMU_GetDataPointer();
    chassis_handle.chassis_can  = &can2_obj;
    chassis_handle.ctrl_mode  = CHASSIS_RELAX;
    chassis_handle.structure.wheel_perimeter = WHEEL_PERIMETER;
    chassis_handle.structure.wheeltrack = WHEELTRACK;
    chassis_handle.structure.wheelbase = WHEELBASE;
    chassis_handle.structure.rotate_x_offset = GIMBAL_X_OFFSET;
    chassis_handle.structure.rotate_y_offset = GIMBAL_Y_OFFSET;
    chassis_handle.SuperPower_State     =0;



    for (uint8_t i=0; i<4; i++)
    {
        chassis_handle.chassis_motor[i].motor_info = ChassisMotor_Pointer(i);
        pid_init(&chassis_handle.chassis_motor[i].pid, POSITION_PID, M3508_MOTOR_MAX_CURRENT, 2000.0f,
                 6.5f, 0.0f, 0.0f);
        adrc_3508[i]=zepi_create_ADRC();
        _adrc_reset_allParam(adrc_3508+i,Default_3508_speed);
    }
    pid_init(&chassis_handle.chassis_follow_pid, POSITION_PID, 300.0f, 50.0f,
             7.0f, 0.0f, 2.0f);//22.2  5.7//24.3 12.5/25.6 12.5


    /*--------------------event-----------------|-------enable-------|-offline time-|-beep_times-*/
     OfflineHandle_Init(OFFLINE_CHASSIS_MOTOR1,  OFFLINE_ERROR_LEVEL,       100,         1);
     OfflineHandle_Init(OFFLINE_CHASSIS_MOTOR2,  OFFLINE_ERROR_LEVEL,       100,         2);
     OfflineHandle_Init(OFFLINE_CHASSIS_MOTOR3,  OFFLINE_ERROR_LEVEL,       100,         3);
     OfflineHandle_Init(OFFLINE_CHASSIS_MOTOR4,  OFFLINE_ERROR_LEVEL,       100,         4);
     OfflineHandle_Init(OFFLINE_PLUCK_MOTOR,     OFFLINE_ERROR_LEVEL,       100,         7);        //三声，云台P轴
     //OfflineHandle_Init(OFFLINE_SUPERPOWER,      OFFLINE_ERROR_LEVEL,       100,         5);  //
     OfflineHandle_Init(OFFLINE_REFEREE_SYSTEM,  OFFLINE_WARNING_LEVEL,     100,         1);  //
     OfflineHandle_Init(OFFLINE_GIMBAL_INFO,     OFFLINE_WARNING_LEVEL,     100,         3);  //
     OfflineHandle_Init(OFFLINE_DBUS,            OFFLINE_WARNING_LEVEL,     100,         0);  //

    Comm_TransmitInit(&chassis_tx_handle, chassis_tx_fifo_buffer, GIMBAL_CHASSIS_DATA_FIFO_SIZE, CAN1_UploadDataHook);
    Comm_ReceiveInit(&chassis_rx_handle, USER_PROTOCOL_HEADER_SOF, chassis_rx_fifo_buffer, GIMBAL_CHASSIS_DATA_FIFO_SIZE, UserProtocol_ParseHandler);

    Comm_TransmitInit(&referee_tx_handle, referee_tx_fifo_buffer, REFEREE_SYSTEM_FIFO_SIZE, CAN1_RefereeDataHook);
    Comm_ReceiveInit(&referee_rx_handle, REFEREE_SYSTEM_HEADER_SOF, referee_rx_fifo_buffer, REFEREE_SYSTEM_FIFO_SIZE, RefereeSystem_ParseHandler);
    SoftwareTimerRegister(Transmit_RefereeData, (void*)NULL, 5);

    BSP_UART_SetRxCallback(&dbus_obj, DBUS_ReceiveCallback);
    BSP_UART_SetRxCallback(&com1_obj, COM1_ReceiveCallback);
    BSP_UART_SetRxCallback(&com2_obj, COM2_ReceiveCallback);
    BSP_CAN_SetRxCallback(&can1_obj, CAN1_ReceiveCallback);
    BSP_CAN_SetRxCallback(&can2_obj, CAN2_ReceiveCallback);
}