
#include "alg_AHRS.h"
#include "alg_ESO.h"
#include "alg_ramp.h"

#include "mdl_Gimbal.h"
#include "task_Gimbal_ctrl.h"
#include "task_Detect.h"

#include "robot_info.h"
#include "user_lib.h"// 一些数学函数

ramp_v0_t yaw_ramp = RAMP_GEN_DAFAULT;
ramp_v0_t pitch_ramp = RAMP_GEN_DAFAULT;

pid_t yaw_vision_pid;
pid_t pitch_vision_pid;
float attitude_yaw_initial;
AutoAim_t yaw_aim;
extern VisionDatabase_t vision_data;

#define BACK_CENTER_TIME 2500
#define KALMAN_FILTER_ANGLE 0
#define KALMAN_FILTER_SPEED 1
#define KALMAN_FILTER_ACCEL 2

#define vision_comps_yaw 0
#define vision_comps_pitch 1

void GimbalSensorUpdata(void)
{
    gimbal_handle.yaw_motor.sensor.relative_angle =  gimbal_handle.yaw_motor.motor_info->relative_angle;
    gimbal_handle.pitch_motor.sensor.relative_angle =  gimbal_handle.pitch_motor.motor_info->relative_angle;

    gimbal_handle.pitch_motor.min_relative_angle = gimbal_handle.pitch_motor.motor_info->min_relative_angle;
    gimbal_handle.pitch_motor.max_relative_angle = gimbal_handle.pitch_motor.motor_info->max_relative_angle;

    gimbal_handle.yaw_motor.sensor.gyro_angle = gimbal_handle.imu->attitude.yaw;
    gimbal_handle.pitch_motor.sensor.gyro_angle = gimbal_handle.imu->attitude.pitch;
    gimbal_handle.yaw_motor.sensor.palstance = gimbal_handle.imu->gyro[2] * RAD_TO_ANGLE;
    gimbal_handle.pitch_motor.sensor.palstance = gimbal_handle.imu->gyro[1] * RAD_TO_ANGLE;

}

void GimbalCtrlModeSwitch(void)                                //云台控制模式选择
{
		gimbal_handle.last_moment_ctrl_mode = gimbal_handle.ctrl_mode;
		GimbalCtrlMode_e ctrl_mode = GIMBAL_RELAX;
		if (gimbal_handle.ctrl_mode != ctrl_mode)
		{
				gimbal_handle.last_ctrl_mode = ctrl_mode;
				ctrl_mode = gimbal_handle.ctrl_mode;
		}
    if (gimbal_handle.console->gimbal_cmd == GIMBAL_RELEASE_CMD)        //gimbal_cmd控制ctrl_mode
    {
        gimbal_handle.ctrl_mode = GIMBAL_RELAX;
    }
    else if (gimbal_handle.console->gimbal_cmd == GIMBAL_INIT_CMD)
    {
        gimbal_handle.ctrl_mode = GIMBAL_INIT;
    }
    else if (gimbal_handle.console->gimbal_cmd == GIMBAL_GYRO_CMD)
    {
        gimbal_handle.ctrl_mode = GIMBAL_GYRO;
    }
    else if (gimbal_handle.console->gimbal_cmd == GIMBAL_RELATIVE_CMD)
    {
        gimbal_handle.ctrl_mode = GIMBAL_RELATIVE;
    }
    else if (gimbal_handle.console->gimbal_cmd == GIMBAL_NORMAL_CMD)
    {
        gimbal_handle.ctrl_mode = GIMBAL_NORMAL;
    }
    else if (gimbal_handle.console->gimbal_cmd == GIMBAL_VISION_CMD)
    {
        gimbal_handle.ctrl_mode = GIMBAL_VISION;
    }
}

void GimbalRampInit()																							//判断是否为遥控器重新上电，是则初始化云台斜坡函数
{
    if(CheckDeviceIsOffline(OFFLINE_DBUS))
    {
        ramp_v0_init(&yaw_ramp, BACK_CENTER_TIME/GIMBAL_CTRL_TASK_PERIOD );
        ramp_v0_init(&pitch_ramp, BACK_CENTER_TIME/GIMBAL_CTRL_TASK_PERIOD );
    }
}

void GimbalRampUpdate()
{
		gimbal_handle.console->gimbal.pitch_ramp = pitch_ramp;
		gimbal_handle.console->gimbal.yaw_ramp = yaw_ramp;
}

void GimbalMotorSendCurrent(int16_t yaw_cur, int16_t pitch_cur)
{
    Motor_SendMessage(gimbal_handle.gimbal_can, GIMBAL_MOTOR_CONTROL_STD_ID, pitch_cur, yaw_cur, 0, 0);
}

void GimbalMotorSendPos(float _pos, float _vel)
{
		Get_MIT_Pointer()->p_int = _pos;
		Get_MIT_Pointer()->v_int = _vel;
		Get_MIT_Pointer()->t_int = -0.0001164*powf(gimbal_motor_pitch.relative_angle,2) + 0.02533*gimbal_motor_pitch.relative_angle+ 0.4923;	          //前馈扭矩
	  ctrl_motor(gimbal_handle.gimbal_can,CAN_ID,Get_MIT_Pointer());
}

void GimbalInitMode(void)
{
    gimbal_handle.yaw_motor.mode = ENCONDE_MODE;
    gimbal_handle.pitch_motor.mode = ENCONDE_MODE;

    gimbal_handle.yaw_motor.given_value = gimbal_handle.yaw_motor.sensor.relative_angle;
    gimbal_handle.pitch_motor.given_value = gimbal_handle.pitch_motor.sensor.relative_angle * (1 - ramp_v0_calculate(&pitch_ramp));
    if (fabsf(gimbal_handle.pitch_motor.sensor.relative_angle) <= 5.0f)
    {
				gimbal_handle.pitch_motor.given_value = 0;
				gimbal_handle.yaw_motor.given_value = gimbal_handle.yaw_motor.sensor.relative_angle * (1 - ramp_v0_calculate(&yaw_ramp));
				if (fabsf(gimbal_handle.yaw_motor.sensor.relative_angle) <= 10.0f)
				{
						gimbal_handle.yaw_motor.given_value = 0;
				}

    }
//    if(gimbal_handle.last_moment_ctrl_mode != GIMBAL_INIT||!CheckDeviceIsOffline(OFFLINE_DBUS))
//	{
        start_motor(gimbal_handle.gimbal_can,CAN_ID);//达妙使能电机
//    }

}

void GimbalGyroAngleMode(void)
{
    gimbal_handle.yaw_motor.mode = GYRO_MODE;
    gimbal_handle.pitch_motor.mode = GYRO_MODE;

    float yaw_target = 0, pitch_target = 0;

//    yaw_target = gimbal_handle.yaw_motor.given_value + gimbal_handle.console->gimbal.yaw_v;
//    pitch_target = gimbal_handle.pitch_motor.given_value + gimbal_handle.console->gimbal.pitch_v;

		gimbal_handle.yaw_motor.given_value	= gimbal_handle.yaw_motor.given_value + gimbal_handle.console->gimbal.yaw_v;
	  gimbal_handle.pitch_motor.motor_info->given_value = gimbal_handle.pitch_motor.motor_info->given_value + gimbal_handle.console->gimbal.pitch_v;

    gimbal_handle.yaw_motor.given_value = AngleTransform(gimbal_handle.yaw_motor.given_value, gimbal_handle.yaw_motor.sensor.gyro_angle);
    gimbal_handle.pitch_motor.motor_info->given_value = AngleTransform(gimbal_handle.pitch_motor.motor_info->given_value, gimbal_handle.pitch_motor.sensor.gyro_angle);

    VAL_LIMIT(gimbal_handle.pitch_motor.motor_info->given_value, gimbal_handle.pitch_motor.min_relative_angle, gimbal_handle.pitch_motor.max_relative_angle);

}

void GimbalRelativeAngleMode(void)                 //SW2上拨杆电机模式设置
{
    gimbal_handle.yaw_motor.mode = ENCONDE_MODE;          //Y轴编码器模式
    gimbal_handle.pitch_motor.mode = ENCONDE_MODE;        //P轴编码器模式

    gimbal_handle.yaw_motor.given_value += gimbal_handle.console->gimbal.yaw_v;       //Y轴补偿值
    gimbal_handle.pitch_motor.given_value += gimbal_handle.console->gimbal.pitch_v;   //P轴补偿值

    VAL_LIMIT(gimbal_handle.yaw_motor.given_value, gimbal_handle.yaw_motor.min_relative_angle, gimbal_handle.yaw_motor.max_relative_angle);
    VAL_LIMIT(gimbal_handle.pitch_motor.given_value, gimbal_handle.pitch_motor.min_relative_angle, gimbal_handle.pitch_motor.max_relative_angle);

}

void GimbalClippedMode(void)//吊射微调模式
{
    gimbal_handle.yaw_motor.mode = ENCONDE_MODE;
    gimbal_handle.pitch_motor.mode = ENCONDE_MODE;

    gimbal_handle.yaw_motor.given_value += gimbal_handle.console->gimbal.yaw_v/1000;
    gimbal_handle.pitch_motor.given_value += gimbal_handle.console->gimbal.pitch_v/1000;

    VAL_LIMIT(gimbal_handle.yaw_motor.given_value, gimbal_handle.yaw_motor.min_relative_angle, gimbal_handle.yaw_motor.max_relative_angle);
    VAL_LIMIT(gimbal_handle.pitch_motor.given_value, gimbal_handle.pitch_motor.min_relative_angle, gimbal_handle.pitch_motor.max_relative_angle);

}



void GimbalNormalMode(void)
{
    float yaw_target = 0;
    float spin90_flag = Console_Pointer()->spin90_flag;
    gimbal_handle.yaw_motor.mode = GYRO_MODE;
    gimbal_handle.pitch_motor.mode = ENCONDE_MODE;
                 /*当前欧拉角+遥控器拨杆速度+旋转90度*/
    yaw_target = gimbal_handle.yaw_motor.given_value + gimbal_handle.console->gimbal.yaw_v + gimbal_handle.console->revolve90*90;

    gimbal_handle.yaw_motor.given_value = AngleTransform(yaw_target, gimbal_handle.yaw_motor.sensor.gyro_angle);
    gimbal_handle.pitch_motor.given_value += gimbal_handle.console->gimbal.pitch_v;
//	gimbal_handle.pitch_motor.given_value -= z1;
    gimbal_handle.console->revolve90 = 0;

    VAL_LIMIT(gimbal_handle.pitch_motor.given_value, gimbal_handle.pitch_motor.min_relative_angle, gimbal_handle.pitch_motor.max_relative_angle);

}

void GimbalVisionAimMode(void)
{
    gimbal_handle.yaw_motor.mode = GYRO_MODE;                                       //使能云台yaw轴陀螺仪模式
    gimbal_handle.pitch_motor.mode = ENCONDE_MODE;                                  //使能云台pitch轴编码器模式

	Aim_contorl(&yaw_aim);
	pid_paramSA(&yaw_aim);//自适应PID

	VisionDatabase_t* info = VisionData_Pointer();//更新视觉数据
//    info->yaw -= z1;
    gimbal_handle.pitch_motor.given_value += gimbal_handle.console->gimbal.pitch_v+vision_comps_pitch*pid_calc(&pitch_vision_pid,gimbal_handle.pitch_motor.given_value,gimbal_handle.pitch_motor.given_value+info->pitch);
    gimbal_handle.yaw_motor.given_value += gimbal_handle.console->gimbal.yaw_v+pid_calc(&yaw_vision_pid,gimbal_handle.yaw_motor.given_value,gimbal_handle.yaw_motor.given_value+info->yaw);

    VAL_LIMIT(gimbal_handle.pitch_motor.given_value, gimbal_handle.pitch_motor.min_relative_angle, gimbal_handle.pitch_motor.max_relative_angle);//pitch角度限位
    gimbal_handle.yaw_motor.given_value  =  AngleTransform(gimbal_handle.yaw_motor.given_value, gimbal_handle.yaw_motor.sensor.gyro_angle);
}

float AngleTransform(float target_angle, float gyro_angle)
{
    float offset = 0, now = 0, target = 0;

    ANGLE_LIMIT_360(target, target_angle);
    ANGLE_LIMIT_360(now, gyro_angle);

    offset = target - now;
    if (offset > 180)
    {
        offset = offset - 360;
    }
    else if (offset < -180)
    {
        offset = offset + 360;
    }
    return gyro_angle + offset;
}

void Aim_contorl(AutoAim_t* Aim)
{
    AimContorlModeSwitch(Aim);
    switch (Aim->aim_mode)
    {
        case RELAX:
        {
            //视觉NUC离线，无法被视觉控制自瞄
            vision_data.yaw_success = AIM_NO;
            Aim->aiming_time = 0 , Aim->stay_time = 0;
            yaw_vision_pid.i=0.00f;
        }break;
        case FOLLOW:
        {
            //自瞄开始，此时还未瞄准到规定角度差
            vision_data.yaw_success = AIM_NO,yaw_vision_pid.p = 0.003;//user set

            if (Aim->aim_flag == 0)
            {
                Aim->Astart_time = Aim->systeam_time,Aim->aim_flag = 1;//记录自瞄开始时间，并进入新的单次自瞄
            }
            else
            {
                Aim->aiming_time = Aim->systeam_time - Aim->Astart_time;//记录自瞄持续时间
            }
        }break;
        case FIRST_AIMING:
        {
            /************记录未进入第一次自瞄的时间**********/
            Aim->Sstart_time = Aim->systeam_time;
            Aim->first_aim = 1, Aim->aiming_time = 0;//进入第一次自瞄
            yaw_vision_pid.p= 0.0001;//user set   //0.06待测试  0.0001
        }break;
        case COMPLETE_AIMING:
        {
            Aim->stay_time = Aim->systeam_time-Aim->Sstart_time;//comparing aimed right time
            if(Aim->stay_time>Aim->tol_time)//自瞄持续时间达到tol_time的标准
            {
                Aim->first_aim = 0,Aim->aim_flag = 0;//认为已完成瞄准，退出此次自瞄及消抖
                if(ABS(vision_data.yaw)<1.0f)vision_data.yaw_success = AIM_RIGHT;//if long enough ,thought to be aimed stable
            }
        }break;
        default:
            break;
    }
    VAL_LIMIT(vision_data.yaw,-15.0f,15.0f);
}

VisionDatabase_t* VisionData_Pointer(void)
{
    return &vision_data;
}

void pid_paramSA(AutoAim_t* Aim)
{
    if(Aim->enable_paramSA)
    {
        if(vision_data.yaw_success == AIM_NO)
        {
            /*时间豁度补偿系数P   TODO 增添I补偿及D补偿*/
            yaw_vision_pid.p += ((float)(Aim->aiming_time*0.0008/Aim->Ap_parm));
            if(Aim->first_aim)//如果进入第一次自瞄
            {
                VAL_LIMIT(Aim->stay_time,1,1000);
                yaw_vision_pid.p -= (Aim->Sp_parm/(float)(Aim->stay_time));
            }
            if(vision_data.yaw==-90)//90位规定的角度
                yaw_vision_pid.i=0;
            VAL_LIMIT(yaw_vision_pid.p,0.00001f,0.02f);
            VAL_LIMIT(yaw_vision_pid.i, 0.0f, 0.000015f);
        }
    }
}
void AimContorlModeSwitch(AutoAim_t* Aim)
{
    if (vision_data.state)//视觉NUC在线时，此条件一直成立
    {
        if (ABS(vision_data.yaw)<Aim->tol_angle)//自瞄已经达到一定角度之内
        {
            //thought to be aimed right认为自瞄正确
            if (vision_data.yaw_success == AIM_NO&&!Aim->first_aim)
            {
                Aim->aim_mode = FIRST_AIMING;
            }
            else if(vision_data.yaw_success == AIM_NO&&Aim->first_aim)
            {
                Aim->aim_mode = COMPLETE_AIMING;
            }
        }
        else
        {
            Aim->aim_mode = FOLLOW;
        }
    }
    else
    {
        Aim->aim_mode = RELAX;
    }
}

float Gimbal_PID_Calc(Gimbal_PID_t* pid, float angle_ref, float angle_fdb, float speed_fdb)
{
    pid->angle_ref = angle_ref;
    pid->angle_fdb = angle_fdb;
    pid_calc(&pid->outer_pid, pid->angle_fdb, pid->angle_ref);
    pid->speed_ref = pid->outer_pid.out;
    pid->speed_fdb = speed_fdb;
    pid_calc(&pid->inter_pid, pid->speed_fdb, pid->speed_ref);
    return pid->inter_pid.out;
}

float Gimbal_ESO_PID_Calc(Gimbal_PID_t* pid, float angle_ref, float angle_fdb, float speed_fdb)
{
    pid->angle_ref = angle_ref;
    pid->angle_fdb = angle_fdb;
    pid_calc(&pid->outer_pid, pid->angle_fdb, pid->angle_ref);
    pid->speed_ref = pid->outer_pid.out - z2;
    pid->speed_fdb = speed_fdb;
    pid_calc(&pid->inter_pid, pid->speed_fdb, pid->speed_ref);
    pid->inter_pid.out -= z3;
    return pid->inter_pid.out;
}


void Gimbal_PID_Clear(Gimbal_PID_t* pid)
{
    pid_clear(&pid->outer_pid);
    pid_clear(&pid->inter_pid);
}

void GimbalMotorChangeProtect(GimbalMotor_t* motor)
{
    if (motor->last_mode != motor->mode)
    {
        if(motor->mode == RAW_VALUE_MODE)
        {
            motor->given_value = motor->current_set;
        }
        else if (motor->mode == GYRO_MODE)
        {
            motor->given_value = motor->sensor.gyro_angle;
        }
        else if (motor->mode == ENCONDE_MODE)
        {
            motor->given_value = motor->sensor.relative_angle;
        }
    }
    motor->last_mode = motor->mode;
}

void GimbalMotorControl(GimbalMotor_t* motor)
{
    GimbalMotorChangeProtect(motor);
    if (motor->mode == RAW_VALUE_MODE)
    {
        motor->current_set = motor->given_value;
        Gimbal_PID_Clear(&motor->pid);
    }
    else if(motor->mode == GYRO_MODE)
    {
        if(motor->motor_info->motor_type == DJ_MOTOR)
        {
            motor->current_set = Gimbal_PID_Calc(&motor->pid,
                                             motor->given_value,
                                             motor->sensor.gyro_angle,
                                             motor->sensor.palstance);
        }
        // else if (motor->motor_info->motor_type == DM_MOTOR)  //不是陀螺仪坐标系，之后需要转换成陀螺仪角度！！
        // {

        //     float relative_angle = DMMotor_RelativePosition(motor->given_value/180.0f*PI, motor->motor_info->offset_value);//将给定的角度转换为相对角度
        //     motor->motor_info->given_value = DMMotor_CalculateAbsolutePosition(relative_angle,motor->motor_info->offset_value);
        // }

    }
    else if(motor->mode == ENCONDE_MODE)
    {
        if(motor->motor_info->motor_type == DJ_MOTOR)
        {
            motor->current_set = Gimbal_PID_Calc(&motor->pid,
                                                 motor->given_value,
                                                 motor->sensor.relative_angle,
                                                 motor->sensor.palstance);
        }
        else if (motor->motor_info->motor_type == DM_MOTOR)
        {

            // float relative_angle = DMMotor_RelativePosition(motor->given_value/180.0f*PI, motor->motor_info->offset_value);//将给定的角度转换为相对角度
            motor->motor_info->given_value = motor->given_value/180.0f*PI+motor->motor_info->offset_value;
        }
    }
}