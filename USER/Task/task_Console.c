
#include "robot_info.h"
#include "cmsis_os.h"

//#include "alg_ramp.h"

#include "drv_timer.h"//定时器函数

#include "mdl_comm.h"//云台通讯数据
#include "mdl_Console.h"//控制台的数据类型和函数

#include "task_Console.h"
#include "task_Detect.h"//离线检测的数据类型和函数
#include "robot_info.h"


RC_Info_t last_rc;
RC_Switch_t wheel_switch;
RC_Switch_t rc_switch2;

uint16_t init_stime,init_etime,l_stime,l_etime;

// ramp_v0_t front_back_ramp = RAMP_GEN_DAFAULT;
// ramp_v0_t left_right_ramp = RAMP_GEN_DAFAULT;
// ramp_v0_t shift_front_ramp = RAMP_GEN_DAFAULT;
// ramp_v0_t shift_left_ramp = RAMP_GEN_DAFAULT;

uint32_t time_ms;
uint32_t time_last;

osThreadId ConsoleTaskHandle;
#if INCLUDE_uxTaskGetStackHighWaterMark
static uint32_t console_task_stack = 0;
#endif

void ConsoleTask(void const *argument)
{
    for(;;)                                      //
    {
        RemoteControlWheelAction();                                //
        RC_SwitchAction(&rc_switch2, console.rc->sw2);             //

			  console.last_ctrl_mode =  console.ctrl_mode;
        switch (console.ctrl_mode)                                 //
        {
            case PREPARE_MODE:                                              //
            {
                if (GimbalInfo_Pointer()->mode != GIMBAL_INIT
                        && GimbalInfo_Pointer()->mode != GIMBAL_RELAX)
                {
						console.ctrl_mode = NORMAL_MODE;
						console.gimbal_cmd = GIMBAL_NORMAL_CMD;
						console.chassis_cmd = CHASSIS_FOLLOW_GIMBAL_CMD;
						console.shoot_cmd = SHOOT_STOP_CMD;
                }
                else
                {
                    time_ms++;
                    if(time_ms>50)
                    {
											  console.ctrl_mode = NORMAL_MODE;
                        console.gimbal_cmd = GIMBAL_RELATIVE_CMD;
                        console.chassis_cmd = CHASSIS_SEPARATE_GIMBAL_CMD;
                        console.shoot_cmd = SHOOT_STOP_CMD;
											 time_ms = 0;
										}
										else
										{
                        console.gimbal_cmd  = GIMBAL_INIT_CMD;
                        console.chassis_cmd = CHASSIS_STOP_CMD;
                        console.shoot_cmd = SHOOT_INIT_CMD;

										}

//												}
                }
            }break;
            case NORMAL_MODE:                                                   //
            {
								time_last = time_ms;
                if (console.rc->sw1 == REMOTE_SWITCH_VALUE_CENTRAL)
                {
                    RemoteControl_Operation();
                }
                else if(console.rc->sw1 == REMOTE_SWITCH_VALUE_UP)
                {
                    Keyboard_Operation();
                }
                else if(console.rc->sw1 == REMOTE_SWITCH_VALUE_DOWN)
                {
                    Other_Operation();
                }
            }break;
            case SAFETY_MODE:
            {
                if(!CheckDeviceIsOffline(OFFLINE_DBUS))
                {
                    console.ctrl_mode = PREPARE_MODE;//遥控器在线
                }
                else                                 //遥控器离线
                {
									  init_stime = BSP_GetTime_ms();
                    console.gimbal_cmd  = GIMBAL_RELEASE_CMD;
                    console.chassis_cmd  = CHASSIS_RELEASE_CMD;
                    console.shoot_cmd = SHOOT_RELEASE_CMD;
                }
            }break;
            default:
                break;
        }

        if(CheckDeviceIsOffline(OFFLINE_DBUS))
        {
            console.ctrl_mode = SAFETY_MODE;
        }
        last_rc = *console.rc;
        osDelay(CONSOLE_TASK_PERIOD);                                  //?
#if INCLUDE_uxTaskGetStackHighWaterMark
        console_task_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

void ConsoleTaskInit(void)
{
    console.rc = RC_GetDataPointer();        //更新遥控器信息
    console.ctrl_mode = PREPARE_MODE;        //控制模式为准备模式
    console.chassis_cmd = CHASSIS_STOP_CMD;  //底盘标志位为停止
    console.gimbal_cmd = GIMBAL_INIT_CMD;    //云台标志位为初始化
    console.shoot_cmd = SHOOT_STOP_CMD;      //射击标志位为停止
    console.shoot.fire_cmd = STOP_FIRE_CMD;  //摩擦轮标志位为停止
    console.supercap_cmd =SUPERCAP_OFF_CMD;
    console.shoot_flag=0;
    console.revolve90 =0;
    console.Yaw_comps=0;                //视觉
    console.spin90_flag=0;              //旋转90度
    console.heat_limit=Heat_limit_on;

    ramp_v0_init(&front_back_ramp, CHASSIS_ACCEL_TIME/CONSOLE_TASK_PERIOD);      //斜波函数
    ramp_v0_init(&left_right_ramp, CHASSIS_ACCEL_TIME/CONSOLE_TASK_PERIOD);      //斜波函数
    ramp_v0_init(&shift_front_ramp,CHASSIS_SHIFT_ACCEL_TIME/CONSOLE_TASK_PERIOD);//斜波函数
    ramp_v0_init(&shift_left_ramp,CHASSIS_SHIFT_ACCEL_TIME/CONSOLE_TASK_PERIOD); //斜波函数
    ramp_v0_init(&console.gimbal.pitch_ramp, 1);																 //斜波函数
    ramp_v0_init(&console.gimbal.yaw_ramp, 1);																	 //斜波函数

    osThreadDef(console_task, ConsoleTask, osPriorityNormal, 0, 256);            //控制任务开始
    ConsoleTaskHandle = osThreadCreate(osThread(console_task), NULL);            //控制任务开始
}