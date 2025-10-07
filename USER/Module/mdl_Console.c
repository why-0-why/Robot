
#include "mdl_Console.h"
#include "mdl_comm.h"
#include "robot_info.h"
Console_t console;
ramp_v0_t front_back_ramp = RAMP_GEN_DAFAULT;
ramp_v0_t left_right_ramp = RAMP_GEN_DAFAULT;
ramp_v0_t shift_front_ramp = RAMP_GEN_DAFAULT;
ramp_v0_t shift_left_ramp = RAMP_GEN_DAFAULT;

static float beta = 1;//不同等级对应的速度增益系数
uint16_t vision_flag;

void RemoteControlWheelAction(void)
{
    static uint8_t wheel_sw = REMOTE_SWITCH_VALUE_CENTRAL;
    if (console.rc->wheel < -440)
    {
        wheel_sw = REMOTE_SWITCH_VALUE_UP;
    }
    else if (console.rc->wheel > -220 && console.rc->wheel < 220)
    {
        wheel_sw = REMOTE_SWITCH_VALUE_CENTRAL;
    }
    else if (console.rc->wheel > 440)
    {
        wheel_sw = REMOTE_SWITCH_VALUE_DOWN;
    }
    RC_SwitchAction(&wheel_switch, wheel_sw);
}

void RemoteControl_Operation(void)
{
    static uint32_t shoot_time = 0;
    console.island_cmd = ISLAND_RELAX_CMD;
    console.SuperPower_cmd	=	Power_IN;
    if (console.rc->sw2 == REMOTE_SWITCH_VALUE_UP)
    {
        console.gimbal_cmd = GIMBAL_VISION_CMD;
        console.chassis_cmd = CHASSIS_SEPARATE_GIMBAL_CMD;

        console.chassis.vx = console.rc->ch4 / RC_RESOLUTION * RC_CHASSIS_MAX_SPEED_X;
        console.chassis.vw = console.rc->ch3 / RC_RESOLUTION * RC_CHASSIS_MAX_SPEED_R;
        console.gimbal.pitch_v = -console.rc->ch2 * RC_GIMBAL_MOVE_RATIO_PIT;
        console.gimbal.yaw_v = -console.rc->ch1 * RC_GIMBAL_MOVE_RATIO_YAW;
    }
    else if(console.rc->sw2 == REMOTE_SWITCH_VALUE_CENTRAL)
    {
        console.gimbal_cmd = GIMBAL_NORMAL_CMD;
        console.chassis_cmd = CHASSIS_FOLLOW_GIMBAL_CMD;

        console.chassis.vx = console.rc->ch4 / RC_RESOLUTION * RC_CHASSIS_MAX_SPEED_X;
        console.chassis.vy = console.rc->ch3 / RC_RESOLUTION * RC_CHASSIS_MAX_SPEED_Y;
        console.gimbal.pitch_v = -console.rc->ch2 * RC_GIMBAL_MOVE_RATIO_PIT;
        console.gimbal.yaw_v = -console.rc->ch1 * RC_GIMBAL_MOVE_RATIO_YAW;
    }
    else if(console.rc->sw2 == REMOTE_SWITCH_VALUE_DOWN)
    {
        console.gimbal_cmd = GIMBAL_VISION_CMD;
        console.chassis_cmd = CHASSIS_SPIN_CMD;

        console.chassis.vx = console.rc->ch4 / RC_RESOLUTION * RC_CHASSIS_MAX_SPEED_X;
        console.chassis.vy = console.rc->ch3 / RC_RESOLUTION * RC_CHASSIS_MAX_SPEED_Y;
        console.gimbal.pitch_v = -console.rc->ch2 * RC_GIMBAL_MOVE_RATIO_PIT;
        console.gimbal.yaw_v = -console.rc->ch1 * RC_GIMBAL_MOVE_RATIO_YAW;
    }

     if (console.shoot_cmd == SHOOT_STOP_CMD)
    {
        if (wheel_switch.switch_state == REMOTE_SWITCH_CHANGE_3TO1)
        {
            console.shoot_cmd = SHOOT_START_CMD;
        }
    }
    else if (console.shoot_cmd == SHOOT_START_CMD)
    {
        if (wheel_switch.switch_state == REMOTE_SWITCH_CHANGE_3TO1)
        {
            console.shoot_cmd = SHOOT_STOP_CMD;
        }
        else if (wheel_switch.switch_state == REMOTE_SWITCH_CHANGE_3TO2)
        {
            console.shoot.fire_cmd = ONE_FIRE_CMD;
        }
        else if (wheel_switch.switch_value_raw == REMOTE_SWITCH_VALUE_DOWN)
        {
            shoot_time++;
            if(shoot_time > 60)
                console.shoot.fire_cmd = ONE_FIRE_CMD ;
            else
                console.shoot.fire_cmd = STOP_FIRE_CMD;
        }
				/* workspace   */
				else if (gimbal_info.can_shoot == 1 && console.gimbal_cmd  ==  GIMBAL_VISION_CMD)  //英雄底盘使用云台传输的视觉CANSHOOT
				{
					console.shoot.fire_cmd = ONE_FIRE_CMD;
				/* workspace   */
				}
        else
        {
            console.shoot.fire_cmd = STOP_FIRE_CMD;
            shoot_time = 0;
        }
    }
}

void Other_Operation(void)                  //调试模式，内含超电测试模式
{
    static uint32_t shoot_time = 0;


    if (console.rc->sw2 == REMOTE_SWITCH_VALUE_UP)
    {
        console.gimbal_cmd 	= GIMBAL_VISION_CMD;
        console.chassis_cmd = CHASSIS_SEPARATE_GIMBAL_CMD;
        console.supercap_cmd = SUPERCAP_OFF_CMD;
        console.chassis.vx = -console.rc->ch4 / RC_RESOLUTION * -RC_CHASSIS_MAX_SPEED_X;
        console.chassis.vy = -console.rc->ch3 / RC_RESOLUTION * -RC_CHASSIS_MAX_SPEED_Y;
        console.gimbal.pitch_v = console.rc->ch2 *-RC_GIMBAL_MOVE_RATIO_PIT;
        console.gimbal.yaw_v = console.rc->ch1 * -RC_GIMBAL_MOVE_RATIO_YAW;
    }

    else if (console.rc->sw2 == REMOTE_SWITCH_VALUE_CENTRAL)
    {
        console.gimbal_cmd 	= GIMBAL_VISION_CMD;
        console.chassis_cmd = CHASSIS_FOLLOW_GIMBAL_CMD;
        console.supercap_cmd = SUPERCAP_OFF_CMD;
        console.chassis.vx = -console.rc->ch4 / RC_RESOLUTION * -RC_CHASSIS_MAX_SPEED_X;
        console.chassis.vy = -console.rc->ch3 / RC_RESOLUTION * -RC_CHASSIS_MAX_SPEED_Y;
        console.gimbal.pitch_v = console.rc->ch2 * -RC_GIMBAL_MOVE_RATIO_PIT;
        console.gimbal.yaw_v = console.rc->ch1 * -RC_GIMBAL_MOVE_RATIO_YAW;
//        console.chassis_cmd = CHASSIS_ISLAND_CMD;
//        console.island_cmd = ISLAND_START_CMD;
    }
    else if (console.rc->sw2 == REMOTE_SWITCH_VALUE_DOWN)
    {
		console.gimbal_cmd  = GIMBAL_VISION_CMD;
        console.chassis_cmd = CHASSIS_SPIN_CMD;
        console.supercap_cmd = SUPERCAP_ON_CMD;
        console.chassis.vx = console.rc->ch4 / RC_RESOLUTION * RC_CHASSIS_MAX_SPEED_X;
        console.chassis.vy = console.rc->ch3 / RC_RESOLUTION * RC_CHASSIS_MAX_SPEED_Y;
        console.gimbal.pitch_v = -console.rc->ch2 * RC_GIMBAL_MOVE_RATIO_PIT;
        console.gimbal.yaw_v = -console.rc->ch1 * RC_GIMBAL_MOVE_RATIO_YAW;
			/*
        console.chassis_cmd = CHASSIS_SEPARATE_GIMBAL_CMD;
			  console.gimbal_cmd 	= GIMBAL_MOVE_CMD;

			  console.chassis.vx = -console.rc->ch4 / RC_RESOLUTION * -RC_CHASSIS_MAX_SPEED_X;
        console.chassis.vy = -console.rc->ch3 / RC_RESOLUTION * -RC_CHASSIS_MAX_SPEED_Y;
        console.gimbal.pitch_v = console.rc->ch2 * RC_GIMBAL_MOVE_RATIO_PIT;
        console.gimbal.yaw_v = -console.rc->ch1 * RC_GIMBAL_MOVE_RATIO_YAW;
			*/
    }

    if (console.shoot_cmd == SHOOT_STOP_CMD)
    {
        if (wheel_switch.switch_state == REMOTE_SWITCH_CHANGE_3TO1)
        {
            console.shoot_cmd = SHOOT_START_CMD;
        }
    }
    else if (console.shoot_cmd == SHOOT_START_CMD)
    {
        if (wheel_switch.switch_state == REMOTE_SWITCH_CHANGE_3TO1)
        {
            console.shoot_cmd = SHOOT_STOP_CMD;
        }
        else if (wheel_switch.switch_state == REMOTE_SWITCH_CHANGE_3TO2)
        {
            console.shoot.fire_cmd = ONE_FIRE_CMD;
        }
        else if (wheel_switch.switch_value_raw == REMOTE_SWITCH_VALUE_DOWN)
        {
            shoot_time++;
            if(shoot_time > 60)
                console.shoot.fire_cmd = ONE_FIRE_CMD ;
            else
                console.shoot.fire_cmd = STOP_FIRE_CMD;
        }
        else
        {
            console.shoot.fire_cmd = STOP_FIRE_CMD;
            shoot_time = 0;
        }
    }
}

/**************在对键盘操作修改时，务必注意状态触发和边沿触发两种触发方式*************/
/**************if (console.rc->kb.bit.F)此为状态触发，F键处于按下状态时触发*********/
/**************if（!last_rc.kb.bit.CTRL&&console.rc->kb.bit.CTRL）此为上升沿触发，CTRL由没按到按下的状态变化时触发 */

void general_keyboard(void)
{
	/*******************************底盘功能***********************************/
	if (!last_rc.kb.bit.F&&console.rc->kb.bit.F)            //正常模式，用来复位 //务必注意此形式为按键处于按下状态时触发
    {
        console.gimbal_cmd = GIMBAL_NORMAL_CMD;
        console.chassis_cmd = CHASSIS_FOLLOW_GIMBAL_CMD;
        console.shoot_cmd = SHOOT_STOP_CMD;
        console.clipped_flag = 0;
        console.r3_flag = 0;
        console.Keyboard_flag = 0;
        console.heat_limit=Heat_limit_on;		
    }   		
   else if(!last_rc.kb.bit.CTRL&&console.rc->kb.bit.CTRL)//云台分离
   {
       console.gimbal_cmd = GIMBAL_RELATIVE_CMD;
       console.chassis_cmd = CHASSIS_SEPARATE_GIMBAL_CMD;
   }

	if (!last_rc.kb.bit.B && console.rc->kb.bit.B)
	{  //2024赛季旧版B键画UI
		// Comm_GimbalInfo_t * gimbal_info = GimbalInfo_Pointer();    
		// uint16_t robot_id = RefereeSystem_GetRobotID();
		// ClientUI_SetHeaderSenderID(robot_id);
		// ClientUI_SetHeaderReceiverID(0x100 + robot_id);
		// ClientUI_DrawLine(&xx7,"xx7",0,-70,-120,50,-120,2,UI_COLOR_YELLOW);
	}

	if (!last_rc.kb.bit.C && console.rc->kb.bit.C)
	{   //2024赛季旧版B键画UI
		// Comm_GimbalInfo_t * gimbal_info = GimbalInfo_Pointer();    
		// uint16_t robot_id = RefereeSystem_GetRobotID();
		// ClientUI_SetHeaderSenderID(robot_id);
		// ClientUI_SetHeaderReceiverID(0x100 + robot_id);
		// ClientUI_DrawLine(&cx2,"cx2",0,-10,200,-10,-200,2,UI_COLOR_YELLOW);
	}
	
	/*****小陀螺开关*****/	
	if(!last_rc.kb.bit.R && console.rc->kb.bit.R)
	{
		if(console.chassis_cmd != CHASSIS_SPIN_CMD)
		{
			console.chassis_cmd = CHASSIS_SPIN_CMD;
		}
		else
		{
			console.chassis_cmd = CHASSIS_FOLLOW_GIMBAL_CMD;
		}
	}
	
    /*****跟随分离切换*****/
	if(!last_rc.kb.bit.CTRL && console.rc->kb.bit.CTRL)//云台底盘跟随
	{
		if(console.chassis_cmd != CHASSIS_FOLLOW_GIMBAL_CMD)
		{
			console.chassis_cmd = CHASSIS_FOLLOW_GIMBAL_CMD;
		}
		else
		{
			console.chassis_cmd = CHASSIS_SEPARATE_GIMBAL_CMD;
		}
	}
	
    /*****一键调头90度*****/
	if(!last_rc.kb.bit.G && console.rc->kb.bit.G)
	{
		console.chassis_cmd = CHASSIS_FOLLOW_GIMBAL_CMD;
		console.revolve90++;
		if(console.revolve90 == 2)
		{
			console.revolve90=0;
		}
	}
	/*******************************云台功能**********************************/
    /*****弹舱盖开关*****/
	if(!last_rc.kb.bit.E && console.rc->kb.bit.E)
	{
		if(console.magazine_cmd != MAGAZINE_OFF_CMD)
		{
			console.magazine_cmd = MAGAZINE_OFF_CMD;//关
    }
		else
		{
			console.magazine_cmd = MAGAZINE_ON_CMD;//开
		}
	}

	if(console.rc->kb.bit.SHIFT)                                       //超电开关
	{
		console.supercap_cmd=SUPERCAP_ON_CMD;
	}
	else
	{
		console.supercap_cmd=SUPERCAP_OFF_CMD;
	}
}
//对底盘VX、VY、VW，云台PITCH_V和YAW_V设置
void movement_keyboard(void)  
{
	float chassis_vx = 0;
    float chassis_vy = 0;
	float chassis_vw = 0;
	float gimbal_vx  = 0;
    float gimbal_vy  = 0;
	/********************************移动模式************************************/			
	ramp_v0_init(&shift_front_ramp,CHASSIS_SHIFT_ACCEL_TIME/CONSOLE_TASK_PERIOD);        //斜坡初始化
	ramp_v0_init(&shift_left_ramp,CHASSIS_SHIFT_ACCEL_TIME/CONSOLE_TASK_PERIOD);
	
//			if(RefereeSystem_RobotState_Pointer()->chassis_power_limit<=55)
//		{
//			beta = 0.4;	
//		}
//		else if(RefereeSystem_RobotState_Pointer()->chassis_power_limit<=60)
//		{
//			beta = 110;
//		}
//		else if(RefereeSystem_RobotState_Pointer()->chassis_power_limit<=65)
//		{
//			beta = 120;
//		}
//		else if(RefereeSystem_RobotState_Pointer()->chassis_power_limit<=70)
//		{
//			beta = 130;
//		}
//		else if(RefereeSystem_RobotState_Pointer()->chassis_power_limit<=75)
//		{
//			beta = 140;
//		}
//		else if(RefereeSystem_RobotState_Pointer()->chassis_power_limit<=80)
//		{
//			beta = 150;
//		}
//		else if(RefereeSystem_RobotState_Pointer()->chassis_power_limit<=85)
//		{
//			beta = 160;
//		}
//		else if(RefereeSystem_RobotState_Pointer()->chassis_power_limit<=90)
//		{
//			beta = 170;
//		}
//		else if(RefereeSystem_RobotState_Pointer()->chassis_power_limit<=100)
//		{
//			beta = 190;
//		}
//		else if(RefereeSystem_RobotState_Pointer()->chassis_power_limit<=120)
//		{
//			beta = 230;
//		}
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
	
        
//	chassis_vx = KB_CHASSIS_MAX_SPEED_X * 0.408f;                                  
//	chassis_vy = KB_CHASSIS_MAX_SPEED_Y * 0.408f; 
//	chassis_vw = KB_CHASSIS_MAX_SPEED_R	* 0.408f;	
	if(console.supercap_cmd==SUPERCAP_ON_CMD)
	{
		chassis_vx = KB_CHASSIS_MAX_SPEED_X*2;                                  
		chassis_vy = KB_CHASSIS_MAX_SPEED_Y*2; 
		chassis_vw = KB_CHASSIS_MAX_SPEED_R*2;	
	}
	else if(console.supercap_cmd==SUPERCAP_OFF_CMD)
	{
		chassis_vx = KB_CHASSIS_MAX_SPEED_X*beta;                                  
		chassis_vy = KB_CHASSIS_MAX_SPEED_Y*beta; 
		chassis_vw = KB_CHASSIS_MAX_SPEED_R*beta;	
	}
	
	if(console.rc->kb.bit.S)                                       //移动
    {
		console.chassis.vx = (chassis_vx*(2*console.Reset_yaw - 1) * ramp_v0_calculate(&front_back_ramp));
    }
    else if (console.rc->kb.bit.W)
    {      
		console.chassis.vx = (chassis_vx*(-2*console.Reset_yaw + 1) * ramp_v0_calculate(&front_back_ramp));
    }
    else
    {
        console.chassis.vx = 0;
        ramp_v0_init(&front_back_ramp, CHASSIS_ACCEL_TIME/CONSOLE_TASK_PERIOD);
    }
	
  if(console.chassis_cmd == CHASSIS_FOLLOW_GIMBAL_CMD)
	{
		if (console.rc->kb.bit.D)
        {
		console.chassis.vy = chassis_vy*(-2*console.Reset_yaw +1) * ramp_v0_calculate(&left_right_ramp);
        }
        else if(console.rc->kb.bit.A)
        {
        console.chassis.vy = chassis_vy*(2*console.Reset_yaw - 1) * ramp_v0_calculate(&left_right_ramp);
        }
        else
        {
        console.chassis.vy = 0;
        ramp_v0_init(&left_right_ramp, CHASSIS_ACCEL_TIME/CONSOLE_TASK_PERIOD);
        }
	}
	
	if(console.chassis_cmd == CHASSIS_SPIN_CMD)
	{
		if (console.rc->kb.bit.D)
        {
		console.chassis.vy = chassis_vy*(-2*console.Reset_yaw +1) * ramp_v0_calculate(&left_right_ramp);
        }
        else if(console.rc->kb.bit.A)
        {
        console.chassis.vy = chassis_vy*(2*console.Reset_yaw - 1) * ramp_v0_calculate(&left_right_ramp);
        }
        else
        {
        console.chassis.vy = 0;
        ramp_v0_init(&left_right_ramp, CHASSIS_ACCEL_TIME/CONSOLE_TASK_PERIOD);
        }
	}
	
	if(console.chassis_cmd == CHASSIS_SEPARATE_GIMBAL_CMD)
	{
		if (console.rc->kb.bit.D)
		{
			console.chassis.vy = chassis_vy*(-2*console.Reset_yaw +1) * ramp_v0_calculate(&left_right_ramp);
		}
		else if(console.rc->kb.bit.A)
		{
			console.chassis.vy = chassis_vy*(2*console.Reset_yaw - 1) * ramp_v0_calculate(&left_right_ramp);
		}
		else
		{
			console.chassis.vy = 0;
			ramp_v0_init(&left_right_ramp, CHASSIS_ACCEL_TIME/CONSOLE_TASK_PERIOD);
		}
	}
	
	console.gimbal.yaw_v = -console.rc->mouse.x * KB_GIMBAL_MOVE_RATIO_YAW;
    console.gimbal.pitch_v = console.rc->mouse.y * KB_GIMBAL_MOVE_RATIO_PIT;
}

void shoot_keyboard(void)
{
	if(!last_rc.kb.bit.V && console.rc->kb.bit.V )   //V键设置否打开自动开火 
  {             
		console.auto_aim_flag++;
		if(console.auto_aim_flag==2)
		{
			 console.auto_aim_flag=0;
		}             
  }
/******************************************************/
	/******摩擦轮开关*****/
	/**************思路：Q的优先级比鼠标右键高，代码执行顺序在其后，
	 即便右键按下鼠标右键，Q不按也不会改变瞄准模式************/
	if (!last_rc.kb.bit.Q && console.rc->kb.bit.Q)//F              //开摩擦轮
	{
		console.q_vision_flag++;
		if(console.q_vision_flag==2)
		{
			 console.q_vision_flag=0;
		}  	          
	}
	if(console.q_vision_flag)
	{
		console.gimbal_cmd = GIMBAL_VISION_CMD;
		console.shoot_cmd = SHOOT_START_CMD; 
	}
	else 
	{	
		console.gimbal_cmd= GIMBAL_NORMAL_CMD;
		if(vision_flag!=1)
		{
			console.shoot_cmd = SHOOT_STOP_CMD;	
		}
	}
/***********************************************************/

	/*****自瞄开火*****/
	if(!last_rc.mouse.r&&console.rc->mouse.r)                                    //开启自瞄
    {
		console.r_vision_flag++;//控制是否进入自瞄模式
		if(console.r_vision_flag==2)
		{
			 console.r_vision_flag=0;
		}  
	}

	if (console.r_vision_flag == 0)
	{
			console.gimbal_cmd = GIMBAL_NORMAL_CMD;		
	}
	else
	{
			console.gimbal_cmd = GIMBAL_VISION_CMD;
	}


/********************************************************/
	if(console.shoot_cmd==SHOOT_START_CMD)
	{
		if(console.gimbal_cmd!=GIMBAL_VISION_CMD)//没开自瞄
		{
			if(console.shoot_flag==0)//此情况默认
			{
				if(!last_rc.mouse.l&&console.rc->mouse.l)//左键上升沿射击
				{
					console.shoot.fire_cmd=ONE_FIRE_CMD;
				}
				else
				{
					console.shoot.fire_cmd=STOP_FIRE_CMD;
				}
			}
		}
		else
		{
			if(console.auto_aim_flag==0)      //开自瞄时，没有自动开火
			{
				if(!last_rc.mouse.l&&console.rc->mouse.l)
				{
					console.shoot.fire_cmd=ONE_FIRE_CMD;
				}
				else
				{
					console.shoot.fire_cmd=STOP_FIRE_CMD;
				}
			}
			else if(console.auto_aim_flag==1)
			{               
            if((gimbal_info.can_shoot == 1 && console.gimbal_cmd  ==  GIMBAL_VISION_CMD)||(console.rc->mouse.l))
				{
					console.shoot.fire_cmd = ONE_FIRE_CMD;
				}
				else
				{
					console.shoot.fire_cmd=STOP_FIRE_CMD;
				}
		    }
           else
           {
           console.shoot.fire_cmd = STOP_FIRE_CMD ;
           }
		}
		
	}
}

void Keyboard_Operation(void)
{
	general_keyboard();//通过通用键位设置运动模式等
	movement_keyboard();//通过专用限位根据不同运动模式设置车体速度
	console.heat_limit = Heat_limit_on;//25赛季无脑开热量限制，24赛季使用下面注释的代码
	shoot_keyboard();//根据键位设置摩擦轮和拨弹轮命令
}

Console_t* Console_Pointer(void)
{
	return &console;
}