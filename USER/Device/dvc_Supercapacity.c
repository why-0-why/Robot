/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2024-08-27 15:06:50
 * @LastEditors: your name
 * @LastEditTime: 2025-02-27 20:12:11
 * @FilePath: \MDK-ARMd:\RM_work\ZJ_Hero_Group_Work\Hero2025_V1.70\Components\drvices\Super_Capacitor\super_capacitor.c
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
/* 包含头文件 ----------------------------------------------------------------*/
#include "dvc_Supercapacity.h"
// TODO：层级不清晰，耦合性高

#include "mdl_Chassis.h"
#include "dvc_Referee_system.h"
/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/
Cap ZFcap_info;
float total_current_g;
static uint16_t super_power_state = 0;

Cap_power cap_info;
/* 扩展变量 ------------------------------------------------------------------*/
/* 私有函数原形 --------------------------------------------------------------*/

/* 函数体 --------------------------------------------------------------------*/




/*************************************************/
 void LONG_CAP_PowerParser(Cap_power *received_data, uint8_t *can_receive_data, uint16_t len)//张泓学长超电的CAN协议
 {
             uint16_t in_v_received = (can_receive_data[1] << 8) | can_receive_data[0];
             uint16_t cap_v_received = (can_receive_data[3] << 8) | can_receive_data[2];
             uint16_t in_c_received = (can_receive_data[5] << 8) | can_receive_data[4];
             uint8_t cap_percent_received = can_receive_data[6];
             uint8_t power_received = can_receive_data[7];

             float in_v = (float)in_v_received / 100.0f;
             float cap_v = (float)cap_v_received / 100.0f;
             float in_c = (float)in_c_received / 100.0f;
             float power_set = (float)power_received;
 						float in_p  = in_v * in_c;
					
             received_data->in_v         = in_v;
             received_data->cap_v        = cap_v;
             received_data->in_c         = in_c;
             received_data->cap_percent  = cap_percent_received;
 					  received_data->in_power		 = in_p;
	
 						RefereeSystem_PowerHeatData_Pointer()->chassis_power = received_data->in_power;
 }

Cap_power* CAP_GetDataPointer(void) //传出一个*P指针访问值。
{
    return &cap_info;             //具体内容
}


/***************************浙纺超电部分*******************************/
/* 
   功能: 通过CAN总线发送超级电容的消息。
   参数: obj - 指向CAN对象的指针，包含要发送的消息。
   返回值: 无。
*/
void SuperCap_SendMessage(CAN_Object_t* obj) //浙纺超电的CAN发送协议,我们24赛季新版超电测试未完成能用但没有低电量保护
{
  uint8_t cmd = chassis_handle.console->supercap_cmd;  //chassis_handle.console->supercap_cmd   chassis_handle.super_flag
  uint8_t mode = 0;//模式默认（暂时）
  uint16_t ref_power_limit = (uint16_t)RefereeSystem_RobotState_Pointer()->chassis_power_limit;
  uint16_t ref_power_buffer = (uint16_t)RefereeSystem_PowerHeatData_Pointer()->chassis_power_buffer;
  uint16_t powerbuffer_limit = (uint16_t)RefereeSystem_PowerHeatData_Pointer()->chassis_power_buffer;
  if(powerbuffer_limit > 60)
      powerbuffer_limit = 250.0f;
  else
      powerbuffer_limit = 60.0f;
  uint8_t TxData[8] = {0};
  TxData[0] = (uint8_t)cmd;
  TxData[1] = (uint8_t)mode;
  TxData[2] = (uint8_t)(ref_power_buffer >> 8);
  TxData[3] = (uint8_t)ref_power_buffer;
  TxData[4] = (uint8_t) (ref_power_limit >> 8);
  TxData[5] = (uint8_t) ref_power_limit;
  TxData[6] = (uint8_t) (powerbuffer_limit >> 8);
  TxData[7] = (uint8_t) powerbuffer_limit;
  BSP_CAN_TransmitData(&can1_obj, Super_Power_Message_ID, TxData, 8);
}

void CAP_PowerParser(Cap*CAP ,uint8_t *data, uint16_t len)//浙纺2023赛季的CAN接收协议
{
  CAP->cap_low_waring = data[0];//低电压警报
  CAP->DischargeATK = data[1];
  CAP->chassis_power = data[2]<<8|data[3];

  CAP->CapQuantity = data[4]<<8|data[5];//电压百分比
}




uint8_t limit_capquantity_rato = 56;
//(24赛季，超电改版此模块未进行测试)
void ChassisSuperCapTest(void)       //超电检测
{
 if (ZFcap_info.CapQuantity<limit_capquantity_rato)
 {
	 chassis_handle.console->supercap_cmd=SUPERCAP_OFF_CMD;	 
 }
}

Cap* ZFCAP_GetDataPointer(void) //传出一个*P指针访问值。
{
    return &ZFcap_info;             //具体内容
} 
