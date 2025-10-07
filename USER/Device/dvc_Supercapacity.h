//
// Created by Administrator on 25-8-5.
//

#ifndef DVC_SUPERCAPACITY_H
#define DVC_SUPERCAPACITY_H

/* 包含头文件 ----------------------------------------------------------------*/
#include "drv_can.h"
/* 类型定义 ------------------------------------------------------------------*/
typedef struct {
    float in_v;       
    float cap_v;       
    float in_c;        
    uint8_t cap_percent;  
    uint8_t in_power;    
} Cap_power;//张泓学长的结构体

typedef struct
{
    float cap_low_waring;
    float DischargeATK;
    float chassis_power;
    float CapQuantity;
}Cap;//浙纺2023赛季超电

/* 宏定义 --------------------------------------------------------------------*/
/*---------------↓ 超电ID ↓---------------*/
#define	Super_Power_Message_ID						0x026
#define LONGSuper_Power_Message_ID                  0x211
/* 扩展变量 ------------------------------------------------------------------*/
extern Cap_power cap_info;
/* 函数声明 ------------------------------------------------------------------*/
Cap_power* CAP_GetDataPointer(void); //传出一个*P指针访问值。
void SuperCap_SendMessage(CAN_Object_t* obj);
void LONG_CAP_PowerParser(Cap_power *received_data, uint8_t *can_receive_data, uint16_t len);  //张弘学长版超电
void CAP_PowerParser(Cap*CAP ,uint8_t *data, uint16_t len);//浙纺2023赛季的CAN接收协议
Cap* ZFCAP_GetDataPointer(void);//传出一个*P指针访问值。
void ChassisSuperCapTest(void);       //超电检测

#endif //DVC_SUPERCAPACITY_H
