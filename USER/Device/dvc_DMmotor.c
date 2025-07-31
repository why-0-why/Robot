/* 包含头文件 ----------------------------------------------------------------*/
#include "dvc_DMmotor.h"

#include "../Module/motor.h"
/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/
Motor_Inf chassis_dmmotor[4];
Motor_Inf gimbal_dmmotor_yaw;
Motor_Inf gimbal_dmmotor_pitch;
Motor_Inf friction_wheel_dmmotor[2];
Motor_Inf pluck_dmmotor;
Motor_Inf cover_dmmotor;

 Motor_MIT_Data_t MIT;
/* 扩展变量 ------------------------------------------------------------------*/

//DM4310变量
Motor_Inf mtr;
static CAN_TxHeaderTypeDef   Tx_Header;
uint8_t dm43_can_send_data[8];
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
/* 私有函数原形 --------------------------------------------------------------*/

/* 函数体 --------------------------------------------------------------------*/
int float_to_uint(float x, float x_min, float x_max, int bits)
{
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}


float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

// 计算电机的相对位置
// 参数：rad - 当前角度（以弧度为单位）
// 参数：center_offset - 中心偏移量（以弧度为单位）
// 返回值：相对位置（以弧度为单位）
float DMMotor_RelativePosition(float rad, float center_offset)
{
    float tmp = 0;
    if (center_offset >= PI)
    {
        if (rad > center_offset - PI)
            tmp = rad - center_offset;
        else
            tmp = rad + 2*PI - center_offset;
    }
    else
    {
        if (rad > center_offset + PI)
            tmp = rad - 2*PI - center_offset;
        else
            tmp = rad - center_offset;
    }
    return tmp;
}


// 计算电机的绝对位置
float DMMotor_CalculateAbsolutePosition(float tmp, float center_offset)
{
    float rad = 0;
    if (center_offset >= PI)
    {
        if (tmp >= 0)
        {
            rad = tmp + center_offset;
        }
        else // tmp < 0
        {
            rad = tmp + center_offset + 2 * PI;
        }
    }
    else // center_offset < PI
    {
        if (tmp >= 0)
        {
            rad = tmp + center_offset;
        }
        else // tmp < 0
        {
            rad = tmp + center_offset - 2 * PI;
        }
    }
    return rad;
}


void DMMotor_DataParse(Motor_Inf* dmmotor,uint8_t data[])
{
       dmmotor->id = (data[0])&0x0F;
       dmmotor->state = (data[0])>>4;
       dmmotor->p_int=(data[1]<<8)|data[2];
       dmmotor->v_int=(data[3]<<4)|(data[4]>>4);
       dmmotor->t_int=((data[4]&0xF)<<8)|data[5];
       dmmotor->pos = uint_to_float(dmmotor->p_int, P_MIN, P_MAX, 16); // (-12.5,12.5)
       dmmotor->vel = uint_to_float(dmmotor->v_int, V_MIN, V_MAX, 12); // (-45.0,45.0)
       dmmotor->toq = uint_to_float(dmmotor->t_int, T_MIN, T_MAX, 12);  // (-18.0,18.0)
       dmmotor->Tmos = (float)(data[6]);
       dmmotor->Tcoil = (float)(data[7]);

	    // 从弧度制转换为角度制
			 dmmotor->pos_deg = dmmotor->pos * (180.0f / PI);

    // 从弧度/秒转换为转速/分钟
			 dmmotor->vel_rpm = dmmotor->vel * (60.0f / (2 * PI));
}


void DMMotor_Init(Motor_Inf* ptr)
{
	ptr->offset_pos= 2.379;/*2549*/   //2066        //复位弧度
    ptr->max_relative_angle = 30;                //P轴最大角度
    ptr->min_relative_angle = -60;                //P轴最低角度
    ptr->dir = -1.0;                            //默认方向为反（上正下负）
//    start_motor(&hcan1,CAN_ID);
    MIT.p_int = 0;						//位置
	  MIT.v_int = 0;            //速度
		MIT.kp_int = 50;						//p增益
		MIT.kd_int = 1.1;
		MIT.t_int = -0.0001164*powf(gimbal_motor_pitch.relative_angle,2) + 0.02533*gimbal_motor_pitch.relative_angle+ 0.4923;	          //前馈扭矩
}

void ctrl_motor(CAN_Object_t *obj,uint16_t id, Motor_MIT_Data_t* _dm43_mit_t) //MIT模式
{

//    uint32_t send_mail_box;
    uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
    uint8_t TxData[8] = {0};

    pos_tmp = float_to_uint(_dm43_mit_t->p_int, P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(_dm43_mit_t->v_int, V_MIN, V_MAX, 12);
    kp_tmp  = float_to_uint(_dm43_mit_t->kp_int, KP_MIN, KP_MAX, 12);
    kd_tmp  = float_to_uint(_dm43_mit_t->kd_int, KD_MIN, KD_MAX, 12);
    tor_tmp = float_to_uint(_dm43_mit_t->t_int, T_MIN, T_MAX, 12);

//	Tx_Header.StdId=id;
//	Tx_Header.IDE=CAN_ID_STD;
//	Tx_Header.RTR=CAN_RTR_DATA;
//	Tx_Header.DLC=0x08;
	TxData[0] = (pos_tmp >> 8);
	TxData[1] = pos_tmp;
	TxData[2] = (vel_tmp >> 4);
	TxData[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	TxData[4] = kp_tmp;
	TxData[5] = (kd_tmp >> 4);
	TxData[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	TxData[7] = tor_tmp;

	BSP_CAN_TransmitData(obj, id, TxData, 8);
}

void ctrl_motor2(CAN_Object_t *obj,uint16_t id, float _pos, float _vel) //位置模式
{
//	    uint32_t send_mail_box;
	uint8_t *pbuf,*vbuf;
	pbuf=(uint8_t*)&_pos;
	vbuf=(uint8_t*)&_vel;
    uint8_t TxData[8] = {0};
//	hcan->pTxMsg->StdId = id;
//	hcan->pTxMsg->IDE = CAN_ID_STD;
//	hcan->pTxMsg->RTR = CAN_RTR_DATA;
//	hcan->pTxMsg->DLC = 0x08;
	TxData[0] = *pbuf;
	TxData[1] = *(pbuf+1);
	TxData[2] = *(pbuf+2);
	TxData[3] = *(pbuf+3);
	TxData[4] = *vbuf;
	TxData[5] = *(vbuf+1);
	TxData[6] = *(vbuf+2);
	TxData[7] = *(vbuf+3);
			// HAL_CAN_AddTxMessage(&hcan1, &Tx_Header, TxData, &send_mail_box);//不要直接调用，否则出现CAN总线错误HAL_CAN_ERROR_PARAM
    BSP_CAN_TransmitData(obj, id, TxData, 8);

}

void ctrl_motor3(CAN_HandleTypeDef* hcan,uint16_t id, float _vel) //速度模式
{
//uint8_t *vbuf;
//	vbuf=(uint8_t*)&_vel;
//
//	hcan->pTxMsg->StdId = id;
//	hcan->pTxMsg->IDE = CAN_ID_STD;
//	hcan->pTxMsg->RTR = CAN_RTR_DATA;
//	hcan->pTxMsg->DLC = 0x04;
//	hcan->pTxMsg->Data[0] = *vbuf;
//	hcan->pTxMsg->Data[1] = *(vbuf+1);
//	hcan->pTxMsg->Data[2] = *(vbuf+2);
//	hcan->pTxMsg->Data[3] = *(vbuf+3);
//
//	HAL_CAN_Transmit(hcan, 100);
}

void start_motor(CAN_Object_t *obj,uint16_t id)
{
//    uint32_t send_mail_box;

//	Tx_Header.StdId=id;
//	Tx_Header.IDE=CAN_ID_STD;
//	Tx_Header.RTR=CAN_RTR_DATA;
//	Tx_Header.DLC=0x08;
	dm43_can_send_data[0] = 0xFF;
	dm43_can_send_data[1] = 0xFF;
	dm43_can_send_data[2] = 0xFF;
	dm43_can_send_data[3] = 0xFF;
	dm43_can_send_data[4] = 0xFF;
	dm43_can_send_data[5] = 0xFF;
	dm43_can_send_data[6] = 0xFF;
	dm43_can_send_data[7] = 0xFC;

//	HAL_CAN_AddTxMessage(&hcan1, &Tx_Header, dm43_can_send_data, &send_mail_box);//不要直接调用，否则出现CAN总线错误HAL_CAN_ERROR_PARAM
	BSP_CAN_TransmitData(obj, id, dm43_can_send_data, 8);
}

void lock_motor(CAN_Object_t *obj,uint16_t id)
{
//    uint32_t send_mail_box;

//	Tx_Header.StdId=id;
//	Tx_Header.IDE=CAN_ID_STD;
//	Tx_Header.RTR=CAN_RTR_DATA;
//	Tx_Header.DLC=0x08;
	dm43_can_send_data[0] = 0xFF;
	dm43_can_send_data[1] = 0xFF;
	dm43_can_send_data[2] = 0xFF;
	dm43_can_send_data[3] = 0xFF;
	dm43_can_send_data[4] = 0xFF;
	dm43_can_send_data[5] = 0xFF;
	dm43_can_send_data[6] = 0xFF;
	dm43_can_send_data[7] = 0xFD;

//	HAL_CAN_AddTxMessage(&hcan1, &Tx_Header, dm43_can_send_data, &send_mail_box);//不要直接调用，否则出现CAN总线错误HAL_CAN_ERROR_PARAM
	BSP_CAN_TransmitData(obj, id, dm43_can_send_data, 8);
}


void can_filter_init(void)
{

    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);


    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);



}


Motor_MIT_Data_t* Get_MIT_Pointer()
{
	return &MIT;
}