/* 包含头文件 ----------------------------------------------------------------*/
#include "dvc_DJmotor.h"
/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/
DJMotorInfo_t chassis_djmotor[4];
DJMotorInfo_t gimbal_djmotor_yaw;
DJMotorInfo_t gimbal_djmotor_pitch;
DJMotorInfo_t friction_wheel_djmotor[2];
DJMotorInfo_t pluck_djmotor;
DJMotorInfo_t cover_djmotor;
/* 扩展变量 ------------------------------------------------------------------*/


/* 私有函数原形 --------------------------------------------------------------*/

/* 函数体 --------------------------------------------------------------------*/
void DJMotor_yaw_Init(DJMotorInfo_t* ptr)
{
	ptr->offset_ecd = 7426;/*2549*/   //2066        //Y轴复位值
    ptr->ecd_ratio = YAW_DJDIR * YAW_DJRATIO / ENCODER_ANGLE_RATIO;
    ptr->max_relative_angle = 90;                //Y轴最大角度
    ptr->min_relative_angle = -90;                //Y轴最低角度
}

void DJMotor_pitch_Init(DJMotorInfo_t* ptr)
{
    ptr->offset_ecd = 3300;/*6944*/ //5336        //P轴复位值
    ptr->ecd_ratio = PITCH_DJDIR * PITCH_DJRATIO / ENCODER_ANGLE_RATIO;
    ptr->max_relative_angle = 38.4;               //P轴最低角度
    ptr->min_relative_angle = -10;              //P轴最高角度
}

void DJMotor_pluck_Init(DJMotorInfo_t* ptr)
{
    ptr->offset_ecd = 0;/*6944*/ //5336        //P轴复位值
    ptr->ecd_ratio = PITCH_DJDIR * PITCH_DJRATIO / ENCODER_ANGLE_RATIO;
//    ptr->max_relative_angle = 38.4;               //P轴最低角度
//    ptr->min_relative_angle = -10;              //P轴最高角度
}

static void Motor_EncoderData(DJMotorInfo_t* ptr, uint8_t data[])
{
    ptr->last_ecd = ptr->ecd;
    ptr->ecd = (uint16_t)(data[0] << 8 | data[1]);

    if (ptr->ecd - ptr->last_ecd > MOTOR_ENCODER_RANGE_HALF)
    {
        ptr->round_cnt--;
        ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd - MOTOR_ENCODER_RANGE;
    }
    else if (ptr->ecd - ptr->last_ecd < -MOTOR_ENCODER_RANGE_HALF)
    {
        ptr->round_cnt++;
        ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd + MOTOR_ENCODER_RANGE;
    }
    else
    {
        ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd;
    }

    ptr->total_ecd = ptr->round_cnt * MOTOR_ENCODER_RANGE + ptr->ecd - ptr->offset_ecd;
    /* total angle, unit is degree */
    ptr->total_angle = ptr->total_ecd / ENCODER_ANGLE_RATIO;

    ptr->speed_rpm = (int16_t)(data[2] << 8 | data[3]);                //?此固定格式作用
    ptr->given_current = (int16_t)(data[4] << 8 | data[5]);
    ptr->temperature = data[6];
    ptr->relative_angle = ptr->ecd_ratio *(float)DJMotor_RelativePosition(ptr->ecd , ptr->offset_ecd);
}

static void Motor_EncoderOffset(DJMotorInfo_t* ptr, uint8_t data[])
{
    ptr->ecd        = (uint16_t)(data[0] << 8 | data[1]);
//    ptr->offset_ecd = ptr->ecd;
}

/* 电机返回数据处理 */
void DJMotor_DataParse(DJMotorInfo_t *ptr, uint8_t data[])
{
    if (ptr == NULL)
        return;
    ptr->msg_cnt++;

//    if (ptr->msg_cnt < 50)        //50次接收到电机反馈信息，则设置初始偏移编码值，浙纺留下来的屎代码
//    {
//        Motor_EncoderOffset(ptr, data);
//        return;
//    }

    Motor_EncoderData(ptr, data);
}

int16_t DJMotor_RelativePosition(int16_t ecd, int16_t center_offset)
{
    int16_t tmp = 0;
    if (center_offset >= MOTOR_ENCODER_RANGE_HALF)
    {
        if (ecd > center_offset - MOTOR_ENCODER_RANGE_HALF)
            tmp = ecd - center_offset;
        else
            tmp = ecd + MOTOR_ENCODER_RANGE - center_offset;
    }
    else
    {
        if (ecd > center_offset + MOTOR_ENCODER_RANGE_HALF)
            tmp = ecd - MOTOR_ENCODER_RANGE - center_offset;
        else
            tmp = ecd - center_offset;
    }
    return tmp;
}

/* 发送电机数据 */
void Motor_SendMessage(CAN_Object_t *obj, uint32_t std_id, int16_t cur1, int16_t cur2, int16_t cur3, int16_t cur4)
{
    uint8_t TxData[8] = {0};
    TxData[0] = (uint8_t)(cur1 >> 8);
    TxData[1] = (uint8_t)cur1;
    TxData[2] = (uint8_t)(cur2 >> 8);
    TxData[3] = (uint8_t)cur2;
    TxData[4] = (uint8_t)(cur3 >> 8);
    TxData[5] = (uint8_t)cur3;
    TxData[6] = (uint8_t)(cur4 >> 8);
    TxData[7] = (uint8_t)cur4;
    BSP_CAN_TransmitData(obj, std_id, TxData, 8);
}

void Motor_QuicklySetID(CAN_Object_t *obj)
{
    uint8_t TxData[8] = {0};
    BSP_CAN_TransmitData(obj, 0x700, TxData, 8);
}

DJMotorInfo_t* ChassisdjMotor_Pointer(uint8_t i)
{
    return &chassis_djmotor[i];
}

DJMotorInfo_t* GimbaldjMotorYaw_Pointer(void)
{
    return &gimbal_djmotor_yaw;
}

DJMotorInfo_t* GimbaldjMotorPitch_Pointer(void)
{
    return &gimbal_djmotor_pitch;
}

DJMotorInfo_t* FrictionWheeldjMotor_1_Pointer(void)
{
    return &friction_wheel_djmotor[0];
}

DJMotorInfo_t* FrictionWheeldjMotor_2_Pointer(void)
{
    return &friction_wheel_djmotor[1];
}

DJMotorInfo_t* PluckdjMotor_Pointer(void)
{
    return &pluck_djmotor;
}

DJMotorInfo_t* MagazinedjMotor_Pointer(void)
{
    return &cover_djmotor;
}