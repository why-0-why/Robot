// //
// // Created by Administrator on 25-7-30.
// //
// #include "mdl_Gimbal.h"
//
// AimMessage_t aim_handle;
// GimbalHandle_t gimbal_handle; //云台处理结构体数值
//
// void GimbalAppConfig(void)
// {
// 	aim_handle.PITCH_RATIO = 0.020f;//STF new
// 	aim_handle.YAW_RATIO = 0.040f;//STF new
//     gimbal_handle.console     = Console_Pointer();                   //云台控制数据更新
//     gimbal_handle.imu  = IMU_GetDataPointer();                       //陀螺仪数据更新
//     gimbal_handle.gimbal_can  = &can1_obj;                           //云台CAN1通道
// 	  gimbal_handle.ctrl_mode = GIMBAL_INIT;                            //云台控制模式为初始化
// 	  gimbal_handle.yaw_motor.motor_info = GimbalMotorYaw_Pointer(); //Y轴电机信息更新
// 	  gimbal_handle.pitch_motor.motor_info = GimbalMotorPitch_Pointer();//P轴电机信息更新
// 	  gimbal_handle.yaw_motor.offset_ecd = 5543;/*2549*/   //2066        //Y轴复位值
//     gimbal_handle.pitch_motor.offset_ecd = 6865;/*6944*/ //5336        //P轴复位值
//     gimbal_handle.yaw_motor.ecd_ratio = YAW_MOTO_POSITIVE_DIR * YAW_REDUCTION_RATIO / ENCODER_ANGLE_RATIO;
//     gimbal_handle.pitch_motor.ecd_ratio = PITCH_MOTO_POSITIVE_DIR * PITCH_REDUCTION_RATIO / ENCODER_ANGLE_RATIO;
//     gimbal_handle.yaw_motor.max_relative_angle = 120;                //Y轴最大角度
//     gimbal_handle.yaw_motor.min_relative_angle = -90;                //Y轴最低角度
//     gimbal_handle.pitch_motor.max_relative_angle = 38.4;               //P轴最低角度
//     gimbal_handle.pitch_motor.min_relative_angle = -39.8;              //P轴最高角度
//     pid_init(&gimbal_handle.yaw_motor.pid.outer_pid, POSITION_PID, 2000.0f, 0.0f,                //Y轴外环PID
//              30.0f, 0.0f, 0.0f);  //30    /*40,0.0,3.5*///30 0 5.5 30 0.15 15
//     pid_init(&gimbal_handle.yaw_motor.pid.inter_pid, POSITION_PID, GM6020_MOTOR_MAX_CURRENT, 3000.0f,      //Y轴内环PID
//              60.0f, 0.0f, 0.0f);  //60    /*60,0.1,12*/ //75 0.03 35  70 0.2 30         //P调大越硬
//     pid_init(&gimbal_handle.pitch_motor.pid.outer_pid, POSITION_PID, 20000.0f, 0.0f,                       //P轴外环PID
//              60.0f, 0.0f, 20.0f);          /*40,0,98有摄像头*/   /*40,0,90无摄像头*/     /*40,0,23 */
//     pid_init(&gimbal_handle.pitch_motor.pid.inter_pid, POSITION_PID, GM6020_MOTOR_MAX_CURRENT, 6000.0f,    //P轴内环PID
//              90.0f, 0.1f, 30.0f);         /*38,0.1,0有摄像头*/    /*38,0.1,0无摄像头*/    /*80,0.1,0*/     //75 0.0 0.0
//  //                     离线事件                                 离线返回码                            叫声次数
//     /*--------------------event------------------------|-------enable-------|-offline time-|-beep_times-*/           //OfflineHandle离线处理
//     OfflineHandle_Init(OFFLINE_GIMBAL_PITCH,            OFFLINE_ERROR_LEVEL,       100,         5);        //云台P轴
//     OfflineHandle_Init(OFFLINE_GIMBAL_YAW,              OFFLINE_ERROR_LEVEL,       100,         4);        //云台Y轴
//     OfflineHandle_Init(OFFLINE_FRICTION_WHEEL_MOTOR1,   OFFLINE_ERROR_LEVEL,       100,         1);        //一声，摩擦轮
//     OfflineHandle_Init(OFFLINE_FRICTION_WHEEL_MOTOR2,   OFFLINE_ERROR_LEVEL,       100,         2);        //两声，摩擦轮
//     OfflineHandle_Init(OFFLINE_PLUCK_MOTOR,             OFFLINE_ERROR_LEVEL,       100,         0);        //三声，云台P轴
// //    OfflineHandle_Init(OFFLINE_COVER_MOTOR,             OFFLINE_ERROR_LEVEL,       100,         0);        //
//     OfflineHandle_Init(OFFLINE_REFEREE_SYSTEM,          OFFLINE_WARNING_LEVEL,     100,         0);  //
//     OfflineHandle_Init(OFFLINE_CHASSIS_INFO,            OFFLINE_WARNING_LEVEL,     100,         0);  //
// 		//OfflineHandle_Init(OFFLINE_INFRARED_INFO,           OFFLINE_WARNING_LEVEL,     100,         0);
//     OfflineHandle_Init(OFFLINE_DBUS,                    OFFLINE_WARNING_LEVEL,     100,         0);  //
// //    OfflineHandle_Init(OFFLINE_VISION_INFO,             OFFLINE_WARNING_LEVEL,     500,         0);
//     OfflineHandle_Init(OFFLINE_KEYBOARDINFO,            OFFLINE_WARNING_LEVEL,     100,         6);        //图传链路键鼠数据
//
//     Comm_TransmitInit(&gimbal_tx_handle, gimbal_tx_fifo_buffer, GIMBAL_CHASSIS_DATA_FIFO_SIZE, CAN1_UploadDataHook);
//     Comm_ReceiveInit(&gimbal_rx_handle, USER_PROTOCOL_HEADER_SOF, gimbal_rx_fifo_buffer, GIMBAL_CHASSIS_DATA_FIFO_SIZE, UserProtocol_ParseHandler);
//     SoftwareTimerRegister(GimbalInfoUploadCallback, (void*)NULL, GIMBAL_UPLOAD_TIMER_PERIOD);
//
//     Comm_TransmitInit(&vision_tx_handle, vision_tx_fifo_buffer, VISION_DATA_FIFO_SIZE, Vision_UploadDataHook);                                     //视觉发送：视觉发送地址，视觉接受信息缓冲区，视觉数据信息大小，视觉数据上载。
//     Comm_ReceiveInit(&vision_rx_handle, VISION_PROTOCOL_HEADER_SOF, vision_rx_fifo_buffer, VISION_DATA_FIFO_SIZE, VisionProtocol_ParseHandler);    //视觉接收: 视觉接受地址，视觉协议帧头起始，视觉接受信息缓冲区，视觉数据信息大小，视觉协议、
//     SoftwareTimerRegister(Vision_RobotInfoUploadCallback, (void*)NULL, 50);                                            //软时钟寄存器：视觉信息回调
//
//     Comm_ReceiveInit(&referee_rx_handle, REFEREE_SYSTEM_HEADER_SOF, referee_rx_fifo_buffer, REFEREE_SYSTEM_FIFO_SIZE, RefereeSystem_ParseHandler);
//
//     BSP_UART_SetRxCallback(&dbus_obj, DBUS_ReceiveCallback);
//     BSP_UART_SetRxCallback(&com1_obj, COM1_ReceiveCallback);
//     BSP_UART_SetRxCallback(&com2_obj, COM2_ReceiveCallback);
//     BSP_CAN_SetRxCallback(&can1_obj, CAN1_ReceiveCallback);
//     BSP_CAN_SetRxCallback(&can2_obj, CAN2_ReceiveCallback);
// }