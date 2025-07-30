/**
* @file alg_pid_capi.cpp
 * @author as17asj
 * @brief pid对象的C语言接口
 * @date 2025-07-25
 *
 */
#include "alg_pid.h"

// 初始化 PID 控制器
void PID_Init(Class_PID *pid,
              float K_P, float K_I, float K_D,
              float K_F, float I_Out_Max, float Out_Max,
              float D_T, float Dead_Zone,
              float I_Variable_Speed_A, float I_Variable_Speed_B,
              float I_Separate_Threshold, Enum_PID_D_First D_First) {
    if (pid) {
        pid->Init(  K_P,  K_I,  K_D,
           K_F,  I_Out_Max,  Out_Max,
           D_T,  Dead_Zone,
           I_Variable_Speed_A,  I_Variable_Speed_B,
           I_Separate_Threshold,  D_First);
    }
}

// 获取积分误差
float PID_Get_Integral_Error(Class_PID *pid) {
    return pid->Get_Integral_Error();  // 假设 integral_error 是 Class_PID 的一个成员
}

// 获取输出
float PID_Get_Out(Class_PID *pid) {
    return pid->Get_Out();  // 假设 output 是 Class_PID 的一个成员
}

// 设置比例系数
void PID_Set_K_P(Class_PID *pid, float K_P) {
    pid->Set_K_P( K_P);
}

// 设置积分系数
void PID_Set_K_I(Class_PID *pid, float K_I) {
    if (pid) {
        pid->Set_K_I(K_I);
    }
}

// 设置微分系数
void PID_Set_K_D(Class_PID *pid, float K_D) {
    if (pid) {
        pid->Set_K_I(K_D);
    }
}

// 设置前馈系数
void PID_Set_K_F(Class_PID *pid, float K_F) {
    if (pid) {
        pid->Set_K_F(K_F);
    }
}

// 设置积分输出最大值
void PID_Set_I_Out_Max(Class_PID *pid, float I_Out_Max) {
    if (pid) {
        pid->Set_I_Out_Max(I_Out_Max);
    }
}

// 设置输出最大值
void PID_Set_Out_Max(Class_PID *pid, float Out_Max) {
    if (pid) {
        pid->Set_Out_Max(Out_Max);
    }
}

// 设置积分速度 A
void PID_Set_I_Variable_Speed_A(Class_PID *pid, float I_Variable_Speed_A) {
    if (pid) {
        pid->Set_I_Variable_Speed_A(I_Variable_Speed_A);
    }
}

// 设置积分速度 B
void PID_Set_I_Variable_Speed_B(Class_PID *pid, float I_Variable_Speed_B) {
    if (pid) {
        pid->Set_I_Variable_Speed_B(I_Variable_Speed_B) ;
    }
}

// 设置积分分离阈值
void PID_Set_I_Separate_Threshold(Class_PID *pid, float I_Separate_Threshold) {
    if (pid) {
        pid->Set_I_Separate_Threshold(I_Separate_Threshold);
    }
}

// 设置目标值
void PID_Set_Target(Class_PID *pid, float Target) {
    if (pid) {
        pid->Set_Target(Target);
    }
}

// 设置当前值
void PID_Set_Now(Class_PID *pid, float Now) {
   if (pid) {
       pid->Set_Now(Now);
   }
}

// 设置积分误差
void PID_Set_Integral_Error(Class_PID *pid, float Integral_Error) {
   if (pid) {
       pid->Set_Integral_Error(Integral_Error);
   }
}

// 定时调整回调
void PID_TIM_Adjust_PeriodElapsedCallback(Class_PID *pid) {
    if (pid) {
        pid->TIM_Adjust_PeriodElapsedCallback();
    }
}