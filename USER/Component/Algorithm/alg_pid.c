/* 包含头文件 ----------------------------------------------------------------*/
#include "alg_pid.h"
#include <math.h>

/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/

/* 私有函数原形 --------------------------------------------------------------*/

/* 函数体 --------------------------------------------------------------------*/
static void abs_limit(float *a, float ABS_MAX)
{
    if (*a > ABS_MAX)
        *a = ABS_MAX;
    if (*a < -ABS_MAX)
        *a = -ABS_MAX;
}

static void pid_param_init(
    pid_t*   pid,
    int      mode,
    float    maxout,
    float    intergral_limit,
    float    kp,
    float    ki,
    float    kd)
{
    pid->pid_mode       = mode;
    pid->max_out        = maxout;
    pid->integral_limit = intergral_limit;

    pid->p = kp;
    pid->i = ki;
    pid->d = kd;

}
/**
 * @brief  在线修改 PID 参数
 * @param[in] pid  PID 结构体指针
 * @param[in] kp   比例系数
 * @param[in] ki   积分系数
 * @param[in] kd   微分系数
 * @retval 无
 */
static void pid_reset(pid_t *pid, float kp, float ki, float kd)
{
    pid->p = kp;
    pid->i = ki;
    pid->d = kd;

    pid->pout = 0;
    pid->iout = 0;
    pid->dout = 0;
    pid->out  = 0;
}

/**
  * @brief     计算增量式PID和位置式PID
  * @param[in] pid: PID控制器结构体指针
  * @param[in] get: 当前测量反馈值
  * @param[in] set: 目标设定值
  * @retval    PID计算输出值
  */
float pid_calc(pid_t *pid, float get, float set)
{
    pid->get = get;
    pid->set = set;
    pid->err[NOW] = set - get;

    if ((pid->input_max_err != 0) && (fabs(pid->err[NOW]) > pid->input_max_err))
        return 0;

    if (pid->pid_mode == POSITION_PID) //position PID
    {
        pid->pout = pid->p * pid->err[NOW];
        pid->iout += pid->i * pid->err[NOW];
        pid->dout = pid->d * (pid->err[NOW] - pid->err[LAST]);

        abs_limit(&(pid->iout), pid->integral_limit);
        pid->out = pid->pout + pid->iout + pid->dout;
        abs_limit(&(pid->out), pid->max_out);
    }
    else if (pid->pid_mode == DELTA_PID) //delta PID
    {
        pid->pout = pid->p * (pid->err[NOW] - pid->err[LAST]);
        pid->iout = pid->i * pid->err[NOW];
        pid->dout = pid->d * (pid->err[NOW] - 2 * pid->err[LAST] + pid->err[LLAST]);

        pid->out += pid->pout + pid->iout + pid->dout;
        abs_limit(&(pid->out), pid->max_out);
    }

    pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST]  = pid->err[NOW];


    if ((pid->output_deadband != 0) && (fabs(pid->out) < pid->output_deadband))
        return 0;
    else
        return pid->out;
}

/**
  * @brief     复位PID控制器
  * @retval    无
  */
void pid_clear(pid_t *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->err[NOW] = pid->err[LAST] = pid->err[LLAST] = 0.0f;
    pid->out = pid->pout = pid->iout = pid->dout = 0.0f;
    pid->get = pid->set = 0.0f;
}

/**
  * @brief     初始化PID所有参数
  * @retval    none
  */
void pid_init(
    pid_t*   pid,
    int   mode,
    float maxout,
    float intergral_limit,

    float kp,
    float ki,
    float kd)
{
    pid_param_init(pid, mode, maxout, intergral_limit, kp, ki, kd);
    pid_reset(pid, kp, ki, kd);
}

/**
  * @brief     双环PID计算函数
  * @param[in] dpid      : 双环PID结构体指针
  * @param[in] outer_ref : 外环（位置/角度等）目标值
  * @param[in] outer_fdb : 外环（位置/角度等）反馈值
  * @param[in] inter_fdb : 内环（速度/电流等）反馈值
  * @retval    最终输出值（内环PID输出）
  */
float DoublePID_Calc(Double_PID_t* dpid, float outer_ref, float outer_fdb, float inter_fdb)
{
    dpid->outer_ref = outer_ref;
    dpid->outer_fdb = outer_fdb;
    pid_calc(&dpid->outer_pid, dpid->outer_fdb, dpid->outer_ref);
    dpid->inter_ref = dpid->outer_pid.out;
    dpid->inter_fdb = inter_fdb;
    pid_calc(&dpid->inter_pid, dpid->inter_fdb, dpid->inter_ref);
    return dpid->inter_pid.out;
}
