#include "dvc_buzzer.h"

/* 私有变量 ------------------------------------------------------------------*/
static uint8_t beep_times;
static uint16_t beep_on_time = BEEP_ON_TIME;
static uint16_t beep_off_time = BEEP_OFF_TIME;
/* IO对象结构 */
GPIO_Object_t buzzer_gpio;

/* 函数体 --------------------------------------------------------------------*/

void Buzzer_Init(void) {
    BSP_GPIO_NewObject_PWM(&buzzer_gpio, &htim4, TIM_CHANNEL_3);
}

/**
  * @brief          设置蜂鸣器的音调和音量
  * @param[in]      tune PWM自动重装载值(ARR)，决定音调频率（值越小频率越高）
  * @param[in]      value PWM比较值(CCR)，决定占空比（音量大小）
  * @retval         void
  */
void Buzzer_SetBeep(uint16_t tune, uint16_t value)
{
    ((TIM_HandleTypeDef *)buzzer_gpio.handle)->Instance->ARR = tune;
    BSP_GPIO_SetPwmValue(&buzzer_gpio, value);     //将&buzzer_gpio这个地址内的数传到obj内
}


/**
  * @brief          蜂鸣器鸣叫处理函数，控制蜂鸣器的鸣叫次数和节奏（鸣叫时长和间隔时长
  * @param[in]
  * @retval         void
  */
void BeepHandler(void)
{
    static uint32_t beep_tick;
    static uint32_t times_tick;
    static uint8_t times;

    uint32_t time_now = BSP_GetTime_ms();
    /* The beep works after the system starts 3s */
    if (time_now / 1000 < 3)
    {
        return;
    }

    if (time_now - beep_tick > BEEP_PERIOD)
    {
        times = beep_times;
        beep_tick = time_now;
        times_tick = time_now;
    }
    else if (times != 0)
    {
        if (time_now - times_tick < beep_on_time)
        {
            Buzzer_SetBeep(BEEP_TUNE_VALUE, BEEP_CTRL_VALUE);
        }
        else if (time_now - times_tick < beep_on_time + beep_off_time)
        {
            Buzzer_SetBeep(0, 0);
        }
        else
        {
            times--;
            times_tick = time_now;
        }
    }
}


/**
  * @brief          设置蜂鸣器鸣叫的次数
  * @param[in]      times 为要鸣叫的次数
  * @retval         void
  */
void BeepTimesSet(uint8_t times)
{
    beep_times = times;
}



/**
  * @brief          设置蜂鸣器每次鸣叫的持续时间和间隔时间
  * @param[in]      on_time 单次鸣叫的持续时间
  * @param[in]      off_time 两次鸣叫之间的间隔时间
  * @retval         void
  */
void BeepTimeSet_ON_OFF(uint16_t on_time, uint16_t off_time)
{
    beep_on_time = on_time;
    beep_off_time = off_time;

}


