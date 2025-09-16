//
// Created by ZJH on 25-8-5.
//
/****************************************************************************
 *  Copyright (C) 2020 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include "alg_ramp.h"

/**
  * @brief     ramp filter initialize
  * @param[in]
  * @retval    void
  */
void ramp_v0_init(ramp_v0_t *ramp, int32_t scale)
{
    ramp->count = 0;
    ramp->scale = scale;
}

/**
  * @brief     caculate output of ramp filter
  * @param[in] ramp: a ramp filter pointer
  * @retval    output
  */
float ramp_v0_calculate(ramp_v0_t *ramp)
{
    if (ramp->scale <= 0)
    {
        return 0;
    }

    if (ramp->count++ >= ramp->scale)
    {
        ramp->count = ramp->scale;
    }

    ramp->out = ramp->count / ((float)ramp->scale);
    return ramp->out;
}

float previous_value = 0.0f;
float previous_time = 0.0f;

// 计算当前值与之前值的导函数
// 输入: 当前值和当前时间
// 输出: 当前值的导数
float calculate_derivative(float current_value, float current_time)
{
    // 计算时间间隔，转换为秒
    float delta_time = (current_time - previous_time) / 1000.0f; // 转换为秒

    // 计算导数，防止除以零
    float derivative = 0.0f;
    if (delta_time > 0) {
        derivative = (current_value - previous_value) / delta_time;
    }

    // 更新之前的值和时间
    previous_value = current_value;
    previous_time = current_time;

    return derivative;
}
