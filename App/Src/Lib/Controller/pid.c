/****************************************************************************
 *  Copyright (C) 2018 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of?
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.? See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

/**
  *********************** (C) COPYRIGHT 2018 DJI **********************
  * @update
  * @history
  * Version     Date              Author           Modification
  * V1.0.0      January-15-2018   ric.luo
  * @verbatim
  *********************** (C) COPYRIGHT 2018 DJI **********************
  */




#include <math.h>
#include "Inc/Lib/Controller/pid.h"

static void abs_limit(float *a, float ABS_MAX)
{
    if (*a > ABS_MAX)
        *a = ABS_MAX;
    if (*a < -ABS_MAX)
        *a = -ABS_MAX;
}

/**
  * @brief     PID 初始化函数
  * @param[in] pid: PID 结构体
  * @param[in] max_out: 最大输出
  * @param[in] intergral_limit: 积分限幅
  * @param[in] kp/ki/kd: 具体 PID 参数
  */
void pid_init(struct Pid *pid, float max_out, float intergral_limit,
              float kp, float ki, float kd)
{
    pid->integral_limit = intergral_limit;
    pid->max_output     = max_out;

    pid->p = kp;
    pid->i = ki;
    pid->d = kd;

    pid->err[NOW]=0;
    pid->err[LAST]=0;
    pid->set=0;
    pid->get=0;
    pid->pout=0;
    pid->iout=0;
    pid->dout=0;
    pid->excess=0;
}

/**
  * @brief     PID 计算函数，使用位置式 PID 计算
  * @param[in] pid: PID 结构体
  * @param[in] get: 反馈数据
  * @param[in] set: 目标数据
  * @retval    PID 计算输出
  */
float pid_calc(struct Pid *pid, float get, float set)
{
    pid->get = get;
    pid->set = set;
    pid->err[NOW] = set - get;

    pid->pout = pid->p * pid->err[NOW];
    pid->iout += pid->i * pid->err[NOW];
    pid->dout = pid->d * (pid->err[NOW] - pid->err[LAST]);

    abs_limit(&(pid->iout), pid->integral_limit);
    pid->out = pid->pout + pid->iout + pid->dout;
    abs_limit(&(pid->out), pid->max_output);

    pid->err[LAST]  = pid->err[NOW];

    return pid->out;
}


float pid_loop_calc(struct Pid *pid,float get,float set,float max_value,float min_value){
    float gap,mid;
    mid=(max_value-min_value)/2;
    gap=set-get;
    if(gap>=mid){
        gap-=max_value-min_value;
        return pid_calc(pid,-gap,0);
    }
    else if(gap<=-mid){
        gap+=max_value-min_value;
        return pid_calc(pid,-gap,0);
    }
    else{
        return pid_calc(pid,get,set);
    }

}

/**
  * @brief     PID 参数复位函数
  * @param[in] pid: PID 结构体
  * @param[in] kp/ki/kd: 具体 PID 参数
  */
void pid_reset(struct Pid *pid, float kp, float ki, float kd)
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
 * @brief 带积分分离的PID计算
 * @param pid
 * @param get
 * @param set
 * @param err_threshold 积分分离阈值
 * @return
 */
float pid_calc_KI_Separation(struct Pid* pid,float get,float set,float err_threshold)
{
    pid->get = get;
    pid->set = set;
    pid->err[NOW] = set - get;

    pid->pout = pid->p * pid->err[NOW];

    if(fabs(pid->err[NOW])>err_threshold)
    {
        pid->iout+=0;
    }
    else{
        pid->iout += pid->i * pid->err[NOW];
    }

    pid->dout = pid->d * (pid->err[NOW] - pid->err[LAST]);
    abs_limit(&(pid->iout), pid->integral_limit);
    pid->out = pid->pout + pid->iout + pid->dout;
    abs_limit(&(pid->out), pid->max_output);

    pid->err[LAST]  = pid->err[NOW];

    return pid->out;
}


float pid_calc_balance(struct Pid *pid, float get, float set,float gyro_y)
{
    pid->get = get;
    pid->set = set;
    pid->err[NOW] = set - get;

    pid->pout = pid->p * pid->err[NOW];
    pid->iout += pid->i * pid->err[NOW];
    pid->dout = pid->d * gyro_y;

    abs_limit(&(pid->iout), pid->integral_limit);
    pid->out = pid->pout + pid->iout + pid->dout;
    abs_limit(&(pid->out), pid->max_output);

    pid->err[LAST]  = pid->err[NOW];

    return pid->out;
}