/*
 * pid.c
 *
 * 增量式带抗饱和的 PI 调节器（全定点 Q24）
 * 写法沿用 Boost 工程：结构体 + PI_Calc 函数
 *
 * 积分项的定点处理：
 *   原来的写法是 Ki * err * Ts，每拍乘两次。
 *   定点下更划算也更准的做法是把采样周期折进系数：KiTs = Ki*Ts，
 *   初始化时由 dcac_cfg.h 的 KI_I_Ts 算好（编译期常量），
 *   中断里只做一次 _IQmpy(pi->KiTs, err)。
 *   这样做还避免了 err*Ts 先算出来太小的问题：Ts = 50us 在 Q24 里只有
 *   838 个刻度，先乘 Ts 会把误差的有效位数吃掉十几位。
 *
 * 采样周期仍然来自 dcac_cfg.h 的 Ts_S（20kHz -> 50us），
 * 换载波频率时 Ki*Ts 会自动跟着变，不用手改
 */

#include "pid.h"
#include "dcac_cfg.h"

// 全局 PI 结构体定义
PI_TypeDef PI_V;
PI_TypeDef PI_I;

// 限幅宏（定点就是整数，直接比较）
#define __clamp(val, min, max) ((val)>(max)?(max):((val)<(min)?(min):(val)))

/*===========================================================================
 * 函数名：PI_Calc
 * 说明：PI 计算核心。先算候选积分和候选输出，只有输出没饱和才提交积分
 *       乘法全部用 _IQmpy，直接写 a*b 得到的是右移 24 位之前的 Q48 值
 *===========================================================================*/
_iq PI_Calc(PI_TypeDef *pi)
{
    _iq err = pi->Ref - pi->Fdb;                    // 定点加减直接用整数运算
    _iq p_term = _IQmpy(pi->Kp, err);               // 比例项

    // 候选积分：KiTs 已经含采样周期，这里只乘一次
    _iq i_temp = pi->Integral + _IQmpy(pi->KiTs, err);
    i_temp = __clamp(i_temp, pi->IntMin, pi->IntMax);

    // 候选未限幅输出
    _iq out_temp = p_term + i_temp;

    // 抗饱和：候选输出饱和且误差同向，则不更新积分
    if (!((out_temp >= pi->OutMax && err > 0) ||
          (out_temp <= pi->OutMin && err < 0)))
    {
        pi->Integral = i_temp;
    }

    pi->Err = err;

    pi->Out = p_term + pi->Integral;
    pi->Out = __clamp(pi->Out, pi->OutMin, pi->OutMax);

    return pi->Out;
}
