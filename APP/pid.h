#ifndef _PID_H
#define _PID_H

#include "IQmathLib.h"

// PI控制器结构体，全部定点 Q24（GLOBAL_Q = 24）
// 量级检查：电压环 Kp=0.0585、误差最大 51V；电流环 Kp=0.8411、误差最大 8A，
//           乘积都不超过 6.8，Q24 的 ±128 装得下
typedef struct
{
    _iq Ref;        // 设定值(目标)
    _iq Fdb;        // 反馈值(采样)
    _iq Err;        // 误差 = Ref - Fdb
    _iq Kp;         // 比例系数
    _iq KiTs;       // 积分增益，已经把采样周期折进去（Ki*Ts），中断里只乘一次
    _iq Integral;   // 积分累加器
    _iq Out;        // PI输出
    _iq OutMax;     // 输出上限
    _iq OutMin;     // 输出下限
    _iq IntMax;     // 积分上限
    _iq IntMin;     // 积分下限
}PI_TypeDef;

// 全局PI结构体实例
extern PI_TypeDef PI_V;  // 备用，本工程电压外环用准PR，见 pr.h
extern PI_TypeDef PI_I;  // 电流内环（本工程为电容电流内环）

// 函数声明
extern _iq PI_Calc(PI_TypeDef *pi);

#endif // _PID_H
