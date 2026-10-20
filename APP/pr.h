/*
 * pr.h
 *
 * 准比例谐振(PR)调节器：Tustin 离散系数设计 + 差分方程计算
 *
 * 连续域（准 PR，带截止频率 wc 抑制谐振尖峰）：
 *                2*wc*s
 *   G(s) = Kp + Kr*------------------------
 *                s^2 + 2*wc*s + w0^2
 *
 * Tustin 离散（Ts = 1/fs）后：
 *   y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
 *
 * 定点化（GLOBAL_Q = 24）：
 *   系数 b0/b1/b2/a1/a2 都是 ±2 以内的小数，Q24 精度足够；
 *   运行量 x = 电压误差 ±51V、y = 电容电流给定 ±8A，都在 Q24 的 ±128 以内；
 *   a1 ≈ -1.9988、y1 最大 8A，a1*y1 ≈ 16，仍在 ±128 以内，
 *   而且 _IQmpy 用 64 位中间结果，不会溢出。
 *
 * 用法和 pid.c 的 PI_Calc 一样：结构体 + Calc 函数
 *   PR_V.Ref = 给定; PR_V.Fdb = 反馈; i_ref = PR_Calc(&PR_V);
 */

#ifndef PR_H
#define PR_H

#include "IQmathLib.h"

// 准 PR 结构体，全部 Q24
typedef struct
{
    // 设计输入
    _iq Kp;           // 比例增益
    _iq Kr;           // 谐振增益

    // Tustin 离散系数（a0 归一化为 1），Q24
    // w0、wc、Ts 在 PR_Design 之前就已经合成 r = w0*Ts/2、q = wc*Ts/2，
    // 由 dcac_cfg.h 按编译期常量算好，运行期不保留这三个量
    _iq b0;
    _iq b1;
    _iq b2;
    _iq a1;
    _iq a2;

    // 运行量
    _iq Ref;        // 设定值
    _iq Fdb;        // 反馈值
    _iq Err;        // 误差 = Ref - Fdb
    _iq X1;         // 误差历史 x[n-1]
    _iq X2;         // 误差历史 x[n-2]
    _iq Y1;         // 输出历史 y[n-1]
    _iq Y2;         // 输出历史 y[n-2]
    _iq Out;        // 输出
    _iq OutMax;     // 输出上限
    _iq OutMin;     // 输出下限
} PR_TypeDef;

// 全局 PR 结构体实例
extern PR_TypeDef PR_V;     // 电压外环

// 按 Tustin 法计算 b0/b1/b2/a1/a2
//   r = w0*Ts/2, q = wc*Ts/2，都是 Q24（w0 超出 Q24 范围，
//   已经在 dcac_cfg.h 里折成 r、q，不会再出现 314 这种数）
void PR_Design(PR_TypeDef *p, _iq Kp, _iq Kr, _iq r, _iq q);

// 差分方程计算，返回限幅后的输出
_iq PR_Calc(PR_TypeDef *p);

// 两个 Q24 限幅
_iq PR_Sat(_iq v, _iq lo, _iq hi);

#endif
