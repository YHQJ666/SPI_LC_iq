/*
 * protect.h
 *
 * 硬件过流保护（不经 CPU，直达 ePWM）
 *
 * F28035 没有 CMPSS 模块，它的等价物是 3 个模拟比较器 COMP1/COMP2/COMP3
 * 加 ePWM 内部 Digital Compare：
 *
 *   COMP1A(ADCINA2 电感电流) --+
 *                              +--> COMP1 --> DCAH/DCBH --+
 *   COMP1 DAC(内部阈值)     --+                          |
 *                                                        +--> TZDCSEL 事件
 *   COMP2A(ADCINA2 电感电流) --+                          |        |
 *                              +--> COMP2 --> DCAL/DCBL --+        v
 *   COMP2 DAC(内部阈值)     --+                     TZSEL 单次/逐周期
 *                                                                  |
 *                                                                  v
 *                                                    TZCTL -> 四管直接拉低
 *
 * 整条链纯硬件，CPU 跑飞也不影响保护动作
 *
 * 注意：COMP1A 固定接在 ADCINA2 上，本工程 ADCINA2 就是电感电流，
 * 所以硬件过流保护的监视对象是电感电流（也就是开关管电流），正合适
 *
 * 定点化：阈值和换算全部用 Q24，最后一步转成 DAC 码值时用移位，
 *         因为 1024 这个数超过 Q24 的 ±128（见 protect.c）
 */

#ifndef PROTECT_H
#define PROTECT_H

#include "DSP28x_Project.h"
#include "IQmathLib.h"
#include "dcac_cfg.h"

// 保护总开关在 dcac_cfg.h 的 Protect_Enable
//   0 = 本模块所有函数直接返回，COMP / Digital Compare / Trip Zone 一概不碰
//   1 = 正常配置
// 注意：整个模块都在这个开关下面，不要单独调用里面的函数去绕过它

// 反向过流用 COMP2（反相输出）实现，需要电感电流信号同时接到
// ADCA2(COMP1A) 和 ADCA4(COMP2A)。
// 默认 0：板子只接了一路时必须保持 0，否则 COMP2A 悬空约 0V，
// 反相输出恒为高 -> TZ 会把四管一直封死，表现为完全没有 SPWM 输出
#define Use_Comp2_Negative  0

// 过流阈值默认值 A（按电感电流传感器量程整定）
// 后缀 _F = 设计期 float 常量，不带后缀的是编译期算好的 Q24（和 dcac_cfg.h 一致）
#define Trip_Amps_Default_F     10.0f
#define Trip_Amps_Default       _IQ(Trip_Amps_Default_F)

// 配置比较器 + ePWM 数字比较触发链；必须在 ePWM 初始化之后、开 PWM 之前调用
void Protect_InitOCP(void);

// 修改过流阈值（安培，双向对称，Q24）
void Protect_SetTripLevel(_iq amps);

// 清硬件触发锁存并重新武装 TZ 中断，允许重新输出
void Protect_ClearTrip(void);

// 主动封锁输出（软件急停）：TZFRC 强制单次触发，四管全关
// 属于主动动作，不置 Protect_TripFlag、不产生 TZ 中断
// 恢复输出用 Protect_ClearTrip()
void Protect_ForceStop(void);

// 硬件触发标志，只由 EPWM1 TZ 中断置位
extern volatile Uint16 Protect_TripFlag;
extern volatile Uint16 Protect_TripSource;      // 1=OST(DCxEVT1) 2=CBC(DCxEVT2)

// 软件过流兜底：返回 1 表示越限（两个参数都是 Q24，直接按整数比较）
Uint16 Protect_SoftwareCheck(_iq i_fb, _iq i_limit);

#endif
