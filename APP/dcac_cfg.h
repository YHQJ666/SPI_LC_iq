/*
 * dcac_cfg.h
 *
 * 单相离网逆变硬件与控制配置（全 CPU 运行，不使用 CLA）
 *
 * 载波：ePWM1 增减计数 TBPRD=1500 -> fsw=20kHz
 * 采样：ePWM1 在 CTR=ZERO 触发 SOCA，ADCINT1 直接进 CPU 中断
 *   SOC0 丢弃
 *   SOC1 = ADCINA0 输出电流 i_o
 *   SOC2 = ADCINA1 输出电压 v_o
 *   SOC3 = ADCINA2 电感电流 i_L
 * 控制：电容电流 i_c = i_L - i_o
 *   电压外环 PR(准比例谐振) -> 电容电流给定 -> 电流内环 PI -> 调制比 mod
 *
 *
 * ============================ 定点化说明 ============================
 * 本工程控制环全部用 TI IQmath 定点运算，全局格式 GLOBAL_Q = 24（_iq）。
 *   Q24 分辨率 2^-24 = 5.96e-8，动态范围只有 ±128
 *   本工程所有运行量都在 ±128 以内：
 *     输出电压 80V、输出电流 22A、电感电流 30A、给定峰值 50.9V、
 *     电容电流给定 8A、调制比 0.85
 *   超过 ±128 的量必须换低 Q 存放，整个工程只有下面 3 处，都加了注释：
 *     a) 谐振角频率 w0 = 314  -> 设计期就折成 R_V = w0*Ts/2 = 0.00785，不再有 w0
 *     b) TBPRD/2 = 750        -> SPWM 里按 Q15 算比较值
 *     c) 2*满量程 = 160(电压) -> ADC 换算系数按 Q17 存放
 *
 * 两条必须记住的规矩：
 *   1) 中断里的乘法一律用 _IQmpy（它带 64 位中间结果并右移 GLOBAL_Q），
 *      直接写 a*b 得到的是 Q48 的截断值，是最隐蔽的错误来源；
 *   2) 加减法可以直接写，定点加减就是整数加减，不会溢出（除非超出 ±128）。
 * ====================================================================
 */

#ifndef CFG_H
#define CFG_H

// ---- 0. IQ 全局格式 ----
// IQmathLib.h 里 GLOBAL_Q 的默认值本来就是 24，这里显式写一遍：
// 换库版本或者有人在编译选项里 -dGLOBAL_Q=xx 时，不会悄悄变掉
#ifndef GLOBAL_Q
#define GLOBAL_Q 24
#endif
#include "IQmathLib.h"

#include "DSP28x_Project.h"

// ---- 1. 环路模式 ----
#define Loop_Open       0
#define Loop_Closed     1
// 0 = 开环（只出正弦，不走 PR/PI），1 = 闭环（电压外环 PR + 电容电流内环 PI）
#define Loop_Mode       0

// ---- 2. 系统时钟 / 载波 / 采样率 ----
// ePWM 增减计数：fsw = TBCLK / (2 * TBPRD)
//   TBCLK = 60MHz, TBPRD = 1500 -> fsw = 20kHz
//   TBCLK = 60MHz, TBPRD = 3000 -> fsw = 10kHz
// 单极性倍频：两个桥臂共用一个三角载波，各用一个比较值
//   EPWM1.CMPA = TBPRD/2 * (1 + mod)   左桥臂，占空 dA = (1 + mod)/2
//   EPWM2.CMPA = TBPRD/2 * (1 - mod)   右桥臂，占空 dB = (1 - mod)/2
//   TBPRD=3000 时即 1500 +- mod*1500
//   uAB = (dA - dB)*Vdc = mod*Vdc，电平只有 +Vdc/0/-Vdc，
//   每个载波周期两个脉冲，等效开关频率 = 2*fsw
#define TBCLK_Hz        60000000L
#define TBPRD           1500U
#define CMPA_Mid        (TBPRD / 2U)    // 750
#define CMPA_Span       (TBPRD / 2U)    // 750

// 下面两个 float 常量只在"设计期"用（算采样周期、相位步进、PR 系数、Ki*Ts），
// 它们只出现在 _IQ() 的参数里，编译期就被折成整数常量了，
// 中断里一个浮点运算都不会剩下
#define Fsw_Hz          ((float)TBCLK_Hz / (2.0f * (float)TBPRD))
#define Ts_S            (1.0f / Fsw_Hz)

// ---- 3. 输出规格 / 正弦表 / 相位步进 ----
// 输入 60V，输出 36Vrms/50Hz
//   Vref_Peak = sqrt(2)*36 = 50.91V
//   Mod_Max   = 50.91 / 60 = 0.85
#define Vbus_Nominal    60.0f
#define Fout_Hz         50.0f
#define Vout_Rms        36.0f
#define Vref_Peak_F     (1.41421356f * Vout_Rms)    // 50.91V，设计期
#define Mod_Max_F       0.85f
// 开环目标调制比幅值 = Vref_Peak / Vbus = 0.85
#define Mod_Amp_Open_F  (Vref_Peak_F / Vbus_Nominal)

#define Vref_Peak       _IQ(Vref_Peak_F)        // Q24：50.91V -> 854156389
#define Mod_Max         _IQ(Mod_Max_F)          // Q24：0.85
#define Mod_Amp_Open    _IQ(Mod_Amp_Open_F)     // Q24：0.85

#define Sin_Table_Size  256U
// phase_step = 256 * Fout / Fsw = 256*50/20000 = 0.64
#define Phase_Step_F    ((float)Sin_Table_Size * Fout_Hz / Fsw_Hz)

// 相位累加器用 Uint32 定点累加，不用 _iq：Q24 装不下 0~256 这个范围
//   高 8 位 = 正弦表下标 0..255，低 24 位 = 下标的小数部分（Q24）
//   每周期加 0.64，累加满 256 与 0x00FFFFFF 相与即自动回绕，
//   和原来的"Phase >= 256 就减 256"完全等价，但不要判断、不要减法
#define Phase_Acc_Bits  24U
#define Phase_Step_Q24  (Uint32)_IQ(Phase_Step_F)       // 0.64 -> 10737418
#define Phase_Mask      0xFFFFFFFFUL

// ---- 4. ADC 通道分配 ----
#define ADC_Ch_Dummy    0U      // SOC0，第一次采样，结果丢弃
#define ADC_Ch_Iout     0U      // SOC1 -> ADCINA0 输出电流
#define ADC_Ch_Vout     1U      // SOC2 -> ADCINA1 输出电压
#define ADC_Ch_IL       2U      // SOC3 -> ADCINA2 电感电流

// ---- 5. 传感器标定 ----
// ADC 12 位单极性 0..4095，中点 2048 对应 0A / 0V
// 工程量 = (raw - 2048) * Full_Scale / 2048
#define ADC_Mid_Code        2048        // 码值中点
#define Vout_Full_Scale_F   80.0f
#define Iout_Full_Scale_F   22.0f
// 电感电流含电容纹波，量程一般比输出电流大，按实际传感器改
#define IL_Full_Scale_F     30.0f

// 一次采样换算的定点写法（以电压为例）：
//   a = raw - 2048          ∈ [-2048, 2047]，整数
//   a << 12                 = a/4096 的 Q24，最大 8.4e6，不会溢出 32 位
//   再乘 2*Full_Scale       即得 a*Full_Scale/2048 的 Q24
// 2*80 = 160 超过 Q24 的 ±128，所以系数用 Q17 存，
// 用 __IQmpy(A,B,Q) = (A*B)>>Q：A 是 Q24、B 是 Q17 时结果正好还是 Q24
#define ADC_Scale_Q17(fs)   ((long)((fs) * 2.0f * 131072.0f))
#define ADC_To_IQ(raw, fs) \
    __IQmpy((_iq)(((long)(raw) - (long)ADC_Mid_Code) << 12), ADC_Scale_Q17(fs), 17)

// 电感电流满量程另外留一份 Q24 的（30 < 128 装得下），保护模块换 DAC 码要用
#define IL_Full_Scale       _IQ(IL_Full_Scale_F)

// ---- 6. 控制环参数（待整定） ----
#define W0_V_F          314.159265f     // 电压环 2*pi*50 谐振角频率
#define Wc_V_F          10.0f           // 电压环准 PR 截止角频率
// PR 的 Tustin 系数只需要下面两个无量纲小量，设计期一次算好：
//   R_V = w0*Ts/2 = 314.159*50us/2 = 7.853982e-3  Q24 -> 131767
//   Q_V = wc*Ts/2 = 10*50us/2      = 2.5e-4       Q24 -> 4194
// w0 = 314 本身超出 Q24 的 ±128，做成 Q8 再乘会有精度损失，
// 直接用编译期常量 _IQ(公式) 算，结果是 Q24 能表示的最接近值
#define R_V             _IQ(W0_V_F * Ts_S * 0.5f)
#define Q_V             _IQ(Wc_V_F * Ts_S * 0.5f)

#define KP_V            _IQ(0.0585f)    // 电压环比例
#define KR_V            _IQ(0.1651f)    // 电压环谐振增益
#define KP_I            _IQ(0.8411f)    // 电流环 Kp [mod/A]

// 电流环 Ki，单位 1/s，0 = 纯 P
#define KI_I            0.0f
// 积分增益把采样周期折进去：KI_I_Ts = KI_I*Ts，中断里只乘一次。
// 这样做一是快，二是保精度：Ts = 50us 在 Q24 里只有 838 个刻度
// （0.1% 量化误差），先乘 Ts 会把误差的有效位数吃掉十几位。
// 折进来以后 KI_I_Ts = KI_I*50us，量级很小，Q24 的 6e-8 分辨率完全够用
#define KI_I_Ts         _IQ(KI_I * Ts_S)

#define Iref_Max_F      8.0f            // 电容电流给定限幅 A
#define Iref_Max        _IQ(Iref_Max_F)

// ---- 7. 保护总开关 ----
// 0 = 全部关闭：硬件 Trip（COMP + Digital Compare）完全不配置，
//     软件过流/过压判断也不执行，用来单独确认 SPWM 输出
// 1 = 全部打开：整机运行时用
// 关掉保护只允许空载/低压短时间调试，绝对不能带功率跑
#define Protect_Enable  0

// ---- 8. 保护阈值与软启动 ----
#define OV_Volts_F      45.0f           // 输出过压阈值 V（峰值）
#define OV_Volts        _IQ(OV_Volts_F)
// 软件过流阈值按电感电流算，要高于硬件 Trip 值（默认 20A）
// 前提是 ADCINA2 真的接了电感电流传感器；没接时采样恒为 0，
// 算出来 IL = -30A 会立刻误报过流，所以调试期必须把 Protect_Enable 置 0
#define OC_Amps_F       25.0f
#define OC_Amps         _IQ(OC_Amps_F)
#define OC_Debounce     3U              // 过流连续几个采样点才认
#define Soft_Start_Ms   500U            // 软启动时长 ms
// 每个中断周期给定增量 = 额定峰值 / (软启动时间 * 中断频率)
#define Soft_Start_Step_F \
    (Vref_Peak_F / ((float)Soft_Start_Ms * 0.001f * Fsw_Hz))
#define Soft_Start_Step _IQ(Soft_Start_Step_F)      // Q24：5.0912e-3 -> 85415

// ---- 9. 中断计时脚 ----
// GPIO4 配成普通 GPIO 输出，进 ADC 中断拉高、退出前拉低
// 用它判断两件事：有没有脉冲 = ADC 中断在不在跑；脉宽 = 控制环耗时
// 定点化以后控制环应该明显快于原来的软件浮点版本，可以直接用脉宽对比
#define ISR_Test_Pin    4U

#endif
