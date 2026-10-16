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
 *   SOC4 = ADCB1 输入电压 v_in（分压 80/3.3，只监视/保护）
 *   SOC5 = ADCB0 输入电流 i_in（放大 150 倍，只监视/保护）
 *   控制环只吃 i_o / v_o / i_L，两路输入量原样存进 VinRaw / IinRaw 留给保护
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
//   引脚按走线方便分配：GPIO0/1 = PWM1AB = 右桥臂（EPWM1），
//                       GPIO2/3 = PWM2AB = 左桥臂（EPWM2）
//   EPWM1.CMPA = TBPRD/2 * (1 - mod)   右桥臂，占空 dR = (1 - mod)/2
//   EPWM2.CMPA = TBPRD/2 * (1 + mod)   左桥臂，占空 dL = (1 + mod)/2
//   TBPRD=3000 时即 1500 +- mod*1500
//   uAB = (dL - dR)*Vdc = mod*Vdc，电平只有 +Vdc/0/-Vdc，
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
// F2803x 的 ADC 有 A/B 两组各 8 个通道，SOCxCTL.CHSEL 的编号是连续的：
//   A 组 ADCINA0~A7 = 通道 0~7
//   B 组 ADCINB0~B7 = 通道 8~15
// 所以 ADCB0 = 8、ADCB1 = 9。
// ADCB0 / ADCB1 在 F2803x 上是纯模拟脚（AIOMUX1 里对应位是保留位），
// 复位后就是模拟输入，不需要像 ADCINA2/A4 那样写 AIOMUX 配置。
//
// SOC 顺序 = 转换顺序：全部 SOC 共用同一个触发源（EPWM1 SOCA）、同样的优先级，
// 所以每个载波周期都按 SOC0 -> SOC1 -> ... 固定顺序转换。
// 这种"固定槽位"写法下第 N 个 SOC 的结果一定落在 ADCRESULTn 里，
// 不依赖轮询指针当拍停在哪儿，比"通道和 ADCRESULT 编号错开"稳得多。
//
// ★ SOC0 和 SOC1 必须读同一个通道，这是 TI 给的 rev0 芯片勘误修法：
//   每个 SOC 序列的"第一次采样"结果不准，所以同一通道连采两次、
//   丢掉第一次只用第二次。这两次必须紧邻，中间不能插别的通道，
//   否则第二次采到的也不是刚建立的那个值，修法就失效了。
//   本板子 ADCA0 = 输出电压，所以 SOC0/SOC1 都放 A0，真值取 SOC1。
#define ADC_Ch_Dummy    0U      // SOC0 -> ADCA0 丢样（与 SOC1 同通道）
#define ADC_Ch_Vout     0U      // SOC1 -> ADCA0 输出电压   ← 取这一次的
#define ADC_Ch_Iout     1U      // SOC2 -> ADCA1 输出电流
#define ADC_Ch_IL       2U      // SOC3 -> ADCA2 电感电流
#define ADC_Ch_Vin      9U      // SOC4 -> ADCB1 输入电压（只监视）
#define ADC_Ch_Iin      8U      // SOC5 -> ADCB0 输入电流（只监视）

// 中断触发点 = 最后一个 SOC。以后再加通道，这里必须跟着改，
// 否则 ADCINT1 会在新通道转换完成之前就进 ISR，读到的还是上一拍的结果
#define ADC_EOC_For_INT1 5U     // EOC5（SOC5 = i_in）触发 ADCINT1

// ---- 5. 采样前端标定 ----
// 调理电路送到 ADC 引脚的电压 Vpin，12 位 ADC 满码 3.3V、4096 码：
//   code = Vpin / 3.3 * 4096
//   电流（IL 和 Iout 调理一样，都是 1mΩ 采样电阻 + 20 倍增益）：
//     Vpin = 2.5 + 0.001*20*I = 2.5 + 0.02*I
//     I    = (code - 2.5/3.3*4096) * 3.3/4096/0.02
//          = (code - 3103) * 40.283mA
//     满码 4096 对应 3.3/0.02 = 165A，实际 0~4095 码 -> -125A ~ +40A
//   电压：
//     Vpin = 1.5 + 0.03125*V
//     V    = (code - 1.5/3.3*4096) * 3.3/4096/0.03125
//          = (code - 1862) * 25.781mV
//     满码 4096 对应 3.3/0.03125 = 105.6V，实际 0~4095 码 -> -48V ~ +57.6V
// 偏置码 3103.03 / 1861.82 取整，残差只有 1.2mA / 4.7mV，
// 比 1 个 LSB（电流 40mA、电压 26mV）还小，可以忽略
#define ADC_IL_Offset       3103        // 电流零点 2.5V 对应的码值
#define ADC_IL_Span         165.0f      // 满码 4096 对应 165A
#define ADC_Iout_Offset     3103
#define ADC_Iout_Span       165.0f
#define ADC_Vout_Offset     1862        // 电压零点 1.5V 对应的码值
#define ADC_Vout_Span       105.6f      // 满码 4096 对应 105.6V

// ---- 5a. 输入侧两路采样（只做监视/保护，不参与控制环） ----
// 输入电压：电阻分压 80/3.3
//   Vpin = Vin * 3.3/80，Vin = 80V 时正好 3.3V（满码）
//   分压比要按可能的最高输入电压选，超过 80V 就被 3.3V 钳位
//   Vin = Vpin * 80/3.3 = (code/4096) * 80
//   零点码 0，满码 4096 对应 80V，即 19.5mV/code
#define ADC_Vin_Offset      0
#define ADC_Vin_Span        80.0f       // 满码 4096 对应 80V

// 输入电流：采样电阻 + 150 倍放大
//   Vpin = 2.5 + 0.001 * 150 * Iin = 2.5 + 0.15 * Iin
//   Iin  = (code - 2.5/3.3*4096) * 3.3/4096/0.15
//        = (code - 3103) * 5.371mA
//   满码 4096 对应 3.3/0.15 = 22A，实际 0~4095 码 -> -16.7A ~ +5.3A
//   ★ 这里的 0.001 是 1mΩ 采样电阻。换采样电阻只改这两个宏：
//       Span = 3.3 / (Rsense * 150)，Offset = 2.5/3.3*4096 = 3103（偏置 2.5V 不变时）
#define ADC_Iin_Offset      3103        // 电流零点 2.5V 对应的码值
#define ADC_Iin_Span        22.0f       // 满码 4096 对应 22A（1mΩ + 150 倍）

// ---- 5b. ADC 零点校准（软件校零） ----
// 上面那组 Offset 是"标称值"，按 2.5V/1.5V 偏置算出来的。实际板子元件有误差
// 和温漂，真零点会偏几十个码。校零做的事：让各路调理电路处于静止零输入，
// 把这几十/几百拍的 ADC 码值平均出来，当成真正的零点码。
//
// 和 TI 的 AdcOffsetSelfCal() 是两回事，别搞混：
//   AdcOffsetSelfCal() 校的是 ADC 内核自己的失调（内部把 VREFLO 短到采样端
//   测出来的），信号路径完全不经过外部调理电路，所以它消不掉 2.5V/1.5V
//   偏置和运放失调。本函数是软件校零，量的是"整条链路"的零点。
//
// 启用条件：校零的那一下，各路输入必须真的是零
//   Vout 通道     -> 输出空载，桥臂不给电压
//   Iout/IL/ICap  -> 采样电阻上没有电流流过
//   两路输入      -> 输入电压/输入电流为 0
// 满足不了就把 ADC_ZeroCal_Enable 置 0，用上面的标称 Offset 也能跑，
// 只是读数会带上板子的实际零点误差
#define ADC_ZeroCal_Enable  1

// 每通道平均多少个采样点（必须是 2 的幂，代码里用右移求平均）
// 256 点 @20kHz = 12.8ms，够把噪声平均掉，也不会校太久
#define ADC_ZeroCal_Samples 256U
#define ADC_ZeroCal_Shift   8U      // log2(ADC_ZeroCal_Samples)

// 一致性检查：用前 64 点的 (最大-最小) 判断这一路是不是悬空/在跳。
// 超过这个码数就认为该通道不可信，放弃校准、保留标称 Offset。
// 静止时正常应该在几个码以内。
// ★ 如果误报了，先确认这一路是不是真的悬空；确实需要放宽再调大这个值
#define ADC_ZeroCal_MaxSpread   64
#define ADC_ZeroCal_SpreadN     64U     // 用前多少点做一致性检查

// ADC 满量程（参考）电压，用来把原始码直接换算成"引脚电压"：
//   引脚电压 V = code / 4096 * ADC_Vref
// 调试器里看 Debug_View.CalPinVxxx 就是拿这个算的，单位 V，
// 可以直接和万用表打在 ADC 引脚上的读数对比，
// 用来验证调理电路的分压/放大倍数和零偏对不对
#define ADC_Vref            3.3f

// 定点换算 value = (raw - Offset) * Span / 4096：
//   (raw-Offset) << 12   = (raw-Offset)/4096 的 Q24，最大 3103*4096 = 1.3e7，不溢出
//   Span 乘 2^17 做成 Q17 常量：165 * 131072 = 2.16e7，32 位装得下
//   __IQmpy(A,B,Q) = (A*B)>>Q，Q24 × Q17 >> 17 的结果正好还是 Q24
// Span 本身超过 Q24 的 ±128（165），所以只能走 Q17，不能写成 _IQmpy
#define ADC_Scale_Q17(span) ((long)((span) * 131072.0f))
#define ADC_To_IQ(raw, off, span) \
    __IQmpy((_iq)(((long)(raw) - (long)(off)) << 12), ADC_Scale_Q17(span), 17)

// 校零之后的换算：零点码不再是编译期常量，而是 main.c 里实测出来的 OffsetNeg
//   记标称零点码 nominal（编译期常量，就是上面的 ADC_*_Offset），
//      实测零点码 raw_zero（运行期测出来的），
//      offneg = nominal - raw_zero
//   要算的是 (raw - raw_zero)，而
//     raw - raw_zero = (raw - nominal) + (nominal - raw_zero)
//                    = (raw - nominal) + offneg
//   所以：
//     (raw - raw_zero)/4096 的 Q24 = ((raw - nominal) + offneg) << 12
//   再乘 Span：和 ADC_To_IQ 完全同构，只是左操作数从编译期常量
//   变成了运行期算术，右操作数仍然是编译期常量 Span*2^17
//
// 量级检查：|raw - raw_zero| <= 4095，<<12 后 <= 1.68e7，做 _IQ() 不会溢出；
// 乘 Span*2^17 时 __IQmpy 走 32x32->64，也不会溢出
#define ADC_To_IQ_Cal(raw, nominal, offneg, span) \
    __IQmpy((_iq)((((long)(raw) - (long)(nominal)) + (long)(offneg)) << 12), \
            ADC_Scale_Q17(span), 17)

// 输入侧两路只存原始码，不换算成工程量：
//   vin = (VinRaw - ADC_Vin_Offset) * ADC_Vin_Span / 4096
//   iin = (IinRaw - ADC_Iin_Offset) * ADC_Iin_Span / 4096
// 后面加保护阈值时，既可以直接比码值（省一次乘法），也可以按上面公式算成
// Q24 再和 _IQ(阈值) 比；原始码在中断里原样存下来最省时间

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

// ---- 10. 调试器观察用浮点镜像 ----
// 控制量都是 Q24，调试器里显示的是原始整数（真值 = 原始整数 / 16777216），
// 看着不直观。置 1 时在后台死循环里把它们换算成工程量存进 main.c 的
// Debug_View，调试器里把 Debug_View 加进 Expressions 展开就是 V / A / 调制比。
// 换算只在后台空转的循环里做，20kHz 中断里一次都不做，不影响控制时序；
// 置 0 时 Debug_View 和换算代码都不编译，不占 Flash 也不多跑一条指令
#define Debug_Float_View    1
// 后台循环每转多少圈折算一次：调小刷新快、后台占用高
#define Debug_Div           256U

// ---- 11. 输出幅值检测（只看，不进控制环） ----
// 目标：在调试器里直接读出输出电压/输出电流/电容电流的"幅值"，而不是瞬时值。
//
// 做法是"按工频周期复位的峰值保持 + 每周期的平均值"：
//   1) 每个中断拍更新本周期的峰值 pk = max(pk, |x|)
//   2) 每满一个工频周期（400 拍 @20kHz）把 pk 提交给平均值并清零 pk
//   3) 平均值按 1/4 权重一阶平均： amp += (pk - amp) >> Amp_Avg_Shift
//
// 为什么不用"峰值瞬跌法"（pk 每拍乘个略小于 1 的系数）：
//   那样读数是"峰值 - 纹波"，仿真过：系数取 1/256 时纹波就有 20%、
//   稳态也偏低 20%，根本没法读。按周期复位就没有这个问题。
//
// 本方案仿真的结果（50Hz 正弦、20kHz 采样）：
//   恒定幅值时：纹波 0.056%，稳态误差 -0.007%（基本就是真幅值）
//   幅值阶跃后：约 0.25s 收敛到 2% 以内
// 因为工频 50Hz、中断 20kHz，一个工频周期正好是 20000/50 = 400 拍，
// 所以计数到 Amp_Cycle_Samples 就是整周期，不需要另外判过零
#define Amp_Track_En        1U
#define Amp_Cycle_Samples   400U    // 一个工频周期对应的中断拍数 = Fsw/Fout
// 每周期的峰值做一阶平均，权重 = 1/2^Amp_Avg_Shift
// 每工频周期(20ms)更新一次，取 2 相当于时间常数约 40ms，
// 比这更大就会像上面说的那样"跟不上幅值下降"，不要随意调大
#define Amp_Avg_Shift       2U
// 峰值 -> 有效值：/sqrt(2) = 0.70710678
// Q24 放不下 1.414，所以用 0.70710678 直接乘（在 Q24 范围内）
#define Amp_PkToRms         _IQ(0.70710678f)    // Q24

#endif
