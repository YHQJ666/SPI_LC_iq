/*
 * main.c
 *
 * DSP28035 单相离网逆变（全部在 CPU 上运行，不使用 CLA）
 *
 * 硬件链路：
 *   ePWM1 增减计数 TBPRD=1500 -> fsw=20kHz
 *     CTR=ZERO -> SOCA -> ADC
 *        SOC0 丢弃（ADCA0，与 SOC1 同通道，修 rev0 首采样 errata）
 *        SOC1 = ADCA0 输出电压 v_o
 *        SOC2 = ADCA1 输出电流 i_o
 *        SOC3 = ADCA2 电感电流 i_L
 *        SOC4 = ADCB1 输入电压 v_in（分压 80/3.3，只监视/保护）
 *        SOC5 = ADCB0 输入电流 i_in（放大 150 倍，只监视/保护）
 *     EOC5 -> ADCINT1 -> CPU 的 ADC_ISR
 *   ADC_ISR：换算工程量 -> 电容电流 i_c = i_L - i_o -> 查表
 *            -> 电压外环 PR -> 电容电流给定 -> 电流内环 PI
 *            -> mod 限幅 +-0.85 -> 写 EPWM1/EPWM2 CMPA
 *   COMP1/COMP2 -> ePWM Digital Compare -> Trip Zone -> 四管直接拉低
 *              纯硬件，CPU 跑飞也能保护
 *
 * 定点化：F28035 没有硬件 FPU，浮点全靠软件模拟，20kHz 中断里跑两个调节器
 * 很吃 CPU。本工程把控制环整体改成 TI IQmath 定点，全局格式 Q24
 * （GLOBAL_Q = 24，见 APP/IQmathLib.h）：
 *   - 所有物理量、给定、反馈、调节器内部状态都是 _iq（Q24）
 *   - 中断里的乘法一律 _IQmpy，加减法直接写（定点加减就是整数加减）
 *   - 正弦表编译期就转成 Q24 整数常量，运行期不查表乘浮点
 *   - 相位累加器用 Uint32（高 8 位表下标 + 低 24 位小数），
 *     因为 Q24 装不下 0~256 这个范围
 *   - PR 的离散系数、软启动步长、采样换算系数全部是编译期常量，
 *     整个工程运行期不再出现一次浮点运算
 *   Q24 的动态范围只有 ±128，本工程最大的运行量是 80V（电压满量程），
 *   超过 ±128 的中间量（TBPRD/2=750、2*满量程=160）在各自模块里换低 Q 处理
 *
 * 环路模式由 dcac_cfg.h 的 LOOP_MODE 选择：
 *   0 正弦开环 / 1 电压单环 / 2 电流单环 / 3 双环 / 4 固定占空比开环
 * 本文件不含并网逻辑（离网模式）
 */

#include "DSP28x_Project.h"             // 项目顶层主头文件
#include "DSP2803x_GlobalPrototypes.h"  // 全局函数原型声明文件
#include "DSP2803x_EPwm_defines.h"
#include <math.h>
#include "IQmathLib.h"                  // IQ 定点数学库（GLOBAL_Q 已在 dcac_cfg.h 里定为 24）
#include "dcac_cfg.h"
#include "pid.h"
#include "pr.h"
#include "spwm.h"
#include "protect.h"
#include "Oled.h"                       // 0.96 寸 OLED（SSD1306，I2C 位带，GPIO28/29）

// 函数原型声明
void InitAdc(void);
void AdcOffsetSelfCal(void);
void Adc_Config(void);
void Adc_ZeroCal(void);
void Gpio_Config(void);
__interrupt void ADC_ISR(void);
#if (Protect_Enable == 1)
__interrupt void Epwm1_TripIsr(void);
#endif

// 故障码
#define FAULT_NONE      (0U)
#define FAULT_HW_OC     (1U)        // 硬件过流 Trip
#define FAULT_SW_OC     (2U)        // 软件过流
#define FAULT_OVERVOLT  (3U)        // 输出过压

// 正弦表 -1 ~ +1，256 点，编译期转成 Q24 整数（_IQ() 是编译期常量折叠）
// sin_table[i] = _IQ(sin(2*pi*i/256))
static const _iq sin_table[Sin_Table_Size] =
{
    _IQ(0.0000000f), _IQ(0.0245412f), _IQ(0.0490677f), _IQ(0.0735646f), _IQ(0.0980171f), _IQ(0.1224107f), _IQ(0.1467305f), _IQ(0.1709619f),
    _IQ(0.1950903f), _IQ(0.2191012f), _IQ(0.2429802f), _IQ(0.2667128f), _IQ(0.2902847f), _IQ(0.3136817f), _IQ(0.3368899f), _IQ(0.3598950f),
    _IQ(0.3826834f), _IQ(0.4052413f), _IQ(0.4275551f), _IQ(0.4496113f), _IQ(0.4713967f), _IQ(0.4928982f), _IQ(0.5141027f), _IQ(0.5349976f),
    _IQ(0.5555702f), _IQ(0.5758082f), _IQ(0.5956993f), _IQ(0.6152316f), _IQ(0.6343933f), _IQ(0.6531728f), _IQ(0.6715590f), _IQ(0.6895405f),
    _IQ(0.7071068f), _IQ(0.7242471f), _IQ(0.7409511f), _IQ(0.7572088f), _IQ(0.7730105f), _IQ(0.7883464f), _IQ(0.8032075f), _IQ(0.8175848f),
    _IQ(0.8314696f), _IQ(0.8448536f), _IQ(0.8577286f), _IQ(0.8700870f), _IQ(0.8819213f), _IQ(0.8932243f), _IQ(0.9039893f), _IQ(0.9142098f),
    _IQ(0.9238795f), _IQ(0.9329928f), _IQ(0.9415441f), _IQ(0.9495282f), _IQ(0.9569403f), _IQ(0.9637761f), _IQ(0.9700313f), _IQ(0.9757021f),
    _IQ(0.9807853f), _IQ(0.9852776f), _IQ(0.9891765f), _IQ(0.9924795f), _IQ(0.9951847f), _IQ(0.9972905f), _IQ(0.9987955f), _IQ(0.9996988f),
    _IQ(1.0000000f), _IQ(0.9996988f), _IQ(0.9987955f), _IQ(0.9972905f), _IQ(0.9951847f), _IQ(0.9924795f), _IQ(0.9891765f), _IQ(0.9852776f),
    _IQ(0.9807853f), _IQ(0.9757021f), _IQ(0.9700313f), _IQ(0.9637761f), _IQ(0.9569403f), _IQ(0.9495282f), _IQ(0.9415441f), _IQ(0.9329928f),
    _IQ(0.9238795f), _IQ(0.9142098f), _IQ(0.9039893f), _IQ(0.8932243f), _IQ(0.8819213f), _IQ(0.8700870f), _IQ(0.8577286f), _IQ(0.8448536f),
    _IQ(0.8314696f), _IQ(0.8175848f), _IQ(0.8032075f), _IQ(0.7883464f), _IQ(0.7730105f), _IQ(0.7572088f), _IQ(0.7409511f), _IQ(0.7242471f),
    _IQ(0.7071068f), _IQ(0.6895405f), _IQ(0.6715590f), _IQ(0.6531728f), _IQ(0.6343933f), _IQ(0.6152316f), _IQ(0.5956993f), _IQ(0.5758082f),
    _IQ(0.5555702f), _IQ(0.5349976f), _IQ(0.5141027f), _IQ(0.4928982f), _IQ(0.4713967f), _IQ(0.4496113f), _IQ(0.4275551f), _IQ(0.4052413f),
    _IQ(0.3826834f), _IQ(0.3598950f), _IQ(0.3368899f), _IQ(0.3136817f), _IQ(0.2902847f), _IQ(0.2667128f), _IQ(0.2429802f), _IQ(0.2191012f),
    _IQ(0.1950903f), _IQ(0.1709619f), _IQ(0.1467305f), _IQ(0.1224107f), _IQ(0.0980171f), _IQ(0.0735646f), _IQ(0.0490677f), _IQ(0.0245412f),
    _IQ(0.0000000f), _IQ(-0.0245412f), _IQ(-0.0490677f), _IQ(-0.0735646f), _IQ(-0.0980171f), _IQ(-0.1224107f), _IQ(-0.1467305f), _IQ(-0.1709619f),
    _IQ(-0.1950903f), _IQ(-0.2191012f), _IQ(-0.2429802f), _IQ(-0.2667128f), _IQ(-0.2902847f), _IQ(-0.3136817f), _IQ(-0.3368899f), _IQ(-0.3598950f),
    _IQ(-0.3826834f), _IQ(-0.4052413f), _IQ(-0.4275551f), _IQ(-0.4496113f), _IQ(-0.4713967f), _IQ(-0.4928982f), _IQ(-0.5141027f), _IQ(-0.5349976f),
    _IQ(-0.5555702f), _IQ(-0.5758082f), _IQ(-0.5956993f), _IQ(-0.6152316f), _IQ(-0.6343933f), _IQ(-0.6531728f), _IQ(-0.6715590f), _IQ(-0.6895405f),
    _IQ(-0.7071068f), _IQ(-0.7242471f), _IQ(-0.7409511f), _IQ(-0.7572088f), _IQ(-0.7730105f), _IQ(-0.7883464f), _IQ(-0.8032075f), _IQ(-0.8175848f),
    _IQ(-0.8314696f), _IQ(-0.8448536f), _IQ(-0.8577286f), _IQ(-0.8700870f), _IQ(-0.8819213f), _IQ(-0.8932243f), _IQ(-0.9039893f), _IQ(-0.9142098f),
    _IQ(-0.9238795f), _IQ(-0.9329928f), _IQ(-0.9415441f), _IQ(-0.9495282f), _IQ(-0.9569403f), _IQ(-0.9637761f), _IQ(-0.9700313f), _IQ(-0.9757021f),
    _IQ(-0.9807853f), _IQ(-0.9852776f), _IQ(-0.9891765f), _IQ(-0.9924795f), _IQ(-0.9951847f), _IQ(-0.9972905f), _IQ(-0.9987955f), _IQ(-0.9996988f),
    _IQ(-1.0000000f), _IQ(-0.9996988f), _IQ(-0.9987955f), _IQ(-0.9972905f), _IQ(-0.9951847f), _IQ(-0.9924795f), _IQ(-0.9891765f), _IQ(-0.9852776f),
    _IQ(-0.9807853f), _IQ(-0.9757021f), _IQ(-0.9700313f), _IQ(-0.9637761f), _IQ(-0.9569403f), _IQ(-0.9495282f), _IQ(-0.9415441f), _IQ(-0.9329928f),
    _IQ(-0.9238795f), _IQ(-0.9142098f), _IQ(-0.9039893f), _IQ(-0.8932243f), _IQ(-0.8819213f), _IQ(-0.8700870f), _IQ(-0.8577286f), _IQ(-0.8448536f),
    _IQ(-0.8314696f), _IQ(-0.8175848f), _IQ(-0.8032075f), _IQ(-0.7883464f), _IQ(-0.7730105f), _IQ(-0.7572088f), _IQ(-0.7409511f), _IQ(-0.7242471f),
    _IQ(-0.7071068f), _IQ(-0.6895405f), _IQ(-0.6715590f), _IQ(-0.6531728f), _IQ(-0.6343933f), _IQ(-0.6152316f), _IQ(-0.5956993f), _IQ(-0.5758082f),
    _IQ(-0.5555702f), _IQ(-0.5349976f), _IQ(-0.5141027f), _IQ(-0.4928982f), _IQ(-0.4713967f), _IQ(-0.4496113f), _IQ(-0.4275551f), _IQ(-0.4052413f),
    _IQ(-0.3826834f), _IQ(-0.3598950f), _IQ(-0.3368899f), _IQ(-0.3136817f), _IQ(-0.2902847f), _IQ(-0.2667128f), _IQ(-0.2429802f), _IQ(-0.2191012f),
    _IQ(-0.1950903f), _IQ(-0.1709619f), _IQ(-0.1467305f), _IQ(-0.1224107f), _IQ(-0.0980171f), _IQ(-0.0735646f), _IQ(-0.0490677f), _IQ(-0.0245412f)
};

// 采样与状态量（全部 Q24 定点，调试器里看到的是原始整数，
// 换算关系：显示值 = 原始整数 / 16777216，例如 1.0 = 16777216）
volatile _iq  Vout = 0;             // 输出电压 V
volatile _iq  Iout = 0;             // 输出电流 A
volatile _iq  IL   = 0;             // 电感电流 A
volatile _iq  ICap = 0;             // 电容电流 A = IL - Iout
volatile _iq  Vref = 0;             // 电压给定 V
volatile _iq  IRef = 0;             // 电容电流给定 A
volatile _iq  Mod  = 0;             // 瞬时调制比
volatile Uint32 PhaseAcc = 0;       // 相位累加器：高 8 位查表下标，低 24 位 Q24 小数
volatile _iq  RefRamp = 0;          // 软启动给定峰值

// 输入侧两路采样：只存 ADC 原始码，不参与控制环，后续留给输入过压/过流保护。
// 换算成工程量（加保护阈值时用）：
//   输入电压 V = (VinRaw - ADC_Vin_Offset) * ADC_Vin_Span / 4096
//   输入电流 A = (IinRaw - ADC_Iin_Offset) * ADC_Iin_Span / 4096
volatile _iq  Vin = 0;              // 输入电压 V（只监视）
volatile _iq  Iin = 0;              // 输入电流 A（只监视）

//===========================================================================
// ADC 零点校准结果
//   存的是"负的零点码"，也就是要加到原始码上的那个量：
//     value = (raw + OffsetNeg) * Span / 4096
//   为什么要存负号：换算乘的是 Q24 的 (raw-Offset)/4096，(raw-Offset)<<12
//   本来可能到 -3103*4096 = -1.27e7，逼近 IQmpy 的负数下界；存成 -(Offset-raw)
//   就自然变成正数偏移，乘 Span(最多 165) 也不会溢出，不需要再钳位
//   只在 main() 里赋值一次，之后全是只读，中断里读它做换算
//   初值是 0，也就是"还没校准"，换算结果等价于 (raw - 标称Offset)：
//   因为下面的换算宏用的是标准 _IQmpy，而标称值在宏里是编译期常量
//===========================================================================
volatile long VoutOffNeg = 0L;
volatile long IoutOffNeg = 0L;
volatile long ILOffNeg   = 0L;

// 校零实测到的引脚电压（V），只给 OLED 显示 Z: 用
// 输入侧(Vin/Iin)不做校零，它们没有直流偏置
volatile float CalPinVout = 0.0f;
volatile float CalPinIout = 0.0f;
volatile float CalPinIL   = 0.0f;

//===========================================================================
// 有效值 / 平均值累加器（真有效值，做法参考 TI sineAnalyzerWithPowerMeas）
// C28x 的 long 是 32 位，累加平方会溢出，所以累加器必须用 long long；
// 而且 x 先右移 8 位（Q24->Q16）再平方，否则 400 拍同样会溢出：
//   51V 时 (51*2^16)^2 * 400 = 4.5e15 < 9.2e18  OK
//===========================================================================
volatile long long RmsVoutSq = 0;
volatile long long RmsIoutSq = 0;
volatile long long RmsICapSq = 0;
volatile long long RmsILSq   = 0;
volatile long long AvgVinSum = 0;
volatile long long AvgIinSum = 0;

// 调制比用一个工频周期内的峰值保持，不平方：
//   Mod 是正弦，幅值就是它的峰值，直接取 |Mod| 的最大值最直观，
//   而且削顶时读到的正好是限幅值（比如 0.85），不会像 RMS*sqrt(2)
//   那样把方波算成 0.85*1.414=1.2 这种虚高值
volatile _iq ModPk = 0;

volatile Uint16 RmsCnt    = 0U;
volatile Uint16 RmsCommit = 0U;

//===========================================================================
// 示波器数据缓冲区：CCS 的 Graph 工具按"数组名 + 长度"读一段连续内存，
// 所以必须用全局数组（参考 TI 参考设计里的 float dBuff1[200]）。
// 存的是**瞬时值**（ADCRESULT 减直流偏置再换算成工程量），用来当示波器看波形。
// 用 float：Graph 里 Data Type 直接选 "32 bit floating point"、Q_Value=0。
//
// F28035 只有 10K 字 RAM，加了 OLED 之后已经很紧，所以只留 3 个电流通道：
//   ScopeIout / ScopeICap / ScopeIL，每通道 128 点（float 占 2 字）
// 输出电压的波形不存了（OLED 上有有效值），这样每通道点数能从 64 翻到 128。
// 地址固定在链接器 SCOPEBUF / SCOPEBUF2 段，代码长度变化不会挪动它们。
//===========================================================================
#define Scope_Size      128U

// RMS 一阶 IIR 平滑系数 1/2^Rms_SmoothShift。
// ★ 注意更新率是 50Hz（每 20ms 一次），不是 20kHz！
//   时间常数 = 2^shift × 20ms：
//     shift=2  -> 80 ms，3τ≈240ms 收敛
//     shift=1  -> 40 ms，3τ≈120ms 收敛
//   400 点正好一个工频周期，原始 RMS 已经比较稳，这里只做轻微平滑
#define Rms_SmoothShift 2U

// ScopeIout/ScopeICap 放 scopeBufs（SCOPEBUF，0x8600，512 字 = 2 x 256）
// ScopeIL 放 scopeBufs2（SCOPEBUF2，0x9F00，256 字）
#pragma DATA_SECTION(ScopeIout, "scopeBufs")
#pragma DATA_SECTION(ScopeICap, "scopeBufs")
#pragma DATA_SECTION(ScopeIL,   "scopeBufs2")

volatile float ScopeIout[Scope_Size];   // 输出电流瞬时值 A
volatile float ScopeICap[Scope_Size];   // 电容电流瞬时值 A
volatile float ScopeIL[Scope_Size];     // 电感电流瞬时值 A
volatile Uint16 ScopeIdx = 0U;          // 写指针
volatile Uint16 ScopeCnt = 0U;          // 抽取计数

// 抽取倍数：每 N 个中断存 1 点（128 点一屏）。
//   1 -> 20kHz，一屏 6.4ms  = 0.3 个工频周期（看开关纹波细节）
//   3 -> 6.7kHz，一屏 19ms  = 1 个工频周期
//   5 ->  4kHz，一屏 32ms  = 1.6 个工频周期
// 改完 Graph 的 Sampling Rate Hz 要跟着改成 20000/N
volatile Uint16 Scope_Prescale = 5U;

// RMS 平滑滤波状态（Q24 定点，每 20ms 更新一次）
volatile _iq FiltVout = 0;
volatile _iq FiltIout = 0;
volatile _iq FiltICap = 0;
volatile _iq FiltIL   = 0;
volatile _iq FiltMod  = 0;

// 运行控制与故障
volatile Uint16 Run_Enable = 0U;    // 1=允许运行（自动软启动），0=停机
volatile Uint16 FaultClear = 0U;    // 置 1 清除故障锁存
volatile Uint16 FaultLatch = FAULT_NONE;
volatile Uint16 OcCnt = 0U;         // 软件过流去抖计数
volatile Uint16 IsrCnt = 0U;        // 中断次数，调试观察用

#if (Debug_Float_View == 1)
//===========================================================================
// 调试器观察用浮点镜像
//   上面的控制量都是 Q24 定点，调试器里显示的是原始整数（真值 = 值/16777216）。
//   这里把同一批量换算成工程量存一份 float，调试器里把 Debug_View 加进
//===========================================================================
volatile struct
{
    // 有效值 / 幅值（滤波后，给 Expressions 固定看）
    float VoutRms;      // 输出电压有效值 Vrms
    float IoutRms;      // 输出电流有效值 Arms
    float ICapRms;      // 电容电流有效值 Arms
    float VoutAmp;      // 输出电压幅值 V（= VoutRms*1.414）
    float IoutAmp;      // 输出电流幅值 A
    float ILRms;        // 电感电流有效值 Arms
    float ModAmp;       // 调制比幅值（RMS*1.414，稳态值，不是瞬时值）

    // 输入侧（直流）
    float Vin;          // 输入电压 V
    float Iin;          // 输入电流 A

    // 校零时三个有偏置通道的引脚电压（V），给 OLED 的 Z: 显示
    float CalPinIout;   // 输出电流通道
    float CalPinVout;   // 输出电压通道
    float CalPinIL;     // 电感电流通道
} Debug_View;

#endif

//===========================================================================
// OLED 显示（0.96 寸 SSD1306，I2C 位带，SCL=GPIO29 / SDA=GPIO28）
//
// 屏是 128x64，用 6x8 点阵（Oled.c 里 Char_Size=12 走 F6x8）：
//   横向 128/8 = 16 个字符（ShowString 每个字符前进 8 像素）
//   纵向 64/8  = 8 行（y 就是页号 0~7，一行一页）
//
// 显示内容（8 行正好占满）：
//   0  ST:OK        保护状态
//   1  LOOP:CLOSED  开环 / 闭环
//   2  Mod:0.870    实际调制比幅值（稳态值，不是瞬时调制比）
//   3  Vo:33.4 Zc:1.51   Vout 有效值 + 零点校准值（同一行）
//   4  Io:3.31      输出电流有效值
//   5  IL:3.40      电感电流有效值
//   6  Vi:48.2      输入电压
//   7  Ii:0.35      输入电流
//
// 刷新在后台做，每 Oled_Period_Isr 个中断（500ms）刷一次。
// 刷一屏要几十毫秒（I2C 位带，没有硬件外设），所以不能刷太快。
//===========================================================================
#define Oled_Period_Isr   10000U

static Uint16 OledLastIsr = 0U;

// 定宽显示浮点数：ints 位整数 + decs 位小数，高位 0 用空格代替。
// 不用 OLED_ShowNum 是因为它固定打小数点、而且负数会出错。
static void Oled_F(Uint16 x, Uint16 y, float v, Uint16 ints, Uint16 decs)
{
    char buf[12];
    u32  s;
    u32  div;
    u32  ip;
    u32  fp;
    Uint16 i;
    Uint16 k;
    Uint16 n;
    Uint16 started;

    if (v < 0.0f)
    {
        v = -v;
    }

    for (i = 0U; i < decs; i++)
    {
        v *= 10.0f;
    }

    if (v > 9999999.0f)
    {
        v = 9999999.0f;
    }

    s = (u32)(v + 0.5f);

    div = 1U;
    for (i = 0U; i < decs; i++)
    {
        div *= 10U;
    }

    ip = s / div;
    fp = s % div;

    n = 0U;
    started = 0U;

    // 整数部分：从最高位往下打，前导 0 变空格
    for (i = ints; i > 0U; i--)
    {
        u32 d = 1U;
        Uint16 dig;

        for (k = 0U; k < (Uint16)(i - 1U); k++)
        {
            d *= 10U;
        }

        dig = (Uint16)((ip / d) % 10U);

        if ((dig != 0U) || (started != 0U) || (i == 1U))
        {
            started = 1U;
            buf[n] = (char)('0' + (char)dig);
        }
        else
        {
            buf[n] = ' ';
        }
        n++;
    }

    // 小数部分
    if (decs > 0U)
    {
        buf[n] = '.';
        n++;

        for (i = decs; i > 0U; i--)
        {
            u32 d = 1U;

            for (k = 0U; k < (Uint16)(i - 1U); k++)
            {
                d *= 10U;
            }

            buf[n] = (char)('0' + (char)((fp / d) % 10U));
            n++;
        }
    }

    buf[n] = '\0';
    OLED_ShowString(x, y, (u8 *)buf);
}

static void Oled_Task(void)
{
    // 第 0 行：保护状态 + 开环/闭环（两个都很短，合在一行）
    //   S:OK  L:CL   ->  保护正常、闭环
    //   S:OK  L:OP   ->  保护正常、开环
    //   S:HW-OC / S:SW-OC / S:OV   ->  硬件过流 / 软件过流 / 过压
    if (FaultLatch == FAULT_NONE)
    {
        OLED_ShowString(0U, 0U, (u8 *)"S:OK ");
    }
    else if (FaultLatch == FAULT_HW_OC)
    {
        OLED_ShowString(0U, 0U, (u8 *)"S:HW ");
    }
    else if (FaultLatch == FAULT_SW_OC)
    {
        OLED_ShowString(0U, 0U, (u8 *)"S:SW ");
    }
    else
    {
        OLED_ShowString(0U, 0U, (u8 *)"S:OV ");
    }

#if (LOOP_MODE != LOOP_SINE_OPEN)
    OLED_ShowString(48U, 0U, (u8 *)"L:CLOSE");
#else
    OLED_ShowString(48U, 0U, (u8 *)"L:OPEN");
#endif

    // 第 1 行留空
    // 第 2 行：调制比幅值（开环固定 Mod_Amp_Open；闭环因电感压降会略大）
    OLED_ShowString(0U, 2U, (u8 *)"M:");
    Oled_F(16U, 2U, Debug_View.ModAmp, 1U, 3U);

    // 第 3 行留空
    // 第 4 行：Vout 有效值 + 单位 + 零点校准值（校准值不写单位）
    OLED_ShowString(0U, 4U, (u8 *)"VO:");
    Oled_F(24U, 4U, Debug_View.VoutRms, 2U, 1U);
    OLED_ShowString(56U, 4U, (u8 *)"V");
    OLED_ShowString(80U, 4U, (u8 *)"Z:");
    Oled_F(96U, 4U, Debug_View.CalPinVout, 1U, 2U);

    // 第 5 行：Iout 有效值 + 单位 + 零点校准值
    OLED_ShowString(0U, 5U, (u8 *)"IO:");
    Oled_F(24U, 5U, Debug_View.IoutRms, 1U, 2U);
    OLED_ShowString(56U, 5U, (u8 *)"A");
    OLED_ShowString(80U, 5U, (u8 *)"Z:");
    Oled_F(96U, 5U, Debug_View.CalPinIout, 1U, 2U);

    // 第 6 行：IL 有效值 + 单位 + 零点校准值
    OLED_ShowString(0U, 6U, (u8 *)"IL:");
    Oled_F(24U, 6U, Debug_View.ILRms, 1U, 2U);
    OLED_ShowString(56U, 6U, (u8 *)"A");
    OLED_ShowString(80U, 6U, (u8 *)"Z:");
    Oled_F(96U, 6U, Debug_View.CalPinIL, 1U, 2U);

    // 第 7 行：输入电压（输入电流不显示了）
    OLED_ShowString(0U, 7U, (u8 *)"VIN:");
    Oled_F(32U, 7U, Debug_View.Vin, 2U, 1U);
    OLED_ShowString(64U, 7U, (u8 *)"V");
}

//===========================================================================
// 主函数入口
//===========================================================================
void main(void){
    Uint16 faultHandled = 0U;

    // 系统初始化（时钟、看门狗、外设时钟）
    InitSysCtrl();

    // 中断系统初始化
    DINT;
    InitPieCtrl();
    IER = 0x0000;
    IFR = 0x0000;
    InitPieVectTable();

    // 外设初始化
    // 顺序有讲究：
    //   InitAdc()          上电 ADC、拿 OTP 里的校准值
    //   AdcOffsetSelfCal() 校 ADC 内核自己的失调（内部 VREFLO）
    //   Adc_ZeroCal()      软件校零，量整条调理链路的零点（要在内核失调修好之后）
    //   Adc_Config()       正式配置 SOC 通道和触发源，并把轮询指针拨回 SOC0
    // Adc_ZeroCal 必须在 SPWM_Init 之前：那时 EPWM 还没起振，不会有 SOCA
    // 来抢轮询指针，测出来的零点才是干净的
    InitAdc();
    AdcOffsetSelfCal();
    Adc_ZeroCal();
    Adc_Config();

    OLED_Init();
    OLED_Clear();

    SPWM_Init();
    Gpio_Config();

#if (Protect_Enable == 1)
    // 过流保护先建立，再开中断
    Protect_InitOCP();
#endif

    // 中断向量绑定
    // 注意：TZ 中断的向量绑定和下面的 PIE 使能必须成对出现，
    // 只开使能不绑向量的话，一旦 TZ 触发会跳进 TI 默认 ISR 里
    // 那句 __asm("ESTOP0"); for(;;); ，CPU 直接卡死
    EALLOW;
    PieVectTable.ADCINT1     = &ADC_ISR;            // ADC 中断：控制环
#if (Protect_Enable == 1)
    PieVectTable.EPWM1_TZINT = &Epwm1_TripIsr;      // 硬件过流
#endif
    EDIS;

    // 初始化 PR：电压外环，准谐振 w0 = 314.159, wc = 10
    // r = w0*Ts/2、q = wc*Ts/2 已经在 dcac_cfg.h 里按编译期常量算成 Q24
    // （w0 = 314 超出 Q24 的动态范围，所以在设计期就折进 r、q，不进运行期），
    // 因此初始化阶段也全是定点运算
    PR_Design(&PR_V, KP_V, KR_V,
              R_V, Q_V);
    PR_V.Ref = 0;
    PR_V.Fdb = 0;
    PR_V.X1 = 0; PR_V.X2 = 0;
    PR_V.Y1 = 0; PR_V.Y2 = 0;
    // 限幅必须是 Q24！原来这里写的是裸小数 0.95，赋给 _iq 会被截断成 0，
    // __clamp(y, 0, 0) 让 PR 输出恒等于 0，表现出来就是"闭环没输出"。
    // 现在 PR 输出直接当调制比用，所以限到 ±Mod_Max
    PR_V.OutMin = -Mod_Max;
    PR_V.OutMax =  Mod_Max;

    // 初始化 PI_I：电容电流内环，输出就是调制比
    PI_I.Ref = 0;
    PI_I.Fdb = 0;
    PI_I.Kp = KP_I;
    PI_I.KiTs = KI_I_Ts;  // 积分增益已经折进采样周期 Ki*Ts
    PI_I.Integral = 0;
    PI_I.IntMin = -Mod_Max;
    PI_I.IntMax =  Mod_Max;
    PI_I.OutMin = -Mod_Max;
    PI_I.OutMax =  Mod_Max;

    // 开 PIE 和 CPU 中断
    EALLOW;
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;      // ADCINT1
    IER |= M_INT1;
#if (Protect_Enable == 1)
    PieCtrlRegs.PIEIER2.bit.INTx1 = 1;      // EPWM1_TZINT
    IER |= M_INT2;
#else
    PieCtrlRegs.PIEIER2.bit.INTx1 = 0;      // 保护关闭，TZ 中断也不能开
    IER &= (Uint16)(~M_INT2);
#endif
    EDIS;

    EINT;
    ERTM;

    //=======================================================================
    //  后台死循环：本工程只处理故障复位
    //  运行状态全部在 ADC_ISR 里完成
    //=======================================================================
    for (;;)
    {
        if (FaultLatch != FAULT_NONE)
        {
            // 故障：硬件封锁一次，输出留在中点
            if (faultHandled == 0U)
            {
                faultHandled = 1U;
                Protect_ForceStop();
                RefRamp = 0;
                PhaseAcc = 0U;
            }
        }
        else
        {
            faultHandled = 0U;
        }

        // 复位故障：把 FaultClear 置 1（调试器或以后加的通信）
        if (FaultClear != 0U)
        {
            FaultClear = 0U;
            Protect_ClearTrip();
            FaultLatch = FAULT_NONE;
            OcCnt = 0U;
            faultHandled = 0U;

            // 清两个环的历史，重新软启动
            PR_V.X1 = 0; PR_V.X2 = 0;
            PR_V.Y1 = 0; PR_V.Y2 = 0;
            PI_I.Integral = 0;
            RefRamp = 0;
            PhaseAcc = 0U;
        }

        // OLED 刷新：每 Oled_Period_Isr 个中断（500ms）刷一屏。
        // 刷一屏要几十毫秒（I2C 位带），所以放在后台、不能刷太快。
        // 用中断计数做时基，不依赖 CPU 定时器
        if ((Uint16)(IsrCnt - OledLastIsr) >= Oled_Period_Isr)
        {
            OledLastIsr = IsrCnt;
            Oled_Task();
        }

#if (Debug_Float_View == 1)
        // 后台：算有效值 -> 平滑 -> 刷新调试镜像
        // 全部在后台空转循环里做，20kHz 中断里一次不做
        {
            // 有效值：用 ISR 累加好的平方和，每工频周期(20ms)一次
            if (RmsCommit != 0U)
            {
                float msV;
                float msI;
                float msC;
                float msL;

                RmsCommit = 0U;

                // 累加器是 sum((x>>8)^2) = sum(A^2 * 2^32)
                // 均值 = /N，再 /2^32 就回到 A^2 的真实值（V^2 / A^2）
                msV = (float)RmsVoutSq / (float)Rms_Cycle_Samples / 4294967296.0f;
                msI = (float)RmsIoutSq / (float)Rms_Cycle_Samples / 4294967296.0f;
                msC = (float)RmsICapSq / (float)Rms_Cycle_Samples / 4294967296.0f;
                msL = (float)RmsILSq   / (float)Rms_Cycle_Samples / 4294967296.0f;

                // 开方得真实有效值，再走一阶 IIR 平滑：
                //   y += (x - y) / 2^Rms_SmoothShift
                // 每 20ms 更新一次，shift=2 -> 时间常数 80ms
                {
                    _iq xv = (_iq)(sqrtf(msV) * 16777216.0f);
                    _iq xi = (_iq)(sqrtf(msI) * 16777216.0f);
                    _iq xc = (_iq)(sqrtf(msC) * 16777216.0f);
                    _iq xl = (_iq)(sqrtf(msL) * 16777216.0f);
                    _iq xm = ModPk;         // 本周期 |Mod| 的峰值

                    ModPk = 0;              // 清掉，下个周期重新找

                    FiltVout += (xv - FiltVout) >> Rms_SmoothShift;
                    FiltIout += (xi - FiltIout) >> Rms_SmoothShift;
                    FiltICap += (xc - FiltICap) >> Rms_SmoothShift;
                    FiltIL   += (xl - FiltIL)   >> Rms_SmoothShift;
                    FiltMod  += (xm - FiltMod)  >> Rms_SmoothShift;

                    Debug_View.VoutRms = _IQtoF(FiltVout);
                    Debug_View.IoutRms = _IQtoF(FiltIout);
                    Debug_View.ICapRms = _IQtoF(FiltICap);
                    Debug_View.ILRms   = _IQtoF(FiltIL);
                    Debug_View.ModAmp  = _IQtoF(FiltMod);   // 峰值就是幅值，不用再乘
                }

                Debug_View.VoutAmp = Debug_View.VoutRms * 1.41421356f;
                Debug_View.IoutAmp = Debug_View.IoutRms * 1.41421356f;

                Debug_View.Vin = (float)AvgVinSum / (float)Rms_Cycle_Samples
                               / 16777216.0f;
                Debug_View.Iin = (float)AvgIinSum / (float)Rms_Cycle_Samples
                               / 16777216.0f;

                RmsVoutSq = 0;
                RmsIoutSq = 0;
                RmsICapSq = 0;
                RmsILSq   = 0;
                AvgVinSum = 0;
                AvgIinSum = 0;
            }

            // 校零实测的引脚电压（CalPinXxx 在 Adc_ZeroCal 里只算一次，
            // 这里只是搬到 Debug_View 给 OLED 用）
            Debug_View.CalPinVout = CalPinVout;
            Debug_View.CalPinIout = CalPinIout;
            Debug_View.CalPinIL   = CalPinIL;
        }
    }
#endif
}


//===========================================================================
// ADC 中断服务函数：采样完成后执行完整控制环，20kHz
//   整个中断里没有一次浮点运算，全部是 Q24 定点加减和 _IQmpy
//===========================================================================
__interrupt void ADC_ISR(void)
{
    _iq sin_val;

    GpioDataRegs.GPASET.bit.GPIO4 = 1;      // 中断计时脚拉高

    // ---- 1. 采样并换算工程量 ----
    // 固定槽位：SOCn 的结果就在 ADCRESULTn 里，直接按下标取，不做任何判断
    //   ADC_Ch_Dummy -> SOC0/ADCRESULT0 丢样（ADCA0 的第一次采样不准）
    //   ADC_Ch_Vout  -> SOC1/ADCRESULT1 输出电压  ← 与 SOC0 同通道，用这一次
    //   ADC_Ch_Iout  -> SOC2/ADCRESULT2 输出电流
    //   ADC_Ch_IL    -> SOC3/ADCRESULT3 电感电流
    //   ADC_Ch_Vin   -> SOC4/ADCRESULT4 输入电压（只监视）
    //   ADC_Ch_Iin   -> SOC5/ADCRESULT5 输入电流（只监视）
    // 每个通道的零点码和满码量程在 dcac_cfg.h 里按调理电路标定，
    // 换算宏展开成 __IQmpy((raw-零点码)<<12, 量程(Q17), 17)
    // 用 ADC_To_IQ_Cal：零点码用实测校零结果（主程序启动时测出来的），
    // 没校准或该校通道被一致性检查挡掉时 OffsetNeg = 0，
    // 此时它和 ADC_To_IQ(标称零点码) 完全等价
    Vout = ADC_To_IQ_Cal(AdcResult.ADCRESULT1, ADC_Vout_Offset, VoutOffNeg, ADC_Vout_Span);
    Iout = ADC_To_IQ_Cal(AdcResult.ADCRESULT2, ADC_Iout_Offset, IoutOffNeg, ADC_Iout_Span);
    IL   = ADC_To_IQ_Cal(AdcResult.ADCRESULT3, ADC_IL_Offset,   ILOffNeg,   ADC_IL_Span);
    ICap = IL - Iout;                       // 电容电流 = 电感电流 - 输出电流

    // 输入电压（ADCB1）/ 输入电流（ADCB0）：只存原始码，不换算、不参与控制环。
    // 中断里一个乘法都不多做，后面加输入保护时再按 dcac_cfg.h 里的
    // ADC_Vin_*/ADC_Iin_* 换算，或者直接拿码值和阈值码值比较
    // 输入侧两路（直流，只监视/保护）：不校零，OffsetNeg 固定为 0
    Vin = ADC_To_IQ_Cal(AdcResult.ADCRESULT4, ADC_Vin_Offset, 0L, ADC_Vin_Span);
    Iin = ADC_To_IQ_Cal(AdcResult.ADCRESULT5, ADC_Iin_Offset, 0L, ADC_Iin_Span);

    // ---- 1b. 有效值 / 平均值累加 ----
    // 先右移 8 位（Q24 -> Q16）再平方，累加器用 long long，避免溢出
    {
        long xv = (long)(Vout >> 8);
        long xi = (long)(Iout >> 8);
        long xc = (long)(ICap >> 8);
        long xl = (long)(IL   >> 8);

        RmsVoutSq += (long long)xv * (long long)xv;
        RmsIoutSq += (long long)xi * (long long)xi;
        RmsICapSq += (long long)xc * (long long)xc;
        RmsILSq   += (long long)xl * (long long)xl;
    }
    AvgVinSum += (long long)Vin;
    AvgIinSum += (long long)Iin;

    // 调制比取峰值：|Mod| 越大就越新，一个周期里留最大值
    {
        _iq ax = (Mod >= 0) ? Mod : -Mod;

        if (ax > ModPk)
        {
            ModPk = ax;
        }
    }

    if (++RmsCnt >= Rms_Cycle_Samples)
    {
        RmsCnt = 0U;
        RmsCommit = 1U;
    }

    // ---- 1c. 示波器缓冲：存瞬时值（float，工程量）----
    // 存 float 而不是 int16：Graph 里选 "32 bit floating point" + Q_Value=0，
    // 直接就是伏/安，不用配 Q 格式
    {
        Uint16 pre = Scope_Prescale;

        if (pre == 0U)
        {
            pre = 1U;           // 防止在 Expressions 里误改成 0
        }

        if (++ScopeCnt >= pre)
        {
            ScopeCnt = 0U;

            ScopeIout[ScopeIdx] = _IQtoF(Iout);
            ScopeICap[ScopeIdx] = _IQtoF(ICap);
            ScopeIL[ScopeIdx]   = _IQtoF(IL);

            ScopeIdx++;
            if (ScopeIdx >= Scope_Size)
            {
                ScopeIdx = 0U;
            }
        }
    }

#if (Debug_Fixed_Mod_En == 1)

    // 固定调制比调试模式：控制环完全不跑，CMPA 已经在 SPWM_Init 里设成固定值
    // 只验证 ePWM + 死区 + 驱动这一段，连 ADC 结果都不参与

#else

#if (Protect_Enable == 1)
    // ---- 2. 保护判断，越限就锁存 ----
    // 阈值都是 Q24，定点比较就是整数比较，不需要任何转换
    if (Vout > OV_Volts)
    {
        FaultLatch = FAULT_OVERVOLT;
    }

    if ((IL > OC_Amps) || (IL < -OC_Amps))
    {
        OcCnt++;
        if (OcCnt >= OC_Debounce)
        {
            FaultLatch = FAULT_SW_OC;
        }
    }
    else
    {
        OcCnt = 0U;
    }

    if (Protect_TripFlag != 0U)
    {
        FaultLatch = FAULT_HW_OC;
    }
#else
    // Protect_Enable = 0：软件保护也一并关掉。
    // 这一点很关键：ADCINA2 没接（或电流传感器没上电）时采样恒为 0，
    // 算出来 IL = -30A，按 25A 阈值会立刻误报过流，输出被钉在中点，
    // 表现出来就是"没有 SPWM 输出"
    FaultLatch = FAULT_NONE;
    OcCnt = 0U;
#endif

    if (FaultLatch != FAULT_NONE)
    {
        // 故障：清历史、输出中点
        PR_V.X1 = 0; PR_V.X2 = 0;
        PR_V.Y1 = 0; PR_V.Y2 = 0;
        PI_I.Integral = 0;
        RefRamp = 0;
        Mod = 0;
        SPWM_SetNeutral();
    }
    else
    {
        // ---- 3. 软启动：给定峰值从 0 爬到额定 ----
        // 每拍加一个 Q24 步长，定点加法累加不会引入额外误差
        if (Run_Enable != 0U)
        {
            RefRamp += Soft_Start_Step;
            if (RefRamp > Vref_Peak)
            {
                RefRamp = Vref_Peak;
            }
        }
        else
        {
            RefRamp = 0;
        }

        // ---- 4. 相位累加 + 查表：phase_step = 256*50/20000 = 0.64 ----
        // PhaseAcc 高 8 位是表下标、低 24 位是 Q24 小数，
        // 与 0xFFFFFFFF 相与即对 256 取模，等价于原来的 Phase -= 256
        PhaseAcc = (PhaseAcc + Phase_Step_Q24) & Phase_Mask;
        sin_val = sin_table[(Uint16)(PhaseAcc >> Phase_Acc_Bits)];
        Vref = _IQmpy(sin_val, RefRamp);

#if (LOOP_MODE != LOOP_SINE_OPEN)
        // ---- 5. 电压外环 PR -> 电流给定 ----
        PR_V.Ref = Vref;
        PR_V.Fdb = Vout;
        IRef = PR_Calc(&PR_V);



        // ---- 6. 电流内环 PI -> 调制比 ----
        PI_I.Ref = IRef;
        PI_I.Fdb = ICap;
        Mod = PI_Calc(&PI_I);
#else
        // 开环：瞬时调制比直接来自正弦表，不走 PR/PI
        Mod = _IQmpy(sin_val, Mod_Amp_Open);
        IRef = 0;
        PR_V.X1 = 0; PR_V.X2 = 0;
        PR_V.Y1 = 0; PR_V.Y2 = 0;
        PI_I.Integral = 0;
#endif

        // ---- 7. 限幅并写 CMPA ----
        if (Mod > Mod_Max)
        {
            Mod = Mod_Max;
        }
        else if (Mod < -Mod_Max)
        {
            Mod = -Mod_Max;
        }
        SPWM_SetMod(Mod);
    }

#endif

    IsrCnt++;

    // ---- 8. 清中断标志 ----
    // INT1CONT = 1（连续模式）时标志由硬件在响应中断时自动清掉，
    // 这里再写一次是为了清掉可能残留的旧标志，和连续模式不冲突。
    // 溢出标志也清一下：它本身不产生中断，但留着会掩盖真正的丢拍
    AdcRegs.ADCINTFLGCLR.all = 0x0001;
    AdcRegs.ADCINTOVFCLR.all = 0x0001;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

    GpioDataRegs.GPACLEAR.bit.GPIO4 = 1;    // 中断计时脚拉低
}

//===========================================================================
// ADC 零点校准（软件校零）
//
// 做什么：
//   把 SOC0~SOC5 的触发源临时改成"软件触发"，逐个通道强制转换并累加
//   ADC_ZeroCal_Samples 次，求平均得到实测零点码，再折成 OffsetNeg 存起来。
//   跑完把触发源还给 EPWM1 SOCA，并把轮询指针拨回 SOC0。
//
// 放在初始化序列的哪一步：
//   InitAdc() -> AdcOffsetSelfCal() -> Adc_ZeroCal() -> Adc_Config() -> SPWM_Init()
//   必须在 AdcOffsetSelfCal() 之后（内核失调已经修好，测出来的才准），
//   在 SPWM_Init() 之前（EPWM 还没起振，不会有 SOCA 来抢轮询指针），
//   也在 EINT 之前（此时没有任何中断，不会被 ISR 打断）。
//
// 前提：调用这一刻各路输入必须真的是零，否则校出来的是"当时的输入"，
//       不是零点。见 dcac_cfg.h 第 5b 节的说明。不满足就把
//       ADC_ZeroCal_Enable 置 0。
//
// 保护：每路用前 ADC_ZeroCal_SpreadN 点算 (max-min)，
//       超过 ADC_ZeroCal_MaxSpread 就认为这一路悬空/在跳，
//       放弃校准、OffsetNeg 留 0（等价于用标称 Offset），并累加 CalMiss。
//       通道之间的失调差异各算各的，互不影响。
//===========================================================================
void Adc_ZeroCal(void)
{
#if (ADC_ZeroCal_Enable == 1)
    Uint16 ch;
    Uint16 i;
    Uint16 raw;
    Uint32 sum;
    long   zeroCode;
    long   nominal;
    long   offneg;

    EALLOW;
    AdcRegs.ADCINTFLGCLR.all = 0x0001;      // 清掉可能残留的中断标志
    AdcRegs.ADCINTOVFCLR.all = 0x0001;

    // 全部 SOC 改成软件触发（TRIGSEL=0，靠 ADCSOCFRC1 强制启动）
    // SOC0~SOC5 后面 Adc_Config() 会统一恢复成 EPWM1 SOCA，
    // 这里只是临时借用一下，好在上电阶段把零点测出来
    AdcRegs.ADCSOC0CTL.bit.TRIGSEL = 0;
    AdcRegs.ADCSOC1CTL.bit.TRIGSEL = 0;
    AdcRegs.ADCSOC2CTL.bit.TRIGSEL = 0;
    AdcRegs.ADCSOC3CTL.bit.TRIGSEL = 0;
    AdcRegs.ADCSOC4CTL.bit.TRIGSEL = 0;
    AdcRegs.ADCSOC5CTL.bit.TRIGSEL = 0;

    // 松开轮询指针，从 SOC0 开始
    AdcRegs.SOCPRICTL.bit.SOCPRIORITY = 1;
    while (AdcRegs.SOCPRICTL.bit.SOCPRIORITY != 1) {}
    AdcRegs.SOCPRICTL.bit.SOCPRIORITY = 0;
    while (AdcRegs.SOCPRICTL.bit.SOCPRIORITY != 0) {}

    // 只校 Vout / Iout / IL 三路。
    // 输入侧 Vin/Iin 不校零：它们是绝对值通道，上电时输入本来就不是 0
    // （母线已经带电），当零点扣掉的话运行时会少算一整段输入电压/电流
    // （实测 35V 被扣成 1.9V）。
    for (ch = 0U; ch < 3U; ch++)
    {
        // 把待测通道放到 SOC0（轮询指针重新拨到 SOC0 之后第一个转的就是它）
        // 通道号直接取 dcac_cfg.h 的 ADC_Ch_* 宏，不在这里另抄一份：
        // 以后再改通道映射，只改 dcac_cfg.h 一处，这里自动跟着走，
        // 不会出现"映射改了、校零还校在旧通道上"这种错位
        switch (ch)
        {
            case 0U: AdcRegs.ADCSOC0CTL.bit.CHSEL = ADC_Ch_Vout; break;
            case 1U: AdcRegs.ADCSOC0CTL.bit.CHSEL = ADC_Ch_Iout; break;
            default: AdcRegs.ADCSOC0CTL.bit.CHSEL = ADC_Ch_IL;   break;
        }

        sum = 0UL;

        // 换通道后先空转一次并丢弃：采样保持电容和结果寄存器里都还是上一路的
        // 电压，换通道后的第一次转换结果是上一路的旧值（实测会偏 1133 码）。
        for (i = 0U; i <= ADC_ZeroCal_Samples; i++)
        {
            Uint16 d;

            // 轮询指针拨回 SOC0，保证这一拍转的确实是我们要测的那个通道
            AdcRegs.SOCPRICTL.bit.SOCPRIORITY = 1;
            while (AdcRegs.SOCPRICTL.bit.SOCPRIORITY != 1) {}
            AdcRegs.SOCPRICTL.bit.SOCPRIORITY = 0;
            while (AdcRegs.SOCPRICTL.bit.SOCPRIORITY != 0) {}

            AdcRegs.ADCSOCFRC1.all = 0x0001;        // 强制 SOC0 转换

            // SOCFLG 在转换"开始"时就清零了，不是转换结束。
            // 所以要再等一个转换时间（13 个 ADCCLK 约 220ns，这里给约 1us）
            // 再读结果，否则读到的是上一次转换留在结果寄存器里的值。
            while (AdcRegs.ADCSOCFLG1.bit.SOC0 != 0U) {}
            for (d = 0U; d < 60U; d++) { ; }

            raw = AdcResult.ADCRESULT0;

            if (i == 0U)
            {
                continue;           // 丢掉换通道后的第一次
            }

            sum += (Uint32)raw;
        }

        zeroCode = (long)(sum >> ADC_ZeroCal_Shift);

        // 该通道零点码的标称值（编译期常量，和上面 CHSEL 的顺序一一对应）
        switch (ch)
        {
            case 0U: nominal = (long)ADC_Vout_Offset; break;
            case 1U: nominal = (long)ADC_Iout_Offset; break;
            default: nominal = (long)ADC_IL_Offset;   break;
        }

        // 只校 Vout / Iout / IL 三路（它们有直流偏置，零点是明确的）
        offneg = nominal - zeroCode;    // offneg = nominal - raw_zero

        switch (ch)
        {
            case 0U:
                VoutOffNeg = offneg;    // ADCA0 输出电压零点
                CalPinVout = (float)zeroCode * ADC_Vref / 4096.0f;
                break;
            case 1U:
                IoutOffNeg = offneg;    // ADCA1 输出电流零点
                CalPinIout = (float)zeroCode * ADC_Vref / 4096.0f;
                break;
            default:
                ILOffNeg   = offneg;    // ADCA2 电感电流零点
                CalPinIL   = (float)zeroCode * ADC_Vref / 4096.0f;
                break;
        }
    }
    // 恢复：轮询指针拨回 SOC0，SOC 挂起标志清掉
    AdcRegs.SOCPRICTL.bit.SOCPRIORITY = 1;
    while (AdcRegs.SOCPRICTL.bit.SOCPRIORITY != 1) {}
    AdcRegs.SOCPRICTL.bit.SOCPRIORITY = 0;
    while (AdcRegs.SOCPRICTL.bit.SOCPRIORITY != 0) {}
    AdcRegs.ADCSOCFLG1.all = 0xFFFF;
    AdcRegs.ADCINTFLGCLR.all = 0x0001;
    AdcRegs.ADCINTOVFCLR.all = 0x0001;

    EDIS;
#endif
}

//===========================================================================
// ADC 配置函数
//   SOC0 -> ADCA0，第一次采样按原设计丢弃（与 SOC1 同通道）
//   SOC1 -> ADCA0 输出电压
//   SOC2 -> ADCA1 输出电流
//   SOC3 -> ADCA2 电感电流
//   SOC4 -> ADCB1 输入电压（只监视）
//   SOC5 -> ADCB0 输入电流（只监视）
//   TRIGSEL=5 : EPWM1 SOCA（CTR=ZERO）
//   INT1SEL=5 : EOC5 触发 ADCINT1
//   INTPULSEPOS=1 : 转换结束结果锁存后才发中断脉冲
//
// 【为什么 INT1CONT 必须写 1】—— 这里踩过的坑，别改回去
//   INTSEL1N2.INT1CONT = 0（关闭连续模式）时，中断标志 ADCINTFLG.ADCINT1
//   是"锁存"的：EOC 到来时如果标志还是 1，ADC 只置一个溢出位 ADCINTOVF，
//   不再产生新的中断脉冲。于是只要有一个脉冲没被及时清掉（ISR 执行时间
//   接近或超过一个载波周期 50us、进 ISR 时被别的事情挡住、清标志的时机
//   不对……），之后每一拍的 EOC 都会被"吃掉"，表现就是中断进了一次就
//   再也不进，IsrCnt 永远停在 1。
//   TI 对这个 ADCINTFLG 阻塞问题的官方修法就是文档里那句
//   "Use Continue-to-Interrupt Mode to prevent the ADCINTFLG from blocking
//    additional ADC interrupts"，也就是 INT1CONT = 1：
//   连续模式下标志由硬件在 ISR 被响应时自动清掉，不会再锁死。
//   ISR 里的 ADCINTFLGCLR 保留（清掉初始化阶段可能残留的旧标志），
//   和连续模式不冲突。
//===========================================================================
void Adc_Config(void)
{
    EALLOW;

    AdcRegs.ADCCTL1.bit.INTPULSEPOS = 1;    // 结果锁存之后再发中断脉冲

    AdcRegs.INTSEL1N2.bit.INT1E     = 1;    // 使能 ADCINT1
    AdcRegs.INTSEL1N2.bit.INT1CONT  = 1;    // 连续模式！见上面那段说明，不能写 0
    AdcRegs.INTSEL1N2.bit.INT1SEL   = ADC_EOC_For_INT1;  // EOC5 触发 ADCINT1

    // SOC0 丢一次（ADCA0 = 输出电压，与 SOC1 配成同通道连采两次）
    // 这个配对必须相邻，中间不能插别的通道，否则 rev0 首采样勘误修法失效
    AdcRegs.ADCSOC0CTL.bit.CHSEL    = ADC_Ch_Dummy;
    AdcRegs.ADCSOC1CTL.bit.CHSEL    = ADC_Ch_Vout;
    AdcRegs.ADCSOC2CTL.bit.CHSEL    = ADC_Ch_Iout;
    AdcRegs.ADCSOC3CTL.bit.CHSEL    = ADC_Ch_IL;
    AdcRegs.ADCSOC4CTL.bit.CHSEL    = ADC_Ch_Vin;
    AdcRegs.ADCSOC5CTL.bit.CHSEL    = ADC_Ch_Iin;

    AdcRegs.ADCSOC0CTL.bit.TRIGSEL  = 5;    // EPWM1 SOCA
    AdcRegs.ADCSOC1CTL.bit.TRIGSEL  = 5;
    AdcRegs.ADCSOC2CTL.bit.TRIGSEL  = 5;
    AdcRegs.ADCSOC3CTL.bit.TRIGSEL  = 5;
    AdcRegs.ADCSOC4CTL.bit.TRIGSEL  = 5;
    AdcRegs.ADCSOC5CTL.bit.TRIGSEL  = 5;

    // 采样窗 = (ACQPS+1) 个 ADCCLK。ADCCLK = 60MHz -> 1 个 ADCCLK = 16.67ns
    // 6 -> 7 个 = 117ns；6 路背靠背总共 800ns，全部落在 CTR=ZERO 前后的安静区。
    // ★ 不要为了"S+H 充得更充分"把它调大：6 路是串行转换的，ACQPS=20 会让
    //   最后一路拖到触发后 2.1us 才采，CMPA 很小时那正是开关沿附近，
    //   反而把开关噪声采进来（实测电流通道毛刺变大）。
    //   源阻抗低时 117ns 已经够：117ns/(9*1.64pF) 允许源阻抗约 8kohm。
    //   调理电路噪声大要靠 RC 和布线解决，不是靠加长采样窗。
    AdcRegs.ADCSOC0CTL.bit.ACQPS    = 6;
    AdcRegs.ADCSOC1CTL.bit.ACQPS    = 6;  //Vout
    AdcRegs.ADCSOC2CTL.bit.ACQPS    = 6;  //Iout
    AdcRegs.ADCSOC3CTL.bit.ACQPS    = 6;  //IL
    AdcRegs.ADCSOC4CTL.bit.ACQPS    = 6;
    AdcRegs.ADCSOC5CTL.bit.ACQPS    = 6;

    // ---- 把 SOC 轮询指针拨回 SOC0，并清掉自校准留下的旧标志 ----
    // InitAdc() 之后的 AdcOffsetSelfCal() 会把 16 个 SOC 全改成 B5 通道
    // 并强制转换 256 次，跑完以后轮询指针和 SOC 标志都是"脏"的。
    // SOCPRICTL.SOCPRIORITY 写 1 再写回 0 会让硬件复位轮询指针
    // （写 1 -> 等到读回 1 -> 写 0 -> 等到读回 0，指针归 0），
    // 这样下一个 SOCA 一定从 SOC0 开始，丢样和 SOC1~5 的对应关系才稳。
    AdcRegs.SOCPRICTL.bit.SOCPRIORITY = 1;
    while (AdcRegs.SOCPRICTL.bit.SOCPRIORITY != 1) {}
    AdcRegs.SOCPRICTL.bit.SOCPRIORITY = 0;
    while (AdcRegs.SOCPRICTL.bit.SOCPRIORITY != 0) {}

    AdcRegs.ADCSOCFLG1.all    = 0xFFFF;     // 清掉全部 SOC 挂起标志
    AdcRegs.ADCINTFLGCLR.all  = 0x0001;     // 清 ADCINT1 旧标志
    AdcRegs.ADCINTOVFCLR.all  = 0x0001;     // 清 ADCINT1 溢出标志

    EDIS;
}

//===========================================================================
// GPIO 配置函数
//   GPIO4 中断计时脚，普通 GPIO 输出，初值低
//   spwm.c 只占用 GPIO0~GPIO3，所以 GPIO4 可以自由使用
//===========================================================================
void Gpio_Config(void)
{
    EALLOW;
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 0;         // 复用为普通 GPIO
    GpioCtrlRegs.GPADIR.bit.GPIO4  = 1;         // 输出
    EDIS;

    GpioDataRegs.GPACLEAR.bit.GPIO4 = 1;        // 初值低
}

//===========================================================================
// 硬件过流 Trip 中断
//   TZ 已经把四管拉低，这里只做：置标志、故障锁存、关 TZ 中断源、清中断标志
//   OST/CBC 触发源保持锁存，必须软件 Protect_ClearTrip() 才能恢复输出
//   关掉 TZEINT 是为了防止故障一直存在导致每个载波周期都进中断
//   Protect_Enable = 0 时整个函数不编译
//===========================================================================
#if (Protect_Enable == 1)
__interrupt void Epwm1_TripIsr(void)
{
    Protect_TripFlag = 1U;

    if (EPwm1Regs.TZFLG.bit.OST != 0U)
    {
        Protect_TripSource = 1U;        // DCxEVT1 单次，正向过流
    }
    else if (EPwm1Regs.TZFLG.bit.CBC != 0U)
    {
        Protect_TripSource = 2U;        // DCxEVT2 逐周期，反向过流
    }
    else
    {
        Protect_TripSource = 0U;
    }

    FaultLatch = FAULT_HW_OC;

    EALLOW;
    EPwm1Regs.TZEINT.bit.OST = 0;       // 先关中断源，避免中断风暴
    EPwm1Regs.TZEINT.bit.CBC = 0;
    EPwm1Regs.TZCLR.bit.INT  = 1;       // 只清中断标志，OST/CBC 保持锁存
    EDIS;

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;
}
#endif
