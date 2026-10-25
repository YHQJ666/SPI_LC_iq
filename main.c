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
 * 环路模式由 dcac_cfg.h 的 Loop_Mode 选择：0 开环，1 闭环
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
volatile long VinOffNeg  = 0L;
volatile long IinOffNeg  = 0L;
volatile Uint16 CalDone  = 0U;      // 1 = 校零已执行（调试观察用）
volatile Uint16 CalMiss  = 0U;      // 因一致性检查没过而放弃校准的通道数

// 校零时实测到的各通道零点数据，调试观察用（索引和 Adc_ZeroCal 的 ch 对应）
//   MeanCode：该通道 ADC 引脚电压平均下来的原始码（12 位，0~4095）
//             引脚电压 = MeanCode / 4096 * 3.3V，调试器里懒得手算就改看
//             Debug_View.CalPinVxxx（已经是 V 了）
//   Spread  ：前 64 点的 (max-min)，判断这一路是不是悬空/在跳
//   Valid   ：1 = 通过了 Spread 检查，校零生效；0 = 放弃，用标称零点
volatile Uint16 CalMeanCode[6] = {0U, 0U, 0U, 0U, 0U, 0U};
volatile Uint16 CalSpread[6]   = {0U, 0U, 0U, 0U, 0U, 0U};
volatile Uint16 CalValid[6]    = {0U, 0U, 0U, 0U, 0U, 0U};

//===========================================================================
// 有效值 / 平均值累加器（真有效值，做法参考 TI sineAnalyzerWithPowerMeas）
// C28x 的 long 是 32 位，累加平方会溢出，所以累加器必须用 long long；
// 而且 x 先右移 8 位（Q24->Q16）再平方，否则 400 拍同样会溢出：
//   51V 时 (51*2^16)^2 * 400 = 4.5e15 < 9.2e18  OK
//===========================================================================
volatile long long RmsVoutSq = 0;
volatile long long RmsIoutSq = 0;
volatile long long RmsICapSq = 0;
volatile long long AvgVinSum = 0;
volatile long long AvgIinSum = 0;
volatile Uint16 RmsCnt    = 0U;
volatile Uint16 RmsCommit = 0U;

//===========================================================================
// 示波器数据缓冲区：CCS 的 Graph 工具按"数组名 + 长度"读一段连续内存，
// 所以必须用全局数组（参考 TI 参考设计里的 float dBuff1[200]）。
// 存的是**瞬时值**（ADCRESULT 减直流偏置再换算成工程量），用来当示波器看波形。
// 用 float：Graph 里 Data Type 直接选 "32 bit floating point"、Q_Value=0，
// 不用配 Q 格式，最不容易出错（int16+Q8 在 CCS 的 Graph 里显示不正常）。
//===========================================================================
// 每通道 160 点（float 占 2 字 -> 320 字/通道，4 通道共 1280 字）。
// 4 个数组固定在链接器 SCOPEBUF 段的 0x9B00 起，代码长度变化不会挪动它们，
// 所以 CCS Graph 里存的地址不会失效。
#define Scope_Size      160U

// RMS 一阶 IIR 平滑系数 1/2^Rms_SmoothShift。
// ★ 注意更新率是 50Hz（每 20ms 一次），不是 20kHz！
//   时间常数 = 2^shift × 20ms：
//     shift=2  -> 80 ms，3τ≈240ms 收敛
//     shift=1  -> 40 ms，3τ≈120ms 收敛
//   400 点正好一个工频周期，原始 RMS 已经比较稳，这里只做轻微平滑
#define Rms_SmoothShift 2U

// 放 scopeBufs 段，避免和主控变量混在 .ebss 里
// ScopeIL 单独放 scopeBufs2（RAML0），因为 RAML1L2L3 剩下的空间不够第 4 个数组
#pragma DATA_SECTION(ScopeVout, "scopeBufs")
#pragma DATA_SECTION(ScopeIout, "scopeBufs")
#pragma DATA_SECTION(ScopeICap, "scopeBufs")
#pragma DATA_SECTION(ScopeIL,   "scopeBufs2")

volatile float ScopeVout[Scope_Size];   // 输出电压瞬时值 V
volatile float ScopeIout[Scope_Size];   // 输出电流瞬时值 A
volatile float ScopeICap[Scope_Size];   // 电容电流瞬时值 A
volatile float ScopeIL[Scope_Size];     // 电感电流瞬时值 A
volatile Uint16 ScopeIdx = 0U;          // 写指针
volatile Uint16 ScopeCnt = 0U;          // 抽取计数

// 抽取倍数：每 N 个中断存 1 点（160 点一屏）。
//   1 -> 20kHz，一屏 8ms   = 0.4 个工频周期（看开关纹波细节）
//   3 -> 6.7kHz，一屏 24ms = 1.2 个工频周期
//   5 ->  4kHz，一屏 40ms  = 2 个工频周期
// 改完 Graph 的 Sampling Rate Hz 要跟着改成 20000/N
volatile Uint16 Scope_Prescale = 5U;

// RMS 平滑滤波状态（Q24 定点，每 20ms 更新一次）
volatile _iq FiltVout = 0;
volatile _iq FiltIout = 0;
volatile _iq FiltICap = 0;

volatile _iq   dbgSin   = 0;

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
    float IL;           // 电感电流瞬时值 A

    // 输入侧（直流）平均电压 / 平均码
    float Vin;          // 输入电压 V
    float Iin;          // 输入电流 A
    float VinCode;      // 输入电压 ADC 平均码（12 位，0~4095）
    float IinCode;      // 输入电流 ADC 平均码
    float VinPinV;      // 输入电压 ADC 引脚平均电压 V（分压后的，标称 0V）
    float IinPinV;      // 输入电流 ADC 引脚平均电压 V（标称 0V）

    // 校零时各通道 ADC 引脚上的实际电压（V）
    // 就是"零输入时引脚上的基准电压"，例如 2.48 / 1.49 这种数，
    // 调试器里可以直接和万用表实测值对比，验调理电路对不对
    float CalPinIout;   // 输出电流通道引脚电压，实测约 2.409V
    float CalPinVout;   // 输出电压通道引脚电压，实测约 1.502V
    float CalPinIL;     // 电感电流通道引脚电压，实测约 2.431V
    float CalPinVin;    // 输入电压通道引脚电压，标称 0V（分压后）
    float CalPinIin;    // 输入电流通道引脚电压，标称 0V
} Debug_View;

#endif

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

                RmsCommit = 0U;

                // 累加器是 sum((x>>8)^2) = sum(A^2 * 2^32)
                // 均值 = /N，再 /2^32 就回到 A^2 的真实值（V^2 / A^2）
                msV = (float)RmsVoutSq / (float)Rms_Cycle_Samples / 4294967296.0f;
                msI = (float)RmsIoutSq / (float)Rms_Cycle_Samples / 4294967296.0f;
                msC = (float)RmsICapSq / (float)Rms_Cycle_Samples / 4294967296.0f;

                // 开方得真实有效值，再走一阶 IIR 平滑：
                //   y += (x - y) / 2^Rms_SmoothShift
                // 每 20ms 更新一次，shift=2 -> 时间常数 80ms
                {
                    _iq xv = (_iq)(sqrtf(msV) * 16777216.0f);
                    _iq xi = (_iq)(sqrtf(msI) * 16777216.0f);
                    _iq xc = (_iq)(sqrtf(msC) * 16777216.0f);

                    FiltVout += (xv - FiltVout) >> Rms_SmoothShift;
                    FiltIout += (xi - FiltIout) >> Rms_SmoothShift;
                    FiltICap += (xc - FiltICap) >> Rms_SmoothShift;

                    Debug_View.VoutRms = _IQtoF(FiltVout);
                    Debug_View.IoutRms = _IQtoF(FiltIout);
                    Debug_View.ICapRms = _IQtoF(FiltICap);
                }

                Debug_View.VoutAmp = Debug_View.VoutRms * 1.41421356f;
                Debug_View.IoutAmp = Debug_View.IoutRms * 1.41421356f;

                Debug_View.Vin = (float)AvgVinSum / (float)Rms_Cycle_Samples
                               / 16777216.0f;
                Debug_View.Iin = (float)AvgIinSum / (float)Rms_Cycle_Samples
                               / 16777216.0f;

                Debug_View.VinCode = (float)AvgVinSum / (float)Rms_Cycle_Samples
                                   / 16777216.0f / ADC_Vin_Span * 4096.0f
                                   + (float)ADC_Vin_Offset;
                Debug_View.IinCode = (float)AvgIinSum / (float)Rms_Cycle_Samples
                                   / 16777216.0f / ADC_Iin_Span * 4096.0f
                                   + (float)ADC_Iin_Offset;
                Debug_View.VinPinV = Debug_View.VinCode * ADC_Vref / 4096.0f;
                Debug_View.IinPinV = Debug_View.IinCode * ADC_Vref / 4096.0f;

                RmsVoutSq = 0;
                RmsIoutSq = 0;
                RmsICapSq = 0;
                AvgVinSum = 0;
                AvgIinSum = 0;
            }

            // 电感电流瞬时值（核对采样用）
            Debug_View.IL = _IQtoF(IL);

            // 校零时各通道 ADC 引脚的实际电压，单位 V
            // = 平均码 / 4096 * 3.3，可以直接和万用表打在 ADC 引脚上的读数对比
            Debug_View.CalPinVout = (float)CalMeanCode[0] * ADC_Vref / 4096.0f;
            Debug_View.CalPinIout = (float)CalMeanCode[1] * ADC_Vref / 4096.0f;
            Debug_View.CalPinIL   = (float)CalMeanCode[2] * ADC_Vref / 4096.0f;
            Debug_View.CalPinVin  = (float)CalMeanCode[3] * ADC_Vref / 4096.0f;
            Debug_View.CalPinIin  = (float)CalMeanCode[4] * ADC_Vref / 4096.0f;
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
    Vin = ADC_To_IQ_Cal(AdcResult.ADCRESULT4, ADC_Vin_Offset, VinOffNeg, ADC_Vin_Span);
    Iin = ADC_To_IQ_Cal(AdcResult.ADCRESULT5, ADC_Iin_Offset, IinOffNeg, ADC_Iin_Span);

    // ---- 1b. 有效值 / 平均值累加 ----
    // 先右移 8 位（Q24 -> Q16）再平方，累加器用 long long，避免溢出
    {
        long xv = (long)(Vout >> 8);
        long xi = (long)(Iout >> 8);
        long xc = (long)(ICap >> 8);

        RmsVoutSq += (long long)xv * (long long)xv;
        RmsIoutSq += (long long)xi * (long long)xi;
        RmsICapSq += (long long)xc * (long long)xc;
    }
    AvgVinSum += (long long)Vin;
    AvgIinSum += (long long)Iin;

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

            ScopeVout[ScopeIdx] = _IQtoF(Vout);
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

        dbgSin =sin_val;

#if (Loop_Mode == Loop_Closed)
        // ---- 5. 电压外环 PR -> 电容电流给定 ----
        PR_V.Ref = Vref;
        PR_V.Fdb = Vout;
//        IRef = PR_Calc(&PR_V);

        Mod = PR_Calc(&PR_V);

        // ---- 6. 电流内环 PI -> 调制比，反馈是电容电流 ----
//        PI_I.Ref = IRef;
//        PI_I.Fdb = ICap;
//        Mod = PI_Calc(&PI_I);
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
    Uint16 mx;
    Uint16 mn;
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

    for (ch = 0U; ch < 5U; ch++)
    {
        // 把待测通道放到 SOC0（轮询指针重新拨到 SOC0 之后第一个转的就是它）
        // 通道号直接取 dcac_cfg.h 的 ADC_Ch_* 宏，不在这里另抄一份：
        // 以后再改通道映射，只改 dcac_cfg.h 一处，这里自动跟着走，
        // 不会出现"映射改了、校零还校在旧通道上"这种错位
        // 顺序和 Adc_Config 里的 SOC1~SOC5 一一对应
        // （SOC0 只是丢样，不单独校准）
        switch (ch)
        {
            case 0U: AdcRegs.ADCSOC0CTL.bit.CHSEL = ADC_Ch_Vout; break;
            case 1U: AdcRegs.ADCSOC0CTL.bit.CHSEL = ADC_Ch_Iout; break;
            case 2U: AdcRegs.ADCSOC0CTL.bit.CHSEL = ADC_Ch_IL;   break;
            case 3U: AdcRegs.ADCSOC0CTL.bit.CHSEL = ADC_Ch_Vin;  break;
            default: AdcRegs.ADCSOC0CTL.bit.CHSEL = ADC_Ch_Iin;  break;
        }

        sum = 0UL;
        mx  = 0U;
        mn  = 4095U;

        // 换通道后先空转一次并丢弃：采样保持电容和结果寄存器里都还是上一路的
        // 电压，换通道后的第一次转换结果不可信。不丢这一次，一致性检查就会
        // 把"上一路到这一路"的电压差当成跳幅直接判死（实测 1133 码 = 913mV，
        // 恰好等于 A0 与 A1 的码差），校零永远通不过。
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

            if (i <= ADC_ZeroCal_SpreadN)
            {
                if (raw > mx) { mx = raw; }
                if (raw < mn) { mn = raw; }
            }

            sum += (Uint32)raw;
        }

        zeroCode = (long)(sum >> ADC_ZeroCal_Shift);    // 256 点求平均 = 右移 8

        // 记录实测数据给调试器看：平均码、跳幅
        // 注意 CalMeanCode 是无条件记录的——就算一致性检查没过、
        // 这一路不参与校零，它的平均引脚电压仍然是真实的，照样能和万用表对
        CalMeanCode[ch] = (Uint16)zeroCode;
        CalSpread[ch]   = (Uint16)(mx - mn);

        // 该通道零点码的标称值（编译期常量，和上面 CHSEL 的顺序一一对应）
        switch (ch)
        {
            case 0U: nominal = (long)ADC_Vout_Offset; break;
            case 1U: nominal = (long)ADC_Iout_Offset; break;
            case 2U: nominal = (long)ADC_IL_Offset;   break;
            case 3U: nominal = (long)ADC_Vin_Offset;  break;
            default: nominal = (long)ADC_Iin_Offset;  break;
        }

        // 一致性检查：跳得太厉害说明这一路悬空或没接好，宁可不校
        if (CalSpread[ch] > (Uint16)ADC_ZeroCal_MaxSpread)
        {
            offneg = 0L;
            CalValid[ch] = 0U;              // 该路不可信，不参与校零
            CalMiss++;
        }
        else
        {
            offneg = nominal - zeroCode;    // offneg = nominal - raw_zero
            CalValid[ch] = 1U;
        }

        switch (ch)
        {
            case 0U: VoutOffNeg = offneg; break;   // ADCA0 输出电压零点
            case 1U: IoutOffNeg = offneg; break;   // ADCA1 输出电流零点
            case 2U: ILOffNeg   = offneg; break;   // ADCA2 电感电流零点
            case 3U: VinOffNeg  = offneg; break;   // ADCB1 输入电压零点
            default: IinOffNeg  = offneg; break;   // ADCB0 输入电流零点
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

    CalDone = 1U;
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
