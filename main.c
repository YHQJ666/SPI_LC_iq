/*
 * main.c
 *
 * DSP28035 单相离网逆变（全部在 CPU 上运行，不使用 CLA）
 *
 * 硬件链路：
 *   ePWM1 增减计数 TBPRD=1500 -> fsw=20kHz
 *     CTR=ZERO -> SOCA -> ADC
 *        SOC0 丢弃
 *        SOC1 = ADCINA0 输出电流 i_o
 *        SOC2 = ADCINA1 输出电压 v_o
 *        SOC3 = ADCINA2 电感电流 i_L
 *     EOC3 -> ADCINT1 -> CPU 的 ADC_ISR
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
 * 环路模式由 dcac_cfg.h 的 DCAC_LOOP_MODE 选择：0 开环，1 闭环
 * 本文件不含并网逻辑（离网模式）
 */

#include "DSP28x_Project.h"             // 项目顶层主头文件
#include "DSP2803x_GlobalPrototypes.h"  // 全局函数原型声明文件
#include "DSP2803x_EPwm_defines.h"
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
void Gpio_Config(void);
__interrupt void ADC_ISR(void);
#if (DCAC_PROTECT_ENABLE == 1)
__interrupt void Epwm1_TripIsr(void);
#endif

// 故障码
#define FAULT_NONE      (0U)
#define FAULT_HW_OC     (1U)        // 硬件过流 Trip
#define FAULT_SW_OC     (2U)        // 软件过流
#define FAULT_OVERVOLT  (3U)        // 输出过压

// 正弦表 -1 ~ +1，256 点，编译期转成 Q24 整数（_IQ() 是编译期常量折叠）
// sin_table[i] = _IQ(sin(2*pi*i/256))
static const _iq sin_table[DCAC_SIN_TABLE_SIZE] =
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
volatile _iq  Mod  = 0;             // 调制比
volatile Uint32 PhaseAcc = 0;       // 相位累加器：高 8 位查表下标，低 24 位 Q24 小数
volatile _iq  RefRamp = 0;          // 软启动给定峰值

volatile _iq   dbgSin   = 0;

// 运行控制与故障
volatile Uint16 Run_Enable = 1U;    // 1=允许运行（自动软启动），0=停机
volatile Uint16 FaultClear = 0U;    // 置 1 清除故障锁存
volatile Uint16 FaultLatch = FAULT_NONE;
volatile Uint16 OcCnt = 0U;         // 软件过流去抖计数
volatile Uint16 IsrCnt = 0U;        // 中断次数，调试观察用

//===========================================================================
// 主函数入口
//===========================================================================
void main(void)
{
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
    InitAdc();
    AdcOffsetSelfCal();
    Adc_Config();

    SPWM_Init();
    Gpio_Config();

#if (DCAC_PROTECT_ENABLE == 1)
    // 过流保护先建立，再开中断
    Protect_InitOCP();
#endif

    // 中断向量绑定
    // 注意：TZ 中断的向量绑定和下面的 PIE 使能必须成对出现，
    // 只开使能不绑向量的话，一旦 TZ 触发会跳进 TI 默认 ISR 里
    // 那句 __asm("ESTOP0"); for(;;); ，CPU 直接卡死
    EALLOW;
    PieVectTable.ADCINT1     = &ADC_ISR;            // ADC 中断：控制环
#if (DCAC_PROTECT_ENABLE == 1)
    PieVectTable.EPWM1_TZINT = &Epwm1_TripIsr;      // 硬件过流
#endif
    EDIS;

    // 初始化 PR：电压外环，准谐振 w0 = 314.159, wc = 10
    // r = w0*Ts/2、q = wc*Ts/2 已经在 dcac_cfg.h 里按编译期常量算成 Q24
    // （w0 = 314 超出 Q24 的动态范围，所以在设计期就折进 r、q，不进运行期），
    // 因此初始化阶段也全是定点运算
    PR_Design(&PR_V, DCAC_PR_KP_DEFAULT, DCAC_PR_KR_DEFAULT,
              DCAC_PR_R_IQ, DCAC_PR_Q_IQ);
    PR_V.Ref = 0;
    PR_V.Fdb = 0;
    PR_V.X1 = 0; PR_V.X2 = 0;
    PR_V.Y1 = 0; PR_V.Y2 = 0;
    PR_V.OutMin = -DCAC_IREF_MAX;       // 电容电流给定限幅
    PR_V.OutMax =  DCAC_IREF_MAX;

    // 初始化 PI_I：电容电流内环，输出就是调制比
    PI_I.Ref = 0;
    PI_I.Fdb = 0;
    PI_I.Kp = DCAC_KI_KP_DEFAULT;
    PI_I.KiTs = DCAC_KI_KI_TS_DEFAULT;  // 积分增益已经折进采样周期 Ki*Ts
    PI_I.Integral = 0;
    PI_I.IntMin = -DCAC_MOD_MAX;
    PI_I.IntMax =  DCAC_MOD_MAX;
    PI_I.OutMin = -DCAC_MOD_MAX;
    PI_I.OutMax =  DCAC_MOD_MAX;

    // 开 PIE 和 CPU 中断
    EALLOW;
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;      // ADCINT1
    IER |= M_INT1;
#if (DCAC_PROTECT_ENABLE == 1)
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
    }
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
    // SOC0 的第一次采样按原设计丢弃，这里从 SOC1 开始读
    // 换算宏展开成 __IQmpy((raw-2048)<<12, 2*满量程(Q17), 17)：
    // (raw-2048)<<12 是 (raw-2048)/4096 的 Q24，乘 2*满量程 正好得到工程量
    Iout = DCAC_ADC_TO_IQ(AdcResult.ADCRESULT1, DCAC_IOUT_FULL_SCALE_F);
    Vout = DCAC_ADC_TO_IQ(AdcResult.ADCRESULT2, DCAC_VOUT_FULL_SCALE_F);
    IL   = DCAC_ADC_TO_IQ(AdcResult.ADCRESULT3, DCAC_IL_FULL_SCALE_F);
    ICap = IL - Iout;                       // 电容电流 = 电感电流 - 输出电流

#if (DCAC_DEBUG_FIXED_MOD_EN == 1)

    // 固定调制比调试模式：控制环完全不跑，CMPA 已经在 SPWM_Init 里设成固定值
    // 只验证 ePWM + 死区 + 驱动这一段，连 ADC 结果都不参与

#else

#if (DCAC_PROTECT_ENABLE == 1)
    // ---- 2. 保护判断，越限就锁存 ----
    // 阈值都是 Q24，定点比较就是整数比较，不需要任何转换
    if (Vout > DCAC_OV_VOLTS)
    {
        FaultLatch = FAULT_OVERVOLT;
    }

    if ((IL > DCAC_OC_AMPS) || (IL < -DCAC_OC_AMPS))
    {
        OcCnt++;
        if (OcCnt >= DCAC_OC_DEBOUNCE)
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
    // DCAC_PROTECT_ENABLE = 0：软件保护也一并关掉。
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
            RefRamp += DCAC_SOFT_START_STEP;
            if (RefRamp > DCAC_VREF_PEAK)
            {
                RefRamp = DCAC_VREF_PEAK;
            }
        }
        else
        {
            RefRamp = 0;
        }

        // ---- 4. 相位累加 + 查表：phase_step = 256*50/20000 = 0.64 ----
        // PhaseAcc 高 8 位是表下标、低 24 位是 Q24 小数，
        // 与 0x00FFFFFF 相与即对 256 取模，等价于原来的 Phase -= 256
        PhaseAcc = (PhaseAcc + DCAC_PHASE_STEP_Q24) & DCAC_PHASE_MASK;
        sin_val = sin_table[(Uint16)(PhaseAcc >> DCAC_PHASE_ACC_BITS)];
        Vref = _IQmpy(sin_val, RefRamp);

        dbgSin =sin_val;

#if (DCAC_LOOP_MODE == DCAC_LOOP_CLOSED)
        // ---- 5. 电压外环 PR -> 电容电流给定 ----
        PR_V.Ref = Vref;
        PR_V.Fdb = Vout;
        IRef = PR_Calc(&PR_V);

        // ---- 6. 电流内环 PI -> 调制比，反馈是电容电流 ----
        PI_I.Ref = IRef;
        PI_I.Fdb = ICap;
        Mod = PI_Calc(&PI_I);
#else
        // 开环：调制比直接来自正弦表，不走 PR/PI
        Mod = _IQmpy(sin_val, DCAC_MOD_AMP_OPEN);
        IRef = 0;
        PR_V.X1 = 0; PR_V.X2 = 0;
        PR_V.Y1 = 0; PR_V.Y2 = 0;
        PI_I.Integral = 0;
#endif

        // ---- 7. 限幅并写 CMPA ----
        if (Mod > DCAC_MOD_MAX)
        {
            Mod = DCAC_MOD_MAX;
        }
        else if (Mod < -DCAC_MOD_MAX)
        {
            Mod = -DCAC_MOD_MAX;
        }
        SPWM_SetMod(Mod);
    }

#endif

    IsrCnt++;

    // ---- 8. 清中断标志 ----
    AdcRegs.ADCINTFLGCLR.all = 0x0001;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

    GpioDataRegs.GPACLEAR.bit.GPIO4 = 1;    // 中断计时脚拉低
}

//===========================================================================
// ADC 配置函数
//   SOC0 -> ADCINA0，第一次采样按原设计丢弃
//   SOC1 -> ADCINA0 输出电流
//   SOC2 -> ADCINA1 输出电压
//   SOC3 -> ADCINA2 电感电流
//   TRIGSEL=5 : EPWM1 SOCA（CTR=ZERO）
//   INT1SEL=3 : EOC3 触发 ADCINT1
//   INTPULSEPOS=1 : 转换结束结果锁存后才发中断脉冲
//===========================================================================
void Adc_Config(void)
{
    EALLOW;

    AdcRegs.ADCCTL1.bit.INTPULSEPOS = 1;    // 结果锁存之后再发中断脉冲

    AdcRegs.INTSEL1N2.bit.INT1E     = 1;    // 使能 ADCINT1
    AdcRegs.INTSEL1N2.bit.INT1CONT  = 0;    // 关闭 ADCINT1 连续模式
    AdcRegs.INTSEL1N2.bit.INT1SEL   = 3;    // EOC3 触发 ADCINT1

    AdcRegs.ADCSOC0CTL.bit.CHSEL    = DCAC_ADC_CH_DUMMY;
    AdcRegs.ADCSOC1CTL.bit.CHSEL    = DCAC_ADC_CH_IOUT;
    AdcRegs.ADCSOC2CTL.bit.CHSEL    = DCAC_ADC_CH_VOUT;
    AdcRegs.ADCSOC3CTL.bit.CHSEL    = DCAC_ADC_CH_IL;

    AdcRegs.ADCSOC0CTL.bit.TRIGSEL  = 5;    // EPWM1 SOCA
    AdcRegs.ADCSOC1CTL.bit.TRIGSEL  = 5;
    AdcRegs.ADCSOC2CTL.bit.TRIGSEL  = 5;
    AdcRegs.ADCSOC3CTL.bit.TRIGSEL  = 5;

    AdcRegs.ADCSOC0CTL.bit.ACQPS    = 6;    // 采样窗 7 个 ADCCLK
    AdcRegs.ADCSOC1CTL.bit.ACQPS    = 6;
    AdcRegs.ADCSOC2CTL.bit.ACQPS    = 6;
    AdcRegs.ADCSOC3CTL.bit.ACQPS    = 6;

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
//   DCAC_PROTECT_ENABLE = 0 时整个函数不编译
//===========================================================================
#if (DCAC_PROTECT_ENABLE == 1)
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
