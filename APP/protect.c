/*
 * protect.c
 *
 * 过流保护实现：COMP1/COMP2 + ePWM Digital Compare + Trip Zone
 *
 * 设计要点：
 *   1) 四管全关是 TZCTL 直接把 EPWMxA / EPWMxB 拉低，与 CMPA 无关，
 *      所以控制环还在算也拦不住保护动作
 *   2) DCxEVT1 -> 单次触发(OST)，锁存，必须软件清 TZCLR 才能恢复
 *      DCxEVT2 -> 逐周期(CBC)，作为反向过流的即时限流
 *   3) TZEINT 只在 EPWM1 上开中断，用一路 ISR 上报即可
 *      （两个模块由同一对比较器驱动，必然同时动作）
 *   4) COMP1A 固定在 ADCINA2，本工程 ADCINA2 = 电感电流，
 *      所以阈值按电感电流传感器的量程换算
 *
 * 定点化：阈值换算全过程 Q24，只有最后乘 1024 转 DAC 码时用移位，
 *         因为 1024 超过 Q24 能表示的 ±128
 */

#include "protect.h"
#include "dcac_cfg.h"

volatile Uint16 Protect_TripFlag   = 0U;
volatile Uint16 Protect_TripSource = 0U;

// 比较器 DAC 参考电压（VDDA = 3.3V，10bit）
#define PROTECT_DAC_FS_V_F      (3.3f)
#define PROTECT_DAC_MAX_CODE    (1023U)
#define PROTECT_SENSE_MID_V_F   (1.65f)     // 双向电流采样中点

#define PROTECT_DAC_FS_V        _IQ(PROTECT_DAC_FS_V_F)
#define PROTECT_SENSE_MID_V     _IQ(PROTECT_SENSE_MID_V_F)

// DAC 码值是 10 位（0~1024），Q24 的比例要乘 1024 = 左移 10 位，
// 反过来就是右移 24-10 = 14 位。写成移位而不是 _IQmpy 是因为
// 1024 已经超出 Q24 的动态范围（±128），不能再当 Q24 数用
#define PROTECT_RATIO_TO_CODE(r)    ((long)((r) >> 14))

/*===========================================================================
 * 函数名：Protect_AmpsToDacCode
 * 说明：把电感电流安培值换算成 COMP DAC 码
 *       Vpin(I) = 1.65 + (I / IL_FULL_SCALE) * 1.65
 *       code    = Vpin / 3.3 * 1024
 *       amps 为 Q24，先除以满量程得到 Q24 的比例，再乘中点电压
 *===========================================================================*/
static Uint16 Protect_AmpsToDacCode(_iq amps)
{
    _iq ratio;
    _iq v;
    long code;

    // I / IL_FULL_SCALE，Q24 的除法，被除数比除数小，精度最好
    ratio = _IQdiv(amps, DCAC_IL_FULL_SCALE);

    // Vpin = 1.65 + ratio*1.65
    v = PROTECT_SENSE_MID_V + _IQmpy(ratio, PROTECT_SENSE_MID_V);

    if (v < 0)
    {
        v = 0;
    }
    else if (v > PROTECT_DAC_FS_V)
    {
        v = PROTECT_DAC_FS_V;
    }

    // code = v/3.3*1024
    code = PROTECT_RATIO_TO_CODE(_IQdiv(v, PROTECT_DAC_FS_V));

    if (code > (long)PROTECT_DAC_MAX_CODE)
    {
        code = (long)PROTECT_DAC_MAX_CODE;
    }

    return (Uint16)code;
}

/*===========================================================================
 * 函数名：Protect_ConfigEpwmTrip
 * 说明：给一个 ePWM 模块挂上数字比较加触发区
 *       正向过流: COMP1OUT -> DCAH/DCBH -> DCAEVT1/DCBEVT1 单次
 *       反向过流: COMP2OUT -> DCAL/DCBL -> DCAEVT2/DCBEVT2 逐周期
 *===========================================================================*/
static void Protect_ConfigEpwmTrip(volatile struct EPWM_REGS *epwm)
{
    // 数字比较输入选择
    epwm->DCTRIPSEL.bit.DCAHCOMPSEL = DC_COMP1OUT;
    epwm->DCTRIPSEL.bit.DCBHCOMPSEL = DC_COMP1OUT;
#if (PROTECT_USE_COMP2_NEGATIVE == 1)
    epwm->DCTRIPSEL.bit.DCALCOMPSEL = DC_COMP2OUT;
    epwm->DCTRIPSEL.bit.DCBLCOMPSEL = DC_COMP2OUT;
#else
    epwm->DCTRIPSEL.bit.DCALCOMPSEL = DC_TZ2;
    epwm->DCTRIPSEL.bit.DCBLCOMPSEL = DC_TZ2;
#endif

    // DCAEVT1 / DCBEVT1：COMP1 输出高 = 正向过流
    epwm->TZDCSEL.bit.DCAEVT1 = TZ_DCAH_HI;
    epwm->DCACTL.bit.EVT1SRCSEL     = DC_EVT1;          // 不经过滤波
    epwm->DCACTL.bit.EVT1FRCSYNCSEL = DC_EVT_ASYNC;     // 异步通道，响应最快

    epwm->TZDCSEL.bit.DCBEVT1 = TZ_DCBH_HI;
    epwm->DCBCTL.bit.EVT1SRCSEL     = DC_EVT1;
    epwm->DCBCTL.bit.EVT1FRCSYNCSEL = DC_EVT_ASYNC;

    // DCAEVT2 / DCBEVT2：COMP2 输出高 = 反向过流
#if (PROTECT_USE_COMP2_NEGATIVE == 1)
    epwm->TZDCSEL.bit.DCAEVT2 = TZ_DCAL_HI;
    epwm->TZDCSEL.bit.DCBEVT2 = TZ_DCBL_HI;
#else
    epwm->TZDCSEL.bit.DCAEVT2 = TZ_EVT_DISABLE;
    epwm->TZDCSEL.bit.DCBEVT2 = TZ_EVT_DISABLE;
#endif
    epwm->DCACTL.bit.EVT2SRCSEL     = DC_EVT2;
    epwm->DCACTL.bit.EVT2FRCSYNCSEL = DC_EVT_ASYNC;
    epwm->DCBCTL.bit.EVT2SRCSEL     = DC_EVT2;
    epwm->DCBCTL.bit.EVT2FRCSYNCSEL = DC_EVT_ASYNC;

    // 选为触发源
    epwm->TZSEL.bit.DCAEVT1 = 1U;       // 单次(锁存)
    epwm->TZSEL.bit.DCBEVT1 = 1U;
#if (PROTECT_USE_COMP2_NEGATIVE == 1)
    epwm->TZSEL.bit.DCAEVT2 = 1U;       // 逐周期(限流)
    epwm->TZSEL.bit.DCBEVT2 = 1U;
#else
    epwm->TZSEL.bit.DCAEVT2 = 0U;
    epwm->TZSEL.bit.DCBEVT2 = 0U;
#endif

    // 动作：四管全关（A/B 都直接拉低）
    epwm->TZCTL.bit.DCAEVT1 = TZ_FORCE_LO;
    epwm->TZCTL.bit.DCAEVT2 = TZ_FORCE_LO;
    epwm->TZCTL.bit.DCBEVT1 = TZ_FORCE_LO;
    epwm->TZCTL.bit.DCBEVT2 = TZ_FORCE_LO;

    // 清标志（TZFLG 只读，用 TZCLR 清）
    epwm->TZCLR.all = 0xFFFFU;
}

/*===========================================================================
 * 函数名：Protect_InitOCP
 * 说明：初始化过流保护。先上 ADC bandgap（比较器共用），再配 COMP1/COMP2，
 *       最后给 EPWM1/EPWM2 挂触发链
 *       DCAC_PROTECT_ENABLE = 0 时直接返回，所有相关寄存器保持复位值
 *===========================================================================*/
void Protect_InitOCP(void)
{
#if (DCAC_PROTECT_ENABLE == 0)
    // 保护关掉时什么都不做
    // 关键：COMP、Digital Compare、Trip Zone 全部不配置，
    // 这样即使比较器输入悬空也不会误触发 TZ 把四管封死
    Protect_TripFlag   = 0U;
    Protect_TripSource = 0U;
    return;
#else
    Uint16 code;

    // 比较器需要 ADC 的 bandgap 做参考，必须先上电并等稳定
    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.ADCENCLK   = 1;
    SysCtrlRegs.PCLKCR3.bit.COMP1ENCLK = 1;
#if (PROTECT_USE_COMP2_NEGATIVE == 1)
    SysCtrlRegs.PCLKCR3.bit.COMP2ENCLK = 1;
#endif
    AdcRegs.ADCCTL1.bit.ADCBGPWD = 1;
    EDIS;

    DELAY_US(1000L);

    code = Protect_AmpsToDacCode(PROTECT_TRIP_AMPS_DEFAULT);

    // COMP1：正向过流。COMPSOURCE=0 用内部 DAC 做反相输入，CMPINV=0 不反相
    // 即 COMP1A(电感电流) > DAC 阈值 时输出高
    EALLOW;
    Comp1Regs.COMPCTL.bit.COMPDACEN  = 1;
    Comp1Regs.COMPCTL.bit.COMPSOURCE = 0;
    Comp1Regs.COMPCTL.bit.CMPINV     = 0;
    Comp1Regs.COMPCTL.bit.QUALSEL    = 0;       // 不做限幅滤波，最快响应
    Comp1Regs.COMPCTL.bit.SYNCSEL    = 0;       // 异步输出
    Comp1Regs.DACVAL.bit.DACVAL      = code;
    EDIS;

#if (PROTECT_USE_COMP2_NEGATIVE == 1)
    // COMP2：反向过流，输出反相，电感电流 < -阈值 时输出高
    EALLOW;
    Comp2Regs.COMPCTL.bit.COMPDACEN  = 1;
    Comp2Regs.COMPCTL.bit.COMPSOURCE = 0;
    Comp2Regs.COMPCTL.bit.CMPINV     = 1;
    Comp2Regs.COMPCTL.bit.QUALSEL    = 0;
    Comp2Regs.COMPCTL.bit.SYNCSEL    = 0;
    Comp2Regs.DACVAL.bit.DACVAL      = code;
    EDIS;
#endif

    // 两个 ePWM 模块都挂上触发链，中断只开 EPWM1 一路用来上报
    EALLOW;
    Protect_ConfigEpwmTrip(&EPwm1Regs);
    Protect_ConfigEpwmTrip(&EPwm2Regs);

    EPwm1Regs.TZEINT.bit.OST = 1;       // DCxEVT1 单次
    EPwm1Regs.TZEINT.bit.CBC = 1;       // DCxEVT2 逐周期
    EDIS;

    Protect_TripFlag   = 0U;
    Protect_TripSource = 0U;
#endif
}

/*===========================================================================
 * 函数名：Protect_SetTripLevel
 * 说明：改过流阈值，单位 A，双向对称（Q24）
 *===========================================================================*/
void Protect_SetTripLevel(_iq amps)
{
#if (DCAC_PROTECT_ENABLE == 0)
    // 保护关闭，不碰比较器
    (void)amps;
    return;
#else
    Uint16 code;

    if (amps < 0)               // 定点取绝对值就是整数取反
    {
        amps = -amps;
    }
    code = Protect_AmpsToDacCode(amps);

    EALLOW;
    Comp1Regs.DACVAL.bit.DACVAL = code;
#if (PROTECT_USE_COMP2_NEGATIVE == 1)
    Comp2Regs.DACVAL.bit.DACVAL = code;
#endif
    EDIS;
#endif
}

/*===========================================================================
 * 函数名：Protect_ClearTrip
 * 说明：清触发锁存并重新打开 TZ 中断源
 *       Trip ISR 里会主动关掉 TZEINT 以防止中断风暴，恢复输出时必须重新打开
 *===========================================================================*/
void Protect_ClearTrip(void)
{
#if (DCAC_PROTECT_ENABLE == 0)
    // 保护关闭，TZ 从来没被配置过，没有锁存要清
    Protect_TripFlag   = 0U;
    Protect_TripSource = 0U;
    return;
#else
    EALLOW;
    EPwm1Regs.TZCLR.bit.OST = 1;
    EPwm1Regs.TZCLR.bit.CBC = 1;
    EPwm1Regs.TZCLR.bit.INT = 1;
    EPwm1Regs.TZCLR.bit.DCAEVT1 = 1;
    EPwm1Regs.TZCLR.bit.DCAEVT2 = 1;
    EPwm1Regs.TZCLR.bit.DCBEVT1 = 1;
    EPwm1Regs.TZCLR.bit.DCBEVT2 = 1;

    EPwm2Regs.TZCLR.bit.OST = 1;
    EPwm2Regs.TZCLR.bit.CBC = 1;
    EPwm2Regs.TZCLR.bit.DCAEVT1 = 1;
    EPwm2Regs.TZCLR.bit.DCAEVT2 = 1;
    EPwm2Regs.TZCLR.bit.DCBEVT1 = 1;
    EPwm2Regs.TZCLR.bit.DCBEVT2 = 1;

    // 重新武装中断源
    EPwm1Regs.TZEINT.bit.OST = 1;
    EPwm1Regs.TZEINT.bit.CBC = 1;
    EDIS;

    Protect_TripFlag   = 0U;
    Protect_TripSource = 0U;
#endif
}

/*===========================================================================
 * 函数名：Protect_ForceStop
 * 说明：软件急停，四管全关。置 TZFRC.OST 后立刻清 INT 标志，
 *       所以不会产生 TZ 中断，也不污染 Protect_TripFlag
 *       DCAC_PROTECT_ENABLE = 0 时是空操作，否则一调用就把输出封死了
 *===========================================================================*/
void Protect_ForceStop(void)
{
#if (DCAC_PROTECT_ENABLE == 0)
    return;
#else
    EALLOW;
    EPwm1Regs.TZFRC.bit.OST = 1;
    EPwm2Regs.TZFRC.bit.OST = 1;
    EPwm1Regs.TZCLR.bit.INT = 1;
    EPwm2Regs.TZCLR.bit.INT = 1;
    EDIS;
#endif
}

/*===========================================================================
 * 函数名：Protect_SoftwareCheck
 * 说明：软件过流兜底，返回 1 表示越限（Q24，整数比较）
 *===========================================================================*/
Uint16 Protect_SoftwareCheck(_iq i_fb, _iq i_limit)
{
    if (i_fb > i_limit)
    {
        return 1U;
    }
    if (i_fb < -i_limit)
    {
        return 1U;
    }
    return 0U;
}
