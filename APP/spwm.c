/*
 * spwm.c
 *
 * 单相全桥单极性倍频 SPWM 实现
 *   一个载波（EPWM1/EPWM2 同步，TBPRD=1500，20kHz）
 *   两路反相调制波（左桥臂 +m*sin，右桥臂 -m*sin）
 *   每个桥臂上下管互补，由死区子模块插入 300ns 死区
 *
 * 载波：TBCLK = SYSCLK = 60MHz，TBPRD = 1500
 *       fsw = 60MHz / (2*1500) = 20kHz
 * ADC：EPWM1 在 CTR=ZERO 触发 SOCA
 *      SOC0 丢弃 SOC1=ADCA0 输出电流 SOC2=ADCA1 输出电压 SOC3=ADCA2 电感电流
 *      SOC4=ADCB1 输入电压 SOC5=ADCB0 输入电流（后两路只监视）
 *      EOC5 -> ADCINT1 -> CPU 的 ADC 中断（控制环在中断里跑）
 *
 * 比较值装载：CMPA 走影子寄存器，CTR=ZERO 装载
 * 这样中断里在任意时刻写 CMPA 都不会产生窄脉冲，生效时刻固定
 *
 * 定点化：mod 是 Q24（±1），但比较值幅度 TBPRD/2 = 750 远超 Q24 的 ±128，
 *         所以这里不把 750 转成 Q24，而是把 mod 降到 Q15 后乘整数跨度，
 *         全程 32 位整数，一次乘法就够，也避免了 Q24 溢出的坑
 *
 * 注意：本工程 EPWMxB 完全由死区子模块从 EPWMxA 取反生成，
 * 所以 AQCTLB 必须清零（保持"无动作"）。如果给 AQCTLB 也配一套互补的
 * 动作限定，会和死区子模块的取反叠加两次，结果 EPWMxB == EPWMxA。这一点
 */

#include "spwm.h"
#include "epwm.h"
#include "DSP2803x_EPwm_defines.h"

// 载波宏来自 dcac_cfg.h，保证 ePWM / 相位步进 / 控制周期三者自洽
#define SPWM_TBPRD      (TBPRD)
#define SPWM_CMPA_MID   (CMPA_Mid)
#define SPWM_CMPA_SPAN  (CMPA_Span)

/*===========================================================================
 * 函数名：SPWM_ModToCmp
 * 说明：Q24 调制比 -> CMPA 计数值
 *         cmp = TBPRD/2 + mod*TBPRD/2
 *       TBPRD/2 = 750 超过 Q24 的 ±128，做法是：
 *         mod(Q24) 右移 (GLOBAL_Q-15)=9 位降到 Q15，范围 ±32768
 *         乘 750 得 ±2.46e7，再加 0.5LSB 做四舍五入，最后右移 15 位还原
 *       Q15 量化误差 3.05e-5，折到比较值上只有 0.023 个计数，可以忽略
 *       最后限幅到 [1, TBPRD-1]，避免 0% / 100% 占空
 *===========================================================================*/
static Uint16 SPWM_ModToCmp(_iq mod)
{
    long m15;
    long cmp;

    m15 = mod >> (GLOBAL_Q - 15);       // Q24 -> Q15

    if (m15 > 32767L)
    {
        m15 = 32767L;
    }
    else if (m15 < -32768L)
    {
        m15 = -32768L;
    }

    cmp = (long)SPWM_CMPA_MID + ((m15 * (long)SPWM_CMPA_SPAN + 0x4000L) >> 15);

    if (cmp < 1L)
    {
        cmp = 1L;
    }
    else if (cmp > (long)(SPWM_TBPRD - 1U))
    {
        cmp = (long)(SPWM_TBPRD - 1U);
    }

    return (Uint16)cmp;
}

/*===========================================================================
 * 函数名：SPWM_Init
 * 说明：ePWM1/2 时基、动作限定、死区、ADC SOCA 触发源配置
 *       两个桥臂共用 EPWM1 的载波，EPWM2 与它同步、相位 0
 *       只占 GPIO0~GPIO3，GPIO4 留给中断计时脚
 *===========================================================================*/
void SPWM_Init(void)
{
    // 关 TBCLKSYNC，配置时基
    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    EDIS;

    // GPIO0~GPIO3 复用为 EPWM1A/1B、EPWM2A/2B
    InitPwmBridgeGpio();

    // 时基 + 死区。EPWM1 为主，EPWM2 同步跟随，相位 0（共用同一个载波）
    EPWM1_Init(SPWM_TBPRD, SPWM_DB_COUNTS, SPWM_DB_COUNTS);
    EPWM2_Init(SPWM_TBPRD, SPWM_DB_COUNTS, SPWM_DB_COUNTS, 0U);

    EALLOW;

    // 关闭 PWM 中断：本方案不用 EPWM 中断，用 ADCINT1
    EPwm1Regs.ETSEL.bit.INTSEL = 0;
    EPwm1Regs.ETSEL.bit.INTEN  = 0;
    EPwm1Regs.ETPS.bit.INTPRD  = 0;
    EPwm2Regs.ETSEL.bit.INTEN  = 0;
    PieCtrlRegs.PIEIER3.bit.INTx1 = 0;
    IER &= (Uint16)(~M_INT3);

    // ADC 触发：EPWM1 在 CTR=ZERO 发 SOCA，每个载波周期一次
    // CTR=ZERO 正好在开关管导通区间正中（开关沿在 +-CMPA 处），
    // 此刻电感电流等于其平均值，采样值不含开关纹波，是最佳采样时刻
    EPwm1Regs.ETSEL.bit.SOCAEN  = 1;
    EPwm1Regs.ETSEL.bit.SOCASEL = ET_CTR_ZERO;
    EPwm1Regs.ETPS.bit.SOCAPRD  = ET_1ST;

    // 初始输出中点：mod = 0，两桥臂都 50%/50%，差模电压 0V
    SPWM_SetMod(_IQ(0.0f));


    // 清 SOCA 标志
    EPwm1Regs.ETCLR.bit.SOCA = 1;
    EDIS;

    // 开 TBCLKSYNC，两个模块同时起跑
    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;
}

/*===========================================================================
 * 函数名：SPWM_SetMod
 * 说明：单极性倍频的两个比较值
 *         cmpA = TBPRD/2 * (1 - mod)   右桥臂 EPWM1（GPIO0/1）
 *         cmpB = TBPRD/2 * (1 + mod)   左桥臂 EPWM2（GPIO2/3）
 *       两者之和恒等于 TBPRD，于是 dA + dB = 1，
 *       uAB = (dA - dB)*Vdc = mod*Vdc，且只有 +Vdc/0/-Vdc 三种电平
 *       mod 是 Q24，超出 ±1 先限幅（定点比较就是整数比较）
 *===========================================================================*/
void SPWM_SetMod(_iq mod)
{

    if (mod > _IQ(1.0f))
    {
        mod = _IQ(1.0f);
    }
    else if (mod < _IQ(-1.0f))
    {
        mod = _IQ(-1.0f);
    }


    // 引脚分配：GPIO0/1 = PWM1AB = 右桥臂，GPIO2/3 = PWM2AB = 左桥臂
    EPwm1Regs.CMPA.half.CMPA = SPWM_ModToCmp(-mod);    // 右桥臂 d = (1 - mod)/2
    EPwm2Regs.CMPA.half.CMPA = SPWM_ModToCmp(mod);     // 左桥臂 d = (1 + mod)/2
}

/*===========================================================================
 * 函数名：SPWM_SetNeutral
 * 说明：输出中点，mod=0，两桥臂都 50%，交流侧平均 0V
 *===========================================================================*/
void SPWM_SetNeutral(void)
{
    SPWM_SetMod(_IQ(0.0f));
}

/*===========================================================================
 * 函数名：SPWM_GetCmp
 * 说明：读回右桥臂（EPWM1，GPIO0/1）当前 CMPA，调试观察用
 *===========================================================================*/
Uint16 SPWM_GetCmp(void)
{
    return EPwm1Regs.CMPA.half.CMPA;
}
