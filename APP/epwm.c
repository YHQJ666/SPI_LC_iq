/*
 * epwm.c
 *
 * ePWM 底层初始化：时基、比较值影子寄存器、死区、GPIO 复用
 *
 * 注意：AQCTLA/AQCTLB 的中文含义是动作限定，本工程最终的动作限定在
 * spwm.c 的 SPWM_Init() 里按双极性 SPWM 覆盖，这里给的是通用默认值
 */

#include "epwm.h"

/*===========================================================================
 * 函数名：EPwmDeadBandConfig
 * 说明：A 通道作死区输入，高低互补输出
 *===========================================================================*/
static void EPwmDeadBandConfig(volatile struct EPWM_REGS *epwmRegs, Uint16 dbRed, Uint16 dbFed)
{
    epwmRegs->DBCTL.bit.IN_MODE  = 0;   // A 通道作为死区输入
    epwmRegs->DBCTL.bit.POLSEL   = 2;   // 高低互补
    epwmRegs->DBCTL.bit.OUT_MODE = 3;   // 使能死区
    epwmRegs->DBRED = dbRed;
    epwmRegs->DBFED = dbFed;
}

/*===========================================================================
 * 函数名：EPWM1_Init
 * 说明：主模块初始化，SYNCOSEL 在 CTR=Zero 发同步脉冲
 *===========================================================================*/
void EPWM1_Init(Uint16 tbprd, Uint16 dbRed, Uint16 dbFed)
{
    SysCtrlRegs.PCLKCR1.bit.EPWM1ENCLK = 1;

    EPwm1Regs.TBCTL.bit.SYNCOSEL   = 1;     // CTR=Zero 发同步脉冲
    EPwm1Regs.TBCTL.bit.PHSEN      = 0;     // 主模块
    EPwm1Regs.TBPHS.half.TBPHS     = 0;
    EPwm1Regs.TBCTR                = 0;
    EPwm1Regs.TBPRD                = tbprd;
    EPwm1Regs.TBCTL.bit.CTRMODE    = 2;     // 上下计数
    EPwm1Regs.TBCTL.bit.HSPCLKDIV  = 0;
    EPwm1Regs.TBCTL.bit.CLKDIV     = 0;

    // 比较值影子寄存器，CTR=ZERO 装载
    EPwm1Regs.CMPCTL.bit.SHDWAMODE = 0;
    EPwm1Regs.CMPCTL.bit.LOADAMODE = 0;
    EPwm1Regs.CMPA.half.CMPA       = 0;
    EPwm1Regs.CMPB                 = 0;

    // ---- AQCTLA：d = CMPA / TBPRD ----
    EPwm1Regs.AQCTLA.bit.ZRO = 0;   // 不动
    EPwm1Regs.AQCTLA.bit.PRD = 0;   // 不动
    EPwm1Regs.AQCTLA.bit.CAU = 1;   // CMPA 向上 → CLEAR
    EPwm1Regs.AQCTLA.bit.CAD = 2;   // CMPA 向下 → SET

    // ---- AQCTLB：必须全 0，EPWMxB 交给死区模块取反 ----
    EPwm1Regs.AQCTLB.bit.ZRO = 0;
    EPwm1Regs.AQCTLB.bit.PRD = 0;
    EPwm1Regs.AQCTLB.bit.CAU = 0;
    EPwm1Regs.AQCTLB.bit.CAD = 0;
    EPwm1Regs.AQCTLB.bit.CBU = 0;
    EPwm1Regs.AQCTLB.bit.CBD = 0;

    EPwmDeadBandConfig(&EPwm1Regs, dbRed, dbFed);
}

// 从模块（直接由 EPWM1 同步）
// 注意：宏体里不能写 // 注释，续行符会把下一行也并进来一起注释掉，
// 所以说明写在这里：SYNCOSEL=0 用同步输入，PHSEN=1 启用相位加载，CTRMODE=2 上下计数
#define DEFINE_EPWM_SLAVE_INIT(module) \
void EPWM##module##_Init(Uint16 tbprd, Uint16 dbRed, Uint16 dbFed, Uint16 phaseShift) \
{ \
    SysCtrlRegs.PCLKCR1.bit.EPWM##module##ENCLK = 1; \
    EPwm##module##Regs.TBCTL.bit.SYNCOSEL = 0;    \
    EPwm##module##Regs.TBCTL.bit.PHSEN    = 1;    \
    EPwm##module##Regs.TBPHS.half.TBPHS   = phaseShift; \
    EPwm##module##Regs.TBCTR              = 0; \
    EPwm##module##Regs.TBPRD              = tbprd; \
    EPwm##module##Regs.TBCTL.bit.CTRMODE  = 2;    \
    EPwm##module##Regs.TBCTL.bit.HSPCLKDIV = 0; \
    EPwm##module##Regs.TBCTL.bit.CLKDIV    = 0; \
    EPwm##module##Regs.CMPCTL.bit.SHDWAMODE = 0; \
    EPwm##module##Regs.CMPCTL.bit.LOADAMODE = 0; \
    EPwm##module##Regs.CMPA.half.CMPA = 0; \
    EPwm##module##Regs.CMPB = 0; \
    EPwm##module##Regs.AQCTLA.bit.ZRO = 0;   \
    EPwm##module##Regs.AQCTLA.bit.CAU = 1;   \
    EPwm##module##Regs.AQCTLA.bit.CAD = 2;   \
    EPwm##module##Regs.AQCTLB.bit.ZRO = 0;   \
    EPwm##module##Regs.AQCTLB.bit.CAU = 0;   \
    EPwm##module##Regs.AQCTLB.bit.CAD = 0;   \
    EPwm##module##Regs.AQCTLB.bit.CBU = 0;   \
    EPwm##module##Regs.AQCTLB.bit.CBD = 0;   \
    EPwmDeadBandConfig(&EPwm##module##Regs, dbRed, dbFed); \
}

DEFINE_EPWM_SLAVE_INIT(2)
DEFINE_EPWM_SLAVE_INIT(3)
DEFINE_EPWM_SLAVE_INIT(4)
DEFINE_EPWM_SLAVE_INIT(5)
DEFINE_EPWM_SLAVE_INIT(6)

// 比较值设置
void EPwm1A_SetCompare(Uint16 val) { EPwm1Regs.CMPA.half.CMPA = val; }
void EPwm2A_SetCompare(Uint16 val) { EPwm2Regs.CMPA.half.CMPA = val; }
void EPwm3A_SetCompare(Uint16 val) { EPwm3Regs.CMPA.half.CMPA = val; }
void EPwm4A_SetCompare(Uint16 val) { EPwm4Regs.CMPA.half.CMPA = val; }
void EPwm5A_SetCompare(Uint16 val) { EPwm5Regs.CMPA.half.CMPA = val; }
void EPwm6A_SetCompare(Uint16 val) { EPwm6Regs.CMPA.half.CMPA = val; }

/*===========================================================================
 * 函数名：InitPwmBridgeGpio
 * 说明：只把 GPIO0~GPIO3 复用为 EPWM1A/1B、EPWM2A/2B
 *       GPIO4 及以上不再占用，留给中断计时脚和其他用途
 *===========================================================================*/
void InitPwmBridgeGpio(void)
{
    EALLOW;
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;     // EPWM1A
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;     // EPWM1B
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;     // EPWM2A
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;     // EPWM2B
    EDIS;
}
