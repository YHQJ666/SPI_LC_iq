/*
 * sfra_cfg.h
 *
 * SFRA 扫频（TI SFRA_IQ v1.20）的配置与对外声明。
 * 只有 SFRA_ENABLE == 1 时才参与编译。
 *
 * 频率范围为什么只有 48 点：
 *   采样/中断是 20kHz，SFRA 的有效上限大致到 fsw/10 = 2kHz。
 *   从 100Hz 起、每点 x1.059253（每十倍频程 40 点），48 点正好到 ~2.1kHz。
 *   若照 Boost 的 161 点会扫到 60kHz，远超声速采样限制，高频段全是假数据。
 */

#ifndef SFRA_CFG_H
#define SFRA_CFG_H

#include "dcac_cfg.h"

#if (SFRA_ENABLE == 1)

#include "C:/ti/controlSUITE/libs/app_libs/SFRA/v1_20_00_00/IQ/include/SFRA_IQ_Include.h"

/* ---- 扫频参数 ---- */
#define SFRA_FREQ_LENGTH    48          /* 点数：100Hz ~ 2.1kHz */
#define SFRA_FREQ_START     100.0f      /* 起始频率 Hz */
#define SFRA_FREQ_STEP      1.059253f   /* 相邻频点倍数 = 10^(1/40)，每十倍频程 40 点 */
#define SFRA_ISR_FREQ       20000.0f    /* 中断频率 = 开关频率，填错频率轴整体偏移 */

/* ---- 上位机 SCI ---- */
#define SFRA_SCI_LSPCLK     15000000L   /* LSPCLK = SYSCLK/2 = 15MHz */
#define SFRA_SCI_BAUDRATE   57600L      /* SFRA GUI 默认 */

/* ---- 扫频工作点（参考冻结，不走 50Hz 正弦） ---- */
#define SFRA_VREF_DC        _IQ(20.0f)  /* 电压环扫频：输出电压直流给定 20V */
#define SFRA_DUTY_DC        _IQ(0.40f)  /* Plant 扫频：固定占空比 0.4 */
#define SFRA_IREF_DC        _IQ(1.0f)   /* 电流环扫频：直流电流给定 1A */

/* ---- 库对象与结果数组 ---- */
extern SFRA_IQ  SFRA1;
extern float32  FreqVect[SFRA_FREQ_LENGTH];
extern int32    Plant_MagVect[SFRA_FREQ_LENGTH];
extern int32    Plant_PhaseVect[SFRA_FREQ_LENGTH];
extern int32    OL_MagVect[SFRA_FREQ_LENGTH];
extern int32    OL_PhaseVect[SFRA_FREQ_LENGTH];

/* ---- 上位机映射表（顺序由 SFRA GUI 写死，不可改） ---- */
extern int16   *varSetTxtList[16];
extern int16   *varSetBtnList[16];
extern int16   *varSetSldrList[16];
extern int16   *varGetList[16];
extern int32   *arrayGetList[16];
extern int32   *dataGetList[16];
extern Uint32  *dataSetList[16];

extern int16    CommsOKflg;
extern int16    SerialCommsTimer;
extern int16    initializationFlag;

/* ---- 函数 ---- */
extern void SFRA_Init(void);
extern void SCIA_Init(long SCI_VBUS_CLOCKRATE, long SCI_BAUDRATE);
extern void SerialHostComms(void);

/* ---- 注入/采集的故障门 ----
 * 保护一动作就停止注入，否则扫出来的数据全是故障期的垃圾：
 *   SFRA_INJ : 正常时注入，故障时原样返回直流工作点（相位不前进，等于暂停）
 *   SFRA_COL : 正常时采集，故障时什么都不做（与 INJECT 严格配对，不采集就不注入）
 */
#define SFRA_INJ(dc)                                                            \
    ((FaultLatch == FAULT_NONE) ? (int32)SFRA_IQ_INJECT((int32)(dc)) : (int32)(dc))

#define SFRA_COL(in, fdb)                                                       \
    do {                                                                        \
        if (FaultLatch == FAULT_NONE) {                                         \
            SFRA_IQ_COLLECT((volatile int32 *)(in), (volatile int32 *)(fdb));   \
        }                                                                       \
    } while (0)

#endif   /* SFRA_ENABLE == 1 */

#endif   /* SFRA_CFG_H */
