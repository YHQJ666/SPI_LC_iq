/* 本文件整体只在 SFRA 扫频版编译；OLED 版下是一个空文件 */
#include "dcac_cfg.h"

#if (SFRA_ENABLE == 1)
/* ==============================================================================
//  FILE:   SciCommsGui_32bit.c
//
//  TITLE:  GP Comms kernel as an interface to external GUI
//
//  说明:   这是 "上位机(GUI) <-> 目标板" 的命令协议实现, 供 SFRA GUI(xds/SCI)
//          以及 powerSUITE 的波形/参数面板使用.
//          本文件取自 controlSUITE 中 F2803x 可用的寄存器版实现
//          (development_kits\ILPFC_v1.3\IL_PFC_F28035_powerSUITE\SciCommsGui_32bit.c),
//          只把 LifePulse 里的 LED 翻转去掉, 其余协议逻辑保持原样, 因为 SFRA GUI
//          是按这套固定协议(命令号/包长/字节序)通讯的, 不要改动协议部分.
//
//          协议要点(逆向自代码):
//            每包 6 字节: Cmd(1) + Index(1) + Data(2, 低字节先) + (保留)
//            命令 0: LifePulse    上位机心跳, 板子回 CommsOKflg=1
//            命令 1: TextSet      16位写: *varSetTxtList[Index]  = Data
//            命令 2: ButtonSet    16位写: *varSetBtnList[Index]  = Data
//            命令 3: SliderSet    16位写: *varSetSldrList[Index] = Data
//            命令 4: VariableGet  读16位: 把 *varGetList[Index] 的内容发回 (Data=字数)
//            命令 5: ArrayGet     读32位: 把 arrayGetList[Index] 指向的数组发回
//            命令 7: DataSet32    32位写(分两包): 写 *dataSetList[Index]
//
//          因为 SFRA 的扫频参数(起始频率/幅值/步进)是 float32/int32, SFRA GUI 用的是
//          命令 5 和 7, 所以本文件必须是这个 "32bit" 版本.
//
//  Target: TMS320F2803x (Piccolo) / 寄存器头文件方式
//
//=============================================================================
//  版权归属: 原始代码 Copyright (C) Texas Instruments Incorporated
//===========================================================================  */

#include <stdint.h>

#ifdef F28x_DEVICE
#include "F28x_Project.h"
#else
#include "DSP28x_Project.h"          /* Boost_SFRA 使用的 F2803x 头文件入口 */
#endif

#define	PktSize				6
#define	CmdNumber			16		/* 命令分发表长度 */
#define	MAX_CMD_NUM			8		/* 实际用到的最大命令号 */
#define	MAX_LIST_IDX		16		/* 各张指针表的长度, 越界就是内存踩踏, 必须挡住 */
#define	MAX_WORDS_GET		512		/* 一次"读数组"命令最多允许发送的字数 (161点x2=322, 留余量) */

/* 说明: 原版TI代码对命令号和列表下标都不做检查. 开关电源板上串口线噪声很大,
   一旦线路噪声凑出一个越界的命令号(比如0x5A), CmdDispatcher[0x5A] 会跳到野地址
   直接死机; 越界的下标则会写坏内存. 下面所有入口都加了边界检查, 对正常通讯没有影响. */


/* ---------------------------------------------------------------------------
// 命令接收状态机的函数原型
// ------------------------------------------------------------------------- */
void GetCmdByte(void);
void EchoCmdByte(void);
void GetSizeByte(void);
void EchoSizeByte(void);
void GetDataByte(void);
void EchoDataByte(void);
void PackWord(void);
void PackArray(void);
void CmdInterpreter(void);

/* ---------------------------------------------------------------------------
// 命令解释/分发函数的原型
// ------------------------------------------------------------------------- */
void LifePulseTsk(void);		/* 0 */
void TextSet(void);				/* 1 */
void ButtonSet(void);			/* 2 */
void SliderSet(void);			/* 3 */
void VariableGet(void);			/* 4 */
void ArrayGet(void);			/* 5 */
void DataGet(void);				/* 6 */
void DataSet32(void);			/* 7 */
void SpareTsk08(void);			/* 8 */

void SendData(void);

/* ---------------------------------------------------------------------------
// 变量定义
// ------------------------------------------------------------------------- */
void (*RcvTaskPointer)(void); 				/* 命令包接收状态指针 */
void (*CmdDispatcher[CmdNumber])(void);		/* 命令号 -> 处理函数 */

/* 下面这些表由 main.c 定义并赋初值, 这里只引用 */
extern int16_t  *varSetTxtList[];
extern int16_t  *varSetBtnList[];
extern int16_t  *varSetSldrList[];
extern int16_t  *varGetList[];
extern int32_t  *arrayGetList[];
extern int32_t  *dataGetList[];
extern uint32_t *dataSetList[];

extern int16_t  CommsOKflg, SerialCommsTimer;

uint16_t	LowByteFlag, SendTaskPtr;
uint16_t	RxChar, RxWord;
uint16_t	CmdPacket[PktSize];
uint16_t	TaskDoneFlag, NumWords, wordsLeftToGet;

uint16_t dataOut16;
int32_t  dataOut32;

int16_t *memDataPtr16;
int32_t *memDataPtr32;

int16_t  RcvTskPtrShdw;		/* 调试用 */

int16_t  delayer;

int16_t  MemGetPtr;
uint32_t MemGetAddress;
int16_t  MemGetAmount;

int16_t  MemSetPtr;
uint32_t MemSetValue;

uint32_t Temp;


/* ===========================================================================
//  SCI-A 初始化
//  参数: SCI_VBUS_CLOCKRATE = LSPCLK 频率(Hz), SCI_BAUDRATE = 波特率
//  前提: InitSysCtrl() 里已经使能 SCI-A 时钟, GPIO28/29 已由 main.c 配成 SCI 功能
// =========================================================================*/
void SCIA_Init(long SCI_VBUS_CLOCKRATE, long SCI_BAUDRATE)
{
	int16_t j = 0;

	SciaRegs.SCICCR.all = 0x0007;		/* 1停止位, 无校验, 8位字符, 异步, idle-line */
	SciaRegs.SCICTL1.all = 0x0003;		/* 使能TX/RX, 内部SCICLK, 关RX ERR/SLEEP/TXWAKE */
	SciaRegs.SCICTL2.all = 0x0003;
	SciaRegs.SCICTL2.bit.TXINTENA   = 0;
	SciaRegs.SCICTL2.bit.RXBKINTENA = 0;
	SciaRegs.SCIHBAUD = 0x0000;
	/* 波特率分频: 实际波特率 = LSPCLK / ((BRR+1)*8), 这里四舍五入取整.
	   57600 @ LSPCLK 15MHz -> BRR = 32 (即TI官方例子里的 0x20, 实际 56818bps, 误差 1.4%),
	   若直接截断会得到 31 (实际 58594bps, 误差 1.7%). 两者都在容差内, 但用32与TI例程完全一致. */
	SciaRegs.SCILBAUD = ((SCI_VBUS_CLOCKRATE + (4 * SCI_BAUDRATE)) / (8 * SCI_BAUDRATE)) - 1;

	SciaRegs.SCICTL1.all = 0x0023;		/* 退出复位 */

	SciaRegs.SCIFFTX.all = 0x8040;		/* 关闭FIFO增强(与TI原始工程一致) */
	SciaRegs.SCIFFRX.all = 0x204f;
	SciaRegs.SCIFFCT.all = 0x0;
	SciaRegs.SCIPRI.bit.SOFT = 0x0;
	SciaRegs.SCIPRI.bit.FREE = 0x1;

	RcvTaskPointer = &GetCmdByte;		/* 状态机复位 */
	RcvTskPtrShdw  = 1;
	SendTaskPtr    = 0;
	LowByteFlag    = 1;					/* 字节->字 打包时先收低字节 */

	dataOut16 = 0;
	dataOut32 = 0;
	delayer   = 0;

	MemGetPtr     = 0;
	MemGetAddress = 0x00000000;
	MemGetAmount  = 0;

	MemSetPtr    = 0;
	MemSetValue  = 0x00000000;

	for (j = 0; j < PktSize; j++)
	{
		CmdPacket[j] = 0x0;
	}

	RcvTskPtrShdw = 0;

	/* 初始化命令分发表: 先全部填成空任务, 再挂上实际用到的命令.
	   这样即使受到噪声干扰凑出 9~15 号命令, 也只是执行一个空任务, 不会跳野地址. */
	for (j = 0; j < CmdNumber; j++)
	{
		CmdDispatcher[j] = SpareTsk08;
	}
	CmdDispatcher[0] = LifePulseTsk;
	CmdDispatcher[1] = TextSet;
	CmdDispatcher[2] = ButtonSet;
	CmdDispatcher[3] = SliderSet;
	CmdDispatcher[4] = VariableGet;
	CmdDispatcher[5] = ArrayGet;
	CmdDispatcher[6] = DataGet;
	CmdDispatcher[7] = DataSet32;
	CmdDispatcher[8] = SpareTsk08;
}

/* ===========================================================================
//  命令接收状态机入口
//  在主循环里尽可能频繁地调用, 每调一次推进一步
// =========================================================================*/
void SerialHostComms()
{
	(*RcvTaskPointer)();
}

void GetCmdByte(void)		/* 状态1: 等命令字节 */
{
	if (SciaRegs.SCIRXST.bit.RXRDY == 1)
	{
		RxChar = SciaRegs.SCIRXBUF.all;
		RcvTaskPointer = &EchoCmdByte;
		SerialCommsTimer = 0;
		EchoCmdByte();
	}
	else if (SciaRegs.SCIRXST.bit.BRKDT == 1 || SerialCommsTimer > 20000)	/* 20s 空闲才复位 */
	{
		/* 检测到break或者长时间空闲, 复位SCI(某些USB转串口在仿真器RST后会卡住)
		   原版是 2.5s。一次扫频要几十秒, 期间上位机可能几秒不发包,
		   2.5s 会把正常空闲当成断开, 表现成「扫到 1/8 就连接丢失」。 */
		SciaRegs.SCICCR.all  = 0x0007;
		SciaRegs.SCICTL1.all = 0x0003;
		SciaRegs.SCICTL2.all = 0x0000;
		SciaRegs.SCICTL1.all = 0x0023;

		asm(" RPT#8 || NOP");

		SendTaskPtr = 0;
		SerialCommsTimer = 0;
		CommsOKflg = 0;
		RcvTaskPointer = &GetCmdByte;
	}
}

void EchoCmdByte(void)		/* 状态2: 回显命令字节 */
{
	if (SciaRegs.SCICTL2.bit.TXRDY == 1)
	{
		SciaRegs.SCITXBUF = RxChar;
		CmdPacket[0] = RxChar;
		RcvTaskPointer = &GetSizeByte;
		SerialCommsTimer = 0;
	}
}

void GetSizeByte(void)		/* 状态3: 等索引/长度字节 */
{
	if (SciaRegs.SCIRXST.bit.RXRDY == 1)
	{
		RxChar = SciaRegs.SCIRXBUF.all;
		RcvTaskPointer = &EchoSizeByte;
		EchoSizeByte();
	}
	else if (SerialCommsTimer > 1000)
	{
		CommsOKflg = 0;
		RcvTaskPointer = &GetCmdByte;
		SerialCommsTimer = 0;
	}
}

void EchoSizeByte(void)		/* 状态4: 回显索引字节 */
{
	if (SciaRegs.SCICTL2.bit.TXRDY == 1)
	{
		SciaRegs.SCITXBUF = RxChar;
		CmdPacket[1] = RxChar;
		RcvTaskPointer = &GetDataByte;
		SerialCommsTimer = 0;
	}
}

void GetDataByte(void)		/* 状态5: 等数据字节 */
{
	if (SciaRegs.SCIRXST.bit.RXRDY == 1)
	{
		RxChar = SciaRegs.SCIRXBUF.all;
		RcvTaskPointer = &EchoDataByte;
		EchoDataByte();
	}
	else if (SerialCommsTimer > 500)
	{
		CommsOKflg = 0;
		RcvTaskPointer = &GetCmdByte;
		SerialCommsTimer = 0;
	}
}

void EchoDataByte(void)		/* 状态6: 回显数据字节 */
{
	if (SciaRegs.SCICTL2.bit.TXRDY == 1)
	{
		SciaRegs.SCITXBUF = RxChar;
		RcvTaskPointer = &PackWord;
	}
}

void PackWord(void)			/* 状态7: 低字节先, 拼成16位字 */
{
	if (LowByteFlag == 1)
	{
		RxWord = RxChar;
		LowByteFlag = 0;
		RcvTaskPointer = &GetDataByte;
		GetDataByte();
	}
	else
	{
		RxWord = RxWord | (RxChar << 8);
		LowByteFlag = 1;
		CmdPacket[2] = RxWord;
		RcvTaskPointer = &CmdInterpreter;
		TaskDoneFlag = 0;			/* 有新任务 */
	}
}

void CmdInterpreter(void)	/* 状态8: 解释并执行 */
{
	if (TaskDoneFlag == 0)
	{
		/* 边界检查: 噪声凑出的命令号(>=16)不能拿去索引函数指针表, 否则直接跳野地址 */
		if (CmdPacket[0] < CmdNumber)
		{
			(*CmdDispatcher[CmdPacket[0]])();
		}
		else
		{
			TaskDoneFlag = 1;		/* 非法命令, 直接丢弃 */
		}
	}

	/* 任务迟迟不结束也要退出, 防止卡死状态机.
	   ★ 但正在发数据的途中绝不能打断 (SendTaskPtr != 0 表示还有字节没发完):
	   一次读 48 点数组要发 192 字节, 57600 下约 33ms, 这段时间收不到任何字节,
	   SerialCommsTimer 会一直往上加. 原版在这里无条件超时退出, 上位机只收到
	   半截数组, 就以为"长度未知", 最后在 CreateCSV 里按 161 点越界崩溃. */
	if (SerialCommsTimer > 2500 && SendTaskPtr == 0)
	{
		CommsOKflg = 0;
		RcvTaskPointer = &GetCmdByte;
		SerialCommsTimer = 0;
	}
	if (TaskDoneFlag == 1)
	{
		RcvTaskPointer = &GetCmdByte;
	}
}

/* ===========================================================================
//  上位机命令对应的从机任务
// =========================================================================*/
void LifePulseTsk(void)		/* 命令0: 心跳, 只回一个 CommsOKflg=1 */
{
	/* 原TI工程在这里翻转控制卡上的LED(GPIO31). Boost_SFRA 没有空闲LED,
	   而且GPIO不能乱动, 所以这里只置通讯OK标志, 上位机同样能识别连接成功. */
	CommsOKflg = 1;
	SerialCommsTimer = 0;
	TaskDoneFlag = 1;
}
/*-----------------------------------------------------------*/
void TextSet(void)			/* 命令1: 16位文本/常数写入 */
{
	if (CmdPacket[1] < MAX_LIST_IDX)			/* 下标越界检查 */
	{
		*varSetTxtList[CmdPacket[1]] = CmdPacket[2];
	}
	TaskDoneFlag = 1;
}
/*-----------------------------------------------------------*/
void ButtonSet(void)		/* 命令2: 按钮(布尔)写入 */
{
	if (CmdPacket[1] < MAX_LIST_IDX)
	{
		*varSetBtnList[CmdPacket[1]] = CmdPacket[2];
	}
	TaskDoneFlag = 1;
}
/*-----------------------------------------------------------*/
void SliderSet(void)		/* 命令3: 滑块写入 */
{
	if (CmdPacket[1] < MAX_LIST_IDX)
	{
		*varSetSldrList[CmdPacket[1]] = CmdPacket[2];
	}
	TaskDoneFlag = 1;
}
/*-----------------------------------------------------------*/
void VariableGet(void)		/* 命令4: 读16位变量 */
{
	SendData();
}
/*-----------------------------------------------------------*/
void ArrayGet(void)			/* 命令5: 读32位数组(SFRA曲线数据走这里) */
{
	SendData();
}
/*-----------------------------------------------------------*/
void DataGet(void)			/* 命令6: 按地址读16位 */
{
	switch (MemGetPtr)
	{
		case 0:
			MemGetAddress = CmdPacket[2];
			MemGetPtr = 1;
			wordsLeftToGet = 1;
			SendTaskPtr = 1;
			TaskDoneFlag = 1;
			break;

		case 1:
			Temp = CmdPacket[2];
			MemGetAddress = MemGetAddress + (Temp << 16);
			memDataPtr16 = (int16_t *)MemGetAddress;
			dataOut16 = *memDataPtr16;
			SendData();

			if (TaskDoneFlag == 1)
			{
				MemGetPtr = 0;
			}
			break;
	}
}
/*-----------------------------------------------------------*/
void DataSet32(void)		/* 命令7: 32位写入(分两包), SFRA参数走这里 */
{
	switch (MemSetPtr)
	{
		case 0:
			MemSetValue = CmdPacket[2];
			MemSetPtr = 1;
			TaskDoneFlag = 1;
			break;

		case 1:
			Temp = CmdPacket[2];
			MemSetValue = MemSetValue + (Temp << 16);
			if (CmdPacket[1] < MAX_LIST_IDX)		/* 下标越界检查 */
			{
				*dataSetList[CmdPacket[1]] = MemSetValue;
			}
			MemSetPtr = 0;
			TaskDoneFlag = 1;
			break;
	}
}
/*-----------------------------------------------------------*/
void SpareTsk08(void)		/* 命令8: 保留 */
{
	TaskDoneFlag = 1;
}
/*-----------------------------------------------------------*/

void SendData(void)
{
	if (CmdPacket[0] == 0x04 || CmdPacket[0] == 0x06)
	{
		/* ---------- 16位变量传输: 一次发2字节 ---------- */
		switch (SendTaskPtr)
		{
		case 0:	/* 取指针 */
			if (CmdPacket[1] >= MAX_LIST_IDX)	/* 下标越界检查: 否则会读野地址 */
			{
				SendTaskPtr = 0;
				TaskDoneFlag = 1;
				break;
			}
			memDataPtr16 = (int16_t *)varGetList[CmdPacket[1]];
			dataOut16 = *memDataPtr16;
			wordsLeftToGet = CmdPacket[2];
			if (wordsLeftToGet > MAX_WORDS_GET) wordsLeftToGet = MAX_WORDS_GET;	/* 限制突发长度 */
			/* 故意不break, 直接进入 case 1 */

		case 1:	/* 发低字节 */
			if (wordsLeftToGet > 0)
			{
				if (SciaRegs.SCICTL2.bit.TXRDY == 1)
				{
					SciaRegs.SCITXBUF = (dataOut16 & 0x00FF);
					SendTaskPtr = 2;
				}
				else
				{
					break;		/* 发送缓冲还满, 下次再发, 不要往下走 */
				}
			}
			else
			{
				SendTaskPtr = 0;
				TaskDoneFlag = 1;
				break;
			}

		case 2:	/* 发高字节 */
			if (SciaRegs.SCICTL2.bit.TXRDY == 1)
			{
				SciaRegs.SCITXBUF = ((dataOut16 >> 8) & 0x00FF);
				memDataPtr16 = memDataPtr16 + 1;
				dataOut16 = *memDataPtr16;
				wordsLeftToGet = wordsLeftToGet - 1;
				SendTaskPtr = 1;
			}
			break;
		}
	}
	else
	{
		/* ---------- 32位数组传输: 一次发4字节, 低字节先 ---------- */
		switch (SendTaskPtr)
		{
		case 0:	/* 取指针 */
			if (CmdPacket[1] >= MAX_LIST_IDX)	/* 下标越界检查: 否则会读野地址 */
			{
				SendTaskPtr = 0;
				TaskDoneFlag = 1;
				break;
			}
			memDataPtr32 = (int32_t *)arrayGetList[CmdPacket[1]];
			dataOut32 = *memDataPtr32;
			wordsLeftToGet = CmdPacket[2];
			if (wordsLeftToGet > MAX_WORDS_GET) wordsLeftToGet = MAX_WORDS_GET;	/* 限制突发长度 */
			/* 故意不break, 直接进入 case 1 */

		case 1:	/* byte0 */
			if (wordsLeftToGet > 0)
			{
				if (SciaRegs.SCICTL2.bit.TXRDY == 1)
				{
					SciaRegs.SCITXBUF = (dataOut32 & 0x000000FF);
					SendTaskPtr = 2;
				}
				else
				{
					break;		/* 发送缓冲还满, 下次再发, 不要往下走 */
				}
			}
			else
			{
				SendTaskPtr = 0;
				TaskDoneFlag = 1;
				break;
			}

		case 2:	/* byte1 */
			if (SciaRegs.SCICTL2.bit.TXRDY == 1)
			{
				SciaRegs.SCITXBUF = ((dataOut32 >> 8) & 0x000000FF);
				SendTaskPtr = 3;
			}
			else
			{
				break;
			}

		case 3:	/* byte2 */
			if (SciaRegs.SCICTL2.bit.TXRDY == 1)
			{
				SciaRegs.SCITXBUF = ((dataOut32 >> 16) & 0x000000FF);
				SendTaskPtr = 4;
			}
			else
			{
				break;
			}

		case 4:	/* byte3 */
			if (SciaRegs.SCICTL2.bit.TXRDY == 1)
			{
				SciaRegs.SCITXBUF = ((dataOut32 >> 24) & 0x000000FF);

				memDataPtr32 = memDataPtr32 + 1;
				dataOut32 = *memDataPtr32;
				wordsLeftToGet = wordsLeftToGet - 1;
				SendTaskPtr = 1;
			}
			break;
		}
	}
}

#endif   /* SFRA_ENABLE == 1 */
