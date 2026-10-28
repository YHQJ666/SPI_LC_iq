/*****************************************************
 * Oled.h
 *
 *  Created on: Jun 25, 2020
 *      Author: 11219
 *
 *      SCL  GPIO32
 *      SDI  GPIO33
 *      x:0-128
 *      y:0-7
 *
 *      OLED_ShowNum(x,y,(float)num,len,lot,size)
 *      			location  num  integer decimals
 *
 *      when (int)a=48,ShowChar(a) is 0.
 *      the char can show is start at 32(Dec).
 *      ascii 	dec
 *      0		48
 *      9		57
 *      A		65
 *      Z		90
 *      a		97
 *      z		122
 *
 *************************************************/

#ifndef OLED_H_
#define OLED_H_

#include <APP/Oledfont.h>
#include "DSP2803x_Device.h"     // DSP2833x 头文件
#include "DSP2803x_Examples.h"   // DSP2833x 例子相关头文件

#define OLED_MODE 0
#define SIZE 8
#define XLevelL		0x00
#define XLevelH		0x10
#define Max_Column	128
#define Max_Row		64
#define	Brightness	0xFF
#define X_WIDTH 	128
#define Y_WIDTH 	64
#define u8 char
#define u32 Uint32
//-----------------OLED IIC端口定义----------------

#define OLED_SCLK_Clr() GpioDataRegs.GPACLEAR.bit.GPIO29=1;
#define OLED_SCLK_Set() GpioDataRegs.GPASET.bit.GPIO29=1;

#define OLED_SDIN_Clr() GpioDataRegs.GPACLEAR.bit.GPIO28=1;
#define OLED_SDIN_Set() GpioDataRegs.GPASET.bit.GPIO28=1;


#define OLED_CMD  0	//写命令
#define OLED_DATA 1	//写数据


//OLED控制用函数
void OLED_WR_Byte(unsigned dat,unsigned cmd);
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Init(void);
void OLED_Clear(void);
void OLED_DrawPoint(u8 x,u8 y,u8 t);
void OLED_Fill(u8 x1,u8 y1,u8 x2,u8 y2,u8 dot);
void OLED_On(void);
void OLED_ShowChar(u8 x,u8 y,u8 chr);
void OLED_ShowNum(u8 x,u8 y,float num,u8 len,u8 lot);
void OLED_ShowString(u8 x,u8 y, u8 *p);
void OLED_Set_Pos(unsigned char x, unsigned char y);
void OLED_DrawBMP(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[]);
void Delay_50ms(unsigned int Del_50ms);
void Delay_1ms(unsigned int Del_1ms);
void Delay_3us(void);
void fill_picture(unsigned char fill_Data);
void Picture();
void IIC_Start();
void IIC_Stop();
void Write_IIC_Command(unsigned char IIC_Command);
void Write_IIC_Data(unsigned char IIC_Data);
void Write_IIC_Byte(unsigned char IIC_Byte);

void IIC_Wait_Ack();


#endif /* OLED_H_ */
