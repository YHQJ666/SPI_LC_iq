#include "Oled.h"
#include "dcac_cfg.h"          /* 只为拿到 SFRA_ENABLE，SFRA 版把整个文件编译掉 */

#if (SFRA_ENABLE == 0)

//初始化SSD1306
void OLED_Init(void)
{
    EALLOW;

    // 配置SDA（GPIO28）
    GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 0;  // 通用GPIO模式
    GpioCtrlRegs.GPAPUD.bit.GPIO28 = 0;   // 开启内部上拉
    GpioCtrlRegs.GPADIR.bit.GPIO28 = 1;   // 输出模式

    // 配置SCL（GPIO29）
    GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 0;  // 通用GPIO模式
    GpioCtrlRegs.GPAPUD.bit.GPIO29 = 0;   // 开启内部上拉
    GpioCtrlRegs.GPADIR.bit.GPIO29 = 1;   // 输出模式

    EDIS;

    Delay_1ms(800);     //delay 800ms
    OLED_WR_Byte(0xAE,OLED_CMD);//--display off
    OLED_WR_Byte(0x00,OLED_CMD);//---set low column address
    OLED_WR_Byte(0x10,OLED_CMD);//---set high column address
    OLED_WR_Byte(0x40,OLED_CMD);//--set start line address
    OLED_WR_Byte(0xB0,OLED_CMD);//--set page address
    OLED_WR_Byte(0x81,OLED_CMD); // contract control
    OLED_WR_Byte(0xFF,OLED_CMD);//--128
    OLED_WR_Byte(0xA1,OLED_CMD);//set segment remap****************
    OLED_WR_Byte(0xA6,OLED_CMD);//--normal / reverse
    OLED_WR_Byte(0xA8,OLED_CMD);//--set multiplex ratio(1 to 64)
    OLED_WR_Byte(0x3F,OLED_CMD);//--1/32 duty
    OLED_WR_Byte(0xC8,OLED_CMD);//Com scan direction**************
    OLED_WR_Byte(0xD3,OLED_CMD);//-set display offset
    OLED_WR_Byte(0x00,OLED_CMD);//

    OLED_WR_Byte(0xD5,OLED_CMD);//set osc division
    OLED_WR_Byte(0x80,OLED_CMD);//

    OLED_WR_Byte(0xD8,OLED_CMD);//set area color mode off
    OLED_WR_Byte(0x05,OLED_CMD);//

    OLED_WR_Byte(0xD9,OLED_CMD);//Set Pre-Charge Period
    OLED_WR_Byte(0xF1,OLED_CMD);//

    OLED_WR_Byte(0xDA,OLED_CMD);//set com pin configuartion
    OLED_WR_Byte(0x12,OLED_CMD);//

    OLED_WR_Byte(0xDB,OLED_CMD);//set Vcomh
    OLED_WR_Byte(0x30,OLED_CMD);//

    OLED_WR_Byte(0x8D,OLED_CMD);//set charge pump enable
    OLED_WR_Byte(0x14,OLED_CMD);//

    OLED_WR_Byte(0xAF,OLED_CMD);//--turn on oled panel

    OLED_Clear();
}

void IIC_Start()
{
    OLED_SDIN_Set();
    OLED_SCLK_Set();
    DELAY_US(5);
    OLED_SDIN_Clr();
    DELAY_US(6);
    OLED_SCLK_Clr();
}

/**********************************************
//IIC Stop
**********************************************/
void IIC_Stop()
{
    OLED_SCLK_Clr();
    OLED_SDIN_Clr();
    OLED_SCLK_Set() ;
    DELAY_US(6);
    OLED_SDIN_Set();
    DELAY_US(6);
}

void IIC_Wait_Ack()
{

    //GPIOB->CRH &= 0XFFF0FFFF; //设置PB12为上拉输入模式
    //GPIOB->CRH |= 0x00080000;
//  OLED_SDA = 1;
//  delay_us(1);
    //OLED_SCL = 1;
    //delay_us(50000);
/*  while(1)
    {
        if(!OLED_SDA)               //判断是否接收到OLED 应答信号
        {
            //GPIOB->CRH &= 0XFFF0FFFF; //设置PB12为通用推免输出模式
            //GPIOB->CRH |= 0x00030000;
            return;
        }
    }
*/
    OLED_SDIN_Set();
    OLED_SCLK_Set() ;
    DELAY_US(3);
    OLED_SCLK_Clr();
    DELAY_US(3);
}
/**********************************************
// IIC Write byte
**********************************************/

void Write_IIC_Byte(unsigned char IIC_Byte)
{
    unsigned char i;
    unsigned char m,da;
    da=IIC_Byte;
    OLED_SCLK_Clr();
    for(i=0;i<8;i++)
    {
            m=da;
        //  OLED_SCLK_Clr();
        m=m&0x80;
        if(m==0x80)
        {
            OLED_SDIN_Set();
        }
        else
        {
            OLED_SDIN_Clr();
        }
            da=da<<1;
        DELAY_US(2);
        OLED_SCLK_Set();
        DELAY_US(2);
        OLED_SCLK_Clr();
        DELAY_US(2);
        }
}


/**********************************************
// IIC Write Command
**********************************************/
void Write_IIC_Command(unsigned char IIC_Command)
{
   IIC_Start();
   Write_IIC_Byte(0x78);            //Slave address,SA0=0
    IIC_Wait_Ack();
   Write_IIC_Byte(0x00);            //write command
    IIC_Wait_Ack();
   Write_IIC_Byte(IIC_Command);
    IIC_Wait_Ack();
   IIC_Stop();
}
/**********************************************
// IIC Write Data
**********************************************/
void Write_IIC_Data(unsigned char IIC_Data)
{
   IIC_Start();
   Write_IIC_Byte(0x78);            //D/C#=0; R/W#=0
    IIC_Wait_Ack();
   Write_IIC_Byte(0x40);            //write data
    IIC_Wait_Ack();
   Write_IIC_Byte(IIC_Data);
    IIC_Wait_Ack();
   IIC_Stop();
}
void OLED_WR_Byte(unsigned dat,unsigned cmd)
{
    if(cmd)
            {

   Write_IIC_Data(dat);

        }
    else {
   Write_IIC_Command(dat);

    }


}


/***********************Delay****************************************/
void Delay_1ms(unsigned int Del_1ms)
{
    unsigned int m;
        for(;Del_1ms>0;Del_1ms--)
            for(m=16500;m>0;m--);
}

//坐标设置

    void OLED_Set_Pos(unsigned char x, unsigned char y)
{   OLED_WR_Byte((0xb0)+y,OLED_CMD);
    OLED_WR_Byte(((x&(0xf0))>>4)|0x10,OLED_CMD);
    OLED_WR_Byte((x&(0x0f)),OLED_CMD);
}
//清屏函数
void OLED_Clear(void)
{
    u8 i,n;
    for(i=0;i<8;i++)
    {
        OLED_WR_Byte (0xb0+i,OLED_CMD);    //设置页地址（0~7）
        OLED_WR_Byte (0x00,OLED_CMD);      //设置显示位置—列低地址
        OLED_WR_Byte (0x10,OLED_CMD);      //设置显示位置—列高地址
        for(n=0;n<128;n++)OLED_WR_Byte(0,OLED_DATA);
    } //更新显示
}
//在指定位置显示一个字符,包括部分字符
//x:0~127
//y:0~63
//mode:0,反白显示;1,正常显示
void OLED_ShowChar(u8 x,u8 y,u8 chr)
{
    unsigned char c=0,i=0;
    c=chr-' ';//得到偏移后的值
    if(x>Max_Column-1){x=0;y=y+2;}

    OLED_Set_Pos(x,y);
    for(i=0;i<6;i++)
    OLED_WR_Byte(F6x8[c][i],OLED_DATA);
}
//m^n函数
u32 oled_pow(u8 m,u8 n)
{
    u32 result=1;
    while(n--)result*=m;
    return result;
}

//显示一个字符号串u8
void OLED_ShowString(u8 x,u8 y,u8 *chr)
{
    unsigned char j=0;
    while (chr[j]!='\0')
    {       OLED_ShowChar(x,y,chr[j]);
            x+=8;
        if(x>120){x=0;y+=2;}
            j++;
    }
}

#endif   /* SFRA_ENABLE == 0 */