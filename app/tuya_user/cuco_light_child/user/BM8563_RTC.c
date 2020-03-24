#include "BM8563_RTC.h"
#include "espressif/c_types.h"
#include "esp8266/pin_mux_register.h"
#include "gpio.h"
#include "device.h"

#if 1
#define		BM8563_SCL_SET	GPIO_OUTPUT(GPIO_Pin_14,1) //<将SCL设置为高电平
#define		BM8563_SCL_CLR	GPIO_OUTPUT(GPIO_Pin_14,0) //<将SCL设置为低电平

#define		BM8563_SDA_SET   GPIO_OUTPUT(GPIO_Pin_2,1) //<将SDA设置为高电平
#define		BM8563_SDA_CLR	GPIO_OUTPUT(GPIO_Pin_2,0) //<将SDA设置为低电平

#define		BM8563_SDA_IN    GPIO_INPUT_GET(GPIO_ID_PIN(2))  //<当SDA设为输入方向时，读取的电平值
#endif 
#if 0
#define I2C_MASTER_SDA_MUX PERIPHS_IO_MUX_GPIO2_U
#define I2C_MASTER_SCL_MUX PERIPHS_IO_MUX_MTMS_U
#define I2C_MASTER_SDA_GPIO 2
#define I2C_MASTER_SCL_GPIO 14
#define I2C_MASTER_SDA_FUNC FUNC_GPIO2
#define I2C_MASTER_SCL_FUNC FUNC_GPIO14

#endif

#define uchar unsigned char
#define Byte unsigned char

static uchar twdata[9]={0x00,0x00,0x58,0x59,0x23,0x31,0x06,0x12,0x04};/*前2个数据用来设置状态寄存器，后 7 个用来设置时间寄存器 */
static uchar trdata[7]; /*定义数组用来存储读取的时间数据 */
static uchar asc[14]; /*定义数组用来存储转换的 asc 码时间数据，供显示用 */
static Byte ack;

/********************************************************************
函 数 名： Start_I2C(void)
功 能：启动 I2C 总线的传输
说 明：
调 用：
入口参数：无
返 回 值：无
***********************************************************************/
static void Start_I2C(void)
{
	BM8563_SDA_SET;
	os_delay_us(10);
	
	BM8563_SCL_SET;
	os_delay_us(20);
	
	BM8563_SDA_CLR;
	os_delay_us(20);
	
	BM8563_SCL_CLR;
	os_delay_us(10);
}

/********************************************************************
函 数 名： Stop_I2C(void)
功 能：停止 I2C 总线的传输
说 明：
调 用：
入口参数：无
返 回 值：无
***********************************************************************/
static void Stop_I2C(void)
{
	BM8563_SCL_CLR;
	os_delay_us(10);

	BM8563_SCL_SET;
	os_delay_us(20);

	BM8563_SDA_SET;
	os_delay_us(20);
}

/********************************************************************
函 数 名： Ack_I2C(void)
功 能：单片机作为接收机时，用来向 BM8563 提供接收应答信号
说 明：每从 BM8563 读取一个字节数据后都要发送应答信号
调 用：被 BM8563(void)
入口参数： ack（应答信号）
返 回 值：无
***********************************************************************/
static void Ack_I2C(char ack)
{
	if(ack==0)
		BM8563_SDA_CLR;
	else 
		BM8563_SDA_SET;
	os_delay_us(20);
	
	BM8563_SCL_SET;
	os_delay_us(20);
	
	BM8563_SCL_CLR;
	os_delay_us(20);
}

/********************************************************************
函 数 名： SendByte(void)
功 能：向 BM8563 写一个字节的数据
说 明： 被 GetBM8563(void) 调用
调 用：被 GetBM8563(void)、 setBM8563(void)调用
入口参数： c（要写入的单字节数据）
返 回 值：无
***********************************************************************/
static void SendByte(char data )
{
	unsigned char i;
	for(i=0;i<8;i++)
	{
		if((data<<i)&0x80)
			BM8563_SDA_SET;
		else 
			BM8563_SDA_CLR;
		os_delay_us(10);
		
		BM8563_SCL_SET;
		os_delay_us(20);
		BM8563_SCL_CLR;
	}
	os_delay_us(10);

	BM8563_SDA_SET;
	os_delay_us(10);
	
	BM8563_SCL_SET;
	
	os_delay_us(20);
	
	if(BM8563_SDA_IN==1)
		ack=0;
	else 
		ack=1;
	BM8563_SCL_CLR;
	os_delay_us(10);
}

/********************************************************************
函 数 名： RcvByte(void)
功 能：从 BM8563 读取一个字节的数据
说 明： 该程序要配合 SendByte()程序一起完成对 BM8563的读操作，被GetBM8563(void)
调用
调 用：被 GetBM8563(void)调用
入口参数：无
返 回 值： rect
***********************************************************************/
uchar RcvByte(void)
{
	uchar i,rect;
	rect=0;
	
	BM8563_SDA_SET;
	for(i=0;i<8;i++)
	{
		os_delay_us(10);
		
		BM8563_SCL_CLR;
		os_delay_us(20);
		
		BM8563_SCL_SET;
		os_delay_us(20);
		
		rect=rect<<1;
		if(BM8563_SDA_IN==1)
			rect=rect+1;
		
		os_delay_us(10);
		
	}
	BM8563_SCL_CLR;
	os_delay_us(10);

	return(rect);
}


/********************************************************************
函 数 名： GetBM8563(void)
功 能：从 BM8563 的内部寄存器（时间、状态、报警等寄存器）读取数据
说 明：该程序函数用来读取 BM8563 的内部寄存器，譬如时间，报警，状态等寄存器
采用页写的方式，设置数据的个数为 no， no 参数设置为 1 就是单字节方式
调 用： Start_I2C()， SendByte()， RcvByte()， Ack_I2C()， Stop_I2C()
入口参数： sla（BM8563 从地址）， suba（BM8563 内部寄存器地址）
*s（设置读取数据存储的指针）， no（传输数据的个数）
返 回 值：有，返回布尔量（bit）用来鉴定传输成功否
***********************************************************************/
BYTE GetBM8563(uchar sla,uchar suba,uchar *s,uchar no)
{
	uchar i;
	
	Start_I2C();
	SendByte(sla);
	if(ack==0)
		return(0);
	SendByte(suba);
	if(ack==0)
		return(0);
	Start_I2C();
	SendByte(sla+1);
	if(ack==0)
		return(0);
	for (i=0;i<no-1;i++)
	{
		*s=RcvByte();
		Ack_I2C(0);
		s++;
	}
	*s=RcvByte();
	Ack_I2C(1);
	Stop_I2C();//除最后一个字节外，其他都要从 MASTER 发应答。
	return(1);
}


/********************************************************************
函 数 名： SetBM8563(void)
功 能：设置 BM8563 的内部寄存器（时间，报警等寄存器）
说 明：该程序函数用来设置 BM8563 的内部寄存器，譬如时间，报警，状态等寄存器
采用页写的方式，设置数据的个数为 no， no 参数设置为 1 就是单字节方式
调 用： Start_I2C()， SendByte()， Stop_I2C()
入口参数： sla（BM8563 从地址）， suba（BM8563 内部寄存器地址）
*s（设置初始化数据的指针）， no（传输数据的个数）
返 回 值：有，返回布尔量（bit）用来鉴定传输成功否
***********************************************************************/
BYTE SetBM8563(uchar sla,uchar suba,uchar *s,uchar no)
{
	uchar i;
	
	Start_I2C();
	SendByte(sla);
	if(ack==0)
		return(0);
	SendByte(suba);
	if(ack==0)
		return(0);
	for(i=0;i<no;i++)
	{
		SendByte(*s);
		if(ack==0)return(0);
		s++;
	}
	Stop_I2C();
	return(1);
}

/********************************************************************
函 数 名： void Bcd2asc(void)
功 能： bcd 码转换成 asc 码，供液晶显示用
说 明：
调 用：
入口参数：
返 回 值：无
***********************************************************************/
void Bcd2asc(void)
{
	uchar i,j;
	
	for (j=0,i=0; i<7; i++)
	{
		asc[j++] =(trdata[i]&0xf0)>>4|0x30 ;/*格式为: 秒 分 时 日 月 星期 年 */
		asc[j++] =trdata[i]&0x0f|0x30;
	}
}

/********************************************************************
函 数 名： datajust(void)
功 能：将读出的时间数据的无关位屏蔽掉
说 明： BM8563 时钟寄存器中有些是无关位，可以将无效位屏蔽掉
调 用：
入口参数：
返 回 值：无
***********************************************************************/
void datajust(void)
{
	trdata[0] = trdata[0]&0x7f;
	trdata[1] = trdata[1]&0x7f;
	trdata[2] = trdata[2]&0x3f;
	trdata[3] = trdata[3]&0x3f;
	trdata[4] = trdata[4]&0x07;
	trdata[5] = trdata[5]&0x1f;
	trdata[6] = trdata[6]&0xff;
}

void BM8563_TIMER_READ(void)
{
    uint8 j;
	
    do{
        j=GetBM8563(0xa2,0x02,trdata,0x07);
    }
    while(j==0);
	
    datajust();

	printf(">>>>>>date:20%02x-%02x-%02x %02x:%02x:%02x \n",trdata[5],trdata[4],trdata[3],
  											              trdata[2],trdata[1],trdata[0]);
}

void BM8563_TIMER_INIT(void)
{
    uint8 j;
	
    do{
      j=SetBM8563(0xa2,0x00,twdata,0x09);
    }while(j==0);
}

#if 0
void BM8563_init(void)
{
	PIN_FUNC_SELECT(I2C_MASTER_SDA_MUX, I2C_MASTER_SDA_FUNC);
    PIN_FUNC_SELECT(I2C_MASTER_SCL_MUX, I2C_MASTER_SCL_FUNC);

	GPIO_REG_WRITE(GPIO_PIN_ADDR(GPIO_ID_PIN(I2C_MASTER_SDA_GPIO)), GPIO_REG_READ(GPIO_PIN_ADDR(GPIO_ID_PIN(I2C_MASTER_SDA_GPIO))) | GPIO_PIN_PAD_DRIVER_SET(GPIO_PAD_DRIVER_ENABLE)); //open drain;
    GPIO_REG_WRITE(GPIO_ENABLE_ADDRESS, GPIO_REG_READ(GPIO_ENABLE_ADDRESS) | (1 << I2C_MASTER_SDA_GPIO));	GPIO_REG_WRITE(GPIO_PIN_ADDR(GPIO_ID_PIN(I2C_MASTER_SCL_GPIO)), GPIO_REG_READ(GPIO_PIN_ADDR(GPIO_ID_PIN(I2C_MASTER_SCL_GPIO))) | GPIO_PIN_PAD_DRIVER_SET(GPIO_PAD_DRIVER_ENABLE)); //open drain;    GPIO_REG_WRITE(GPIO_ENABLE_ADDRESS, GPIO_REG_READ(GPIO_ENABLE_ADDRESS) | (1 << I2C_MASTER_SCL_GPIO));
	
}
#endif 

