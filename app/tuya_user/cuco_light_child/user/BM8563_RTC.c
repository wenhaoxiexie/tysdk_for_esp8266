#include "BM8563_RTC.h"
#include "espressif/c_types.h"
#include "esp8266/pin_mux_register.h"
#include "gpio.h"
#include "device.h"

#if 1
#define		BM8563_SCL_SET	GPIO_OUTPUT(GPIO_Pin_14,1) //<��SCL����Ϊ�ߵ�ƽ
#define		BM8563_SCL_CLR	GPIO_OUTPUT(GPIO_Pin_14,0) //<��SCL����Ϊ�͵�ƽ

#define		BM8563_SDA_SET   GPIO_OUTPUT(GPIO_Pin_2,1) //<��SDA����Ϊ�ߵ�ƽ
#define		BM8563_SDA_CLR	GPIO_OUTPUT(GPIO_Pin_2,0) //<��SDA����Ϊ�͵�ƽ

#define		BM8563_SDA_IN    GPIO_INPUT_GET(GPIO_ID_PIN(2))  //<��SDA��Ϊ���뷽��ʱ����ȡ�ĵ�ƽֵ
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

static uchar twdata[9]={0x00,0x00,0x58,0x59,0x23,0x31,0x06,0x12,0x04};/*ǰ2��������������״̬�Ĵ������� 7 ����������ʱ��Ĵ��� */
static uchar trdata[7]; /*�������������洢��ȡ��ʱ������ */
static uchar asc[14]; /*�������������洢ת���� asc ��ʱ�����ݣ�����ʾ�� */
static Byte ack;

/********************************************************************
�� �� ���� Start_I2C(void)
�� �ܣ����� I2C ���ߵĴ���
˵ ����
�� �ã�
��ڲ�������
�� �� ֵ����
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
�� �� ���� Stop_I2C(void)
�� �ܣ�ֹͣ I2C ���ߵĴ���
˵ ����
�� �ã�
��ڲ�������
�� �� ֵ����
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
�� �� ���� Ack_I2C(void)
�� �ܣ���Ƭ����Ϊ���ջ�ʱ�������� BM8563 �ṩ����Ӧ���ź�
˵ ����ÿ�� BM8563 ��ȡһ���ֽ����ݺ�Ҫ����Ӧ���ź�
�� �ã��� BM8563(void)
��ڲ����� ack��Ӧ���źţ�
�� �� ֵ����
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
�� �� ���� SendByte(void)
�� �ܣ��� BM8563 дһ���ֽڵ�����
˵ ���� �� GetBM8563(void) ����
�� �ã��� GetBM8563(void)�� setBM8563(void)����
��ڲ����� c��Ҫд��ĵ��ֽ����ݣ�
�� �� ֵ����
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
�� �� ���� RcvByte(void)
�� �ܣ��� BM8563 ��ȡһ���ֽڵ�����
˵ ���� �ó���Ҫ��� SendByte()����һ����ɶ� BM8563�Ķ���������GetBM8563(void)
����
�� �ã��� GetBM8563(void)����
��ڲ�������
�� �� ֵ�� rect
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
�� �� ���� GetBM8563(void)
�� �ܣ��� BM8563 ���ڲ��Ĵ�����ʱ�䡢״̬�������ȼĴ�������ȡ����
˵ �����ó�����������ȡ BM8563 ���ڲ��Ĵ�����Ʃ��ʱ�䣬������״̬�ȼĴ���
����ҳд�ķ�ʽ���������ݵĸ���Ϊ no�� no ��������Ϊ 1 ���ǵ��ֽڷ�ʽ
�� �ã� Start_I2C()�� SendByte()�� RcvByte()�� Ack_I2C()�� Stop_I2C()
��ڲ����� sla��BM8563 �ӵ�ַ���� suba��BM8563 �ڲ��Ĵ�����ַ��
*s�����ö�ȡ���ݴ洢��ָ�룩�� no���������ݵĸ�����
�� �� ֵ���У����ز�������bit��������������ɹ���
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
	Stop_I2C();//�����һ���ֽ��⣬������Ҫ�� MASTER ��Ӧ��
	return(1);
}


/********************************************************************
�� �� ���� SetBM8563(void)
�� �ܣ����� BM8563 ���ڲ��Ĵ�����ʱ�䣬�����ȼĴ�����
˵ �����ó������������� BM8563 ���ڲ��Ĵ�����Ʃ��ʱ�䣬������״̬�ȼĴ���
����ҳд�ķ�ʽ���������ݵĸ���Ϊ no�� no ��������Ϊ 1 ���ǵ��ֽڷ�ʽ
�� �ã� Start_I2C()�� SendByte()�� Stop_I2C()
��ڲ����� sla��BM8563 �ӵ�ַ���� suba��BM8563 �ڲ��Ĵ�����ַ��
*s�����ó�ʼ�����ݵ�ָ�룩�� no���������ݵĸ�����
�� �� ֵ���У����ز�������bit��������������ɹ���
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
�� �� ���� void Bcd2asc(void)
�� �ܣ� bcd ��ת���� asc �룬��Һ����ʾ��
˵ ����
�� �ã�
��ڲ�����
�� �� ֵ����
***********************************************************************/
void Bcd2asc(void)
{
	uchar i,j;
	
	for (j=0,i=0; i<7; i++)
	{
		asc[j++] =(trdata[i]&0xf0)>>4|0x30 ;/*��ʽΪ: �� �� ʱ �� �� ���� �� */
		asc[j++] =trdata[i]&0x0f|0x30;
	}
}

/********************************************************************
�� �� ���� datajust(void)
�� �ܣ���������ʱ�����ݵ��޹�λ���ε�
˵ ���� BM8563 ʱ�ӼĴ�������Щ���޹�λ�����Խ���Чλ���ε�
�� �ã�
��ڲ�����
�� �� ֵ����
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

