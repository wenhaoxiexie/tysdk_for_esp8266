#include "ws100t10.h"
#include "gpio.h"

#define	SPI_SDA_HIGH	 GPIO_OUTPUT(GPIO_Pin_4,1)
#define	SPI_SDA_LOW 	 GPIO_OUTPUT(GPIO_Pin_4,0)

#define	SPI_SCL_HIGH	 GPIO_OUTPUT(GPIO_Pin_5,1)
#define	SPI_SCL_LOW 	 GPIO_OUTPUT(GPIO_Pin_5,0)

#define	SPI_STB_HIGH	 gpio16_output_set(1)
#define	SPI_STB_LOW 	 gpio16_output_set(0)

#define LIGHT_MAX        76

/*************************************************************
function: ws100t10_init
return: void
note: ��ʼ��gpio״̬
author: xwh 
date: 2020-03-11
**************************************************************/
void ws100t10_init(void)
{
	gpio16_output_conf();

	SPI_SDA_HIGH;
	SPI_STB_HIGH;
	SPI_SCL_HIGH;
}
/*************************************************************
function: ws100t10_SendData
ptr1: cmd (����ֵ)
ptr2: channel (����ͨ�� channel 0 ��channel 1)
return: void
note: ����ֵ��Χ0~81 ����ο�ws100t10�����ĵ�
author: xwh 
date: 2020-03-11
**************************************************************/
static void ws100t10_SendData(u8 cmd,u8 channel)
{
	if(cmd<0||cmd>81)
		return ;
	
	u8 i,dl,dh; 
	uint32 data;
	
	dh = cmd;
	if(channel == 1) 
		dh |= 0x80;
	dl = ~dh; 
	data = dl;
	data |= dh<<8;
		
	SPI_STB_LOW; 
	os_delay_us(200);
	for(i=0;i<16;i++)
	{
		SPI_SCL_LOW;
		os_delay_us(500); 
		if(data & 0x8000) 
			SPI_SDA_HIGH;
		else 
			SPI_SDA_LOW;
		SPI_SCL_HIGH;
		os_delay_us(500); 
		data <<= 1;
	}
	SPI_SDA_HIGH;
	SPI_STB_HIGH;
	SPI_SCL_HIGH;
}
/*************************************************************
function: ws100t10_ControlLight
ptr1: lightness (����ֵ)
ptr2: channel (����ͨ�� channel 0 ��channel 1)
return: void
note: ����ֵ��ͨ�������������ƣ��������ⷶΧ0~150
author: xwh 
date: 2020-03-11
**************************************************************/
void ws100t10_ControlLight(u8 lightness,u8 channel)
{
	if(lightness<0||lightness>150)
		return;
	
	u8 cmd= LIGHT_MAX-lightness/2;
	
	ws100t10_SendData(cmd,channel);
}

