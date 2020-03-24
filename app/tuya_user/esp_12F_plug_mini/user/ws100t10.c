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
note: 初始化gpio状态
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
ptr1: cmd (亮度值)
ptr2: channel (负载通道 channel 0 、channel 1)
return: void
note: 亮度值范围0~81 详情参考ws100t10技术文档
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
ptr1: lightness (亮度值)
ptr2: channel (负载通道 channel 0 、channel 1)
return: void
note: 亮度值是通过触摸板来控制，触摸板检测范围0~150
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

