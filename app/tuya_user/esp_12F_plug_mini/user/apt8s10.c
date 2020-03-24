#include "apt8s10.h"
#include "device.h"
#include "esp8266/ets_sys.h"
#include "esp8266/gpio_register.h"
#include "esp8266/pin_mux_register.h"

#define I2C_MASTER_SDA_MUX PERIPHS_IO_MUX_GPIO2_U
#define I2C_MASTER_SCL_MUX PERIPHS_IO_MUX_MTMS_U
#define I2C_MASTER_SDA_GPIO 2
#define I2C_MASTER_SCL_GPIO 14
#define I2C_MASTER_SDA_FUNC FUNC_GPIO2
#define I2C_MASTER_SCL_FUNC FUNC_GPIO14

#define RGW_CPU  // 瑞格微触摸芯片

uint8 APT8S10_ADDR = 0xa6;	

extern u8 power_led_status;

void I2C_Delay(void)
{
  os_delay_us(50);
}


static void I2C_Start(void)
{
  I2C_SCL_HIGH;                                  //SCL高
  I2C_Delay();
 
  I2C_SDA_HIGH;                                  //SDA高 -> 低
  I2C_Delay();

  I2C_SDA_LOW;                                   //SDA低
  I2C_Delay();

  I2C_SCL_LOW;                                   //SCL低(待写地址/数据)
  I2C_Delay();
}
static void I2C_Stop(void)
{
  I2C_SDA_LOW;                                   //SDA低 -> 高
  I2C_Delay();

  I2C_SCL_HIGH;                                  //SCL高
 I2C_Delay();

  I2C_SDA_HIGH;                                  //SDA高
 I2C_Delay();
}
static void I2C_PutAck(uint8 Ack)
{
  I2C_SCL_LOW;                                   //SCL低
  I2C_Delay();

  if(I2C_ACK == Ack)
    I2C_SDA_LOW;                                 //应答
  else
    I2C_SDA_HIGH;                                //非应答
  I2C_Delay();

  I2C_SCL_HIGH;                                  //SCL高 -> 低
  I2C_Delay();
  
  I2C_SCL_LOW;                                   //SCL低
  I2C_Delay();
}
static uint8 I2C_GetAck(void)
{
  uint8 ack;

  I2C_SCL_LOW;                                   //SCL低 -> 高
  I2C_Delay();
  I2C_Delay();
  I2C_Delay();
  I2C_Delay();

  I2C_SDA_HIGH;                                  //释放SDA(开漏模式有效)
  
  I2C_Delay();
  I2C_Delay();
  I2C_Delay();
  I2C_Delay();
	
  I2C_SCL_HIGH;//SCL高(读取应答位)
  
  I2C_Delay();
  I2C_Delay();
  I2C_Delay();
  I2C_Delay();   
  
  if(I2C_SDA_READ())
    ack = I2C_NOACK;                             //非应答
  else
    ack = I2C_ACK;                               //应答
  
  I2C_SCL_LOW;                                   //SCL低
  I2C_Delay();
  I2C_Delay();
  I2C_Delay();
  I2C_Delay();

  return ack;
}

static void I2C_WriteByte(uint8 Data)
{
  uint8 cnt;

  for(cnt=0; cnt<8; cnt++)
  {
    I2C_SCL_LOW;                                 //SCL低(SCL为低电平时变化SDA有效)
    I2C_Delay();
	I2C_Delay();

    if(Data & 0x80)
      I2C_SDA_HIGH;                              //SDA高
    else
      I2C_SDA_LOW;                               //SDA低
    Data <<= 1;
    I2C_Delay();
	I2C_Delay();

    I2C_SCL_HIGH;                                //SCL高(发送数据)
    I2C_Delay();
	I2C_Delay();
  }
  I2C_SCL_LOW;                                   //SCL低(等待应答信号)
  I2C_Delay();
  I2C_Delay();
  
}
static uint8 I2C_ReadByte(uint8 ack)
{
  uint8 cnt;
  uint8 data;

  I2C_SCL_LOW;                                   //SCL低
  I2C_Delay();

  I2C_SDA_HIGH;                                  //释放SDA(开漏模式有效)

  for(cnt=0; cnt<8; cnt++)
  {
    I2C_SCL_HIGH;                                //SCL高(读取数据)
    I2C_Delay();

    data <<= 1;
    if(I2C_SDA_READ())
      data |= 0x01;                              //SDA为高(数据有效)

    I2C_SCL_LOW;                                 //SCL低
    I2C_Delay();
  }

  I2C_PutAck(ack);                               //产生应答(或者非应答)位

  return data;  
}

/**********************************************************
**Name: 	apt8s10_init
**Func: 	1.引脚功能选择为GPIO 
		2.初始化I2C状态
**author: xwh 2020/03/04	
**********************************************************/
void apt8s10_init(void)
{	

	PIN_FUNC_SELECT(I2C_MASTER_SDA_MUX, I2C_MASTER_SDA_FUNC);
    PIN_FUNC_SELECT(I2C_MASTER_SCL_MUX, I2C_MASTER_SCL_FUNC);
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0RXD_U,FUNC_GPIO3);

	
	//PIN_FUNC_SELECT(PERIPHS_IO_MUX_SD_DATA0_U,FUNC_GPIO9);
	//GPIO_AS_OUTPUT(GPIO_Pin_9);
	
	GPIO_AS_INPUT(GPIO_Pin_3);

	
    GPIO_REG_WRITE(GPIO_PIN_ADDR(GPIO_ID_PIN(I2C_MASTER_SDA_GPIO)), GPIO_REG_READ(GPIO_PIN_ADDR(GPIO_ID_PIN(I2C_MASTER_SDA_GPIO))) | GPIO_PIN_PAD_DRIVER_SET(GPIO_PAD_DRIVER_ENABLE)); //open drain;
    GPIO_REG_WRITE(GPIO_ENABLE_ADDRESS, GPIO_REG_READ(GPIO_ENABLE_ADDRESS) | (1 << I2C_MASTER_SDA_GPIO));

	GPIO_REG_WRITE(GPIO_PIN_ADDR(GPIO_ID_PIN(I2C_MASTER_SCL_GPIO)), GPIO_REG_READ(GPIO_PIN_ADDR(GPIO_ID_PIN(I2C_MASTER_SCL_GPIO))) | GPIO_PIN_PAD_DRIVER_SET(GPIO_PAD_DRIVER_ENABLE)); //open drain;
    GPIO_REG_WRITE(GPIO_ENABLE_ADDRESS, GPIO_REG_READ(GPIO_ENABLE_ADDRESS) | (1 << I2C_MASTER_SCL_GPIO));

}
bool apt8s10_writeData(u8* buf, u8 len)
{

#ifdef RGW_CPU // 瑞格微
	I2C_Start();

	os_delay_us(4000);
	
	I2C_WriteByte(APT8S10_ADDR); 	

	if (I2C_GetAck()) {
		PR_ERR("touch addr not ack when tx write cmd!");		
		I2C_Stop();
		return FALSE;
	}
		
	I2C_WriteByte(0x00);	

	if (I2C_GetAck()) {
		PR_ERR("touch addr not ack when tx write cmd!");		
		I2C_Stop();
		return FALSE;
	}
	while(len--) {
		I2C_WriteByte(*buf);
		if (I2C_GetAck()) {
			PR_ERR("touch not ack when tx write data!");		
			I2C_Stop();
			return FALSE;
		}
		buf++;
	}
	I2C_Stop();
		
	return TRUE;
#else  // 晶格
	I2C_Start();
		
	I2C_WriteByte(APT8S10_ADDR & 0xFE);
			
	if (I2C_GetAck()) {
		PR_ERR("touch addr not ack when tx write cmd!");		
		I2C_Stop();
		return FALSE;
	}
		
	while(len--) {
		I2C_WriteByte(*buf);
		if (I2C_GetAck()) {
			PR_ERR("touch not ack when tx write data!");		
			I2C_Stop();
			return FALSE;
		}	  
		buf++;
	}
	
	I2C_Stop();
		
	return TRUE;
#endif
}

bool apt8s10_readData(u8* buf, u8 len)
{
#ifdef RGW_CPU 
	I2C_Start();
		
	I2C_WriteByte(APT8S10_ADDR);	
	
	if (I2C_GetAck()) {
		PR_ERR("touch addr not ack when tx write cmd!");		
		I2C_Stop();
		return FALSE;
	}

	I2C_WriteByte(0x01);	

	if (I2C_GetAck()) {
		PR_ERR("touch addr not ack when tx write cmd!");		
		I2C_Stop();
		return FALSE;
	}	

	I2C_Start();	
	
	I2C_WriteByte(APT8S10_ADDR|0x01);

	if (I2C_GetAck()) {
		PR_ERR("touch addr not ack when tx write cmd!");		
		I2C_Stop();
		return FALSE;
	}
	
	*buf = I2C_ReadByte(1);	
		
	I2C_Stop();
		
	return TRUE;
#else // 晶格

	I2C_Start();
		
	I2C_WriteByte(APT8S10_ADDR | 0x01);
	
	if (I2C_GetAck()) {
		PR_ERR("touch addr not ack when tx write cmd!");		
		I2C_Stop();
		return FALSE;
	}

#if 	0	
	while(len) {
	   *buf = I2C_ReadByte(1);
		   
	   if(1 == len)
		  I2C_PutAck(0);
	   else 
		  I2C_PutAck(1);
		   
	   buf++;
	   len--;
	}
#else
	   *buf = I2C_ReadByte(1);

#endif	
		
	I2C_Stop();
		
	return TRUE;
#endif 
}

void apt8s10_DirLed_OnOff(u8 ch)
{
	apt8s10_writeData(&ch, 1);	
	SystemSleep(10);
}

void apt8s10_DirLed_level(u8 bright_level, u8 power_status)
{
   u8 i, ctrl_value[9] = {0x01, 0x03, 0x07, 0x0f, 0x1f, 0x3f, 0x7f, 0xff, 0x00};

   for (i = 0; i < bright_level; i++) {	
   	    //if (power_status)
			// apt8s10_DirLed_OnOff(ctrl_value[i] | 0x80);		
		//else
			 apt8s10_DirLed_OnOff(ctrl_value[i]);						
		SystemSleep(200);
   }
}
u8 apt8s10_DirLed_level_change(u8 key_value)
{
	u8 bright_level;
	
	if (key_value > 0)
		bright_level = key_value / 20 + 1;		
	else
		bright_level = 0;
	
	if (bright_level > 7)
		bright_level = 7; 

	return bright_level;
}
u8 apt8s10_DirLed_ctrl_value_set(u8 bright_level)
{
	u8 i, ret = 0;

	for (i = 0; i < bright_level; i++) {
		ret <<= 1;
		ret |= 0x01;
	}

	if (power_led_status)
	  ret |= 0x80;
	
	return ret;
}
u8 app_bright_level_change(u8 bright_value)
{
	u8 level;

	bright_value -= 25;
	
	if (bright_value > 0)
		level = bright_value / 33 + 1;		
	else
		level = 0;
	
	if (level > 7)
		level = 7; 

	return level;
}
u8 app_bright_value_back(u8 bright_level)
{
	u8 bright_value = 0;

	if (bright_level == 0)
		bright_value = 25;
	else if (bright_level == 7)
		bright_value = 255;
	else 
		bright_value = bright_level * 33; 

	return bright_value;
}


