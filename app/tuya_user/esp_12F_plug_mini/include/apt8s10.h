#ifndef __APT8S10_H__
#define __APT8S10_H__

#include "espressif/c_types.h"
#include "esp8266/pin_mux_register.h"
#include "gpio.h"

extern uint8 APT8S10_ADDR;	

#define NOP() 	     	 asm("nop")

#define	I2C_SDA_HIGH	 GPIO_OUTPUT(GPIO_Pin_2,1)
#define	I2C_SDA_LOW 	 GPIO_OUTPUT(GPIO_Pin_2,0)

#define	I2C_SCL_HIGH	 GPIO_OUTPUT(GPIO_Pin_14,1)
#define	I2C_SCL_LOW 	 GPIO_OUTPUT(GPIO_Pin_14,0)

#define	I2C_SDA_READ()   GPIO_INPUT_GET(GPIO_ID_PIN(2))

#define INT_STATE() 	 GPIO_INPUT_GET(GPIO_ID_PIN(3)) 

#define I2C_ACK              0                        //Ó¦´ð
#define I2C_NOACK            1                        //·ÇÓ¦´ð

void apt8s10_init(void);
bool apt8s10_readData(u8* buf, u8 len);
bool apt8s10_writeData(u8* buf, u8 len);

void apt8s10_DirLed_OnOff(u8 ch);
void apt8s10_DirLed_level(u8 bright_level,u8 power_status);
u8 apt8s10_DirLed_level_change(u8 key_value);
u8 apt8s10_DirLed_ctrl_value_set(u8 bright_level);

u8 app_bright_level_change(u8 bright_value);
u8 app_bright_value_back(u8 bright_level);

#endif // __APTS810_H__

