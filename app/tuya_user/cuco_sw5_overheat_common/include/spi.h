#ifndef __SPI_H
#define __SPI_H

#include "device.h"
#include "gpio.h"


#define SPI3_SPEED 20

#define NOP() 	asm("nop")

typedef unsigned char byte;
typedef unsigned int  word;

#define GPO3_H() 	(GPIO_INPUT_GET(GPIO_ID_PIN(15)) ? 1 : 0)
#define GPO3_L()    (GPIO_INPUT_GET(GPIO_ID_PIN(15)) ? 0 : 1 )

#define	SetCSB()	GPIO_OUTPUT(GPIO_Pin_12,1)
#define	ClrCSB()	GPIO_OUTPUT(GPIO_Pin_12,0)
      
#define	SetFCSB()	GPIO_OUTPUT(GPIO_Pin_13,1)
#define	ClrFCSB()	GPIO_OUTPUT(GPIO_Pin_13,0)
      
#define	SetSDCK()	GPIO_OUTPUT(GPIO_Pin_5,1)
#define	ClrSDCK()	GPIO_OUTPUT(GPIO_Pin_5,0)
      
#define	SetSDIO()	GPIO_OUTPUT(GPIO_Pin_4,1)
#define	ClrSDIO()	GPIO_OUTPUT(GPIO_Pin_4,0)
      
#define InputSDIO()		GPIO_AS_INPUT(GPIO_Pin_4)
#define	OutputSDIO()	GPIO_AS_OUTPUT(GPIO_Pin_4)
      
#define	SDIO_H()	GPIO_INPUT_GET(GPIO_ID_PIN(4))
#define	SDIO_L()	GPIO_INPUT_GET(GPIO_ID_PIN(4))

void Delay_us(uint32 us);
void Delay_ms(uint32 us);

typedef struct SPI_RF{
	void (*vSpi3Init)(void);				/** initialize software SPI-3 **/	
	void (*vSpi3Write)(word dat);			/** SPI-3 send one word **/
	byte (*bSpi3Read)(byte addr);			/** SPI-3 read one byte **/

	void (*vSpi3WriteFIFO)(byte dat);		/** SPI-3 send one byte to FIFO **/
	byte (*bSpi3ReadFIFO)(void);			/** SPI-3 read one byte from FIFO **/
	void (*vSpi3BurstWriteFIFO)(byte ptr[], byte length);			/** SPI-3 burst send N byte to FIFO**/
	void (*vSpi3BurstReadFIFO)(byte ptr[], byte length);			/** SPI-3 burst read N byte to FIFO**/

	void (*vSpi3WriteByte)(byte dat);		/** SPI-3 send one byte **/
	byte (*bSpi3ReadByte)(void);			/** SPI-3 read one byte **/
}SPI_RF;

extern SPI_RF Spi3;
void spi_init(void);

#endif 

