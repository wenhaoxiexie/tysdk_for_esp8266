
/**********************************************************
3-wire spi
**********************************************************/

#include "spi.h"
#include "stdlib.h"

SPI_RF Spi3;

/**********************************************************
**Name: 	Delay_us
**Func: 	tiem delay
**Note: 	delay 0.0625us  time = us/0.0625
**********************************************************/
void Delay_us(uint32 us)
{
	while(us--)
		NOP();
}
/**********************************************************
**Name: 	Delay_ms
**Func: 	tiem delay
**Note: 	delay 1ms
**********************************************************/
void Delay_ms(uint32 us)
{
	while(us--)
	{
		NOP();
		NOP();
	}
}


/**********************************************************
**Name: 	vSpi3Init
**Func: 	Init Spi-3 Config
**Note: 	GPIO4-->SPI_SDIO¡¢GPIO5-->SPI_SCLK
	     GPIO12-->SPI_CSB¡¢GPIO13-->FCSB
**********************************************************/
void vSpi3Init(void)
{
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDI_U,FUNC_GPIO12);
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U,FUNC_GPIO13);
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDO_U,FUNC_GPIO15);

	GPIO_AS_OUTPUT(GPIO_Pin_4);
	GPIO_AS_OUTPUT(GPIO_Pin_5);
	GPIO_AS_OUTPUT(GPIO_Pin_12);
	GPIO_AS_OUTPUT(GPIO_Pin_13);
	GPIO_AS_INPUT(GPIO_Pin_15);
	
	SetCSB();
	SetFCSB();
	SetSDIO();
	ClrSDCK();
}

/**********************************************************
**Name: 	vSpi3WriteByte
**Func: 	SPI-3 send one byte
**Input:
**Output:  
**********************************************************/
void vSpi3WriteByte(byte dat)
{

 	byte bitcnt;	
 
	SetFCSB();				//FCSB = 1;
 
 	OutputSDIO();			//SDA output mode
 	OutputSDIO();			//SDA output mode
 	SetSDIO();				//    output 1
 
 	ClrSDCK();				
 	ClrCSB();

 	for(bitcnt=8; bitcnt!=0; bitcnt--)
 		{
		ClrSDCK();	
		//Delay_us(SPI3_SPEED);
		Delay_us(320);	
 		if(dat&0x80)
 			SetSDIO();
 		else
 			ClrSDIO();
		SetSDCK();
 		dat <<= 1; 		
 		//Delay_us(SPI3_SPEED);
 		Delay_us(320);
 		}
 	ClrSDCK();		
 	SetSDIO();
}

/**********************************************************
**Name: 	bSpi3ReadByte
**Func: 	SPI-3 read one byte
**Input:
**Output:  
**********************************************************/
byte bSpi3ReadByte(void)
{
	byte RdPara = 0;
 	byte bitcnt;
  
 	ClrCSB(); 
 	InputSDIO();			
  	InputSDIO();		
 	for(bitcnt=8; bitcnt!=0; bitcnt--)
 		{
 		ClrSDCK();
 		RdPara <<= 1;
 		//Delay_us(SPI3_SPEED);
 		Delay_us(320);
 		SetSDCK();
 		//Delay_us(SPI3_SPEED);
 		Delay_us(320);
 		if(SDIO_H())
 			RdPara |= 0x01;
 		else
 			RdPara |= 0x00;
 		} 
 	ClrSDCK();
 	OutputSDIO();
	OutputSDIO();
 	SetSDIO();
 	SetCSB();			
 	return(RdPara);	
}

/**********************************************************
**Name:	 	vSpi3Write
**Func: 	SPI Write One word
**Input: 	Write word
**Output:	none
**********************************************************/
void vSpi3Write(word dat)
{
 	vSpi3WriteByte((byte)(dat>>8)&0x7F);
 	vSpi3WriteByte((byte)dat);
 	SetCSB();
}

/**********************************************************
**Name:	 	bSpi3Read
**Func: 	SPI-3 Read One byte
**Input: 	readout addresss
**Output:	readout byte
**********************************************************/
byte bSpi3Read(byte addr)
{
  	vSpi3WriteByte(addr|0x80);
 	return(bSpi3ReadByte());
}

/**********************************************************
**Name:	 	vSpi3WriteFIFO
**Func: 	SPI-3 send one byte to FIFO
**Input: 	one byte buffer
**Output:	none
**********************************************************/
void vSpi3WriteFIFO(byte dat)
{
 	byte bitcnt;	
 
 	SetCSB();	
	OutputSDIO();	
	ClrSDCK();
 	ClrFCSB();			//FCSB = 0
	for(bitcnt=8; bitcnt!=0; bitcnt--)
 		{
 		ClrSDCK();
 		
 		if(dat&0x80)
			SetSDIO();		
		else
			ClrSDIO();
		//Delay_us(SPI3_SPEED);
		Delay_us(320);
		SetSDCK();
		//Delay_us(SPI3_SPEED);
		Delay_us(320);
 		dat <<= 1;
 		}
 	ClrSDCK();	
 	//Delay_us(SPI3_SPEED);		//Time-Critical
 	Delay_us(320);
 	//Delay_us(SPI3_SPEED);		//Time-Critical
 	Delay_us(320);
 	SetFCSB();
	SetSDIO();
 	//Delay_us(SPI3_SPEED);		//Time-Critical
 	Delay_us(320);
 	//Delay_us(SPI3_SPEED);		//Time-Critical
 	Delay_us(320);
}

/**********************************************************
**Name:	 	bSpi3ReadFIFO
**Func: 	SPI-3 read one byte to FIFO
**Input: 	none
**Output:	one byte buffer
**********************************************************/
byte bSpi3ReadFIFO(void)
{
	byte RdPara;
 	byte bitcnt;	
 	
 	SetCSB();
	InputSDIO();
 	ClrSDCK();
	ClrFCSB();
		
 	for(bitcnt=8; bitcnt!=0; bitcnt--)
 		{
 		ClrSDCK();
 		RdPara <<= 1;
 		//Delay_us(SPI3_SPEED);
 		Delay_us(320);
		SetSDCK();
		//Delay_us(SPI3_SPEED);
		Delay_us(320);
 		if(SDIO_H())
 			RdPara |= 0x01;		//NRZ MSB
 		else
 		 	RdPara |= 0x00;		//NRZ MSB
 		}
 	
 	ClrSDCK();
 	//Delay_us(SPI3_SPEED);		//Time-Critical
 	Delay_us(320);
 	//Delay_us(SPI3_SPEED);		//Time-Critical
 	Delay_us(320);
 	SetFCSB();
	OutputSDIO();
	SetSDIO();
 	//Delay_us(SPI3_SPEED);		//Time-Critical
 	Delay_us(320);
 	//Delay_us(SPI3_SPEED);		//Time-Critical
 	Delay_us(320);
 	return(RdPara);
}

/**********************************************************
**Name:	 	vSpi3BurstWriteFIFO
**Func: 	burst wirte N byte to FIFO
**Input: 	array length & head pointer
**Output:	none
**********************************************************/
void vSpi3BurstWriteFIFO(byte ptr[], byte length)
{
 	byte i;
 	if(length!=0x00)
	 	{
 		for(i=0;i<length;i++)
 			vSpi3WriteFIFO(ptr[i]);
 		}
 	return;
}

/**********************************************************
**Name:	 	vSpiBurstRead
**Func: 	burst wirte N byte to FIFO
**Input: 	array length  & head pointer
**Output:	none
**********************************************************/
void vSpi3BurstReadFIFO(byte ptr[], byte length)
{
	byte i;
 	if(length!=0)
 		{
 		for(i=0;i<length;i++)
 			ptr[i] = bSpi3ReadFIFO();
 		}	
 	return;
}
    
/**********************************************************
**Name:	spi_init
**Func: 	spi init 
**Input: none
**Output:	none
**********************************************************/  
void spi_init(void)
{
	
	Spi3.vSpi3Init=vSpi3Init;
	Spi3.vSpi3Write=vSpi3Write;
	Spi3.bSpi3Read=bSpi3Read;
	
	Spi3.vSpi3WriteFIFO=vSpi3WriteFIFO;
	Spi3.bSpi3ReadFIFO=bSpi3ReadFIFO;
	Spi3.vSpi3BurstWriteFIFO=vSpi3BurstWriteFIFO;
	
	Spi3.vSpi3BurstReadFIFO=vSpi3BurstReadFIFO;
	Spi3.vSpi3WriteByte=vSpi3WriteByte;
	Spi3.bSpi3ReadByte=bSpi3ReadByte;

}  

