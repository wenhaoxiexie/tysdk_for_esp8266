#include "CMT2300drive.h"

cmt2300aEasy radio;
/**********************************************
note : 参照文件5-pre-2219b.exp 
**********************************************/
word CMTBank[12] = {
					0x0002,
					0x0166,
					0x02EC,
					0x031C,
					0x0430,
					0x0580,
					0x0614,
					0x0708,
					0x0811,
					0x0902,
					0x0A02,
					0x0B00
				   };

word SystemBank[12] = {
						0x0CAE,
						0x0DE0,
						0x0E70,
						0x0F00,
						0x1000,
						0x11F4,
						0x1210,
						0x13E2,
						0x1442,
						0x1520,
						0x1600,
						0x1781
					  };
	
word FrequencyBank[8] = {
						0x1846,
						0x196D,
						0x1A80,
						0x1B86,
						0x1C46,
						0x1D62,
						0x1E27,
						0x1F16
						};

word DataRateBank[24] = {
						0x2043,
						0x2193,
						0x22B1,
						0x2333,
						0x2400,
						0x2500,
						0x2600,
						0x2700,
						0x2800,
						0x2900,
						0x2A00,
						0x2B29,
						0x2CC0,
						0x2D8A,
						0x2E02,
						0x2F6B,
						0x3017,
						0x3100,
						0x3250,
						0x3315,
						0x3400,
						0x3501,
						0x3605,
						0x3705
						};

word BasebandBank[29] = {
						0x382A,
						0x3908,
						0x3A00,
						0x3BAA,
						0x3C02,
						0x3D00,
						0x3E00,
						0x3F00,
						0x4000,
						0x4100,
						0x4200,
						0x43D4,
						0x442D,
						0x4500,
						0x460B,
						0x4700,
						0x4800,
						0x4900,
						0x4A00,
						0x4B00,
						0x4C00,
						0x4D00,
						0x4E00,
						0x4F60,
						0x50FF,
						0x5101,
						0x5200,
						0x531F,
						0x5410
						};

word TXBank[11] = {
					0x5541,
					0x564D,
					0x5706,
					0x5800,
					0x5900,
					0x5A30,
					0x5B00,
					0x5C03,
					0x5D01,
					0x5E3F,
					0x5F7F
				  };

/**********************************************************
**Name:     bGoStandby
**Function: Entry Standby Mode
**Input:    none
**Output:   none
**********************************************************/
bool bGoStandby(void)
{
 byte tmp, i;	
 
 radio.RssiTrig = false;
 Spi3.vSpi3Write(((word)CMT23_MODE_CTL<<8)+MODE_GO_STBY);	
 for(i=0; i<50; i++)
 	{
 	//Delay_us(400);
 	Delay_us(6400);
	tmp = (MODE_MASK_STA & Spi3.bSpi3Read(CMT23_MODE_STA));	
	
	if(tmp==MODE_STA_STBY)
		break;
	}
 if(i>=50)
 	return(false);
 else
 	return(true);
}

bool bGoTx(void)
{
byte tmp, i;

INIT_TX:
 Spi3.vSpi3Write(((word)CMT23_MODE_CTL<<8)+MODE_GO_TX);		
 for(i=0; i<50; i++)
 	{
 	//Delay_us(200);
 	Delay_us(3200);	
	tmp = (MODE_MASK_STA & Spi3.bSpi3Read(CMT23_MODE_STA));	
	if(tmp==MODE_STA_TX)
		break;
	}
 if(i>=50)
 	{
 	
		bGoStandby();
 		goto INIT_TX;
 	}
 else
 	return(true);
}

/**********************************************************
**Name:     bGoRx
**Function: Entry Rx Mode
**Input:    none
**Output:   none
**********************************************************/

byte vReadIngFlag1(void)
{
	return(Spi3.bSpi3Read((byte)(CMT23_INT_FLG>>8)));

}

byte vReadIngFlag2(void)
{
	return(Spi3.bSpi3Read((byte)(CMT23_INT_CLR1	>>8)));

}


bool bGoRx(void)
{
 byte tmp, i;
 
 radio.RssiTrig = false;

 INIT_RX:
 Spi3.vSpi3Write(((word)CMT23_MODE_CTL<<8)+MODE_GO_RX);		
 for(i=0; i<50; i++)
 	{
 	//Delay_us(200);
 	Delay_us(3200);	
	tmp = (MODE_MASK_STA & Spi3.bSpi3Read(CMT23_MODE_STA));	
	
	if(tmp==MODE_STA_RX)
		break;
	}
 if(i>=50)
 	{
		bGoStandby();
 		goto INIT_RX;
 	}
 else
 	return(true);
}

/**********************************************************
**Name:     bGoSleep
**Function: Entry Sleep Mode
**Input:    none
**Output:   none
**********************************************************/
bool bGoSleep(void)
{
 byte tmp;
 
 Spi3.vSpi3Write(((word)CMT23_MODE_CTL<<8)+MODE_GO_SLEEP);	
 Delay_ms(100);		//enough?
 tmp = (MODE_MASK_STA & Spi3.bSpi3Read(CMT23_MODE_STA));
 if(tmp==MODE_STA_SLEEP)
 	return(true);
 else
 	return(false);
}

/**********************************************************
**Name:     vSoftReset
**Function: Software reset Chipset
**Input:    none
**Output:   none
**********************************************************/
void vSoftReset(void)
{
 Spi3.vSpi3Write(((word)CMT23_SOFTRST<<8)+0xFF); 
 //Delay_us(1000);				//enough?
 Delay_us(16000);	
}

/**********************************************************
**Name:     bReadStatus
**Function: read chipset status
**Input:    none
**Output:   none
**********************************************************/
byte bReadStatus(void)
{
 return(MODE_MASK_STA & Spi3.bSpi3Read(CMT23_MODE_STA));		
}

/**********************************************************
**Name:     bReadRssi
**Function: Read Rssi
**Input:    true------dBm;
            false-----Code;
**Output:   none
**********************************************************/
byte bReadRssi(bool unit_dbm)
{
 if(unit_dbm)
 	return(Spi3.bSpi3Read(CMT23_RSSI_DBM));
 else		
 	return(Spi3.bSpi3Read(CMT23_RSSI_CODE));
}

/**********************************************************
GPIO & Interrupt CFG
**********************************************************/
/**********************************************************
**Name:     vGpioFuncCfg
**Function: GPIO Function config
**Input:    none
**Output:   none
**********************************************************/
void vGpioFuncCfg(byte io_cfg)
{
 Spi3.vSpi3Write(((word)CMT23_IO_SEL<<8)+io_cfg);
}

/**********************************************************
**Name:     vIntSrcCfg
**Function: config interrupt source  
**Input:    int_1, int_2
**Output:   none
**********************************************************/
void vIntSrcCfg(byte int_1, byte int_2)
{
 byte tmp;
 tmp = INT_MASK & Spi3.bSpi3Read(CMT23_INT1_CTL);
 Spi3.vSpi3Write(((word)CMT23_INT1_CTL<<8)+(tmp|int_1));
 
 tmp = INT_MASK & Spi3.bSpi3Read(CMT23_INT2_CTL);
 Spi3.vSpi3Write(((word)CMT23_INT2_CTL<<8)+(tmp|int_2));
}

/**********************************************************
**Name:     vEnableAntSwitch
**Function:  
**Input:    
**Output:   none
**********************************************************/
void vEnableAntSwitch(byte mode)
{
 byte tmp;
 tmp = Spi3.bSpi3Read(CMT23_INT1_CTL);
 tmp&= 0x3F;
 switch(mode)
 	{
 	case 1:
 		tmp |= RF_SWT1_EN; break;		//GPO1=RxActive; GPO2=TxActive	
 	case 2:
 		tmp |= RF_SWT2_EN; break;		//GPO1=RxActive; GPO2=!RxActive
 	case 0:
 	default:
 		break;							//Disable	
 	}
 Spi3.vSpi3Write(((word)CMT23_INT1_CTL<<8)+tmp);
}


/**********************************************************
**Name:     vIntSrcEnable
**Function: enable interrupt source 
**Input:    en_int
**Output:   none
**********************************************************/
void vEnablePLLcheck(void)
{
  byte tmp;
  tmp = Spi3.bSpi3Read(CMT23_EN_CTL);
  tmp |= LD_STOP_EN; 	
  Spi3.vSpi3Write(((word)CMT23_EN_CTL<<8)+tmp);
}

/**********************************************************
**Name:     vIntSrcEnable
**Function: enable interrupt source 
**Input:    en_int
**Output:   none
**********************************************************/
void vJumpFRQ(byte channel)
{
  Spi3.vSpi3Write(((word)CMT23_FREQ_OFS<<8)+0xC8); //分辨率500K
  Spi3.vSpi3Write(((word)CMT23_FREQ_CHNL<<8)+channel);
}

/**********************************************************
**Name:     vEnablePLLcheck
**Function: enable PLLcheck 
**Input:    en_int
**Output:   none
**********************************************************/
void vIntSrcEnable(byte en_int)
{
 Spi3.vSpi3Write(((word)CMT23_INT_EN<<8)+en_int);				
}


/**********************************************************
**Name:     vIntSrcFlagClr
**Function: clear flag
**Input:    none
**Output:   equ CMT23_INT_EN
**********************************************************/
byte bIntSrcFlagClr(void)
{
 byte tmp;
 byte int_clr2 = 0;
 byte int_clr1 = 0;
 byte flg = 0;
 
 tmp = Spi3.bSpi3Read(CMT23_INT_FLG);
 if(tmp&LBD_STATUS_FLAG)		//LBD_FLG_Active
 	int_clr2 |= LBD_CLR;
 
 if(tmp&PREAMBLE_PASS_FLAG)		//Preamble Active
 	{
 	int_clr2 |= PREAMBLE_PASS_CLR;
 	flg |= PREAMBLE_PASS_EN;
	}
 if(tmp&SYNC_PASS_FLAG)			//Sync Active
 	{
 	int_clr2 |= SYNC_PASS_CLR;		
 	flg |= SYNC_PASS_EN;		
 	}
 if(tmp&NODE_PASS_FLAG)			//Node Addr Active
 	{
 	int_clr2 |= NODE_PASS_CLR;	
 	flg |= NODE_PASS_EN;
 	}
 if(tmp&CRC_PASS_FLAG)			//Crc Pass Active
 	{
 	int_clr2 |= CRC_PASS_CLR;
 	flg |= CRC_PASS_EN;
 	}
 if(tmp&RX_DONE_FLAG)			//Rx Done Active
 	{
 	int_clr2 |= RX_DONE_CLR;
 	flg |= PKT_DONE_EN;
 	}
 	
 if(tmp&COLLISION_ERR_FLAG)		//这两个都必须通过RX_DONE清除
 	int_clr2 |= RX_DONE_CLR;
 if(tmp&DC_ERR_FLAG)
 	int_clr2 |= RX_DONE_CLR;
 	
 Spi3.vSpi3Write(((word)CMT23_INT_CLR2<<8)+int_clr2);	//Clear flag
 
 
 tmp = Spi3.bSpi3Read(CMT23_INT_CLR1);
 if(tmp&TX_DONE_FLAG)
 	{
 	int_clr1 |= TX_DONE_CLR;
 	flg |= TX_DONE_EN;
 	}	
 if(tmp&SLEEP_TIMEOUT_FLAG)
 	{
 	int_clr1 |= SLEEP_TIMEOUT_CLR;
 	flg |= SLEEP_TMO_EN;
 	} 
 if(tmp&RX_TIMEOUT_FLAG)
 	{
 	int_clr1 |= RX_TIMEOUT_CLR;
 	flg |= RX_TMO_EN;
 	}	
 Spi3.vSpi3Write(((word)CMT23_INT_CLR1<<8)+int_clr1);	//Clear flag 
 
 return(flg);
}

/**********************************************************
**Name:     bClearFIFO
**Function: clear FIFO buffer
**Input:    none
**Output:   FIFO state
**********************************************************/
byte vClearFIFO(void)
{
 byte tmp;	
 tmp = Spi3.bSpi3Read(CMT23_FIFO_FLG);
 Spi3.vSpi3Write(((word)CMT23_FIFO_CLR<<8)+FIFO_CLR_RX+FIFO_CLR_TX);
 return(tmp);
}

void vEnableWrFifo(void)
{
 byte tmp;
 tmp = Spi3.bSpi3Read(CMT23_FIFO_CTL);
 tmp |= (SPI_FIFO_RD_WR_SEL|FIFO_RX_TX_SEL);
 Spi3.vSpi3Write(((word)CMT23_FIFO_CTL<<8)+tmp);
}

void vEnableRdFifo(void)
{
 byte tmp;
 tmp = Spi3.bSpi3Read(CMT23_FIFO_CTL);
 tmp &= (~(SPI_FIFO_RD_WR_SEL|FIFO_RX_TX_SEL));
 Spi3.vSpi3Write(((word)CMT23_FIFO_CTL<<8)+tmp);
}

/**********************************************************
CFG
**********************************************************/
/**********************************************************
**Name:     vInit
**Function: Init. CMT2300A
**Input:    none
**Output:   none
**********************************************************/
void vInit(void)
{
 //byte i;
 byte tmp;
 bool tmp1;
 //word len;
 Spi3.vSpi3Init();
 //GPO3In();

 vSoftReset();
 Delay_ms(20);
tmp1 = bGoStandby();

if(tmp1 == false)
{
	while(1);
}

 tmp = Spi3.bSpi3Read(CMT23_MODE_STA);
 tmp|= EEP_CPY_DIS;
 tmp&= (~RSTN_IN_EN);			//Disable RstPin	
 Spi3.vSpi3Write(((word)CMT23_MODE_STA<<8)+tmp);

 bIntSrcFlagClr();
 
}

void vCfgBank(word cfg[], byte length)
{
 byte i;
 
 if(length!=0)
 	{	
 	for(i=0; i<length; i++)	
 		Spi3.vSpi3Write(cfg[i]);
 	}	
}


/******************************************************************************
**函数名称：bGetMessage
**函数功能：接收一包数据
**输入参数：无
**输出参数：非0——接收成功
**          0——接收失败
******************************************************************************/
byte bGetMessage(byte msg[])
{
 byte i;	
 
 vEnableRdFifo();	
 if(radio.FixedPktLength)
 	{
  	Spi3.vSpi3BurstReadFIFO(msg, radio.PayloadLength);
	i = radio.PayloadLength;
	}
 else
 	{
	i = Spi3.bSpi3ReadFIFO();	
 	Spi3.vSpi3BurstReadFIFO(msg, i);
 	}
 return(i);
}

byte bGetMessageByFlag(byte msg[])
{
 byte tmp;
 byte tmp1;
 byte rev = 0;
 tmp = Spi3.bSpi3Read(CMT23_INT_FLG);
 if((tmp&SYNC_PASS_FLAG)&&(!radio.RssiTrig))
 	{
 	radio.PktRssi = bReadRssi(false);
 	radio.RssiTrig = true;
 	}
 
 tmp1 = Spi3.bSpi3Read(CMT23_CRC_CTL);
 vEnableRdFifo();	 
 if(tmp1&CRC_ENABLE)		//Enable CrcCheck
 	{
 	if(tmp&CRC_PASS_FLAG)
 		{
 		if(radio.FixedPktLength)
 			{
  			Spi3.vSpi3BurstReadFIFO(msg, radio.PayloadLength);
			rev = radio.PayloadLength;
			}
 		else
 			{	
			rev = Spi3.bSpi3ReadFIFO();	
 			Spi3.vSpi3BurstReadFIFO(msg, rev);
 			}
 		radio.RssiTrig = false;
 		}
 	}
 else
 	{
	if(tmp&RX_DONE_FLAG) 		
		{
 		if(radio.FixedPktLength)
 			{
  			Spi3.vSpi3BurstReadFIFO(msg, radio.PayloadLength);
			rev = radio.PayloadLength;
			}
 		else
 			{	
			rev = Spi3.bSpi3ReadFIFO();	
 			Spi3.vSpi3BurstReadFIFO(msg, rev);
 			}	
 		radio.RssiTrig = false;		
		}
 	}
 
 if(tmp&COLLISION_ERR_FLAG)			//错误处理
	rev = 0xFF;
 return(rev);
}

void vSetTxPayloadLength(word length)
{
 byte tmp;	
 byte len;
 bGoStandby();
 tmp = Spi3.bSpi3Read(CMT23_PKT_CTRL1);
 tmp&= 0x8F;
 
 if(length!=0)
 	{
 	if(radio.FixedPktLength)
		len = length-1;
 	else
		len = length;
	}
 else
 	len = 0;
 
 tmp|= (((byte)(len>>8)&0x07)<<4);
 Spi3.vSpi3Write(((word)CMT23_PKT_CTRL1<<8)+tmp);
 Spi3.vSpi3Write(((word)CMT23_PKT_LEN<<8)+(byte)len);	//Payload length

}


/******************************************************************************
**函数名称：bSendMessage
**函数功能：发射一包数据
**输入参数：无
**输出参数：
**          
******************************************************************************/
bool bSendMessage(byte msg[], byte length)
{
 //mode1
 //vSetTxPayloadLength(length);
 //bGoStandby();
 //vEnableWrFifo();	
 //Spi3.vSpi3BurstWriteFIFO(msg, length);
 //bGoTx();
 
 //mode2
 bIntSrcFlagClr();  //清中断
 vSetTxPayloadLength(length);
 bGoTx();
 vEnableWrFifo();	
 Spi3.vSpi3BurstWriteFIFO(msg, length);
 return(true);
}

void CMT2300_Band(void)
{
	//State Ctrl
	radio.bGoTx=bGoTx;
	radio.bGoRx=bGoRx;
	radio.bGoSleep=bGoSleep;
	radio.bGoStandby=bGoStandby;
	radio.vSoftReset=vSoftReset;
	radio.bReadStatus=bReadStatus;
	radio.bReadRssi=bReadRssi;

	//GPIO & Interrupt CFG
	radio.vGpioFuncCfg=vGpioFuncCfg;
	radio.vIntSrcCfg=vIntSrcCfg;
	radio.vEnableAntSwitch=vEnableAntSwitch;
	radio.vIntSrcEnable=vIntSrcEnable;
	radio.vEnablePLLcheck=vEnablePLLcheck;
	radio.vJumpFRQ=vJumpFRQ;
	radio.bIntSrcFlagClr=bIntSrcFlagClr;
	radio.vClearFIFO=vClearFIFO;
	radio.vEnableRdFifo=vEnableRdFifo;
	radio.vEnableWrFifo=vEnableWrFifo;
	radio.vSetTxPayloadLength=vSetTxPayloadLength;
	radio.vReadIngFlag1=vReadIngFlag1;
	radio.vReadIngFlag2=vReadIngFlag2;
 
	//CFG
	radio.vInit=vInit;
	radio.vCfgBank=vCfgBank;
	
	radio.bGetMessage=bGetMessage;
	radio.bGetMessageByFlag=bGetMessageByFlag;
	radio.bSendMessage=bSendMessage;
}

/**
  * @brief Device Initialize configuration
  * @param None
  * @retval None
  */

void CMT2300_Init()
{
	spi_init();
	CMT2300_Band();
	/**********基础设置初始化一次即可*******/
	radio.FixedPktLength    = true;				
	radio.PayloadLength     = LEN;	
	radio.vInit();
	radio.vCfgBank(CMTBank, 12);
	radio.vCfgBank(SystemBank, 12);
	radio.vCfgBank(FrequencyBank, 8);
	radio.vCfgBank(DataRateBank, 24);
	radio.vCfgBank(BasebandBank, 29);
	radio.vCfgBank(TXBank, 11);
	radio.vEnablePLLcheck();
	radio.vJumpFRQ(1);
	radio.bGoSleep();  				//让配置生效
	
	/**************************************/
}

void setup_Rx(void)
{

	radio.bGoStandby();   //进入配置模式
	radio.vEnableAntSwitch(0); //为 1 时 GPIO1 和 GPIO2 不可用
	radio.vGpioFuncCfg(GPIO1_INT1+GPIO2_Dout+GPIO3_INT2);  //IO口的功能映射

	//radio.vIntSrcCfg(INT_RSSI_VALID, INT_CRC_PASS);   //GPO3映射成CRC_pass中断，此处如果要用该中断，RFPDK需要配置CRC
	radio.vIntSrcCfg(INT_PKT_DONE, INT_PKT_DONE);  //GPO3映射成PKT_DONE中断 //IO口中断的映射
	radio.vIntSrcEnable(PKT_DONE_EN);          //中断使能 
	
	radio.vClearFIFO();
	radio.bGoSleep();           //让配置生效
	radio.bGoRx();              //for Rx

}


