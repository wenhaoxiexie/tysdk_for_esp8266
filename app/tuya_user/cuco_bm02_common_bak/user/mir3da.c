#if 1
#include "c_types.h"
#include "esp_libc.h"
#include "system/uni_log.h"
#include "com_def.h"
#include "c_types.h"
#include "mir3da.h"

bool com_iic_readData(UCHAR chip_addr, UCHAR reg_addr, UCHAR* buf, UCHAR len);
bool com_iic_writeData(UCHAR chip_addr, UCHAR reg_addr, UCHAR* buf, UCHAR len);


//unsigned char mir3da_i2c_addr = 0x27;//SA0 PIN LOW:0x26 ;SA0 PIN HIGH OR FLOATTING:0x27
unsigned char mir3da_i2c_addr = 0x4F;//SA0 PIN LOW:0x26 ;SA0 PIN HIGH OR FLOATTING:0x27

#define YZ_CROSS_TALK_ENABLE   0 

#if YZ_CROSS_TALK_ENABLE 
static short yzcross=0;
#endif    
/*******************************************************************************
* Function Name: mir3da_register_read
* Description  : Read registers.
* Arguments    : addr -> register start address
                 data -> the point of register data
                 datalen -> the lenth of data to read
* Return Value : 0 -> OK; -1 -> Error
********************************************************************************/
signed char mir3da_register_read( unsigned char addr, unsigned char *data,unsigned char datalen)
{
    //TODO 

    if (com_iic_readData(mir3da_i2c_addr, addr, data, datalen) == FALSE) 
	 return -1;
	
    return 0;
}

/*******************************************************************************
* Function Name: mir3da_register_write
* Description  : Write a register.
* Arguments    : addr -> register address
                 data -> the data to write
* Return Value : 0 -> OK; -1 -> Error
********************************************************************************/
signed char mir3da_register_write( unsigned char addr, unsigned char data)
{
    //TODO 

    if (com_iic_writeData(mir3da_i2c_addr, addr, &data, 1) == FALSE)  
   	 return -1;
	
    return 0;
}

/*******************************************************************************
* Function Name: mir3da_register_mask_write
* Description  : Write a register with mask.
* Arguments    : addr -> register address
                 mask -> register mask
                 data -> register data to write
* Return Value : 0 -> OK; -1 -> Error
********************************************************************************/
signed char mir3da_register_mask_write(unsigned char addr, unsigned char mask, unsigned char data)
{
    signed char        res = -1;
    unsigned char      tmp_data=0;

    res = mir3da_register_read(addr, &tmp_data,1);
    if(res) 
    {
        return res;
    }

    tmp_data &= ~mask; 
    tmp_data |= data & mask;
    res = mir3da_register_write(addr, tmp_data);

    return res;
}

/*******************************************************************************
* Function Name: Delay_ms
* Description  : delay ms
* Arguments    : ms
* Return Value : void
********************************************************************************/
void Delay_ms(unsigned int ms)
{
	  SystemSleep(ms);	
}

/*******************************************************************************
* Function Name: mir3da_init
* Description  : This function initialization mir3da chip.
* Arguments    : None
* Return Value : 0 -> OK; -1 -> Error
********************************************************************************/
signed char mir3da_init(void)
{
    signed char   res = -1;
    unsigned char data=0;
    unsigned char i=0;

    mir3da_register_read(NSA_REG_WHO_AM_I,&data,1);
    
    if(0x13 != data)
    {
        for(i=0;i<5;i++)
		{
                 mir3da_register_read(NSA_REG_WHO_AM_I,&data,1);
		  if(0x13 == data)
		  	break;
		  
		  Delay_ms(20);
        }
		if(i==5)
		{
		  //TRACE("mir3da_init Can't read chipid=0x%x \r\n",data);
		  PR_DEBUG("mir3da_init Can't read chipid=0x%x \r\n",data);
          	   return -1;
		}
    }

	
	//TRACE("mir3da_init chipid=0x%x \r\n",data);
	PR_DEBUG("mir3da_init chipid=0x%x \r\n",data);

    res =  mir3da_register_mask_write(NSA_REG_SPI_I2C, 0x24, 0x24);

    Delay_ms(50);

    //res |= mir3da_register_write(NSA_REG_G_RANGE,0x41);//Enable I2C watch dog & 14 bit adc & 4g range
    res |= mir3da_register_write(NSA_REG_G_RANGE,0x01);//Enable I2C watch dog & 14 bit adc & 4g range
    
    //res |= mir3da_register_write(NSA_REG_POWERMODE_BW,0x04);//OSR 1X & 1/10 ODR
    res |= mir3da_register_write(NSA_REG_POWERMODE_BW,0x14);//OSR 1X & 1/10 ODR
    
   //// res |= mir3da_register_write(NSA_REG_ODR_AXIS_DISABLE,0x07);//ENABLE XYZ & SET ODR 125Hz
   res |= mir3da_register_write(NSA_REG_ODR_AXIS_DISABLE,0xc7);//ENABLE XYZ & SET ODR 125Hz
		
   // res |= mir3da_register_write(NSA_REG_ENGINEERING_MODE,0x83);
   // res |= mir3da_register_write(NSA_REG_ENGINEERING_MODE,0x69);
   // res |= mir3da_register_write(NSA_REG_ENGINEERING_MODE,0xBD);
	
#if YZ_CROSS_TALK_ENABLE 
    res = mir3da_register_read(NSA_REG_CHIP_INFO_SECOND, &data, 1);
    if(res != 0)
    {
        return res;
    }
	           
	if(data&0x10)
		yzcross = -(data&0x0f);
	else
		yzcross = (data&0x0f);
#endif	

    if(mir3da_i2c_addr == 0x26)
        res |= mir3da_register_mask_write(0x8c, 0x40, 0x00);	
	
    return res;	    	
}

/*******************************************************************************
* Function Name: mir3da_set_enable
* Description  : This function enable or disable mir3da chip.
* Arguments    : enable -> 1:enable  0:disable 
* Return Value : 0 -> OK; -1 -> Error
********************************************************************************/
signed char mir3da_set_enable(unsigned char enable)
{
    signed char res = -1;
		
    if(enable)
        res = mir3da_register_write(NSA_REG_POWERMODE_BW,0x04);   //Normal mode
    else	
        res = mir3da_register_write(NSA_REG_POWERMODE_BW,0x84);   //Suspend mode

	//TRACE("mir3da_set_enable %d\r\n",enable);
	PR_DEBUG("mir3da_set_enable %d\r\n",enable);
	
	return res;	
}

/*******************************************************************************
* Function Name: mir3da_open_interrupt
* Description  : This function enable double-tap interrupt.
* Arguments    : None
* Return Value : 0 -> OK; -1 -> Error
********************************************************************************/
signed char mir3da_open_interrupt()
{
    signed char   res = -1;

    res = mir3da_register_write(NSA_REG_INT_LATCH,0xee);//Latch 50 ms  

    res |= mir3da_register_write(NSA_REG_TAP_DURATION,0x45 );//Quiet 30ms & Shock 70ms &Dur 700ms  

    res |= mir3da_register_write(NSA_REG_TAP_THRESHOLD,0x04 );//125mg * 0x0D   
    
    res |= mir3da_register_write(NSA_REG_INTERRUPT_MAPPING1,0x10 );//Mappint the double-tap interrupt to int1

    res |= mir3da_register_write(NSA_REG_INTERRUPT_SETTINGS1,0x50);//Enable double-tap interrupt

	//TRACE("mir3da_open_interrupt \r\n");
	PR_DEBUG("mir3da_open_interrupt \r\n");
	
    return res;
}

/*******************************************************************************
* Function Name: mir3da_close_interrupt
* Description  : This function disable double-tap interrupt.
* Arguments    : None
* Return Value : 0 -> OK; -1 -> Error
********************************************************************************/
signed char mir3da_close_interrupt()
{
    signed char   res = 0;

    res = mir3da_register_write(NSA_REG_INTERRUPT_MAPPING1,0x00 );
	
    res |= mir3da_register_write(NSA_REG_INTERRUPT_SETTINGS1,0x00 );

	//TRACE("mir3da_close_interrupt \r\n");
	PR_DEBUG("mir3da_close_interrupt \r\n");
	

	return res;
}

/*******************************************************************************
* Function Name: mir3da_read_data
* Description  : Read the xyz axis data
* Arguments    : x/y/z
* Return Value : 0 -> OK; -1 -> Error
********************************************************************************/
signed char mir3da_read_data(short *x, short *y, short *z)
{
    signed char      res = -1;
    unsigned char    tmp_data[6] = {0};

#if 1
    res = mir3da_register_read(NSA_REG_ACC_X_LSB,tmp_data,6);
#else
    res = mir3da_register_read(NSA_REG_ACC_Z_LSB,tmp_data,2);
#endif

    if (res)
	return res;

#if 1
    *x = ((short)(tmp_data[1] << 8 | tmp_data[0]))>> 3;
    *y = ((short)(tmp_data[3] << 8 | tmp_data[2]))>> 3;
    *z = ((short)(tmp_data[5] << 8 | tmp_data[4]))>> 3;
#else
    *z = ((short)(tmp_data[1] << 8 | tmp_data[0]))>> 3;

#endif

#if 0
    #if YZ_CROSS_TALK_ENABLE
    if(yzcross)
          *y=*y-(*z)*yzcross/100;
    #endif	
#endif	
	
	//TRACE("xyz %d %d %d \r\n",*x,*y,*z);
	//PR_DEBUG("xyz %d %d %d \r\n",*x,*y,*z);	
    	//PR_DEBUG("z: %d   tmp_data_1: %02X   tmp_data_0: %02X \r\n",*z, tmp_data[1], tmp_data[0]);
	//if (*z < 500 && *z > -500)
    	//	PR_DEBUG("------------------------------------------z error!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! \r\n");
		
	
    return res;
}

/*******************************************************************************
* Function Name: mir3da_get_chip_id
* Description  : Get the chipid
* Arguments    : None
* Return Value : Chipid
********************************************************************************/
unsigned char mir3da_get_chip_id(void)
{
    unsigned char data=0;

    if(mir3da_register_read(NSA_REG_WHO_AM_I,&data,1))
        return 0;
    
    return data;	    	
}


#endif
