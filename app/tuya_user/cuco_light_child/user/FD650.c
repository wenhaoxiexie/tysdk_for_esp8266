/** 
* @file 		  FD650.C   			��д��ʵ�ֺ�����оƬ������������
* @brief        ��д��ʵ�ֺ�����оƬ������������
* @author        FD.chd 
* @author        Jun
* @version        A3 
* @date 			2010/07/25    ����˵�� :  �����ļ� by FD.chd 
* @date 			2012/07/25    ����˵�� :  ���ע�ͣ�doxygen��by jun
* @copyright Copyright (c) 2012 by FUZHOU FUDA HISI MICROELECTRONICS CO.,LTD.              
*/
#include "FD650.h"	
#include "espressif/c_types.h"
#include "esp8266/pin_mux_register.h"
#include "gpio.h"
#include "device.h"


#define		FD650_SCL_SET	GPIO_OUTPUT(GPIO_Pin_14,1) //<��SCL����Ϊ�ߵ�ƽ
#define		FD650_SCL_CLR	GPIO_OUTPUT(GPIO_Pin_14,0) //<��SCL����Ϊ�͵�ƽ
#define		FD650_SCL_D_OUT	   {}	 ///<����SCLΪ�������,����˫��I/O���л�Ϊ���

#define		FD650_SDA_SET   GPIO_OUTPUT(GPIO_Pin_2,1) //<��SDA����Ϊ�ߵ�ƽ
#define		FD650_SDA_CLR	GPIO_OUTPUT(GPIO_Pin_2,0) //<��SDA����Ϊ�͵�ƽ

#define		FD650_SDA_IN    GPIO_INPUT_GET(GPIO_ID_PIN(2))  //<��SDA��Ϊ���뷽��ʱ����ȡ�ĵ�ƽֵ
#define		FD650_SDA_D_OUT	{} //GPIO_AS_OUTPUT(GPIO_Pin_2) 	 ///<����SDAΪ�������,����˫��I/O���л�Ϊ���,���鲻Ҫ���ó����죬����ACK���Ľ���״���γ��֡�
#define		FD650_SDA_D_IN	{} //GPIO_AS_INPUT(GPIO_Pin_2)	 ///<����SDAΪ���뷽��,����˫��I/O���л�Ϊ����

#define I2C_MASTER_SDA_MUX PERIPHS_IO_MUX_GPIO2_U
#define I2C_MASTER_SCL_MUX PERIPHS_IO_MUX_MTMS_U
#define I2C_MASTER_SDA_GPIO 2
#define I2C_MASTER_SCL_GPIO 14
#define I2C_MASTER_SDA_FUNC FUNC_GPIO2
#define I2C_MASTER_SCL_FUNC FUNC_GPIO14


/** 
 * @brief   ����FD650
 * @param   ��
 * @return  ��
 * @note   ��SCL�ߵ�ƽ�ڼ䲶��SDA���½��أ�ʹFD650��ʼ����     
 */ 
void FD650_Start( void )
{
	FD650_SDA_SET;  
	FD650_SDA_D_OUT;
	FD650_SCL_SET;
	FD650_SCL_D_OUT;
	os_delay_us(6);
	FD650_SDA_CLR;
	os_delay_us(6);      
	FD650_SCL_CLR;
}

/** 
 * @brief   ֹͣFD650
 * @param   ��
 * @return  ��
 * @note   ��SCL�ߵ�ƽ�ڼ䲶��SDA�������أ�ʹFD650ֹͣ����     
 */ 
void FD650_Stop( void )
{
	FD650_SDA_CLR;
	FD650_SDA_D_OUT;
	os_delay_us(6);
	FD650_SCL_SET;
	os_delay_us(6);
	FD650_SDA_SET;
	os_delay_us(6);
	FD650_SDA_D_IN;
}

/** 
 * @brief   ����һ���ֽڣ�8λ�����ݸ�FD650
 * @param   dat �޷���8λ����
 * @return  ��
 * @note   ��SCL������д��FD650 ��������9���ֽڶ���Ӧ���ź�	    
 */ 
void FD650_WrByte( u_int8 dat )
{
	u_int8 i;
	FD650_SDA_D_OUT;
	for( i = 0; i != 8; i++ )
	{
		if( dat & 0x80 ) 
		{
		    FD650_SDA_SET;
		}
		else 
		{
		    FD650_SDA_CLR;
		}
		os_delay_us(6);
		FD650_SCL_SET;
		dat <<= 1;
		os_delay_us(6);  // ��ѡ��ʱ
		FD650_SCL_CLR;
	}
	FD650_SDA_SET;
	FD650_SDA_D_IN;
	os_delay_us(6);
	FD650_SCL_SET;
	os_delay_us(6);
	FD650_SCL_CLR;
}

/** 
 * @brief   ��ȡFD650�İ�����ֵ
 * @param   ��
 * @return  �����޷���8λ����
 * @note   ��SCL�½��ض�ȡFD650 ��������9���ֽڷ�����ЧӦ��    
 */ 
u_int8  FD650_RdByte( void )
{
	u_int8 dat,i;
	FD650_SDA_SET;
	FD650_SDA_D_IN;
	dat = 0;
	for( i = 0; i != 8; i++ )
	{
		os_delay_us(6);  // ��ѡ��ʱ
		FD650_SCL_SET;
		os_delay_us(6);  // ��ѡ��ʱ
		dat <<= 1;
		if( FD650_SDA_IN )
			dat++;
		FD650_SCL_CLR;
	}
	FD650_SDA_SET;
	os_delay_us(6);
	FD650_SCL_SET;
	os_delay_us(6);
	FD650_SCL_CLR;
	printf(">>>>:dat:%d \n",dat);
	return dat;
}

void FD650_init(void)
{
	PIN_FUNC_SELECT(I2C_MASTER_SDA_MUX, I2C_MASTER_SDA_FUNC);
    PIN_FUNC_SELECT(I2C_MASTER_SCL_MUX, I2C_MASTER_SCL_FUNC);

    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U, FUNC_GPIO13);

	GPIO_REG_WRITE(GPIO_PIN_ADDR(GPIO_ID_PIN(I2C_MASTER_SDA_GPIO)), GPIO_REG_READ(GPIO_PIN_ADDR(GPIO_ID_PIN(I2C_MASTER_SDA_GPIO))) | GPIO_PIN_PAD_DRIVER_SET(GPIO_PAD_DRIVER_ENABLE)); //open drain;
    GPIO_REG_WRITE(GPIO_ENABLE_ADDRESS, GPIO_REG_READ(GPIO_ENABLE_ADDRESS) | (1 << I2C_MASTER_SDA_GPIO));

	GPIO_REG_WRITE(GPIO_PIN_ADDR(GPIO_ID_PIN(I2C_MASTER_SCL_GPIO)), GPIO_REG_READ(GPIO_PIN_ADDR(GPIO_ID_PIN(I2C_MASTER_SCL_GPIO))) | GPIO_PIN_PAD_DRIVER_SET(GPIO_PAD_DRIVER_ENABLE)); //open drain;
    GPIO_REG_WRITE(GPIO_ENABLE_ADDRESS, GPIO_REG_READ(GPIO_ENABLE_ADDRESS) | (1 << I2C_MASTER_SCL_GPIO));
	
}

/** 
 * @defgroup FD650_Driver FD650 ����ģ�� 
 * FD650 ����ģ���ṩ��ͳһ�Ľӿں����Լ��û��ĺ궨�����������FD650����
 */ 

/** 
 * @ingroup FD650_Driver
 * @brief   ����FD650��������� cmd
 * @param cmd FD650��������������޷���16λ2�ֽ��� �ɺ궨�����ȡֵ
 * @return  ��
 * @note ����һ��16λ2�ֽڵ���������������FD650�������ã�������ʾ����.
	   �������ʾֱ��ʹ��FD650_Write ���� �������Ӧ��ָ��� ��ָ���ʽΪ��  
 �����λѡ��(����) |   �������ʾ��ֵ(����) |   �����С����(��ѡ)  
 * @code 
	FD650_Write(FD650_SYSON_8);// ������ʾ�ͼ��̣�8����ʾ��ʽ
	//����ʾ����
	FD650_Write( FD650_DIG0 | (u_int8)data[0] );    //������һ�������
	if(sec_flag)
		FD650_Write( FD650_DIG1 | (u_int8)data[1] | FD650_DOT ); //�����ڶ��������,����ʾС����
	else
		FD650_Write( FD650_DIG1 | (u_int8)data[1] ); 
	if(Lock)
		FD650_Write( FD650_DIG2 | (u_int8)data[2] | FD650_DOT ); //��������������ܣ�����ʾС����
	else
		FD650_Write( FD650_DIG2 | (u_int8)data[2] );
	FD650_Write( FD650_DIG3 | (u_int8)data[3] ); //�������ĸ������
 * @endcode 
 */ 
void FD650_Write( u_int16 cmd )	
{
	FD650_Start(); 
	FD650_WrByte(((u_int8)(cmd>>7)&0x3E)|0x40);
	FD650_WrByte((u_int8)cmd);
	FD650_Stop();

	return;
}

/** 
 * @ingroup FD650_Driver
 * @brief   ��ȡFD650�İ������� 
 * @param   ��
 * @return  keycode ������������״̬�ļ�ֵ ���޷���8λ1�ֽ�����������Ч����ֵ����0
 * @note 	��ȡFD650�İ������� ���м����·��ؼ�ֵ���޼����·���0��������ֵ����±�
 * @code 
 *	������ֵ��				 
 *	��ַ   DIG3 DIG2 DIG1 DIG0 
 *	KI1  	47H 46H 45H 44H 
 *	KI2 	4FH 4EH 4DH 4CH 
 *	KI3  	57H 56H 55H 54H 
 *	KI4 	5FH 5EH 5DH 5CH 
 *	KI5  	67H 66H 65H 64H 
 *	KI6 	6FH 6EH 6DH 6CH 
 *	KI7  	77H 76H 75H 74H 
 *	KI1+KI2 7FH 7EH 7DH 7CH 

 *	Keycode=FD650_Read( );		 // keycode Ϊ��ֵ
 *	Key_Fun ( Keycode) ;		 // ���������� 
 * @endcode 
 */ 
u_int8 FD650_Read( void )		
{
	u_int8 keycode = 0;
	
	FD650_Start(); 

	FD650_WrByte((u_int8)(FD650_GET_KEY>>7)&0x3E|0x01|0x40);

	keycode=FD650_RdByte();
	FD650_Stop();
	if( (keycode&0x00000040) ==0)
		keycode = 0;
	return keycode;
}


