#include "FD650App.h"
#include "string.h"

/** 字符转数码管显示码字的数组*/
const led_bitmap BCD_decode_tab[LEDMAPNUM] = 
{	
	{'0', 0x3F}, {'1', 0x06}, {'2', 0x5B}, {'3', 0x4F},
	{'4', 0x66}, {'5', 0x6D}, {'6', 0x7D}, {'7', 0x07},
	{'8', 0x7F}, {'9', 0x6F}, {'a', 0x77}, {'A', 0x77},
	{'b', 0x7C}, {'B', 0x7C}, {'c', 0x58}, {'C', 0x39},
	{'d', 0x5E}, {'D', 0x5E}, {'e', 0x79}, {'E', 0x79},
	{'f', 0x71}, {'F', 0x71} 
};//BCD码字映射

/** 
 * @brief   转换字符为数码管的显示码
 * @param   cTemp 待转换为显示码的字符
 * @return  显示码值,8位无符号
 * @note    码值见BCD_decode_tab[LEDMAPNUM]，如遇到无法转换的字符返回0  
 */ 
 u_int8 Led_Get_Code(char cTemp)
{
	u_int8 i, bitmap=0x00;

	for(i=0; i<LEDMAPNUM; i++)
	{
		if(BCD_decode_tab[i].character == cTemp)
		{
			bitmap = BCD_decode_tab[i].bitmap;
			break;
		}
	}

	return bitmap;
}

/** 
 * @brief   简单的数码管显示应用示例
 * @param   acFPStr  显示的字符串
 * @param   sec_flag	  开启小数点标志位；
 * @param   Lock		  开启小数点标志位；	
 * @return  无
 * @note    以字符串形式写入,经转换为数码管显示的内容,本示例为4位数码管
   @code 
	   Led_Show_650（"ABCD",1,1）;
   @endcode
 */ 
void Led_Show_650( char *acFPStr, unsigned char sec_flag, unsigned char Lock,int syson)
{
	int i, iLenth;
	int	data[4]={0x00, 0x00, 0x00, 0x00};
	if( strcmp(acFPStr, "") == 0 )
	{
		return;
	}
	iLenth = strlen(acFPStr);
	if(iLenth>4)
		iLenth = 4;
	
	for(i=0; i<iLenth; i++)
	{
		data[i] = Led_Get_Code(acFPStr[i]);	 
	}
	FD650_Write(syson);// 开启显示和键盘，8段显示方式
	//发显示数据
	FD650_Write( FD650_DIG0 | (u_int8)data[0] );	//点亮第一个数码管
	
	if(sec_flag)
		FD650_Write( FD650_DIG1 | (u_int8)data[1] | FD650_DOT ); //点亮第二个数码管
	else
		FD650_Write( FD650_DIG1 | (u_int8)data[1] ); 
	if(Lock)
		FD650_Write( FD650_DIG2 | (u_int8)data[2] | FD650_DOT ); //点亮第三个数码管
	else
		FD650_Write( FD650_DIG2 | (u_int8)data[2] );
	FD650_Write( FD650_DIG3 | (u_int8)data[3] ); //点亮第四个数码管
	
}

u_int8 key_Show_650(void)
{
	u_int8 key_code;
	key_code = FD650_Read();
	printf(">>>>>>key_code: 0X%X\r\n",key_code);
	if((key_code!=0X00)&&(key_code!=0XFF)){
		printf("0X%X\r\n",key_code);	
	}
	return key_code;
}



