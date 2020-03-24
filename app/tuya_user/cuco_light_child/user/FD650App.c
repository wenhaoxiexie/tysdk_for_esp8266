#include "FD650App.h"
#include "string.h"

/** �ַ�ת�������ʾ���ֵ�����*/
const led_bitmap BCD_decode_tab[LEDMAPNUM] = 
{	
	{'0', 0x3F}, {'1', 0x06}, {'2', 0x5B}, {'3', 0x4F},
	{'4', 0x66}, {'5', 0x6D}, {'6', 0x7D}, {'7', 0x07},
	{'8', 0x7F}, {'9', 0x6F}, {'a', 0x77}, {'A', 0x77},
	{'b', 0x7C}, {'B', 0x7C}, {'c', 0x58}, {'C', 0x39},
	{'d', 0x5E}, {'D', 0x5E}, {'e', 0x79}, {'E', 0x79},
	{'f', 0x71}, {'F', 0x71} 
};//BCD����ӳ��

/** 
 * @brief   ת���ַ�Ϊ����ܵ���ʾ��
 * @param   cTemp ��ת��Ϊ��ʾ����ַ�
 * @return  ��ʾ��ֵ,8λ�޷���
 * @note    ��ֵ��BCD_decode_tab[LEDMAPNUM]���������޷�ת�����ַ�����0  
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
 * @brief   �򵥵��������ʾӦ��ʾ��
 * @param   acFPStr  ��ʾ���ַ���
 * @param   sec_flag	  ����С�����־λ��
 * @param   Lock		  ����С�����־λ��	
 * @return  ��
 * @note    ���ַ�����ʽд��,��ת��Ϊ�������ʾ������,��ʾ��Ϊ4λ�����
   @code 
	   Led_Show_650��"ABCD",1,1��;
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
	FD650_Write(syson);// ������ʾ�ͼ��̣�8����ʾ��ʽ
	//����ʾ����
	FD650_Write( FD650_DIG0 | (u_int8)data[0] );	//������һ�������
	
	if(sec_flag)
		FD650_Write( FD650_DIG1 | (u_int8)data[1] | FD650_DOT ); //�����ڶ��������
	else
		FD650_Write( FD650_DIG1 | (u_int8)data[1] ); 
	if(Lock)
		FD650_Write( FD650_DIG2 | (u_int8)data[2] | FD650_DOT ); //���������������
	else
		FD650_Write( FD650_DIG2 | (u_int8)data[2] );
	FD650_Write( FD650_DIG3 | (u_int8)data[3] ); //�������ĸ������
	
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



