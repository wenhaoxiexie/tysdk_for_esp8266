/** 
* @file 		  FD650.H    			������Ӳ�����IO �������Լ�оƬ������ò��� 
* @brief        ������Ӳ�����IO �������Լ�оƬ������ò���
* @author        FD.chd 
* @author        Jun
* @version        A3 
* @date 			2010/07/25    ����˵�� :  �����ļ� by FD.chd 
* @date 			2012/07/25    ����˵�� :  ���ע�ͣ�doxygen��by jun
* @copyright Copyright (c) 2012 by FUZHOU FUDA HISI MICROELECTRONICS CO.,LTD.              
*/            

// Ӳ����ض���, �����ʵ��Ӳ���޸ı��ļ�
#ifndef __FD650H__
#define __FD650H__

typedef unsigned char   u_int8;
typedef unsigned short  u_int16;	
typedef unsigned long 	u_int32;

#define	FD650_SCL      1//  ()       ///<����IO�˿�
#define	FD650_SDA       0//()        ///<����IO�˿�

// ����ϵͳ��������

#define FD650_BIT_ENABLE	0x01		///< ����/�ر�λ
#define FD650_BIT_SLEEP		0x04		///< ˯�߿���λ
//#define FD650_BIT_7SEG		0x08			///< 7�ο���λ
#define FD650_BIT_INTENS1	0x10		///< 1������
#define FD650_BIT_INTENS2	0x20		///< 2������
#define FD650_BIT_INTENS3	0x30		///< 3������
#define FD650_BIT_INTENS4	0x40		///< 4������
#define FD650_BIT_INTENS5	0x50		///< 5������
#define FD650_BIT_INTENS6	0x60		///< 6������
#define FD650_BIT_INTENS7	0x70		///< 7������
#define FD650_BIT_INTENS8	0x00		///< 8������

#define FD650_SYSOFF	0x0400			///< �ر���ʾ���رռ���
#define FD650_SYSON		( FD650_SYSOFF | FD650_BIT_ENABLE )	///< ������ʾ������
#define FD650_SLEEPOFF	FD650_SYSOFF	///< �ر�˯��
#define FD650_SLEEPON	( FD650_SYSOFF | FD650_BIT_SLEEP )	///< ����˯��
//#define FD650_7SEG_ON	( FD650_SYSON | FD650_BIT_7SEG )	///< �����߶�ģʽ
#define FD650_8SEG_ON	( FD650_SYSON | 0x00 )	///< �����˶�ģʽ
#define FD650_SYSON_1	( FD650_SYSON | FD650_BIT_INTENS1 )	///< ������ʾ�����̡�1������
//�Դ�����
#define FD650_SYSON_4	( FD650_SYSON | FD650_BIT_INTENS4 )	///< ������ʾ�����̡�4������
//�Դ�����
#define FD650_SYSON_8	( FD650_SYSON | FD650_BIT_INTENS8 )	///< ������ʾ�����̡�8������


// ��������������
#define FD650_DIG0		0x1400			///< �����λ0��ʾ,�����8λ����
#define FD650_DIG1		0x1500			///< �����λ1��ʾ,�����8λ����
#define FD650_DIG2		0x1600			///< �����λ2��ʾ,�����8λ����
#define FD650_DIG3		0x1700			///< �����λ3��ʾ,�����8λ����

#define FD650_DOT			0x0080			///< �����С������ʾ

// ��ȡ������������
#define FD650_GET_KEY	0x0700					///< ��ȡ����,���ذ�������


// �ⲿ����
extern void FD650_init(void);
extern	u_int8 FD650_Read( void );				///< ��FD650��ȡ��������
extern  void FD650_Write( u_int16 cmd );	///< ��FD650������������

#endif

