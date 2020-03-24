#ifndef __FD650APP_H__
#define __FD650APP_H__

#include "FD650.h"

#define LEDMAPNUM 22		///<数码管码字数组最大值

/** 字符转数码管显示码字*/
typedef struct _led_bitmap
{
	u_int8 character;
	u_int8 bitmap;
} led_bitmap;

u_int8 Led_Get_Code(char cTemp);
void Led_Show_650( char *acFPStr, unsigned char sec_flag, unsigned char Lock,int syson);
u_int8 key_Show_650(void);

#endif // __FD650APP_H__
