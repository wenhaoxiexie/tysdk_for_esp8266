#ifndef __HLW8012_H__
#define __HLW8012_H__

#include "sys_adapter.h"
#include "esp_common.h"
#include "com_def.h"


#ifdef __cplusplus
	extern "C" {
#endif



//--------------------------------------------------------------------------------------------
#define D_ERR_MODE                	0x00        //错误提示模式
#define D_NORMAL_MODE		      	0x10	    //正常工作模式
#define D_CAL_START_MODE		    0x21	    //校正模式，启动
#define D_CAL_END_MODE		        0x23	    //校正模式，完成
//以下可选择上报或不上报
#define DP_PTRSLT  7//产测结果，0为失败，1为成功，2为未产测
#define DP_VREF  8//电压系数
#define DP_IREF  9//电流系数
#define DP_PREF  10//功率系数
#define DP_EREF  11//电量系数



typedef enum 
{
	IO_DRIVE_LEVEL_HIGH,		// 高电平有效
	IO_DRIVE_LEVEL_LOW,			// 低电平有效
	IO_DRIVE_LEVEL_NOT_EXIST	// 该IO不存在
}IO_DRIVE_TYPE;

typedef struct
{
	IO_DRIVE_TYPE type;	// 有效电平类型
	unsigned char pin;	// 引脚号
}IO_CONFIG;

typedef struct{
	unsigned char epin;
	unsigned char ivpin;
	IO_CONFIG	  ivcpin;
	unsigned int  v_ref;
	unsigned int  i_ref;
	unsigned int  p_ref;
	unsigned int  e_ref;
    unsigned int  v_def;
    unsigned int  i_def;
    unsigned int  p_def;
	BOOL		  if_have;
}DLTJ_CONFIG;


VOID hlw8012_init(DLTJ_CONFIG *dltj);
VOID get_ele_par(OUT UINT *P,OUT UINT *V,OUT UINT *I);
VOID get_ele(OUT UINT *E);
OPERATE_RET report_coe_data(VOID);
OPERATE_RET ele_cnt_init(INT mode);
OPERATE_RET get_prod_test_data(INT *state);
OPERATE_RET save_prod_test_data(INT state);

#ifdef __cplusplus
}
#endif
#endif

