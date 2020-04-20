/***********************************************************
*  File: device.c 
*  Author: nzy
*  Date: 20150605
***********************************************************/
#define __DEVICE_GLOBALS
#include "device.h"
#include "mem_pool.h"
#include "smart_wf_frame.h"
#include "key.h"
#include "led_indicator.h"
#include "system/sys_timer.h"
#include "system/uni_thread.h"
#include "system/uni_mutex.h"
#include "system/uni_semaphore.h"
#include "uart.h"
#include "smart_link.h"
#include "pwm.h"
#include "wf_sdk_adpt.h"
#include "i2c_master.h"
#include "mir3da.h"


/***********************************************************
*************************micro define***********************
***********************************************************/
#define PWM_0_OUT_IO_MUX PERIPHS_IO_MUX_MTCK_U
#define PWM_0_OUT_IO_FUNC FUNC_GPIO13
#define PWM_0_OUT_IO_NUM 13

#define PWM_3_OUT_IO_MUX PERIPHS_IO_MUX_MTDI_U
#define PWM_3_OUT_IO_FUNC FUNC_GPIO12
#define PWM_3_OUT_IO_NUM 12

#define PWM_4_OUT_IO_MUX PERIPHS_IO_MUX_GPIO4_U
#define PWM_4_OUT_IO_FUNC FUNC_GPIO4
#define PWM_4_OUT_IO_NUM 4 //4 

#define PWM_2_OUT_IO_MUX PERIPHS_IO_MUX_GPIO5_U
#define PWM_2_OUT_IO_FUNC FUNC_GPIO5
#define PWM_2_OUT_IO_NUM 5

#define PWM_1_OUT_IO_MUX PERIPHS_IO_MUX_MTDO_U
#define PWM_1_OUT_IO_FUNC FUNC_GPIO15
#define PWM_1_OUT_IO_NUM 15
		
#define CHAN_NUM 4
#define PWM_MIN 0
#if 0
#define PWM_MAX 1024
#else
#define PWM_MAX (1024 * 5)
#endif
		
uint32 duty[CHAN_NUM] = {100};
uint32 pwm_val = 0;

uint32 io_info[][3] ={
	{PWM_0_OUT_IO_MUX,PWM_0_OUT_IO_FUNC,PWM_0_OUT_IO_NUM},
	{PWM_1_OUT_IO_MUX,PWM_1_OUT_IO_FUNC,PWM_1_OUT_IO_NUM},
	{PWM_2_OUT_IO_MUX,PWM_2_OUT_IO_FUNC,PWM_2_OUT_IO_NUM},
	{PWM_3_OUT_IO_MUX,PWM_3_OUT_IO_FUNC,PWM_3_OUT_IO_NUM},
	{PWM_4_OUT_IO_MUX,PWM_4_OUT_IO_FUNC,PWM_4_OUT_IO_NUM},
};


#define REG_WRITE(_r,_v)    (*(volatile uint32 *)(_r)) = (_v)
#define REG_READ(_r)        (*(volatile uint32 *)(_r))
#define WDEV_NOW()          REG_READ(0x3ff20c00)


#define PIN_DI              13
#define PIN_DCKI            15

#define DPID_1               1
#define DPID_2               2
#define DPID_3               3
#define DPID_4               4
#define DPID_5               5
#define DPID_6               6
#define DPID_7               7
#define DPID_8               8
#define DPID_9               9
#define DPID_10              10

#define KEY_SWITCH   16
#define KEY_LEFT_1    14
#define KEY_LEFT_2    13
#define KEY_LEFT_3    3
#define KEY_LEFT_4    2
#define KEY_LEFT_5    1
#define KEY_LEFT_6    20
#define KEY_LEFT_7    19
#define KEY_RIGHT_1    9
#define KEY_RIGHT_2    8
#define KEY_RIGHT_3    7
#define KEY_RIGHT_4    6
#define KEY_RIGHT_5    5
#define KEY_RIGHT_6    4
#define KEY_RIGHT_7    15



typedef enum {
    WHITE_MODE = 0,
    COLOUR_MODE,
    SCENE_MODE,
    ROUGUANG_SCENE,
    BINFENG_SCENE,
    XUANCAI_SCENE,
    BANLAN_SCENE,
}SCENE_MODE_E;

typedef struct {
    UCHAR RED_VAL;
    UCHAR GREEN_VAL;
	UCHAR BLUE_VAL;
	UCHAR WHITE_VAL;
	UCHAR WARM_VAL;
    UCHAR LAST_RED_VAL;
    UCHAR LAST_GREEN_VAL;
	UCHAR LAST_BLUE_VAL;
	UCHAR LAST_WHITE_VAL;
	UCHAR LAST_WARM_VAL;
	UCHAR FIN_RED_VAL;
    UCHAR FIN_GREEN_VAL;
	UCHAR FIN_BLUE_VAL;
	UCHAR FIN_WHITE_VAL;
	UCHAR FIN_WARM_VAL;
	USHORT HUE;
	UCHAR SATURATION;
	UCHAR VALUE;
}LIGHT_DATA_DEF;
STATIC LIGHT_DATA_DEF light_data;

typedef struct {
    UCHAR RED_VAL;
	UCHAR GREEN_VAL;
	UCHAR BLUE_VAL;
}DATA_GROUP_DEF;

typedef struct {
    UCHAR BRIGHT;
    UCHAR COL_TEMP;
	UCHAR SPEED;
	UCHAR NUM;
	DATA_GROUP_DEF data_group[6];
}FLASH_LIGHT_DATA_DEF;
STATIC FLASH_LIGHT_DATA_DEF flash_light_data;


typedef struct 
{
	BOOL scale_flag;
    THREAD gra_thread;
    SEM_HANDLE gra_key_sem;
}L_GRA_CHANGE_DEF;
STATIC L_GRA_CHANGE_DEF gra_change;

typedef struct 
{
    INT r_delata;
	INT g_delata;
	INT b_delata;
	MUTEX_HANDLE  mutex;
    THREAD flash_scene_thread;
    xSemaphoreHandle flash_scene_sem;
}FLASH_SCENE_HANDLE_DEF;
STATIC FLASH_SCENE_HANDLE_DEF flash_scene_handle;

typedef struct 
{
	BOOL SWITCH;
	SCENE_MODE_E WORK_MODE;
    UCHAR BRIGHT;
	UCHAR COL_TEMPERATURE;
	UCHAR COLOUR_DATA[15];
	UCHAR SCENE_DATA[15];
	UCHAR ROUGUANG_SCENE_DATA[15];
	UCHAR BINFENG_SCENE_DATA[45];
	UCHAR XUANCAI_SCENE_DATA[15];
	UCHAR BANLAN_SCENE_DATA[45];
}DP_DEF;
STATIC DP_DEF dp_data;

typedef enum {
    FUC_TEST1 = 0,
	AGING_TEST,
	FUC_TEST2,
}TEST_MODE_DEF;

typedef enum {
	FUN_SUC = 0,
    NO_KEY,
	WIFI_TEST_ERR,
}FUN_TEST_RET;


typedef struct 
{
	UINT pmd_times;
	UINT aging_times;
	UINT aging_tested_time;
	UINT aging_left_time;
	BOOL wf_test_ret;
	FUN_TEST_RET fun_test_ret;
	TEST_MODE_DEF test_mode;
	TIMER_ID fuc_test_timer;
	TIMER_ID aging_test_timer;
}TEST_DEF;
STATIC TEST_DEF test_handle;
#define AGING_TEST_TIME 60
#define AGING_TEST_W_TIME 45
#define AGING_TEST_RGB_TIME AGING_TEST_TIME-AGING_TEST_W_TIME
#define TIME_SAVE_INTERVAL 1


#define DEVICE_MOD "device_mod"
#define DEVICE_PART "device_part"
#define FASE_SW_CNT_KEY "fsw_cnt_key"
#define DP_DATA_KEY   "dp_data_key"
#define LIGHT_TEST_KEY   "light_test_key"
#define AGING_TESTED_TIME  "aging_tested_time"


#define BRIGHT_INIT_VALUE 25
#define COL_TEMP_INIT_VALUE 255
#define TEMP_FULL_VALUE 255
#define RED_MAX_VALUE 255
#define GREEN_MAX_VALUE 38
#define BLUE_MAX_VALUE 0
#define WHITE_DEL_VALUE 60
#define BREATH_DELAY_TIME 40
#define NORMAL_DELAY_TIME 3
#define RESO_VAL 4

STATIC UCHAR TEST_R_BRGHT = 10;
STATIC UCHAR TEST_G_BRGHT = 10;
STATIC UCHAR TEST_B_BRGHT = 10;
STATIC UCHAR TEST_W_BRGHT = 1;
volatile STATIC BOOL flash_scene_flag = TRUE;

volatile STATIC BOOL gsensor_trigger = FALSE;

STATIC u8 key_switch_now = 0;
STATIC u8 key_switch_single = 0;
STATIC u8 key_switch_double = 0;
STATIC u8 key_left = 1;
STATIC u8 key_right = 1;
STATIC u8 key_left_temp = 1;
STATIC u8 key_right_temp = 1;
static short  g_x = 0;
static short  g_y = 0;
static short  g_z = 0;

static short  x_n = 0;
static short  y_n = 0;
static short  z_n = 0;

STATIC u8 click_now = 0;
STATIC u8 click_single = 0;
STATIC u8 click_double = 0;
STATIC u8 action = 0;

STATIC u16 inc_sum_record = 0;

#if 0
#define COLOUR_1    "ff006f014effff"
#define COLOUR_2    "ff00dd0134ffff"
#define COLOUR_3    "aa00ff0118ffff"
#define COLOUR_4    "0900ff00f2ffff"
#define COLOUR_5    "0084ff00d1ffff"
#define COLOUR_6    "00eaff00b9ffff"
#define COLOUR_7    "00ff040079ffff"
#else

#define COLOUR_1_1    "ff00150163ffff"      // 100
#define COLOUR_1_2    "d900120163ffd5"   //  83
#define COLOUR_1_3    "b0000f0163ffac"    //  66
#define COLOUR_1_4    "8c00090164ff85"   //  50
#define COLOUR_1_5    "6900070164ff5e"   //  34 
#define COLOUR_1_6    "4000040164ff35"   //  17
#define COLOUR_1_7    "1c00010165ff0e"    //  1

#define COLOUR_2_1    "ff00ee0130ffff"        // 100
#define COLOUR_2_2    "d900ca0130ffd5"     // 83 
#define COLOUR_2_3    "b000a40130ffac"    // 66
#define COLOUR_2_4    "8c00830130ff85"    // 50
#define COLOUR_2_5    "6900620130ff5e"    // 34 
#define COLOUR_2_6    "40003c0130ff35"    // 17
#define COLOUR_2_7    "1c001a0130ff0e"    // 1

#define COLOUR_3_1    "1700ff00f5ffff"     // 100
#define COLOUR_3_2    "1200d900f5ffd5"    //  83
#define COLOUR_3_3    "0f00b000f5ffac"    // 66
#define COLOUR_3_4    "0c008c00f5ff85"    // 50
#define COLOUR_3_5    "09006900f5ff5e"   // 34
#define COLOUR_3_6    "05004000f5ff35"   // 17
#define COLOUR_3_7    "02001c00f5ff0e"   // 1

#define COLOUR_4_1    "00a7ff00c8ffff"   // 100
#define COLOUR_4_2    "0091d900c8ffd5"   // 83
#define COLOUR_4_3    "0075b000c8ffac"   // 66
#define COLOUR_4_4    "005e8c00c8ff85"   // 50
#define COLOUR_4_5    "00466900c8ff5e"   // 34
#define COLOUR_4_6    "002b4000c8ff35"   // 17
#define COLOUR_4_7    "00131c00c8ff0e"  // 1

#define COLOUR_5_1    "00ff0a007affff"      // 100
#define COLOUR_5_2    "00d907007affd5"   // 83
#define COLOUR_5_3    "00b006007affac"   // 66
#define COLOUR_5_4    "008c05007aff85"   // 50 
#define COLOUR_5_5    "006903007aff5e"   // 34
#define COLOUR_5_6    "004002007aff35"   // 17
#define COLOUR_5_7    "001c01007aff0e"   // 1

#define COLOUR_6_1    "f6ff00003effff"  // 100
#define COLOUR_6_2    "d2d900003effd5"  // 83
#define COLOUR_6_3    "aab000003effac"  // 66
#define COLOUR_6_4    "888c00003eff85"  // 50 
#define COLOUR_6_5    "656900003eff5e"  // 34
#define COLOUR_6_6    "3e4000003eff35"  // 17
#define COLOUR_6_7    "1b1c00003eff0e"  // 1

#define COLOUR_7_1    "ff6e000019ffff"  // 100
#define COLOUR_7_2    "d95a000019ffd5"  // 83
#define COLOUR_7_3    "b049000019ffac"  // 66
#define COLOUR_7_4    "8c3a000019ff85"  // 50
#define COLOUR_7_5    "692c000019ff5e"  // 34 
#define COLOUR_7_6    "401b000019ff35"  // 17
#define COLOUR_7_7    "1c0c000019ff0e"  // 1

#endif

const char* colour_table[7][7] = {
	{COLOUR_1_1,COLOUR_1_2,COLOUR_1_3,COLOUR_1_4,COLOUR_1_5,COLOUR_1_6,COLOUR_1_7},
	{COLOUR_2_1,COLOUR_2_2,COLOUR_2_3,COLOUR_2_4,COLOUR_2_5,COLOUR_2_6,COLOUR_2_7},
	{COLOUR_3_1,COLOUR_3_2,COLOUR_3_3,COLOUR_3_4,COLOUR_3_5,COLOUR_3_6,COLOUR_3_7},
	{COLOUR_4_1,COLOUR_4_2,COLOUR_4_3,COLOUR_4_4,COLOUR_4_5,COLOUR_4_6,COLOUR_4_7},
	{COLOUR_5_1,COLOUR_5_2,COLOUR_5_3,COLOUR_5_4,COLOUR_5_5,COLOUR_5_6,COLOUR_5_7},
	{COLOUR_6_1,COLOUR_6_2,COLOUR_6_3,COLOUR_6_4,COLOUR_6_5,COLOUR_6_6,COLOUR_6_7},
	{COLOUR_7_1,COLOUR_7_2,COLOUR_7_3,COLOUR_7_4,COLOUR_7_5,COLOUR_7_6,COLOUR_7_7}
};

/***********************************************************
*************************function define********************
***********************************************************/
STATIC OPERATE_RET device_differ_init(VOID);
STATIC INT ABS(INT value);
STATIC VOID wf_direct_timer_cb(UINT timerID,PVOID pTimerArg);
STATIC VOID reset_fsw_cnt_cb(UINT timerID,PVOID pTimerArg);
STATIC OPERATE_RET dev_inf_get(VOID);
STATIC OPERATE_RET dev_inf_set(VOID);
STATIC VOID init_upload_proc(VOID);
STATIC VOID sl_datapoint_proc(cJSON *root);
STATIC USHORT byte_combine(UCHAR hight, UCHAR low);
STATIC VOID idu_timer_cb(UINT timerID,PVOID pTimerArg);
STATIC VOID char_change(UINT temp, UCHAR *hight, UCHAR *low);
STATIC INT string_combine_int(u32 a,u32 b, u32 c,u32 d);
STATIC VOID start_gra_change(TIME_MS delay_time);
STATIC VOID light_gra_change(PVOID pArg);
STATIC VOID flash_scene_change(PVOID pArg);
STATIC INT ty_get_enum_id(UCHAR dpid, UCHAR *enum_str);
STATIC UCHAR *ty_get_enum_str(DP_CNTL_S *dp_cntl, UCHAR enum_id);
STATIC VOID wfl_timer_cb(UINT timerID,PVOID pTimerArg);
STATIC VOID work_mode_change(SCENE_MODE_E mode);
STATIC VOID hw_test_timer_cb(void);
STATIC VOID get_light_data(VOID);
STATIC VOID set_default_dp_data(VOID);
STATIC VOID send_light_data(u8 R_value, u8 G_value, u8 B_value, u8 CW_value, u8 WW_value);
STATIC VOID com_iic_proc(PVOID pArg);




/***********************************************************
*************************variable define********************
***********************************************************/
// KEY

TIMER_ID wf_stat_dir;
TIMER_ID timer_init_dpdata;
TIMER_ID gradua_timer;
TIMER_ID timer;
TIMER_ID data_save_timer;

THREAD   com_iic_thread;	  	  


STATIC UINT irq_cnt = 0;
STATIC UINT num_cnt = 0;
STATIC INT flash_dir = 0;

BOOL sta_cha_flag = FALSE;

UCHAR com_iic_addr = 0x4d;

UCHAR touch_addr = 0xa6;
UCHAR touch_key_read_data = 0;

STATIC BYTE gamma_8_correction[]={0,1,1,1,1,2,2,3,3,4,4,5,6,6,7,7,8,9,9,10,11,11,12,13,13,
								  14,15,15,16,17,18,18,19,20,21,21,22,23,24,24,25,26,27,28,
								  28,29,30,31,32,32,33,34,35,36,37,37,38,39,40,41,42,43,44,
								  44,45,46,47,48,49,50,51,52,52,53,54,55,56,57,58,59,60,61,
								  62,63,64,65,66,66,67,68,69,70,71,72,73,74,75,76,77,78,79,
								  80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,96,97,98,
								  99,100,101,103,104,105,106,107,108,109,110,111,112,113,114,
								  115,116,117,118,119,121,122,123,124,125,126,127,128,129,130,
								  131,132,134,135,136,137,138,139,140,141,142,144,145,146,147,
								  148,149,150,151,152,154,155,156,157,158,159,160,162,163,164,
								  165,166,167,168,170,171,172,173,174,175,177,178,179,180,181,
								  182,184,185,186,187,188,189,191,192,193,194,195,196,198,199,
								  200,201,202,204,205,206,207,208,210,211,212,213,214,216,217,
								  218,219,220,222,223,224,225,227,228,229,230,231,233,234,235,
								  236,238,239,240,241,243,244,245,246,248,249,250,251,253,254,255};

STATIC BYTE gamma_6_70_correction[]={0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,2,2,2,2,2,3,3,3,3,3,4,4,4,
								  4,5,5,5,6,6,6,7,7,7,7,8,8,8,9,9,10,10,10,11,11,11,12,12,
								  13,13,13,14,14,15,15,16,16,16,17,17,18,18,19,19,20,20,21,
								  21,22,22,23,23,24,24,25,25,26,26,27,27,28,29,29,30,30,31,
								  31,32,33,33,34,34,35,36,36,37,38,38,39,39,40,41,41,42,43,
								  43,44,45,45,46,47,47,48,49,49,50,51,52,52,53,54,54,55,56,
								  57,57,58,59,60,60,61,62,63,63,64,65,66,66,67,68,69,70,70,
								  71,72,73,74,75,75,76,77,78,79,80,80,81,82,83,84,85,86,86,
								  87,88,89,90,91,92,93,94,94,95,96,97,98,99,100,101,102,103,
								  104,105,106,106,107,108,109,110,111,112,113,114,115,116,117,
								  118,119,120,121,122,123,124,125,126,127,128,129,130,131,132,
								  133,134,135,136,137,139,140,141,142,143,144,145,146,147,148,
								  149,150,151,152,154,155,156,157,158,159,160,161,162,164,165,
								  166,167,168,169,170,172,173,174,175,176,177,179};
/*STATIC BYTE gamma_6_70_correction[]={0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,2,2,2,2,2,3,3,3,3,3,4,4,4,
								  4,5,5,5,6,6,6,7,7,7,7,8,8,8,9,9,10,10,10,11,11,11,12,12,
								  13,13,13,14,14,15,15,16,16,16,17,17,18,18,19,19,20,20,21,
								  21,22,22,23,23,24,24,25,25,26,26,27,27,28,29,29,30,30,31,
								  31,32,33,33,34,34,35,36,36,37,38,38,39,39,40,41,41,42,43,
								  43,44,45,45,46,47,47,48,49,49,50,51,52,52,53,54,54,55,56,
								  57,57,58,59,60,60,61,62,63,63,64,65,66,66,67,68,69,70,70,
								  71,72,73,74,75,75,76,77,78,79,80,80,81,82,83,84,85,86,86,
								  87,88,89,90,91,92,93,94,94,95,96,97,98,99,100,101,102,103,
								  104,105,106,106,107,108,109,110,111,112,113,114,115,116,117,
								  118,119,120,121,122,123,124,125,126,127,128,129,130,131,132,
								  133,134,135,136,137,139,140,141,142,143,144,145,146,147,148,
								  149,150,151,152,154,155,156,157,158,159,160,161,162,164,165,
								  166,167,168,169,170,172,173,174,175,176,177,179};*/

STATIC BYTE gamma_6_75_correction[]={0,0,0,0,0,0,0,0,1,1,1,1,1,1,2,2,2,2,2,3,3,3,3,3,4,4,4,5,
									 5,5,5,6,6,6,7,7,7,8,8,8,9,9,9,10,10,11,11,11,12,12,13,13,
									 14,14,14,15,15,16,16,17,17,18,18,19,19,20,20,21,21,22,22,
									 23,23,24,24,25,25,26,27,27,28,28,29,29,30,31,31,32,32,33,
									 34,34,35,36,36,37,38,38,39,40,40,41,42,42,43,44,44,45,46,
									 46,47,48,49,49,50,51,51,52,53,54,54,55,56,57,58,58,59,60,
									 61,61,62,63,64,65,65,66,67,68,69,70,70,71,72,73,74,75,76,
									 76,77,78,79,80,81,82,83,83,84,85,86,87,88,89,90,91,92,93,
									 94,94,95,96,97,98,99,100,101,102,103,104,105,106,107,108,
									 109,110,111,112,113,114,115,116,117,118,119,120,121,122,
									 123,124,125,127,128,129,130,131,132,133,134,135,136,137,
									 138,139,141,142,143,144,145,146,147,148,150,151,152,153,
									 154,155,156,158,159,160,161,162,163,165,166,167,168,169,
									 170,172,173,174,175,176,178,179,180,181,183,184,185,186,
									 188,189,190,191};
/***********************************************************
*************************function define********************
***********************************************************/


void pre_app_init()
{

}


STATIC USHORT byte_combine(UCHAR hight, UCHAR low)
{
    USHORT temp;
    temp = (hight << 8) | low;
    return temp;
}
STATIC VOID char_change(UINT temp, UCHAR *hight, UCHAR *low)
{
    *hight = (temp & 0xff00) >> 8;
    *low = temp & 0x00ff;
}

STATIC INT string_combine_byte(u32 a,u32 b)
{
   INT combine_data = (a<<4)|(b&0xf);
   return combine_data;
}
STATIC INT string_combine_short(u32 a,u32 b, u32 c,u32 d)
{
   INT combine_data = (a<<12)|(b<<8)|(c<<4)|(d&0xf);
   return combine_data;
}


STATIC INT ABS(INT value)
{
	if(value < 0){
		return 0-value;
	}else{
		return value;
	}
}

STATIC UCHAR get_max_value(UCHAR a, UCHAR b, UCHAR c, UCHAR d, UCHAR e)
{
	int x = a > b ? a : b; //1´Î±È½Ï£¬1´Î¸³Öµ
	int y = c > d ? c : d; //1´Î±È½Ï£¬1´Î¸³Öµ
	int z = x > y ? x : y;
	return z > e ? z : e;  //1´Î±È½Ï 
}


STATIC CHAR* my_itoa(int num,char*str,int radix)
{
/*Ë÷Òý±í*/
    char index[]="0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
    unsigned unum;/*ÖÐ¼ä±äÁ¿*/
    int i=0,j,k;
    char temp;
    /*È·¶¨unumµÄÖµ*/
    if(radix==10&&num<0)/*Ê®½øÖÆ¸ºÊý*/
    {
        unum=(unsigned)-num;
        str[i++]='-';
    }
    else unum=(unsigned)num;/*ÆäËûÇé¿ö*/
    /*×ª»»*/
    do{
        str[i++]=index[unum%(unsigned)radix];
        unum/=radix;
    }while(unum);
    str[i]='\0';
    /*ÄæÐò*/
    if(str[0]=='-')k=1;/*Ê®½øÖÆ¸ºÊý*/
    else k=0;
    for(j=k;j<=(i-1)/2;j++)
    {
        temp=str[j];
        str[j]=str[i-1+k-j];
        str[i-1+k-j]=temp;
    }
    return str;
}
static unsigned char abcd_to_asc(unsigned char ucBcd)
{
	unsigned char ucAsc = 0;
	
	ucBcd &= 0x0f;
	if (ucBcd <= 9)
		ucAsc = ucBcd + '0';
	else
		ucAsc = ucBcd + 'A' - 10;
	return (ucAsc);
}

void BcdToAsc_Api(char * sAscBuf, unsigned char * sBcdBuf, int iAscLen)
{
	int i, j;j = 0;

	if((sBcdBuf == NULL) || (sAscBuf == NULL) || (iAscLen < 0))
		return;
	
	for (i = 0; i < iAscLen / 2; i++) 
	{
		sAscBuf[j] = (sBcdBuf[i] & 0xf0) >> 4;
		sAscBuf[j] = abcd_to_asc(sAscBuf[j]);
		j++;
		sAscBuf[j] = sBcdBuf[i] & 0x0f;
		sAscBuf[j] = abcd_to_asc(sAscBuf[j]);
		j++;
	}
	if (iAscLen % 2) 
	{
		sAscBuf[j] = (sBcdBuf[i] & 0xf0) >> 4;
		sAscBuf[j] = abcd_to_asc(sAscBuf[j]);
	}
}

VOID device_cb(SMART_CMD_E cmd,cJSON *root)
{
    BOOL op_led = FALSE;
    CHAR *buf = cJSON_PrintUnformatted(root);
    if(NULL == buf) {
        PR_ERR("malloc error");
        return;
    }
    //PR_DEBUG("the cmd is %d",cmd);
    PR_DEBUG("the receive buf is %s",buf);
    cJSON *nxt = root->child;
    while(nxt) {
        sl_datapoint_proc(nxt);
        nxt = nxt->next; 
        op_led = TRUE;
    }
    if(TRUE == op_led) {
		if(!IsThisSysTimerRun(data_save_timer)){
        	sys_start_timer(data_save_timer,5000,TIMER_CYCLE);
		}
        op_led = FALSE;
    }
    //·µ»ØÐÅÏ¢
    OPERATE_RET op_ret = sf_obj_dp_report(get_single_wf_dev()->dev_if.id, buf);
    if(OPRT_OK != op_ret) {
        PR_ERR("sf_obj_dp_report err:%d",op_ret);
        PR_DEBUG_RAW("%s\r\n",buf);
        Free(buf);
        return;
    }
    Free(buf);
}

STATIC VOID data_save_timer_cb(UINT timerID,PVOID pTimerArg)
{
     dev_inf_set();
	 sys_stop_timer(data_save_timer);
}

#if  1
STATIC VOID send_light_data(u8 R_value, u8 G_value, u8 B_value, u8 CW_value, u8 WW_value)
{
	pwm_val = PWM_MAX*R_value/255;
	pwm_set_duty(pwm_val, 1);
	pwm_val = PWM_MAX*G_value/255;
	pwm_set_duty(pwm_val, 3);
	pwm_val = PWM_MAX*B_value/255;
	pwm_set_duty(pwm_val, 2);
	pwm_val = PWM_MAX*CW_value/255;
	pwm_set_duty(pwm_val, 0);
	//pwm_val = PWM_MAX*WW_value/255;
	//pwm_set_duty(pwm_val, 4);
	pwm_start();	
}
#else

STATIC VOID send_light_data(u8 R_value, u8 G_value, u8 B_value, u8 CW_value, u8 WW_value)
{
       if (R_value < 245) {
		pwm_val = PWM_MAX*R_value/255;
		pwm_set_duty(pwm_val, 1);
	}
        else {
    		PR_DEBUG("----------stop pwm to level high---------\r\n");	   	
		pwm_stop(0x0f);
	}
		
	pwm_val = PWM_MAX*R_value/255;
	pwm_set_duty(pwm_val, 1);
	pwm_val = PWM_MAX*G_value/255;
	pwm_set_duty(pwm_val, 3);
	pwm_val = PWM_MAX*B_value/255;
	pwm_set_duty(pwm_val, 2);
	pwm_val = PWM_MAX*CW_value/255;
	pwm_set_duty(pwm_val, 0);
	//pwm_val = PWM_MAX*WW_value/255;
	//pwm_set_duty(pwm_val, 4);
	pwm_start();	
}

STATIC VOID send_light_data(u8 CW_value)
{
	PR_DEBUG_RAW("--------CW_value:%d--------\r\n",CW_value);

       if (CW_value < 245) {
    		PR_DEBUG("----------normal pwm---------\r\n");	   	
		pwm_val = PWM_MAX*CW_value/255;
		pwm_set_duty(pwm_val, 0);
		pwm_start();	
	}
        else {
    		PR_DEBUG("----------stop pwm to level high---------\r\n");	   	
		pwm_stop(0x0f);
	}
}
#endif

/*************************************************************light test********************************/

STATIC OPERATE_RET get_light_test_flag(VOID)
{
	OPERATE_RET op_ret;

	UCHAR *buf;
	buf = Malloc(32);
	if(NULL == buf) {
		PR_ERR("Malloc failed");
		goto ERR_EXT;
	}

	op_ret = msf_get_single(DEVICE_MOD, LIGHT_TEST_KEY, buf, 32);
	if(OPRT_OK != op_ret){
		PR_ERR("msf_get_single failed");
		Free(buf);
		goto ERR_EXT;
	}
	PR_DEBUG("buf:%s",buf);
	
	cJSON *root = NULL;
	root = cJSON_Parse(buf);
	if(NULL == root) {
		Free(buf);
		test_handle.test_mode = FUC_TEST1;
		return OPRT_CJSON_PARSE_ERR;
	}
	Free(buf);

	cJSON *json;
	json = cJSON_GetObjectItem(root,"test_mode");
	if(NULL == json) {
		test_handle.test_mode = FUC_TEST1;
	}else{
		test_handle.test_mode = json->valueint;
	}
	
	cJSON_Delete(root);
	return OPRT_OK;
		
ERR_EXT:	
	test_handle.test_mode = FUC_TEST1;
	return OPRT_COM_ERROR;

}

STATIC OPERATE_RET set_light_test_flag(VOID)
{
	OPERATE_RET op_ret;
    INT i = 0;
	CHAR *out = NULL;

	cJSON *root_test = NULL;
	root_test = cJSON_CreateObject();
	if(NULL == root_test) {
		PR_ERR("json creat failed");
		goto ERR_EXT;
	}
	
	cJSON_AddNumberToObject(root_test, "test_mode", test_handle.test_mode);
	
	out=cJSON_PrintUnformatted(root_test);
	cJSON_Delete(root_test);
	if(NULL == out) {
		PR_ERR("cJSON_PrintUnformatted err:");
		Free(out);
		goto ERR_EXT;
	}
	//PR_DEBUG("out[%s]", out);
	op_ret = msf_set_single(DEVICE_MOD, LIGHT_TEST_KEY, out); 
	if(OPRT_OK != op_ret) {
		PR_ERR("data write psm err: %d",op_ret);
		Free(out);
		goto ERR_EXT;
	}
	Free(out);
	return OPRT_OK;
ERR_EXT:	
	return OPRT_COM_ERROR;

}

STATIC OPERATE_RET get_aging_tested_time(VOID)
{
	OPERATE_RET op_ret;

	UCHAR *buf;
	buf = Malloc(64);
	if(NULL == buf) {
		PR_ERR("Malloc failed");
		goto ERR_EXT;
	}

	op_ret = msf_get_single(DEVICE_MOD, AGING_TESTED_TIME, buf, 64);
	if(OPRT_OK != op_ret){
		PR_ERR("msf_get_single failed");
		Free(buf);
		goto ERR_EXT;
	}
	PR_DEBUG("buf:%s",buf);
	
	cJSON *root = NULL;
	root = cJSON_Parse(buf);
	if(NULL == root) {
		Free(buf);
		test_handle.aging_tested_time = 0;
		return OPRT_CJSON_PARSE_ERR;
	}
	Free(buf);

	cJSON *json;
	json = cJSON_GetObjectItem(root,"aging_tested_time");
	if(NULL == json) {
		test_handle.aging_tested_time = 0;
	}else{
		test_handle.aging_tested_time = json->valueint;
	}

	cJSON_Delete(root);
	return OPRT_OK;
		
ERR_EXT:
	test_handle.aging_tested_time = 0;
	return OPRT_COM_ERROR;

}

STATIC OPERATE_RET set_aging_tested_time(VOID)
{
	OPERATE_RET op_ret;
    INT i = 0;
	CHAR *out = NULL;

	cJSON *root_test = NULL;
	root_test = cJSON_CreateObject();
	if(NULL == root_test) {
		PR_ERR("json creat failed");
		goto ERR_EXT;
	}
	
	cJSON_AddNumberToObject(root_test, "aging_tested_time", test_handle.aging_tested_time);
	
	out=cJSON_PrintUnformatted(root_test);
	cJSON_Delete(root_test);
	if(NULL == out) {
		PR_ERR("cJSON_PrintUnformatted err:");
		Free(out);
		goto ERR_EXT;
	}
	PR_DEBUG("out[%s]", out);
	op_ret = msf_set_single(DEVICE_MOD, AGING_TESTED_TIME, out); 
	if(OPRT_OK != op_ret) {
		PR_ERR("data write psm err: %d",op_ret);
		Free(out);
		goto ERR_EXT;
	}
	Free(out);
	return OPRT_OK;
ERR_EXT:	
	return OPRT_COM_ERROR;

}

STATIC INT get_reset_cnt(VOID)
{
	OPERATE_RET op_ret;
	INT cnt = 0;
	UCHAR *buf;
	buf = Malloc(32);
	if(NULL == buf) {
		PR_ERR("Malloc failed");
		goto ERR_EXT;
	}

	op_ret = msf_get_single(DEVICE_MOD, FASE_SW_CNT_KEY, buf, 32);
	if(OPRT_OK != op_ret){
		PR_ERR("get_reset_cnt failed");
		Free(buf);
		goto ERR_EXT;
	}
	PR_DEBUG("buf:%s",buf);
	
	cJSON *root = NULL;
	root = cJSON_Parse(buf);
	if(NULL == root) {
		Free(buf);
		goto ERR_EXT;
	}
	Free(buf);

	cJSON *json;
	json = cJSON_GetObjectItem(root,"fsw_cnt_key");
	if(NULL == json) {
		cnt = 0;
	}else{
		cnt = json->valueint;
	}
	
	cJSON_Delete(root);
	return cnt;
		
ERR_EXT:	
	return 0;

}

STATIC OPERATE_RET set_reset_cnt(INT val)
{
	OPERATE_RET op_ret;
	CHAR *out = NULL;

	cJSON *root_test = NULL;
	root_test = cJSON_CreateObject();
	if(NULL == root_test) {
		PR_ERR("json creat failed");
		goto ERR_EXT;
	}
	
	cJSON_AddNumberToObject(root_test, "fsw_cnt_key", val);
	
	out=cJSON_PrintUnformatted(root_test);
	cJSON_Delete(root_test);
	if(NULL == out) {
		PR_ERR("cJSON_PrintUnformatted err:");
		Free(out);
		goto ERR_EXT;
	}
	PR_DEBUG("out[%s]", out);
	op_ret = msf_set_single(DEVICE_MOD, FASE_SW_CNT_KEY, out); 
	if(OPRT_OK != op_ret) {
		PR_ERR("data write psm err: %d",op_ret);
		Free(out);
		goto ERR_EXT;
	}
	Free(out);
	return OPRT_OK;
ERR_EXT:	
	return OPRT_COM_ERROR;

}

STATIC VOID fuc_test_timer_cb(UINT timerID,PVOID pTimerArg)
{
	test_handle.pmd_times ++;
	switch(test_handle.pmd_times % 3){
		case 1: send_light_data(TEST_R_BRGHT, 0, 0, 0, 0); break;
		case 2: send_light_data(0, TEST_G_BRGHT, 0, 0, 0); break;
		case 0: send_light_data(0, 0, TEST_B_BRGHT, 0, 0); break;
		default:break;
	}
	if(test_handle.pmd_times == 120){
		if(test_handle.test_mode == FUC_TEST1){
			test_handle.test_mode = AGING_TEST;
			if(OPRT_OK != set_light_test_flag()) { 
				sys_stop_timer(test_handle.fuc_test_timer);
				send_light_data(0x00, 0x00, 0x00, 0x00, 0x00);
			}else{
				SystemReset();
			}
		}
	}
}

STATIC VOID aging_test_timer_cb(UINT timerID,PVOID pTimerArg)
{
	test_handle.aging_times ++;
	if(test_handle.aging_times%TIME_SAVE_INTERVAL == 0){
		test_handle.aging_tested_time++;
		if(OPRT_OK != set_aging_tested_time()){
			send_light_data(0x00, 0x00, 0x00, 0x00, 0x00);
			sys_stop_timer(test_handle.aging_test_timer);
		}
	}
	if(test_handle.aging_times == test_handle.aging_left_time){
		sys_stop_timer(test_handle.aging_test_timer);
		send_light_data(0, 0xff, 0, 0, 0);
		test_handle.test_mode = FUC_TEST2;
		test_handle.aging_tested_time = 0;
		if(OPRT_OK != set_light_test_flag()){
			send_light_data(0x00, 0x00, 0x00, 0x00, 0x00);
		}
		if(OPRT_OK != set_aging_tested_time()){
			send_light_data(0x00, 0x00, 0x00, 0x00, 0x00);
		}
	}else{
		if(AGING_TEST_RGB_TIME >= (test_handle.aging_left_time-test_handle.aging_times)){
			send_light_data(0xff, 0xff, 0xff, 0x00, 0x00);
		}
	}
}

STATIC VOID gpio_func_test_cb(UINT timerID,PVOID pTimerArg)
{
	 STATIC u32 times = 0;
	 times ++;
	 switch(times % 3){
	 	case 1: send_light_data(2, 0, 0, 0, 0); break;
		case 2: send_light_data(0, 2, 0, 0, 0); break;
		case 0: send_light_data(0, 0, 2, 0, 0); break;
		default:break;
	 }
}

VOID set_firmware_tp(IN OUT CHAR *firm_name, IN OUT CHAR *firm_ver)
{
	strcpy(firm_name,APP_BIN_NAME);
	strcpy(firm_ver,USER_SW_VER);
	return;
}

BOOL gpio_func_test(VOID)
{
	STATIC TIMER_ID gpio_func_test_timer;
	STATIC BOOL timer_add_flag = FALSE;

	if(timer_add_flag == FALSE){
		if(sys_add_timer(gpio_func_test_cb,NULL,&gpio_func_test_timer) != OPRT_OK) { 
			return FALSE;
		}
		timer_add_flag = TRUE;
	}
	
	if(!IsThisSysTimerRun(gpio_func_test_timer)){
		if(OPRT_OK != sys_start_timer(gpio_func_test_timer, 500, TIMER_CYCLE)){
			return FALSE;
		}
	}

	return TRUE;
}

/*********************************************************************************************************/
#if 0
VOID prod_test(BOOL flag, CHAR rssi)
{
	OPERATE_RET op_ret;
	flash_scene_flag = FALSE;
	print_port_init(UART0);
	PR_DEBUG("rssi:%d", rssi);
	set_reset_cnt(0);
	//prod thread create and start
	if( rssi < -60 || flag == FALSE) {
		send_light_data(TEST_R_BRGHT, 0, 0, 0, 0);
		return;
	}

	if(OPRT_OK != get_light_test_flag()){
		PR_ERR("get_light_test_flag err.......");
	}
	test_handle.test_mode = FUC_TEST2;
	if(test_handle.test_mode == AGING_TEST){
		get_aging_tested_time();
		test_handle.aging_left_time = AGING_TEST_TIME - test_handle.aging_tested_time*TIME_SAVE_INTERVAL;
		op_ret = sys_add_timer(aging_test_timer_cb,NULL,&test_handle.aging_test_timer);
	    if(OPRT_OK != op_ret) {
			send_light_data(0x00, 0x00, 0x00, 0x00, 0x00);
	        return;
	    }
		if(AGING_TEST_RGB_TIME >= test_handle.aging_left_time){
			send_light_data(0xff, 0xff, 0xff, 0x00, 0x00);
		}else{
			send_light_data(0x00, 0x00, 0x00, 0xff, 0xff);
		}
    	
		test_handle.aging_times = 0;
        sys_start_timer(test_handle.aging_test_timer, 60000, TIMER_CYCLE);
	}else{
		op_ret = sys_add_timer(fuc_test_timer_cb,NULL,&test_handle.fuc_test_timer);
	    if(OPRT_OK != op_ret) {  
			send_light_data(0x00, 0x00, 0x00, 0x00, 0x00);
	        return;
	    }
		test_handle.pmd_times = 0;
		if(test_handle.test_mode == FUC_TEST1){
			sys_start_timer(test_handle.fuc_test_timer, 1000, TIMER_CYCLE);
		}else{
			sys_start_timer(test_handle.fuc_test_timer, 500, TIMER_CYCLE);
		}
	}
	return;	
}
#endif


VOID reset_light_sta(VOID)
{
 	//?????£¤?????
    dev_inf_get();
	if(dp_data.SWITCH == TRUE){
		get_light_data();
		
		//_DEBUG("R:%d G:%d B:%d W:%d WW:%d",light_data.FIN_RED_VAL, light_data.FIN_GREEN_VAL, light_data.FIN_BLUE_VAL, light_data.FIN_WHITE_VAL, light_data.FIN_WARM_VAL);
						
		switch(dp_data.WORK_MODE)
		{
			case WHITE_MODE:
			case COLOUR_MODE:
			case SCENE_MODE:
				light_data.LAST_RED_VAL = light_data.FIN_RED_VAL/RESO_VAL;
				light_data.LAST_GREEN_VAL = light_data.FIN_GREEN_VAL/RESO_VAL;
				light_data.LAST_BLUE_VAL = light_data.FIN_BLUE_VAL/RESO_VAL;
				light_data.LAST_WHITE_VAL = light_data.FIN_WHITE_VAL/RESO_VAL;
				light_data.LAST_WARM_VAL = light_data.FIN_WARM_VAL/RESO_VAL;
				send_light_data(light_data.FIN_RED_VAL,light_data.FIN_GREEN_VAL,light_data.FIN_BLUE_VAL,light_data.FIN_WHITE_VAL,light_data.FIN_WARM_VAL);
				break;
				break;
			default:
				light_data.LAST_RED_VAL = 0;
				light_data.LAST_GREEN_VAL = 0;
				light_data.LAST_BLUE_VAL = 0;
				light_data.LAST_WHITE_VAL = 0;
				light_data.LAST_WARM_VAL = 0;
				flash_scene_flag = TRUE;
				break;		
		}
	}
}


BOOL prod_test(BOOL flag, CHAR rssi)
{
	OPERATE_RET op_ret;
	flash_scene_flag = FALSE;
	//print_port_init(UART0);
	print_port_full_init(UART0,BIT_RATE_115200,UART_WordLength_8b,USART_Parity_None,USART_StopBits_1);

	//PR_DEBUG("---------------------------come into prod_test---------------------");
	PR_ERR("--------------------come into prod_test------------------");		
	
	PR_DEBUG("rssi:%d", rssi);
	//set_reset_cnt(0);
	//prod thread create and start
	if( rssi < -60 || flag == FALSE) {
		send_light_data(TEST_R_BRGHT, 0, 0, 0, 0);
		PR_ERR("--------------------the wifi rssi too low!------------");		
		return;
	}

       op_ret = CreateAndStart(&com_iic_thread,com_iic_proc,NULL,1024,TRD_PRIO_2,"cuco_task");
       if(op_ret != OPRT_OK) {
           return op_ret;
       }	


     return TRUE;	
}

VOID light_init(VOID)
{
	OPERATE_RET op_ret;
	
	//app_cfg_set(WCM_OLD, NULL);
	app_cfg_set(WCM_SPCL_AUTOCFG, prod_test);
	
	set_ap_ssid("SmartLife");
	
   	pwm_init(5000, &pwm_val, CHAN_NUM, io_info);
	
	GPIO_ConfigTypeDef config0 = {BIT(PWM_0_OUT_IO_NUM), GPIO_Mode_Output, GPIO_PullUp_EN, GPIO_PIN_INTR_DISABLE};
	gpio_config(&config0);
	GPIO_ConfigTypeDef config1 = {BIT(PWM_1_OUT_IO_NUM), GPIO_Mode_Output, GPIO_PullUp_EN, GPIO_PIN_INTR_DISABLE};
	gpio_config(&config1);
	GPIO_ConfigTypeDef config2 = {BIT(PWM_2_OUT_IO_NUM), GPIO_Mode_Output, GPIO_PullUp_EN, GPIO_PIN_INTR_DISABLE};
	gpio_config(&config2);
	GPIO_ConfigTypeDef config3 = {BIT(PWM_3_OUT_IO_NUM), GPIO_Mode_Output, GPIO_PullUp_EN, GPIO_PIN_INTR_DISABLE};
	gpio_config(&config3);
	//GPIO_ConfigTypeDef config4 = {BIT(PWM_4_OUT_IO_NUM), GPIO_Mode_Output, GPIO_PullUp_EN, GPIO_PIN_INTR_DISABLE};
	//gpio_config(&config4);
	
	op_ret = msf_register_module(DEVICE_MOD, DEVICE_PART);
	if(op_ret != OPRT_OK && \
		op_ret != OPRT_PSM_E_EXIST) {
		PR_ERR("msf_register_module err:%d",op_ret);
		return;
	}

	op_ret = CreateMutexAndInit(&flash_scene_handle.mutex);
    if(op_ret != OPRT_OK) {
        return ;
    }
	
	gra_change.gra_key_sem = CreateSemaphore();
	if(NULL == gra_change.gra_key_sem){
		return ;
	}
    op_ret = InitSemaphore(gra_change.gra_key_sem,0,1);
    if(OPRT_OK != op_ret) {
        return ;
    }
	
   	hw_timer_init(1,hw_test_timer_cb);
	//1ms ?¡ì?????
	hw_timer_arm(2000);
    hw_timer_disable();
	
   op_ret = CreateAndStart(&gra_change.gra_thread, light_gra_change, NULL,1024+512,TRD_PRIO_2,"gra_task");
   if(op_ret != OPRT_OK) {
       return ;
   }

   op_ret = CreateAndStart(&flash_scene_handle.flash_scene_thread, flash_scene_change, NULL,1024+512,TRD_PRIO_2,"flash_scene_task");
	if(op_ret != OPRT_OK) {
        return ;
    }
}

STATIC VOID reset_fsw_cnt_cb(UINT timerID,PVOID pTimerArg)
{
    PR_DEBUG("%s",__FUNCTION__);

    set_reset_cnt(0);
}

VOID dev_reset_judge(VOID)
{
	OPERATE_RET op_ret;
	struct rst_info *rst_inf = system_get_rst_info();
	PR_DEBUG("rst_inf:%d",rst_inf->reason);
	if((rst_inf->reason == REASON_DEFAULT_RST) || (rst_inf->reason == REASON_EXT_SYS_RST)) {
		set_reset_cnt(get_reset_cnt()+1);
	    TIMER_ID timer;
		op_ret = sys_add_timer(reset_fsw_cnt_cb,NULL,&timer);
		if(OPRT_OK != op_ret) {
			PR_ERR("reset_fsw_cnt timer add err:%d",op_ret);
		    return;
		}else {
		    sys_start_timer(timer,5000,TIMER_ONCE);
		}
	}
}

VOID app_init(VOID)
{
    set_console(FALSE);
    print_port_full_init(UART0,BIT_RATE_115200,UART_WordLength_8b,USART_Parity_None,USART_StopBits_1);
           
   //set gpio default status
   light_init();
   reset_light_sta();
   dev_reset_judge();
}

/***********************************************************
*  Function: device_init
*  Input: 
*  Output: 
*  Return: 
***********************************************************/
OPERATE_RET device_init(VOID)
{
    OPERATE_RET op_ret;
	print_port_full_init(UART0,BIT_RATE_115200,UART_WordLength_8b,USART_Parity_None,USART_StopBits_1);
		 

    op_ret = smart_frame_init(device_cb,USER_SW_VER);
    if(op_ret != OPRT_OK) {
        PR_ERR("smart_frame_init failed");
    }
	
	op_ret = device_differ_init();
    if(op_ret != OPRT_OK) {
        return op_ret;
    }
	
    DEV_DESC_IF_S def_dev_if;
    strcpy(def_dev_if.product_key,PRODECT_KEY);
    strcpy(def_dev_if.sw_ver,SW_VER);
    def_dev_if.ability = DEV_SINGLE;
    op_ret = single_wf_device_init_pk(&def_dev_if);
	print_port_full_init(UART0,BIT_RATE_115200,UART_WordLength_8b,USART_Parity_None,USART_StopBits_1);
	
    return op_ret;
}

STATIC OPERATE_RET device_differ_init(VOID)
{
    OPERATE_RET op_ret;    

	if(get_reset_cnt() >= 3){
		set_reset_cnt(0);
		single_dev_reset_factory();
	}

	if(UN_ACTIVE == get_gw_status()){
		set_default_dp_data();
	}
	
    //¶ÁÈ¡´æ´¢µÄÐÅÏ¢
    dev_inf_get();

    op_ret = sys_add_timer(wfl_timer_cb,NULL,&timer);
    if(OPRT_OK != op_ret) {
        return op_ret;
    }else {
        sys_start_timer(timer,300,TIMER_CYCLE);
    }
	
	op_ret = sys_add_timer(idu_timer_cb,NULL,&timer_init_dpdata);
    if(OPRT_OK != op_ret) {
        return op_ret;
    }else {
        sys_start_timer(timer_init_dpdata,300,TIMER_CYCLE);
    }
	
    op_ret = sys_add_timer(wf_direct_timer_cb,NULL,&wf_stat_dir);
    if(OPRT_OK != op_ret) {
        return op_ret;
    }

	op_ret = sys_add_timer(data_save_timer_cb,NULL,&data_save_timer);
    if(OPRT_OK != op_ret) {
        return op_ret;
    }

    op_ret = CreateAndStart(&com_iic_thread,com_iic_proc,NULL,1024,TRD_PRIO_2,"cuco_task");
    if(op_ret != OPRT_OK) {
        return op_ret;
    }	

    return OPRT_OK;
}
//mqttÁ¬½Ó³É¹¦ºó·¢ËÍ³õÊ¼»¯ÏûÏ¢
STATIC VOID idu_timer_cb(UINT timerID,PVOID pTimerArg)
{
    if(get_gw_mq_conn_stat()== TRUE){
        init_upload_proc();
        sys_stop_timer(timer_init_dpdata);
    }else {
        ;
    }    
}

STATIC VOID get_light_data(VOID)
{
    INT i;
	PR_DEBUG("dp_data.WORK_MODE :%d ",dp_data.WORK_MODE );
	switch(dp_data.WORK_MODE){
		case WHITE_MODE:
			light_data.FIN_WHITE_VAL = dp_data.BRIGHT*dp_data.COL_TEMPERATURE/255;
			light_data.FIN_WARM_VAL = dp_data.BRIGHT - light_data.FIN_WHITE_VAL;
			light_data.FIN_RED_VAL = 0;
			light_data.FIN_GREEN_VAL = 0;
			light_data.FIN_BLUE_VAL = 0;
			light_data.RED_VAL = light_data.FIN_RED_VAL/RESO_VAL;
			light_data.GREEN_VAL = light_data.FIN_GREEN_VAL/RESO_VAL;
			light_data.BLUE_VAL = light_data.FIN_BLUE_VAL/RESO_VAL;
			light_data.WHITE_VAL = light_data.FIN_WHITE_VAL/RESO_VAL;
			light_data.WARM_VAL = light_data.FIN_WARM_VAL/RESO_VAL;
			PR_DEBUG("dp_data.BRIGHT :%d ",dp_data.BRIGHT );
			PR_DEBUG("dp_data.COL_TEMPERATURE :%d ",dp_data.COL_TEMPERATURE );
			PR_DEBUG("dp_data.FIN_WARM_VAL :%d ",light_data.FIN_WARM_VAL );
			//PR_DEBUG("R:%d G:%d B:%d W:%d WW:%d",light_data.FIN_RED_VAL, light_data.FIN_GREEN_VAL, light_data.FIN_BLUE_VAL, light_data.FIN_WHITE_VAL, light_data.FIN_WARM_VAL);
				

			
			break;
		case COLOUR_MODE:
			light_data.FIN_RED_VAL = gamma_8_correction[string_combine_byte(asc2hex(dp_data.COLOUR_DATA[0]), asc2hex(dp_data.COLOUR_DATA[1]))];//gamma_9_correction
			light_data.FIN_GREEN_VAL = gamma_6_70_correction[string_combine_byte(asc2hex(dp_data.COLOUR_DATA[2]), asc2hex(dp_data.COLOUR_DATA[3]))];//gamma_5_correction
			light_data.FIN_BLUE_VAL = gamma_6_70_correction[string_combine_byte(asc2hex(dp_data.COLOUR_DATA[4]), asc2hex(dp_data.COLOUR_DATA[5]))];//gamma_6_70_correction
			light_data.FIN_WHITE_VAL = 0;
			light_data.FIN_WARM_VAL = 0;
			light_data.RED_VAL = light_data.FIN_RED_VAL/RESO_VAL;
			light_data.GREEN_VAL = light_data.FIN_GREEN_VAL/RESO_VAL;
			light_data.BLUE_VAL = light_data.FIN_BLUE_VAL/RESO_VAL;
			light_data.WHITE_VAL = light_data.FIN_WHITE_VAL/RESO_VAL;
			light_data.WARM_VAL = light_data.FIN_WARM_VAL/RESO_VAL;
			light_data.HUE = string_combine_short(asc2hex(dp_data.COLOUR_DATA[6]), asc2hex(dp_data.COLOUR_DATA[7]), asc2hex(dp_data.COLOUR_DATA[8]), asc2hex(dp_data.COLOUR_DATA[9]));
			light_data.SATURATION = string_combine_byte(asc2hex(dp_data.COLOUR_DATA[10]), asc2hex(dp_data.COLOUR_DATA[11]));
			light_data.VALUE= string_combine_byte(asc2hex(dp_data.COLOUR_DATA[12]), asc2hex(dp_data.COLOUR_DATA[13]));
			PR_ERR("light_data.VALUE: %x", light_data.VALUE);

			break;
		case SCENE_MODE:
			light_data.FIN_RED_VAL = string_combine_byte(asc2hex(dp_data.SCENE_DATA[0]), asc2hex(dp_data.SCENE_DATA[1]));
			light_data.FIN_GREEN_VAL = string_combine_byte(asc2hex(dp_data.SCENE_DATA[2]), asc2hex(dp_data.SCENE_DATA[3]));
			light_data.FIN_BLUE_VAL = string_combine_byte(asc2hex(dp_data.SCENE_DATA[4]), asc2hex(dp_data.SCENE_DATA[5]));
			light_data.FIN_WHITE_VAL = 0;
			light_data.FIN_WARM_VAL = 0;
			light_data.RED_VAL = light_data.FIN_RED_VAL/RESO_VAL;
			light_data.GREEN_VAL = light_data.FIN_GREEN_VAL/RESO_VAL;
			light_data.BLUE_VAL = light_data.FIN_BLUE_VAL/RESO_VAL;
			light_data.WHITE_VAL = light_data.FIN_WHITE_VAL/RESO_VAL;
			light_data.WARM_VAL = light_data.FIN_WARM_VAL/RESO_VAL;
			light_data.HUE = string_combine_short(asc2hex(dp_data.SCENE_DATA[6]), asc2hex(dp_data.SCENE_DATA[7]), asc2hex(dp_data.SCENE_DATA[8]), asc2hex(dp_data.SCENE_DATA[9]));
			light_data.SATURATION = string_combine_byte(asc2hex(dp_data.SCENE_DATA[10]), asc2hex(dp_data.SCENE_DATA[11]));
			light_data.VALUE= string_combine_byte(asc2hex(dp_data.SCENE_DATA[12]), asc2hex(dp_data.SCENE_DATA[13]));
			break;
		case ROUGUANG_SCENE:
			flash_light_data.BRIGHT = string_combine_byte(asc2hex(dp_data.ROUGUANG_SCENE_DATA[0]), asc2hex(dp_data.ROUGUANG_SCENE_DATA[1]));
			flash_light_data.COL_TEMP = string_combine_byte(asc2hex(dp_data.ROUGUANG_SCENE_DATA[2]), asc2hex(dp_data.ROUGUANG_SCENE_DATA[3]));
			flash_light_data.SPEED = string_combine_byte(asc2hex(dp_data.ROUGUANG_SCENE_DATA[4]), asc2hex(dp_data.ROUGUANG_SCENE_DATA[5]));
			flash_light_data.NUM = string_combine_byte(asc2hex(dp_data.ROUGUANG_SCENE_DATA[6]), asc2hex(dp_data.ROUGUANG_SCENE_DATA[7]));
			for(i=0; i<flash_light_data.NUM; i++){
				flash_light_data.data_group[i].RED_VAL = string_combine_byte(asc2hex(dp_data.ROUGUANG_SCENE_DATA[i*6+8]), asc2hex(dp_data.ROUGUANG_SCENE_DATA[i*6+9]));
				flash_light_data.data_group[i].GREEN_VAL = string_combine_byte(asc2hex(dp_data.ROUGUANG_SCENE_DATA[i*6+10]), asc2hex(dp_data.ROUGUANG_SCENE_DATA[i*6+11]));
				flash_light_data.data_group[i].BLUE_VAL = string_combine_byte(asc2hex(dp_data.ROUGUANG_SCENE_DATA[i*6+12]), asc2hex(dp_data.ROUGUANG_SCENE_DATA[i*6+13]));
			}
			light_data.FIN_WHITE_VAL = 0;
			light_data.WHITE_VAL = 0;
			light_data.FIN_WARM_VAL = 0;
			light_data.WARM_VAL = 0;
			break;
		case BINFENG_SCENE:
			flash_light_data.BRIGHT = string_combine_byte(asc2hex(dp_data.BINFENG_SCENE_DATA[0]), asc2hex(dp_data.BINFENG_SCENE_DATA[1]));
			flash_light_data.COL_TEMP = string_combine_byte(asc2hex(dp_data.BINFENG_SCENE_DATA[2]), asc2hex(dp_data.BINFENG_SCENE_DATA[3]));
			flash_light_data.SPEED = string_combine_byte(asc2hex(dp_data.BINFENG_SCENE_DATA[4]), asc2hex(dp_data.BINFENG_SCENE_DATA[5]));
			flash_light_data.NUM = string_combine_byte(asc2hex(dp_data.BINFENG_SCENE_DATA[6]), asc2hex(dp_data.BINFENG_SCENE_DATA[7]));
			for(i=0; i<flash_light_data.NUM; i++){
				flash_light_data.data_group[i].RED_VAL = string_combine_byte(asc2hex(dp_data.BINFENG_SCENE_DATA[i*6+8]), asc2hex(dp_data.BINFENG_SCENE_DATA[i*6+9]));
				flash_light_data.data_group[i].GREEN_VAL = string_combine_byte(asc2hex(dp_data.BINFENG_SCENE_DATA[i*6+10]), asc2hex(dp_data.BINFENG_SCENE_DATA[i*6+11]));
				flash_light_data.data_group[i].BLUE_VAL = string_combine_byte(asc2hex(dp_data.BINFENG_SCENE_DATA[i*6+12]), asc2hex(dp_data.BINFENG_SCENE_DATA[i*6+13]));
			}
			light_data.FIN_WHITE_VAL = 0;
			light_data.WHITE_VAL = 0;
			light_data.FIN_WARM_VAL = 0;
			light_data.WARM_VAL = 0;
			break;
		case XUANCAI_SCENE:
			flash_light_data.BRIGHT = string_combine_byte(asc2hex(dp_data.XUANCAI_SCENE_DATA[0]), asc2hex(dp_data.XUANCAI_SCENE_DATA[1]));
			flash_light_data.COL_TEMP = string_combine_byte(asc2hex(dp_data.XUANCAI_SCENE_DATA[2]), asc2hex(dp_data.XUANCAI_SCENE_DATA[3]));
			flash_light_data.SPEED = string_combine_byte(asc2hex(dp_data.XUANCAI_SCENE_DATA[4]), asc2hex(dp_data.XUANCAI_SCENE_DATA[5]));
			flash_light_data.NUM = string_combine_byte(asc2hex(dp_data.XUANCAI_SCENE_DATA[6]), asc2hex(dp_data.XUANCAI_SCENE_DATA[7]));
			for(i=0; i<flash_light_data.NUM; i++){
				flash_light_data.data_group[i].RED_VAL = string_combine_byte(asc2hex(dp_data.XUANCAI_SCENE_DATA[i*6+8]), asc2hex(dp_data.XUANCAI_SCENE_DATA[i*6+9]));
				flash_light_data.data_group[i].GREEN_VAL = string_combine_byte(asc2hex(dp_data.XUANCAI_SCENE_DATA[i*6+10]), asc2hex(dp_data.XUANCAI_SCENE_DATA[i*6+11]));
				flash_light_data.data_group[i].BLUE_VAL = string_combine_byte(asc2hex(dp_data.XUANCAI_SCENE_DATA[i*6+12]), asc2hex(dp_data.XUANCAI_SCENE_DATA[i*6+13]));
			}
			light_data.FIN_WHITE_VAL = 0;
			light_data.WHITE_VAL = 0;
			light_data.FIN_WARM_VAL = 0;
			light_data.WARM_VAL = 0;
			break;
		case BANLAN_SCENE:
			flash_light_data.BRIGHT = string_combine_byte(asc2hex(dp_data.BANLAN_SCENE_DATA[0]), asc2hex(dp_data.BANLAN_SCENE_DATA[1]));
			flash_light_data.COL_TEMP = string_combine_byte(asc2hex(dp_data.BANLAN_SCENE_DATA[2]), asc2hex(dp_data.BANLAN_SCENE_DATA[3]));
			flash_light_data.SPEED = string_combine_byte(asc2hex(dp_data.BANLAN_SCENE_DATA[4]), asc2hex(dp_data.BANLAN_SCENE_DATA[5]));
			flash_light_data.NUM = string_combine_byte(asc2hex(dp_data.BANLAN_SCENE_DATA[6]), asc2hex(dp_data.BANLAN_SCENE_DATA[7]));
			for(i=0; i<flash_light_data.NUM; i++){
				flash_light_data.data_group[i].RED_VAL = string_combine_byte(asc2hex(dp_data.BANLAN_SCENE_DATA[i*6+8]), asc2hex(dp_data.BANLAN_SCENE_DATA[i*6+9]));
				flash_light_data.data_group[i].GREEN_VAL = string_combine_byte(asc2hex(dp_data.BANLAN_SCENE_DATA[i*6+10]), asc2hex(dp_data.BANLAN_SCENE_DATA[i*6+11]));
				flash_light_data.data_group[i].BLUE_VAL = string_combine_byte(asc2hex(dp_data.BANLAN_SCENE_DATA[i*6+12]), asc2hex(dp_data.BANLAN_SCENE_DATA[i*6+13]));
			}
			light_data.FIN_WHITE_VAL = 0;
			light_data.WHITE_VAL = 0;
			light_data.FIN_WARM_VAL = 0;
			light_data.WARM_VAL = 0;
			break;
		default:
			break;
	}

}
STATIC VOID set_default_dp_data(VOID)
{
	dp_data.SWITCH = TRUE;
	dp_data.WORK_MODE = WHITE_MODE;
	dp_data.BRIGHT = BRIGHT_INIT_VALUE;
	dp_data.COL_TEMPERATURE = COL_TEMP_INIT_VALUE;
	memcpy(dp_data.COLOUR_DATA, "1900000000ff19", 14);
	memcpy(dp_data.SCENE_DATA, "00ff0000000000", 14);
	memcpy(dp_data.ROUGUANG_SCENE_DATA, "ffff500100ff00", 14);
	memcpy(dp_data.BINFENG_SCENE_DATA, "ffff8003ff000000ff000000ff000000000000000000", 44);
	memcpy(dp_data.XUANCAI_SCENE_DATA, "ffff5001ff0000", 14);
	memcpy(dp_data.BANLAN_SCENE_DATA, "ffff0505ff000000ff00ffff00ff00ff0000ff000000", 44);
	dev_inf_set();
}

STATIC VOID start_gra_change(TIME_MS delay_time)
{
	gra_change.scale_flag = FALSE;
	hw_timer_arm(2000*delay_time);
    hw_timer_enable();
}

STATIC VOID hw_test_timer_cb(void)
{
	PostSemaphore(gra_change.gra_key_sem);
}


STATIC VOID light_gra_change(PVOID pArg)
{
	signed int delata_red = 0;
	signed int delata_green = 0;
    signed int delata_blue = 0;
    signed int delata_white = 0;
	signed int delata_warm = 0;
    UCHAR MAX_VALUE;
	STATIC FLOAT r_scale;
	STATIC FLOAT g_scale;
	STATIC FLOAT b_scale;
	STATIC FLOAT w_scale;
	STATIC FLOAT ww_scale;
	UINT RED_GRA_STEP = 1;
	UINT GREEN_GRA_STEP = 1;
	UINT BLUE_GRA_STEP = 1;
	UINT WHITE_GRA_STEP = 1;
	UINT WARM_GRA_STEP = 1;

	while(1)
	{
	    WaitSemaphore(gra_change.gra_key_sem);
		if(light_data.WHITE_VAL != light_data.LAST_WHITE_VAL || light_data.WARM_VAL != light_data.LAST_WARM_VAL || light_data.RED_VAL != light_data.LAST_RED_VAL ||\
			light_data.GREEN_VAL != light_data.LAST_GREEN_VAL || light_data.BLUE_VAL != light_data.LAST_BLUE_VAL)
	    {
			delata_red = light_data.RED_VAL - light_data.LAST_RED_VAL;
			delata_green = light_data.GREEN_VAL - light_data.LAST_GREEN_VAL;
			delata_blue = light_data.BLUE_VAL - light_data.LAST_BLUE_VAL;
			delata_white = light_data.WHITE_VAL - light_data.LAST_WHITE_VAL;
			delata_warm = light_data.WARM_VAL - light_data.LAST_WARM_VAL;
			MAX_VALUE = get_max_value(ABS(delata_red), ABS(delata_green), ABS(delata_blue), ABS(delata_white), ABS(delata_warm));
			//PR_DEBUG("MAX: %d", MAX_VALUE);
			if(gra_change.scale_flag == FALSE){	
				r_scale = ABS(delata_red)/1.0/MAX_VALUE;
				g_scale = ABS(delata_green)/1.0/MAX_VALUE;
				b_scale = ABS(delata_blue)/1.0/MAX_VALUE;
				w_scale = ABS(delata_white)/1.0/MAX_VALUE;
				ww_scale = ABS(delata_warm)/1.0/MAX_VALUE;
				gra_change.scale_flag = TRUE;
			}
			if(MAX_VALUE == ABS(delata_red)){
				RED_GRA_STEP = 1;
			}else{
				RED_GRA_STEP =  ABS(delata_red) - MAX_VALUE*r_scale;
			}
			if(MAX_VALUE == ABS(delata_green)){
				GREEN_GRA_STEP = 1;
			}else{
				GREEN_GRA_STEP =  ABS(delata_green) - MAX_VALUE*g_scale;
			}
			if(MAX_VALUE == ABS(delata_blue)){
				BLUE_GRA_STEP = 1;
			}else{
				BLUE_GRA_STEP =  ABS(delata_blue) - MAX_VALUE*b_scale;
			}
			if(MAX_VALUE == ABS(delata_white)){
				WHITE_GRA_STEP = 1;
			}else{
				WHITE_GRA_STEP =  ABS(delata_white) - MAX_VALUE*w_scale;
			}
			if(MAX_VALUE == ABS(delata_warm)){
				WARM_GRA_STEP = 1;
			}else{
				WARM_GRA_STEP =  ABS(delata_warm) - MAX_VALUE*ww_scale;
			}

			if(delata_red != 0){
			    if(ABS(delata_red) < RED_GRA_STEP)
			    { 
					 light_data.LAST_RED_VAL += delata_red;
				}else{
					if(delata_red < 0)
						light_data.LAST_RED_VAL -= RED_GRA_STEP;
					else
						light_data.LAST_RED_VAL += RED_GRA_STEP;
				}
			}
			if(delata_green != 0){
			    if(ABS(delata_green) < GREEN_GRA_STEP)
			    { 
					 light_data.LAST_GREEN_VAL += delata_green;
				}else{
					if(delata_green < 0)
						light_data.LAST_GREEN_VAL -= GREEN_GRA_STEP;
					else
						light_data.LAST_GREEN_VAL += GREEN_GRA_STEP;
				}
			}
			if(delata_blue != 0){
			    if(ABS(delata_blue) < BLUE_GRA_STEP)
			    { 
					 light_data.LAST_BLUE_VAL += delata_blue;
				}else{
					if(delata_blue < 0)
						light_data.LAST_BLUE_VAL -= BLUE_GRA_STEP;
					else
						light_data.LAST_BLUE_VAL += BLUE_GRA_STEP;
				}
			}
			if(delata_white != 0){
			    if(ABS(delata_white) < WHITE_GRA_STEP)
			    { 
					 light_data.LAST_WHITE_VAL += delata_white;
				}else{
					if(delata_white < 0)
						light_data.LAST_WHITE_VAL -= WHITE_GRA_STEP;
					else
						light_data.LAST_WHITE_VAL += WHITE_GRA_STEP;
				}
			}
			if(delata_warm != 0){
			    if(ABS(delata_warm) < WARM_GRA_STEP)
			    { 
					 light_data.LAST_WARM_VAL += delata_warm;
				}else{
					if(delata_warm < 0)
						light_data.LAST_WARM_VAL -= WARM_GRA_STEP;
					else
						light_data.LAST_WARM_VAL += WARM_GRA_STEP;
				}
			}
			//PR_DEBUG("r:%d g:%d b:%d w:%d ",light_data.LAST_RED_VAL,light_data.LAST_GREEN_VAL,light_data.LAST_BLUE_VAL,light_data.LAST_WHITE_VAL);
			if(dp_data.SWITCH == FALSE && (dp_data.WORK_MODE == ROUGUANG_SCENE || dp_data.WORK_MODE == BANLAN_SCENE)){
				;
			}else{
				MutexLock(flash_scene_handle.mutex);
				if(dp_data.WORK_MODE != BINFENG_SCENE && dp_data.WORK_MODE != XUANCAI_SCENE)
					send_light_data(light_data.LAST_RED_VAL*RESO_VAL, light_data.LAST_GREEN_VAL*RESO_VAL, light_data.LAST_BLUE_VAL*RESO_VAL, light_data.LAST_WHITE_VAL*RESO_VAL, light_data.LAST_WARM_VAL*RESO_VAL);
				PR_DEBUG("R:%d G:%d B:%d W:%d WW:%d",light_data.FIN_RED_VAL, light_data.FIN_GREEN_VAL, light_data.FIN_BLUE_VAL, light_data.FIN_WHITE_VAL, light_data.FIN_WARM_VAL);
				MutexUnLock(flash_scene_handle.mutex);
			}
		}else{
			if(dp_data.WORK_MODE != ROUGUANG_SCENE && dp_data.WORK_MODE != BANLAN_SCENE){		
				send_light_data(light_data.FIN_RED_VAL, light_data.FIN_GREEN_VAL, light_data.FIN_BLUE_VAL, light_data.FIN_WHITE_VAL, light_data.FIN_WARM_VAL);
				PR_DEBUG("R:%d G:%d B:%d W:%d WW:%d",light_data.FIN_RED_VAL, light_data.FIN_GREEN_VAL, light_data.FIN_BLUE_VAL, light_data.FIN_WHITE_VAL, light_data.FIN_WARM_VAL);
				hw_timer_disable();
			}
		}
	}
}

STATIC VOID flash_scene_change(PVOID pArg)
{
	BYTE require_time;
	while(1)
	{
		MutexLock(flash_scene_handle.mutex);
		if(dp_data.SWITCH == TRUE && flash_scene_flag == TRUE ){
			switch(dp_data.WORK_MODE){
				case ROUGUANG_SCENE:
				case BANLAN_SCENE:
					if(dp_data.WORK_MODE == ROUGUANG_SCENE){
						require_time = flash_light_data.SPEED*2+15;
					}else{
						require_time = flash_light_data.SPEED+10;
					}
					if(sta_cha_flag == TRUE){
						irq_cnt = require_time;
						sta_cha_flag = FALSE;
					}else{
						irq_cnt ++;
					}
					if(irq_cnt >= require_time){
						if(flash_light_data.NUM == 1){
							if(flash_dir == 0){
								flash_dir = 1;
								light_data.FIN_RED_VAL = flash_light_data.data_group[0].RED_VAL;
								light_data.FIN_GREEN_VAL = flash_light_data.data_group[0].GREEN_VAL;
								light_data.FIN_BLUE_VAL = flash_light_data.data_group[0].BLUE_VAL;
								light_data.RED_VAL = light_data.FIN_RED_VAL/RESO_VAL;
								light_data.GREEN_VAL = light_data.FIN_GREEN_VAL/RESO_VAL;
								light_data.BLUE_VAL = light_data.FIN_BLUE_VAL/RESO_VAL;
							}else{
								flash_dir = 0;
								light_data.FIN_RED_VAL = 0;
								light_data.FIN_GREEN_VAL = 0;
								light_data.FIN_BLUE_VAL = 0;
								light_data.RED_VAL = 0;
								light_data.GREEN_VAL = 0;
								light_data.BLUE_VAL = 0;
							}
						}else{
							light_data.FIN_RED_VAL = flash_light_data.data_group[num_cnt].RED_VAL;
							light_data.FIN_GREEN_VAL = flash_light_data.data_group[num_cnt].GREEN_VAL;
							light_data.FIN_BLUE_VAL = flash_light_data.data_group[num_cnt].BLUE_VAL;
							light_data.RED_VAL = light_data.FIN_RED_VAL/RESO_VAL;
							light_data.GREEN_VAL = light_data.FIN_GREEN_VAL/RESO_VAL;
							light_data.BLUE_VAL = light_data.FIN_BLUE_VAL/RESO_VAL;
						}
						num_cnt ++;
						if(num_cnt == flash_light_data.NUM)
							 num_cnt = 0;
						irq_cnt = 0;

						if(dp_data.WORK_MODE == ROUGUANG_SCENE){
							start_gra_change(flash_light_data.SPEED*2/5+2+flash_light_data.SPEED*(255-flash_light_data.BRIGHT)/100/4);
						}else{
							start_gra_change(flash_light_data.SPEED/7+3);
						}
					}
					break;
				case BINFENG_SCENE:
				case XUANCAI_SCENE:
					if(dp_data.WORK_MODE == BINFENG_SCENE){
						require_time = flash_light_data.SPEED+7;
					}else{
						require_time = flash_light_data.SPEED+2;
					}
					if(sta_cha_flag == TRUE){
						irq_cnt = require_time;
						sta_cha_flag = FALSE;
					}else{
						irq_cnt ++;
					}
					if(irq_cnt >= require_time){
						if(flash_light_data.NUM == 1){
							if(flash_dir == 0){
								if(dp_data.SWITCH == TRUE)
									send_light_data(flash_light_data.data_group[0].RED_VAL, flash_light_data.data_group[0].GREEN_VAL, \
												flash_light_data.data_group[0].BLUE_VAL, 0, 0);
								flash_dir = 1;
							}else{
								flash_dir = 0;
								if(dp_data.SWITCH == TRUE)
									send_light_data(0, 0, 0, 0, 0);
							}
						}else{
							if(dp_data.SWITCH == TRUE)
								send_light_data(flash_light_data.data_group[num_cnt].RED_VAL, flash_light_data.data_group[num_cnt].GREEN_VAL, \
												flash_light_data.data_group[num_cnt].BLUE_VAL, 0, 0);
						}
						num_cnt ++;
						if(num_cnt == flash_light_data.NUM)
							 num_cnt = 0;
						irq_cnt = 0;
					}
					
					break;
				default:
					break;
			}
		}
		MutexUnLock(flash_scene_handle.mutex);
		SystemSleep(20);
	}
}

STATIC VOID wfl_timer_cb(UINT timerID,PVOID pTimerArg)
{
    OPERATE_RET op_ret;
    STATIC UINT last_wf_stat = 0xffffffff;
	STATIC BOOL config_flag = FALSE;
    GW_WIFI_STAT_E wf_stat = get_wf_gw_status();
    if(last_wf_stat != wf_stat) {
        PR_DEBUG("wf_stat:%d",wf_stat);
		PR_DEBUG("size:%d",system_get_free_heap_size());
        switch(wf_stat) {
            case STAT_UNPROVISION: {
				config_flag = TRUE;
				flash_scene_flag = FALSE;
              sys_start_timer(wf_stat_dir, 250, TIMER_CYCLE);
            }
            break;  
            
            case STAT_AP_STA_UNCONN: {
				 config_flag = TRUE;
				 flash_scene_flag = FALSE;
                 sys_start_timer(wf_stat_dir, 1500, TIMER_CYCLE);                    
            }
            break;
            
            case STAT_STA_UNCONN:
			case STAT_LOW_POWER:
				 if(IsThisSysTimerRun(wf_stat_dir)){
				 	sys_stop_timer(wf_stat_dir);
				 }
				 if(wf_stat == STAT_STA_UNCONN) {
				 	PR_DEBUG("config_flag:%d",config_flag);
				 	if(config_flag == TRUE){
						config_flag = FALSE;
						reset_light_sta();
					}
				    PR_DEBUG("STAT_STA_UNCONN");
                 }else {
                	send_light_data(0, 0, 0, BRIGHT_INIT_VALUE, 0);
                    PR_DEBUG("LOW POWER");
                 }
                 break;
            case STAT_STA_CONN: 
            case STAT_AP_STA_CONN:{
            }
            break;
        }
        last_wf_stat = wf_stat;
    }
}

STATIC VOID wf_direct_timer_cb(UINT timerID,PVOID pTimerArg)
{
    STATIC INT flag = 0; 
    if(flag == 0) {
        flag = 1;
		send_light_data(0, 0, 0, 0, 0);
    }else {
        flag = 0;
		send_light_data(0, 0, 0, BRIGHT_INIT_VALUE, 0);
    }
}

STATIC OPERATE_RET dev_inf_get(VOID)
{
    OPERATE_RET op_ret;
	
	UCHAR *buf;

	buf = Malloc(378);
	
	if(NULL == buf) {
		PR_ERR("Malloc failed");
		goto ERR_EXT;
	}

	op_ret = msf_get_single(DEVICE_MOD, DP_DATA_KEY, buf, 378);
	if(OPRT_OK != op_ret){
		PR_ERR("msf_get_single failed");
		Free(buf);
		goto ERR_EXT;
	}
	PR_DEBUG("buf:%s",buf);
	
	cJSON *root = NULL;
	root = cJSON_Parse(buf);
	if(NULL == root) {
		Free(buf);
		return OPRT_CJSON_PARSE_ERR;
	}
	Free(buf);

	cJSON *json;
	
	json = cJSON_GetObjectItem(root,"switch");
	if(NULL == json) {
		dp_data.SWITCH = TRUE;
	}else{
		dp_data.SWITCH = json->valueint;
	}
	if(FALSE == dp_data.SWITCH) {
		  struct rst_info *rst_inf = system_get_rst_info();
		  //PR_DEBUG("rst_inf->reaso is %d",rst_inf->reason);
		  if((rst_inf->reason == REASON_DEFAULT_RST)  || (rst_inf->reason == REASON_EXT_SYS_RST)) {
			  dp_data.SWITCH = TRUE;
		  }
	 } 
	
	json = cJSON_GetObjectItem(root,"work_mode");
	if(NULL == json) {
		dp_data.WORK_MODE = WHITE_MODE;
	}else{
		dp_data.WORK_MODE = json->valueint;
	}
	json = cJSON_GetObjectItem(root,"bright");
	if(NULL == json) {
		dp_data.BRIGHT = BRIGHT_INIT_VALUE;
	}else{
		dp_data.BRIGHT = json->valueint;
	}
	json = cJSON_GetObjectItem(root,"temper");
	if(NULL == json) {
		dp_data.COL_TEMPERATURE= COL_TEMP_INIT_VALUE;
	}else{
		dp_data.COL_TEMPERATURE= json->valueint;
	}
	json = cJSON_GetObjectItem(root,"colour_data");
	if(NULL == json) {
		memcpy(dp_data.COLOUR_DATA, "1900000000ff19", 14);
	}else{
		memcpy(dp_data.COLOUR_DATA, json->valuestring, 14);
	}
	json = cJSON_GetObjectItem(root,"scene_data");
	if(NULL == json) {
		memcpy(dp_data.SCENE_DATA, "00ff0000000000", 14);
	}else{
		memcpy(dp_data.SCENE_DATA, json->valuestring, 14);
	}
	json = cJSON_GetObjectItem(root,"rouguang_scene_data");
	if(NULL == json) {
		memcpy(dp_data.ROUGUANG_SCENE_DATA, "ffff500100ff00", 14);
	}else{
		memcpy(dp_data.ROUGUANG_SCENE_DATA, json->valuestring, 14);
	}
	json = cJSON_GetObjectItem(root,"binfeng_scene_data");
	if(NULL == json) {
		memcpy(dp_data.BINFENG_SCENE_DATA, "ffff8003ff000000ff000000ff000000000000000000", 44);
	}else{
		memcpy(dp_data.BINFENG_SCENE_DATA, json->valuestring, 44);
	}
	json = cJSON_GetObjectItem(root,"xuancai_scene_data");
	if(NULL == json) {
		memcpy(dp_data.XUANCAI_SCENE_DATA, "ffff5001ff0000", 14);
	}else{
		memcpy(dp_data.XUANCAI_SCENE_DATA, json->valuestring, 14);
	}
	json = cJSON_GetObjectItem(root,"banlan_scene_data");
	if(NULL == json) {
		memcpy(dp_data.BANLAN_SCENE_DATA, "ffff0505ff000000ff00ffff00ff00ff0000ff000000", 44);
	}else{
		memcpy(dp_data.BANLAN_SCENE_DATA, json->valuestring, 44);
	}
	cJSON_Delete(root);
	return OPRT_OK;
ERR_EXT:	
	set_default_dp_data();
	return OPRT_COM_ERROR;
}

STATIC OPERATE_RET dev_inf_set(VOID)
{
    OPERATE_RET op_ret;
    INT i = 0;
	CHAR *out = NULL;

	cJSON *root_test = NULL;
	root_test = cJSON_CreateObject();

	PR_DEBUG("------------333 come dev_inf_set ---------");	
	
	if(NULL == root_test) {
		PR_ERR("json creat failed");
		goto ERR_EXT;
	}
	//dp_data.WORK_MODE=WHITE_MODE;

	cJSON_AddBoolToObject(root_test, "switch", dp_data.SWITCH);
	cJSON_AddNumberToObject(root_test, "work_mode", dp_data.WORK_MODE);
	cJSON_AddNumberToObject(root_test, "bright", dp_data.BRIGHT);
	//cJSON_AddNumberToObject(root_test, "temper", dp_data.COL_TEMPERATURE);
	cJSON_AddStringToObject(root_test, "colour_data", dp_data.COLOUR_DATA);
	cJSON_AddStringToObject(root_test, "scene_data", dp_data.SCENE_DATA);
	cJSON_AddStringToObject(root_test, "rouguang_scene_data", dp_data.ROUGUANG_SCENE_DATA);
	cJSON_AddStringToObject(root_test, "binfeng_scene_data", dp_data.BINFENG_SCENE_DATA);
	cJSON_AddStringToObject(root_test, "xuancai_scene_data", dp_data.XUANCAI_SCENE_DATA);
	cJSON_AddStringToObject(root_test, "banlan_scene_data", dp_data.BANLAN_SCENE_DATA);
	out=cJSON_PrintUnformatted(root_test);
	cJSON_Delete(root_test);
	if(NULL == out) {
		PR_ERR("cJSON_PrintUnformatted err:");
        Free(out);
		goto ERR_EXT;
	}
	PR_DEBUG("write psm[%s]", out);
	op_ret = msf_set_single(DEVICE_MOD, DP_DATA_KEY, out); 
	if(OPRT_OK != op_ret) {
		PR_ERR("data write psm err: %d",op_ret);
		Free(out);
		goto ERR_EXT;
	}
	Free(out);
    return OPRT_OK;
ERR_EXT:	
	return OPRT_COM_ERROR;
}


//µÚÒ»´ÎÁ¬½Ó·þÎñÆ÷Ê±ÉÏ´«ÐÅÏ¢
STATIC VOID init_upload_proc(VOID)
{
    cJSON *root_test = NULL;
    OPERATE_RET op_ret;
    CHAR *out = NULL;

    if( get_gw_status() != STAT_WORK ) {
		return;
	}

	PR_DEBUG("------------444 come init_upload_proc ---------");	
	

	DEV_CNTL_N_S *dev_cntl = get_single_wf_dev();
	if( dev_cntl == NULL )
		return;
	DP_CNTL_S *dp_cntl =  NULL;
	dp_cntl = &dev_cntl->dp[1];

	
	root_test = cJSON_CreateObject();
	if(NULL == root_test) {
		return;
	}
	cJSON_AddBoolToObject(root_test, "1", dp_data.SWITCH);
	cJSON_AddStringToObject(root_test,"2",ty_get_enum_str(dp_cntl,(UCHAR)dp_data.WORK_MODE));
       cJSON_AddNumberToObject(root_test, "3", dp_data.BRIGHT);
	//cJSON_AddNumberToObject(root_test, "4", dp_data.COL_TEMPERATURE);
	cJSON_AddStringToObject(root_test, "5", dp_data.COLOUR_DATA);
	cJSON_AddStringToObject(root_test, "6", dp_data.SCENE_DATA);
	cJSON_AddStringToObject(root_test, "7", dp_data.ROUGUANG_SCENE_DATA);
	cJSON_AddStringToObject(root_test, "8", dp_data.BINFENG_SCENE_DATA);
	cJSON_AddStringToObject(root_test, "9", dp_data.XUANCAI_SCENE_DATA);
	cJSON_AddStringToObject(root_test, "10", dp_data.BANLAN_SCENE_DATA);

    out=cJSON_PrintUnformatted(root_test);
	cJSON_Delete(root_test);
	if(NULL == out) {
		PR_ERR("cJSON_PrintUnformatted err:");
        Free(out);
		return;
	}
	PR_DEBUG("out[%s]", out);
    op_ret = sf_obj_dp_report(get_single_wf_dev()->dev_if.id,out);
    if(OPRT_OK != op_ret) {
        PR_ERR("sf_obj_dp_report err:%d",op_ret);
        //PR_DEBUG_RAW("%s\r\n",buf);
        Free(out);
        return;
    }
	Free(out);
	return;

}
STATIC VOID work_mode_change(SCENE_MODE_E mode)
{
	if(dp_data.WORK_MODE == mode){
		;
    }else {
    	dp_data.WORK_MODE = mode;
    	MutexLock(flash_scene_handle.mutex);
		get_light_data();
		switch(mode)
		{
			case WHITE_MODE:
			case COLOUR_MODE:
			case SCENE_MODE:
				start_gra_change(NORMAL_DELAY_TIME);
				break;
			case ROUGUANG_SCENE:
			case BINFENG_SCENE:
			case XUANCAI_SCENE:
			case BANLAN_SCENE:
				if(dp_data.WORK_MODE == BINFENG_SCENE || dp_data.WORK_MODE == XUANCAI_SCENE){
					hw_timer_disable();
				}
				sta_cha_flag = TRUE;
				num_cnt = 0;
				flash_dir = 0;
				light_data.LAST_RED_VAL = 0;
				light_data.LAST_GREEN_VAL = 0;
				light_data.LAST_BLUE_VAL = 0;
				light_data.LAST_WHITE_VAL = 0;
				light_data.LAST_WARM_VAL = 0;
				send_light_data(0, 0, 0, 0, 0);
				break;
			default:
				break;	
		}
		MutexUnLock(flash_scene_handle.mutex);
    }
}


STATIC VOID light_switch(bool value)
{
	if (value) {
   	     //¿ªµÆ
	     MutexLock(flash_scene_handle.mutex);
	     dp_data.SWITCH = TRUE;
	     get_light_data();
	     switch(dp_data.WORK_MODE) {
		  case WHITE_MODE:
		  case COLOUR_MODE:
		  case SCENE_MODE:
		  	  MutexUnLock(flash_scene_handle.mutex);
		 	  start_gra_change(NORMAL_DELAY_TIME);
		  	   break;
		  
		 default:
			   sta_cha_flag = TRUE;
			   num_cnt = 0;
			   flash_dir = 0;
			   MutexUnLock(flash_scene_handle.mutex);
			   break;	
	       }
	}
	else {
  		//¹ØµÆ
		dp_data.SWITCH = FALSE;
		MutexLock(flash_scene_handle.mutex);
		light_data.RED_VAL = 0;
		light_data.GREEN_VAL = 0;
		light_data.BLUE_VAL = 0;
		light_data.WHITE_VAL = 0;
		light_data.WARM_VAL = 0;
		light_data.FIN_RED_VAL = 0;
		light_data.FIN_GREEN_VAL = 0;
		light_data.FIN_BLUE_VAL = 0;
		light_data.FIN_WHITE_VAL = 0;
		light_data.FIN_WARM_VAL = 0;
		switch(dp_data.WORK_MODE) {
			case WHITE_MODE:
			case COLOUR_MODE:
			case SCENE_MODE:
				MutexUnLock(flash_scene_handle.mutex);
				start_gra_change(NORMAL_DELAY_TIME);
				break;
			case ROUGUANG_SCENE:
			case BANLAN_SCENE:
				hw_timer_disable();
				send_light_data(light_data.RED_VAL, light_data.GREEN_VAL, light_data.BLUE_VAL, light_data.WHITE_VAL, light_data.WARM_VAL);
				MutexUnLock(flash_scene_handle.mutex);
				break;
			default:
				send_light_data(light_data.RED_VAL, light_data.GREEN_VAL, light_data.BLUE_VAL, light_data.WHITE_VAL, light_data.WARM_VAL);
				MutexUnLock(flash_scene_handle.mutex);
				break;	
		}
	}
}


STATIC VOID sl_datapoint_proc(cJSON *root)
{
	UCHAR dpid, type;
	WORD len, rawlen;
//	PR_DEBUG("remain size:%d",system_get_free_heap_size());
//    PR_ERR("the data is dfddfsfs");
    dpid = atoi(root->string);
	PR_DEBUG("dpid: %d",dpid);

    switch(dpid) {
        case DPID_1:
			switch(root->type) {
			    case cJSON_False: 
			        //¹ØµÆ
			        dp_data.SWITCH = FALSE;
					MutexLock(flash_scene_handle.mutex);
					light_data.RED_VAL = 0;
					light_data.GREEN_VAL = 0;
					light_data.BLUE_VAL = 0;
					light_data.WHITE_VAL = 0;
					light_data.WARM_VAL = 0;
					light_data.FIN_RED_VAL = 0;
					light_data.FIN_GREEN_VAL = 0;
					light_data.FIN_BLUE_VAL = 0;
					light_data.FIN_WHITE_VAL = 0;
					light_data.FIN_WARM_VAL = 0;
					switch(dp_data.WORK_MODE)
					{
						case WHITE_MODE:
						case COLOUR_MODE:
						case SCENE_MODE:
							MutexUnLock(flash_scene_handle.mutex);
							start_gra_change(NORMAL_DELAY_TIME);
							break;
						case ROUGUANG_SCENE:
				        case BANLAN_SCENE:
							hw_timer_disable();
							send_light_data(light_data.RED_VAL, light_data.GREEN_VAL, light_data.BLUE_VAL, light_data.WHITE_VAL, light_data.WARM_VAL);
							MutexUnLock(flash_scene_handle.mutex);
							break;
						default:
							send_light_data(light_data.RED_VAL, light_data.GREEN_VAL, light_data.BLUE_VAL, light_data.WHITE_VAL, light_data.WARM_VAL);
							MutexUnLock(flash_scene_handle.mutex);
							break;	
					}
					
			        break;
			    case cJSON_True:
			        //¿ªµÆ
			        MutexLock(flash_scene_handle.mutex);
					dp_data.SWITCH = TRUE;
					get_light_data();
					switch(dp_data.WORK_MODE)
					{
						case WHITE_MODE:
						case COLOUR_MODE:
						case SCENE_MODE:
							MutexUnLock(flash_scene_handle.mutex);
							start_gra_change(NORMAL_DELAY_TIME);
							break;
						default:
							sta_cha_flag = TRUE;
							num_cnt = 0;
							flash_dir = 0;
							MutexUnLock(flash_scene_handle.mutex);
							break;	
					}
			        break;                
			    default:
			        break;
			}
			break;
		case DPID_2:
			if(dp_data.SWITCH== FALSE){
			   break;
			}
			work_mode_change(ty_get_enum_id(atoi(root->string), root->valuestring));
            break;
        case DPID_3:
			//°×¹âÄ£Ê½ÏÂµÄÊý¾Ý
			if(root->valueint < 11 || root->valueint >255){
				PR_ERR("the data length is wrong: %d", root->valueint);      
			}else {    
				dp_data.BRIGHT = root->valueint;
				
				get_light_data();
			//	PR_DEBUG("R:%d G:%d B:%d W:%d WW:%d",light_data.FIN_RED_VAL, light_data.FIN_GREEN_VAL, light_data.FIN_BLUE_VAL, light_data.FIN_WHITE_VAL, light_data.FIN_WARM_VAL);
				
				start_gra_change(NORMAL_DELAY_TIME);
			} 
			
			break;
		case DPID_4:
			if(root->valueint < 0 || root->valueint >255){
				PR_ERR("the data length is wrong: %d", root->valueint); 	  
		    }else {
				dp_data.COL_TEMPERATURE = root->valueint;
				get_light_data();
				start_gra_change(NORMAL_DELAY_TIME);
			} 
			break;
			
		case DPID_5:
			len = strlen(root->valuestring);
			if(len != 14){
				PR_ERR("the data length is wrong: %d", len);   
		    }else {
		    	memcpy(dp_data.COLOUR_DATA, root->valuestring, len);
				PR_ERR("dp_data.COLOUR_DATA: %s", dp_data.COLOUR_DATA);
				get_light_data();
				start_gra_change(NORMAL_DELAY_TIME);
			} 
		break;
		case DPID_6:
			len = strlen(root->valuestring);
			if(len != 14){
				PR_ERR("the data length is wrong: %d", len);   
		    }else {
		    	memcpy(dp_data.SCENE_DATA, root->valuestring, len);
				get_light_data();
				start_gra_change(NORMAL_DELAY_TIME);
			} 
		break;
		case DPID_7:
			len = strlen(root->valuestring);
			if(len != 14){
				PR_ERR("the data length is wrong: %d", len);   
		    }else {
		    	memcpy(dp_data.ROUGUANG_SCENE_DATA, root->valuestring, len);
				MutexLock(flash_scene_handle.mutex);
				sta_cha_flag = TRUE;
				num_cnt = 0;
				flash_dir = 0;
				get_light_data();
				MutexUnLock(flash_scene_handle.mutex);
			} 
		break;
		case DPID_8:
			len = strlen(root->valuestring);
			if(len > 44){
				PR_ERR("the data length is wrong: %d", len);   
		    }else {
		    	memcpy(dp_data.BINFENG_SCENE_DATA, root->valuestring, len);
				MutexLock(flash_scene_handle.mutex);
				sta_cha_flag = TRUE;
				num_cnt = 0;
				flash_dir = 0;
				get_light_data();
				MutexUnLock(flash_scene_handle.mutex);
			} 
		break;
		case DPID_9:
			len = strlen(root->valuestring);
			if(len != 14){
				PR_ERR("the data length is wrong: %d", len);   
		    }else {
		    	memcpy(dp_data.XUANCAI_SCENE_DATA, root->valuestring, len);
				MutexLock(flash_scene_handle.mutex);
				sta_cha_flag = TRUE;
				num_cnt = 0;
				flash_dir = 0;
				get_light_data();
				MutexUnLock(flash_scene_handle.mutex);
			} 
		break;
		case DPID_10:
			len = strlen(root->valuestring);
			if(len > 44){
				PR_ERR("the data length is wrong: %d", len);   
		    }else {
		    	memcpy(dp_data.BANLAN_SCENE_DATA, root->valuestring, len);
				MutexLock(flash_scene_handle.mutex);
				sta_cha_flag = TRUE;
				num_cnt = 0;
				flash_dir = 0;
				get_light_data();
				MutexUnLock(flash_scene_handle.mutex);
			} 
		break;
         default:
                break;
    }
}

STATIC INT ty_get_enum_id(UCHAR dpid, UCHAR *enum_str)
{
	UCHAR i = 0;
	UCHAR enum_id = 0;
	DP_CNTL_S *dp_cntl =  NULL;	
	DEV_CNTL_N_S *dev_cntl = get_single_wf_dev();

	for(i = 0; i < dev_cntl->dp_num; i++) {
		if(dev_cntl->dp[i].dp_desc.dp_id == dpid) {
			dp_cntl = &dev_cntl->dp[i];
			break;
		}
	}

	if(i >= dev_cntl->dp_num) {
		PR_ERR("not find enum_str");
		return -1;
	}

	if( dp_cntl == NULL ) {
		PR_ERR("dp_cntl is NULL");
		return -1;
	}

	for( i = 0; i < dp_cntl->prop.prop_enum.cnt; i++ )
	{
		if( strcmp(enum_str, dp_cntl->prop.prop_enum.pp_enum[i]) == 0 )
			break;
	}

	return i;	
}

STATIC UCHAR *ty_get_enum_str(DP_CNTL_S *dp_cntl, UCHAR enum_id)
{
	if( dp_cntl == NULL ) {
		return NULL;
	}

	if( enum_id >= dp_cntl->prop.prop_enum.cnt ) {
		return NULL;
	}
	
	return dp_cntl->prop.prop_enum.pp_enum[enum_id];	
}


  void gpio_interrupt_cb(void)
  {
	  unsigned int gpio_status;
	  
	  _xt_isr_mask(1<<ETS_GPIO_INUM);	 //disable interrupt
	  gpio_status = GPIO_REG_READ(GPIO_STATUS_ADDRESS);
  
         //PR_DEBUG("-------come to gpio_interrupt_cb--------\r\n");		
	 //PR_DEBUG("gpio_status:0x%X\r\n",gpio_status);
	  
	  
	  //GPIO_OUTPUT_SET(GPIO_ID_PIN(12), 0);
  
	  if(gpio_status & GPIO_Pin_4){
		  //printf("GPIO_Pin_5 be trigger\r\n");
  		PR_DEBUG("gsensor int trigger!\r\n");		
		gsensor_trigger = TRUE;   

	  }
	  
	  GPIO_REG_WRITE( GPIO_STATUS_W1TC_ADDRESS, gpio_status); //clear interrupt mask
	  _xt_isr_unmask(1 << ETS_GPIO_INUM);					  //Enable the GPIO interrupt
  }
  
  void gsensor_int_gpio_init(void)
  {   
	GPIO_ConfigTypeDef gpio_in_cfg; 					  //Define GPIO Init Structure
	gpio_in_cfg.GPIO_IntrType = GPIO_PIN_INTR_POSEDGE;	  //ä¸‹é™æ²¿è§¦å‘
	gpio_in_cfg.GPIO_Mode = GPIO_Mode_Input;			  //Input mode
	gpio_in_cfg.GPIO_Pullup = GPIO_PullUp_EN;			  //å†…éƒ¨ä¸Šæ‹‰ä½¿èƒ½
	gpio_in_cfg.GPIO_Pin = GPIO_Pin_4;					  // Enable GPIO
	gpio_config(&gpio_in_cfg);							  //Initialization function
  
	GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, GPIO_Pin_All); 	//clear all interrupt mask
	gpio_intr_handler_register(gpio_interrupt_cb);				// Register the interrupt function
	_xt_isr_unmask(1 << ETS_GPIO_INUM); 						//Enable the GPIO interrupt
  }  

bool com_iic_writeData(UCHAR chip_addr, UCHAR reg_addr, UCHAR* buf, UCHAR len)
{
	i2c_master_start();
	
	i2c_master_writeByte(chip_addr & 0xFE);
			
	if (!i2c_master_checkAck()) {
		PR_DEBUG("000 icc addr not ack!");		
		i2c_master_stop();
		return false;
	}

	i2c_master_writeByte(reg_addr);

	if (!i2c_master_checkAck()) {
		PR_DEBUG("111 reg addr not ack!");		
		i2c_master_stop();
		return false;
	}	
		
	while(len--) {
		i2c_master_writeByte(*buf);
		if (!i2c_master_checkAck()) {
			PR_DEBUG("222 data not ack!");		
			i2c_master_stop();
			return false;
		}	  
		buf++;
	}
	
	i2c_master_stop();
		
	return TRUE;
}
	
bool com_iic_readData(UCHAR chip_addr, UCHAR reg_addr, UCHAR* buf, UCHAR len)
{		
	i2c_master_start();

	i2c_master_writeByte(chip_addr & 0xFE);

	if (!i2c_master_checkAck()) {
		PR_DEBUG("000 icc addr not ack!");		
		i2c_master_stop();
		return false;
	}	
	
	i2c_master_writeByte(reg_addr);

	if (!i2c_master_checkAck()) {
		PR_DEBUG("111 reg addr not ack!");		
		i2c_master_stop();
		return false;
	}	

	i2c_master_start();	
		
	i2c_master_writeByte(chip_addr | 0x01);
	
	if (!i2c_master_checkAck()) {
		PR_DEBUG("222 icc addr not ack!");		
		i2c_master_stop();
		return false;
	}		
		
	while(len) {
	   *buf = i2c_master_readByte();
		   
	   if(1 == len)
		  i2c_master_send_nack();
	   else 
		  i2c_master_send_ack();
	   
	   buf++;
	   len--;
	}
		
	i2c_master_stop();
		
	return TRUE;
}

void touch_get_key_rslt(u8 key_value)
{
	if (key_value == KEY_SWITCH)
		key_switch_now = 1;
	else if (key_value == KEY_LEFT_1)
		key_left_temp = 1;
	else if (key_value == KEY_LEFT_2)
		key_left_temp = 2;
	else if (key_value == KEY_LEFT_3)
		key_left_temp = 3;
	else if (key_value == KEY_LEFT_4)
		key_left_temp = 4;	
	else if (key_value == KEY_LEFT_5)
		key_left_temp = 5;
	else if (key_value == KEY_LEFT_6)
		key_left_temp = 6;
	else if (key_value == KEY_LEFT_7)
		key_left_temp = 7;	
	else if (key_value == KEY_RIGHT_1)
		key_right_temp = 1;	
	else if (key_value == KEY_RIGHT_2)
		key_right_temp = 2;
	else if (key_value == KEY_RIGHT_3)
		key_right_temp = 3;	
	else if (key_value == KEY_RIGHT_4)
		key_right_temp = 4;
	else if (key_value == KEY_RIGHT_5)
		key_right_temp = 5;
	else if (key_value == KEY_RIGHT_6)
		key_right_temp = 6;	
	else if (key_value == KEY_RIGHT_7)
		key_right_temp = 7;
	else {
		key_switch_now = 0;
		key_left_temp = 0;
		key_right_temp = 0;
	}

	//PR_DEBUG("----------- key_switch_now: %d-------------", key_switch_now);	
	//PR_DEBUG("----------- key_left_temp: %d-------------", key_left_temp);	
	//PR_DEBUG("----------- key_right_temp: %d-------------", key_right_temp);	
	
}

static u8 com_iic_touch_readData(void)
{	
	u8 key_value = 0;
	
	i2c_master_start();	
		
	i2c_master_writeByte(0x5a | 0x01);
	
	if (!i2c_master_checkAck()) {
		PR_DEBUG("-----------000 icc addr not ack!-------------");		
		i2c_master_stop();
		return 0;
	}	

	key_value = i2c_master_readByte();
	//PR_DEBUG("-----------111 key_value: %d-------------", key_value);	
		
	i2c_master_stop();

	return key_value;
}

#if 1
static void key_get(void)
{
	static u8 key_value_last = 0;
	u8 key_value_now = 0;	
	
	key_value_now = com_iic_touch_readData();

	if (key_value_now == key_value_last) 
		touch_get_key_rslt(key_value_now);	

	key_value_last = key_value_now;
}
#else
static void key_get(void)
{
	u8 key_value_now = 0;	
	
	key_value_now = com_iic_touch_readData();

	touch_get_key_rslt(key_value_now);	
}
#endif

#if 0
static u8 bright_change(u8 key_level)
{
	u8 bright = 25;
	
	if (key_level == 1 || key_level == 0)
		bright = 25;
	else if (key_level == 7)
		bright = 255;
	else
		bright = (key_level - 1) * 38 + 25;

	return bright;
}
#else
static u8 bright_change(u8 key_level)
{
	u8 bright = 12;
	
	if (key_level == 0)
		bright = 12;
	else if (key_level == 6)
		bright = 255;
	else
		bright = (key_level - 1) * 40 + 45;

	if (bright < 12)
		bright = 12;
	
	if (bright > 255)
		bright = 255;	

	return bright;
}
#endif


#if 0
static void key_proc(void)
{
	static u8 key_switch_last = 0, key_fix_cnt = 0, key_single_cnt = 0;
	static u8 key_left_last = 0, key_right_last = 0;
       const char * colour_str;

	if (key_switch_last == key_switch_now) {
		key_fix_cnt++;
	}
	else if (key_switch_now == 0){
		key_fix_cnt = 0;		
	}
	else if (key_switch_now == 1){
		key_fix_cnt = 0;
		if (key_switch_single) 
			key_switch_double = 1;
		else {
			key_switch_single = 1;
			key_single_cnt = 0;
		}
	}		

	if (key_switch_single) {
		if (key_single_cnt++ > 40) {
			key_single_cnt = 0;						
			if (key_switch_double) {
 				PR_DEBUG("------------get a double key---------");	
				if(dp_data.WORK_MODE == WHITE_MODE) 
					work_mode_change(COLOUR_MODE);
				else
					work_mode_change(WHITE_MODE);	
				//light_switch(TRUE);	
			}
			else {
 				PR_DEBUG("------------get a short key---------");	
				if (dp_data.SWITCH) {
			              PR_DEBUG("------------touch single turn off---------");				
					light_switch(FALSE);				
				}
				else {
			              PR_DEBUG("------------touch single turn on---------");								
					light_switch(TRUE);	
				}			
			}
			//dev_inf_set();		
			sys_start_timer(data_save_timer,3000,TIMER_CYCLE);			
	        	init_upload_proc();				
			key_switch_single = 0;
			key_switch_double = 0;			
		}
	}
	else
		key_single_cnt = 0;
	
	if (key_fix_cnt > 60 && key_switch_now) {
		key_fix_cnt = 0;
 		PR_DEBUG("------------get a long key, come to reset factory---------");		
		single_dev_reset_factory();
	}

	if (key_left_temp != key_left_last && key_left_temp != 0) {
		 PR_DEBUG("------------000 come key_left ---------");	
		key_left = key_left_temp;
		 PR_DEBUG("------------key_left: %d ---------", key_left);			
		if(dp_data.WORK_MODE == WHITE_MODE) {			
		     dp_data.BRIGHT = bright_change(key_left);
		     get_light_data();
		     //PR_DEBUG("R:%d G:%d B:%d W:%d WW:%d",light_data.FIN_RED_VAL, light_data.FIN_GREEN_VAL, light_data.FIN_BLUE_VAL, light_data.FIN_WHITE_VAL, light_data.FIN_WARM_VAL);
		     start_gra_change(NORMAL_DELAY_TIME);
		}
		else {
			colour_str = colour_table[key_right - 1][7 - key_left];
		    	memcpy(dp_data.COLOUR_DATA, colour_str, 14);
			PR_ERR("dp_data.COLOUR_DATA: %s", dp_data.COLOUR_DATA);
			get_light_data();
			start_gra_change(NORMAL_DELAY_TIME);
		}
		//dev_inf_set();	
		sys_start_timer(data_save_timer,3000,TIMER_CYCLE);		
	       	init_upload_proc();	
	}

	if (key_right_temp != key_right_last && key_right_temp != 0) {
		PR_DEBUG("------------111 come key_right ---------");	
		key_right = key_right_temp;
		 PR_DEBUG("------------key_right: %d ---------", key_right);					
		if(dp_data.WORK_MODE == WHITE_MODE) {
		     dp_data.BRIGHT = bright_change(key_right);
		     get_light_data();
		     //PR_DEBUG("R:%d G:%d B:%d W:%d WW:%d",light_data.FIN_RED_VAL, light_data.FIN_GREEN_VAL, light_data.FIN_BLUE_VAL, light_data.FIN_WHITE_VAL, light_data.FIN_WARM_VAL);
		     start_gra_change(NORMAL_DELAY_TIME);
		}
		else {
			colour_str = colour_table[key_right - 1][0];
	    		memcpy(dp_data.COLOUR_DATA, colour_str, 14);
			PR_ERR("dp_data.COLOUR_DATA: %s", dp_data.COLOUR_DATA);
			get_light_data();
			start_gra_change(NORMAL_DELAY_TIME);
		}
		//dev_inf_set();	
		sys_start_timer(data_save_timer,3000,TIMER_CYCLE);		
	       	init_upload_proc();			
	}	

	key_switch_last = key_switch_now;
	key_left_last = key_left_temp;
	key_right_last = key_right_temp;
}
#else
static void key_proc(void)
{
	static u8 key_switch_cnt = 0, key_left_last = 0, key_right_last = 0;
       const char * colour_str;

	if (key_switch_now ==1) {
		if (key_switch_cnt > 150) {
			key_switch_cnt = 0;
	 		PR_DEBUG("------------get a long key, come to reset factory---------");		
			single_dev_reset_factory();			
		}		
	}
	else if (key_switch_now == 0){
#if 	0	
		if (key_switch_cnt > 10) {
			key_switch_cnt = 0;
	 		PR_DEBUG("------------get a middle key---------");	
			if(dp_data.WORK_MODE == WHITE_MODE) 
				work_mode_change(COLOUR_MODE);
			else
				work_mode_change(WHITE_MODE);	
			//dev_inf_set();	
			sys_start_timer(data_save_timer,3000,TIMER_CYCLE);
	        	init_upload_proc();
		}
#endif		
		 if (key_switch_cnt > 0){
			key_switch_cnt = 0;
			PR_DEBUG("------------get a short key---------");	
#if 	0		
			if (dp_data.SWITCH) {
		              PR_DEBUG("------------touch single turn off---------");				
				light_switch(FALSE);				
			}
			else {
		              PR_DEBUG("------------touch single turn on---------");								
				light_switch(TRUE);	
			}
#else
			if (dp_data.SWITCH) {
				if(dp_data.WORK_MODE == WHITE_MODE) 
					work_mode_change(COLOUR_MODE);
				else
					work_mode_change(WHITE_MODE);
				//dev_inf_set();				
				sys_start_timer(data_save_timer,3000,TIMER_CYCLE);			
		        	init_upload_proc();				
			}
#endif				
		}	
	}

	if (key_switch_now == 1) 
		key_switch_cnt++;
	else
		key_switch_cnt = 0;

	if (key_left_temp != key_left_last && key_left_temp != 0) {
		PR_DEBUG("------------000 come key_left ---------");	
		key_left = key_left_temp;
		PR_DEBUG("------------key_left: %d ---------", key_left);	
		if (dp_data.SWITCH) {
			if(dp_data.WORK_MODE == WHITE_MODE) {			
			     dp_data.BRIGHT = bright_change(7 - key_left);
			     get_light_data();
			     //PR_DEBUG("R:%d G:%d B:%d W:%d WW:%d",light_data.FIN_RED_VAL, light_data.FIN_GREEN_VAL, light_data.FIN_BLUE_VAL, light_data.FIN_WHITE_VAL, light_data.FIN_WARM_VAL);
			     start_gra_change(NORMAL_DELAY_TIME);
			}
			else {
				colour_str = colour_table[key_right - 1][key_left - 1];
			    	memcpy(dp_data.COLOUR_DATA, colour_str, 14);
				PR_ERR("dp_data.COLOUR_DATA: %s", dp_data.COLOUR_DATA);
				get_light_data();
				start_gra_change(NORMAL_DELAY_TIME);	
			}
			//dev_inf_set();	
			sys_start_timer(data_save_timer,3000,TIMER_CYCLE);		
		       	init_upload_proc();
		}	
	}

	if (key_right_temp != key_right_last && key_right_temp != 0) {
		PR_DEBUG("------------111 come key_right ---------");	
		key_right = key_right_temp;
		 PR_DEBUG("------------key_right: %d ---------", key_right);		
		if (dp_data.SWITCH) {
			if(dp_data.WORK_MODE == WHITE_MODE) {
			     dp_data.BRIGHT = bright_change(7 - key_right);
			     get_light_data();
			     //PR_DEBUG("R:%d G:%d B:%d W:%d WW:%d",light_data.FIN_RED_VAL, light_data.FIN_GREEN_VAL, light_data.FIN_BLUE_VAL, light_data.FIN_WHITE_VAL, light_data.FIN_WARM_VAL);
			     start_gra_change(NORMAL_DELAY_TIME);
			}
			else {
				colour_str = colour_table[key_right - 1][0];
		    		memcpy(dp_data.COLOUR_DATA, colour_str, 14);
				PR_ERR("dp_data.COLOUR_DATA: %s", dp_data.COLOUR_DATA);
				get_light_data();
				start_gra_change(NORMAL_DELAY_TIME);	
			}
			//dev_inf_set();	
			sys_start_timer(data_save_timer,3000,TIMER_CYCLE);		
		       	init_upload_proc();	
		}		
	}	

	key_left_last = key_left_temp;
	key_right_last = key_right_temp;
}
#endif

static void gsensor_data_analy(void)
{
#if 0
#if 0
	if (abs_s_n > 100) {
		if (click_now == 0){
			inc_sum_record = abs_s_n / 2;
		       PR_DEBUG("----------------------inc_sum_record:  %d ---------", inc_sum_record);				
		}
		click_now = 1;	
	}
	else if (abs_s_n < inc_sum_record && abs_sum_l < inc_sum_record) {
		click_now = 0;	
	}
#else
	if (click_single) {
	    if (abs_s_n > (inc_sum_record * 2 /3) && abs_sum_l > (inc_sum_record * 2 /3) ) {
		 click_now = 1;						
	    }	
	    else if (abs_s_n < (inc_sum_record / 3) && abs_sum_l < (inc_sum_record / 3)) {
		 click_now = 0;				
	    }
	}
	else {
		if (abs_s_n > 50 && abs_sum_l > 50) {
			if (click_now == 0){
				inc_sum_record = abs_s_n;
			       PR_DEBUG("----------------------inc_sum_record:  %d ---------", inc_sum_record);				
			}
			click_now = 1;
		}
		else {
			click_now = 0;
		}
	}

#endif

#endif

       //PR_DEBUG("--click_now:  %d ---------", click_now);	

}

static short x_r[100] = {0};
static short y_r[100] = {0};
static short z_r[100] = {0};

static void gsensor_proc(void)
{
	static int  x_l = 0, y_l = 0, z_l = 0,  x_s = 0, y_s = 0, z_s = 0, rec = 0, cnt = 0;
	static u32 abs_s_l = 0;
	int x_c = 0, y_c = 0, z_c = 0;
	u32  abs_s_n = 0;

	x_c =  x_n - x_l; 
	y_c =  y_n - y_l; 
	z_c =  z_n - z_l; 
	
	if (abs(x_c) < 10)
		x_c = 0;
	if (abs(y_c) < 10)
		y_c = 0;
	if (abs(z_c) < 10)
		z_c = 0;	

	abs_s_n = abs(x_c) + abs(y_c) + abs(z_c);		

       PR_DEBUG("src:   %4d  %4d  %4d  %5d ", x_c, y_c, z_c,abs_s_n);	

#if 0
	if (abs_s_n > 300) {
		if (click_now == 0){
			inc_sum_record = abs_s_n / 3;
		       PR_DEBUG("----------------------inc_sum_record:  %d ---------", inc_sum_record);				
		}
		click_now = 1;	
	}
	else if (abs_s_n < inc_sum_record && abs_s_l < inc_sum_record) {
		click_now = 0;	
	}	

       PR_DEBUG("--click_now:  %d ---------", click_now);	
#endif	   
	
	x_l = x_n;
	y_l = y_n;
	z_l = z_n;
	abs_s_l = abs_s_n;
}

static void click_handle(void)
{
	static u8 click_last = 0, cnt = 0;
	u16 fox = 0;

	if (click_now != click_last && click_now == 1){
		if (click_single) {
			click_double = 1;				
		       	PR_DEBUG("--------------------------------get click_double ------------------------------");			
		}
		else {
			click_single = 1;	
			cnt = 0;
		       PR_DEBUG("---------------------------------get click_single ------------------------------");							
		}
	}		

	if (click_single) {
		//PR_DEBUG("------------come click_single,  cnt:  %d  ---------",  cnt);									
		if (cnt++ > 50) {
			if (click_double) {
				if (dp_data.SWITCH) {
			              PR_DEBUG("------------double click turn off---------");				
					light_switch(FALSE);				
				}
				else {
			              PR_DEBUG("------------double click turn on---------");								
					light_switch(TRUE);	
				}
				//dev_inf_set();	
				sys_start_timer(data_save_timer,3000,TIMER_CYCLE);				
		        	init_upload_proc();	
			}
			cnt = 0;									
			click_single = 0;
			click_double = 0;	
			action = 0;
		}
	}
	else {
		cnt = 0;
	}

	click_last = click_now;

}

STATIC VOID com_iic_proc(PVOID pArg)
{
	static u8 g_last_key_read = 0;
	UCHAR chip_id = 0;
	//UCHAR buf[50];
      GW_WIFI_STAT_E wf_stat; 
      signed char      res = -1;
	

	PR_DEBUG("first up power!");	
	SystemSleep(100);	
	
	i2c_master_gpio_init();
	i2c_master_init();	
	gsensor_int_gpio_init();
	
	mir3da_init();	
	mir3da_open_interrupt();
	mir3da_set_enable(1);
	
	while( 1 ) {  
#if 1	
		  SystemSleep(20);	
#else
		  SystemSleep(40);		
#endif		  

 		  //PR_DEBUG("------------come to com_iic_proc---------");

		  wf_stat = get_wf_gw_status();
#if 1 
		  //if (wf_stat != STAT_UNPROVISION && wf_stat != STAT_AP_STA_UNCONN) {
	         	 key_get();
		  	 key_proc();
		  //}
#endif		  

#if 0
		  res = mir3da_read_data(&g_x, &g_y, &g_z);	
    	         //PR_DEBUG("src: xyz %d %d %d \r\n",g_x,g_y,g_z);
		  x_n = g_x -4;
		  y_n = g_y - 90;
		  z_n = g_z + 970;
    	         //PR_DEBUG("cal: x y z  %d %d %d \r\n",x_n,y_n,z_n);		  

	         if (res == 0) {			 	
		  	gsensor_proc();
		       //click_handle();			
		 }			 	
#endif		  
		  

#if 1
		  if (gsensor_trigger) {
			gsensor_trigger = FALSE;
			if (dp_data.SWITCH) {
		              PR_DEBUG("------------double click turn off---------");				
				light_switch(FALSE);				
			}
			else {
		              PR_DEBUG("------------double click turn on---------");								
				light_switch(TRUE);	
			}
		       //dev_inf_set();	
			sys_start_timer(data_save_timer,3000,TIMER_CYCLE);			   
        		init_upload_proc();							   
  		       //SystemSleep(2000);
		  }
#endif		  


	}	
}


