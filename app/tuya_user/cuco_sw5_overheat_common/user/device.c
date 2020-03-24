/***********************************************************
*  File: device.c  
*  Author: husai
*  Date: 20151215
***********************************************************/
#define __DEVICE_GLOBALS
#include "device.h"
#include "tuya_smart_api.h"
#include "gpio.h"
#include "tuya_smart_api.h"
#include "wf_sdk_adpt.h"

#include "CMT2300drive.h"

/***********************************************************
*************************micro define***********************
***********************************************************/
#define SYN_TIME  2  //同步开关状态
#define ONE_SEC   1
#define WM_SUCCESS 0
#define WM_FAIL 1
  
#define DEVICE_MOD    "device_mod"
#define DEVICE_PART   "device_part"
#define APPT_POSIX_KEY   "appt_posix_key"
#define POWER_STAT_KEY  "power_stat_key"

#define RF_MOD    "device_mod"
#define RF_PART   "device_part"
#define RF_USER_ID_KEY  "rfUser_stat_key"

#define TIME_POSIX_2016 1451577600 //2016年时间戳
  
  // reset key define
#define WF_RESET_KEY GPIO_ID_PIN(0)  //WIFI复位按键13
  // wifi direct led
#define WF_DIR_LED GPIO_ID_PIN(2)  //WiFi指示灯
#define POWER_LED  GPIO_ID_PIN(16)   //电源指示灯
  // power contrl
#define POWER_CTRL GPIO_ID_PIN(14)  //继电器控制

#define DP_SWITCH            1
#define DP_COUNT_DOWN  9
#define DP_OVER_HEAT      101

#define DP_SWITCH_CHAR            "1"
#define DP_COUNT_DOWN_CHAR  "9"
#define DP_OVER_HEAT_CHAR      "101"

typedef struct 
{
    INT power;
    INT appt_sec;
    TIMER_ID syn_timer;
    TIMER_ID count_timer;
    TIMER_ID saver_timer;
    THREAD msg_thread;
    THREAD sec_thread;   
    THREAD adc_thread;
	THREAD rf_rec_thread;
	THREAD rf_del_thread;
	THREAD led_thread;
    SEM_HANDLE press_key_sem;
    SEM_HANDLE sec_up_sem; 
	MSG_QUE_HANDLE ledMsgQueenHandle;
	MSG_QUE_HANDLE rfMsgQueenHandle;
}TY_MSG_S;

enum{
	msgLedId,
	msgRfId
};

/***********************************************************
*************************variable define********************
***********************************************************/
STATIC TY_MSG_S ty_msg;
LED_HANDLE wf_light = NULL;
LED_HANDLE power_ctrl = NULL;
LED_HANDLE power_led = NULL;

#define KEY_NUM 4

#if 1
#define TEM_PATI_RESIST  33   // 单位k
#else
#define TEM_PATI_RESIST  30   // 单位k
#endif

#if 1
#define TEM_ALARM_RESIST  17    // 95 摄氏度
#else
#define TEM_ALARM_RESIST  37    // 60 摄氏度
#endif

static uint16 adc_value = 0;
static uint32 tem_resist = 0;   // 单位k
static uint16 temperature = 25;
static uint8   tem_alarm = 0;
// 增加led处理相关变量
byte getstr[LEN+1];
static bool rfStudy_flag=false;
static MSG_LIST* ledMsg=NULL;
static MSG_LIST* rfMsg=NULL;

//单位： 0.1k 欧姆
#define TEMP_25  100
#define TEMP_30  90
#define TEMP_35  80
#define TEMP_40  70
#define TEMP_45  60
#define TEMP_50  50
#define TEMP_55  44
#define TEMP_60  40
#define TEMP_65  35
#define TEMP_70  32
#define TEMP_75  28
#define TEMP_80  26
#define TEMP_85  24
#define TEMP_90  22
#define TEMP_95  21
#define TEMP_100  10
#define TEMP_105  9
#define TEMP_110  8
#define TEMP_115  7
#define TEMP_120  6

/***********************************************************
*************************function define***********************
***********************************************************/
static void adc_proc();
static void rf_recv_task();
static void rf_proc_task();
static void led_proc_task();

STATIC VOID key_process(INT gpio_no,PUSH_KEY_TYPE_E type,INT cnt);
STATIC VOID wfl_timer_cb(UINT timerID,PVOID pTimerArg);
STATIC OPERATE_RET device_differ_init(VOID);
u8 prod_cucotest(void);
//u8 prod_cucotest(BOOL flag, CHAR rssi);

void over_usb_protect();
KEY_ENTITY_S key_tbl[] = {
    {WF_RESET_KEY,3000,key_process,0,0,0,0},
};
 
void pre_app_init()
{

}

BOOL gpio_func_test(VOID)
{
    return TRUE;
}
  
VOID set_firmware_tp(IN OUT CHAR *firm_name, IN OUT CHAR *firm_ver)
{
    strcpy(firm_name, APP_BIN_NAME);
    strcpy(firm_ver, USER_SW_VER);
    return;
}
  
STATIC INT msg_upload_proc(INT state,char i)
{
    GW_WIFI_STAT_E wf_stat = tuya_get_wf_status();

    if( STAT_UNPROVISION == wf_stat || STAT_STA_UNCONN == wf_stat || (tuya_get_gw_status() != STAT_WORK)) {
		return WM_FAIL;
    }
	
    cJSON *root = cJSON_CreateObject();
    if( NULL == root ) {
        return WM_FAIL;
    }
		 
     cJSON_AddBoolToObject(root,DP_SWITCH_CHAR,state);
     cJSON_AddNumberToObject(root, DP_COUNT_DOWN_CHAR, 0);
#if 	1 
     cJSON_AddBoolToObject(root,DP_OVER_HEAT_CHAR,tem_alarm);
#endif
	  
     CHAR *out = cJSON_PrintUnformatted(root);
     cJSON_Delete(root);
     if( NULL == out ) {
	  PR_ERR("cJSON_PrintUnformatted err:");
	  return WM_FAIL;
     }
		  
     PR_DEBUG("msg_upload_proc [%s]", out);
     OPERATE_RET op_ret =tuya_obj_dp_report(out);  // tuya_obj_dp_trans_report(out); 
     PR_DEBUG("op_ret %d", op_ret);
     Free(out);
		 
     if( OPRT_OK == op_ret ) {
	  return WM_SUCCESS;
     }
     else {
	  return WM_FAIL;
     }
}
  
STATIC INT msg_upload_sec(INT sec,char num)
{
     GW_WIFI_STAT_E wf_stat = tuya_get_wf_status();
	 
     if( STAT_UNPROVISION == wf_stat ||STAT_STA_UNCONN == wf_stat ||(tuya_get_gw_status() != STAT_WORK)) {
	   return WM_FAIL;
     }
		  
     cJSON *root = cJSON_CreateObject();
     if(NULL == root) {
	  return WM_FAIL;
     }

     cJSON_AddNumberToObject(root, DP_COUNT_DOWN_CHAR, sec);
     CHAR *out = cJSON_PrintUnformatted(root);
     cJSON_Delete(root);
	 
     if( NULL == out ) {
	  PR_ERR("cJSON_PrintUnformatted err:");
	  return WM_FAIL;
     }
		  
      PR_DEBUG("msg_upload_sec[%s]", out);
      OPERATE_RET op_ret;

      op_ret = tuya_obj_dp_report(out);
      PR_DEBUG("op_ret=%d", op_ret);
      Free(out);
	  
      if( OPRT_OK == op_ret ) {
	   return WM_SUCCESS;
      }
      else {
	   return WM_FAIL;
      }
}
  
STATIC VOID msg_send(INT cmd,char i)
{
     if( cmd == ty_msg.power) {
 	  msg_upload_proc(cmd,0);
     }
     else {
	  ty_msg.power = cmd;
	  PostSemaphore(ty_msg.press_key_sem);
     }
}
  
STATIC VOID ctrl_power_led(INT state,char i)
{
#if 0
	GW_WIFI_STAT_E wf_stat;

	wf_stat = tuya_get_wf_status();
#endif

	if( state ) {
	     tuya_set_led_type(power_ctrl,OL_HIGH,0);
	     PR_DEBUG("power on");	  
	}
	else {
	     tuya_set_led_type(power_ctrl,OL_LOW,0);	
	     PR_DEBUG("power off");	  		 
	}

	sys_start_timer(ty_msg.saver_timer,3000,TIMER_ONCE);				

#if  0
	if (ty_msg.power) {	
		if (wf_stat == STAT_UNPROVISION || wf_stat == STAT_AP_STA_UNCONN ||wf_stat == STAT_AP_STA_CONN)
 			tuya_set_led_type(power_led,OL_HIGH,0);			
		else
	        	tuya_set_led_type(power_led,OL_LOW,0);			
	}
	else
	     tuya_set_led_type(power_led,OL_HIGH,0);
#else
	if (ty_msg.power) {
	     tuya_set_led_type(power_led,OL_LOW,0);	
	     PR_DEBUG("power led on");	  		 	 
	}
	else {
	     tuya_set_led_type(power_led,OL_HIGH,0);
	     PR_DEBUG("power led off");	  		 		 
	}
#endif
}

STATIC VOID msg_proc(PVOID pArg)
{
      while( 1 ) {
	   WaitSemaphore(ty_msg.press_key_sem);
		  
	    //根据power状态控制继电器和电源指示灯
	   SystemSleep(10);
	   ctrl_power_led(ty_msg.power,0);
	   if( ty_msg.appt_sec > 0 ) {
		  ty_msg.appt_sec = 0;
	   }
	   
	   SystemSleep(10);
	   //sys_start_timer(ty_msg.saver_timer,3000,TIMER_ONCE);
	   msg_upload_proc(ty_msg.power,0);
	   SystemSleep(10);
      }
}
  
STATIC VOID sec_proc(PVOID pArg)
{
      while( 1 ) {
	   WaitSemaphore(ty_msg.sec_up_sem);
	    // PR_ERR("sec_proc");
	    
	    INT sec = wmtime_time_get_posix();
		
	    if( ty_msg.appt_sec == 0 ) {
		 msg_upload_sec(0,0);
	    }
	    else if( ty_msg.appt_sec > sec ) {
		  msg_upload_sec(ty_msg.appt_sec - sec,0);
	    }
	    else {
		  msg_upload_sec(0,0);
	    }

	    SystemSleep(20);
      }
}

/***********************************************************
*  Function: deal_dps_handle
*  Input: 
*  Output: 
*  Return: 
***********************************************************/
VOID deal_dps_handle(UCHAR dps,cJSON *nxt)
{
     OPERATE_RET op_ret;

     switch(dps) {
	  case DP_SWITCH:		
		   if( nxt->type == cJSON_False) {
		        msg_send(0,DP_SWITCH);
		   }
		   else if(nxt->type == cJSON_True) {
		        msg_send(1,DP_SWITCH);
		   }
		   break;

	  case DP_COUNT_DOWN:
	      	   //PR_ERR("DP_COUNT_DOWN");
		   if( nxt->type == cJSON_Number) {
	    	    if( nxt->valueint == 0 ){ 
			     ty_msg.appt_sec = 0;
			     msg_upload_sec(0,0);
			    }
		        else{
			      ty_msg.appt_sec = wmtime_time_get_posix() + nxt->valueint;
			      msg_upload_sec(nxt->valueint,0);
	      	    }
		    }
		    break;
				
	   default:
		  break;
      }
}
  
/***********************************************************
*************************function define********************
***********************************************************/
VOID device_cb(SMART_CMD_E cmd,cJSON *root)
{  
      //PR_DEBUG("cmd:%d",cmd);
      cJSON *nxt = root->child; 
      cJSON *last;
      UCHAR dp_id = 0;
	  
      while(nxt) {
	   dp_id = atoi(nxt->string);
	   PR_ERR("dp_id=%d",dp_id);
	   
	   if(dp_id) {
		  deal_dps_handle(dp_id,nxt);
		  nxt = cJSON_False;
	   }

	   SystemSleep(10);		  
      }
}
  
STATIC VOID syn_timer_cb(UINT timerID,PVOID pTimerArg)
{
      static char Count_timer = 0;

      if(++Count_timer > KEY_NUM) {
	       Count_timer = 0;
      }
	  
      if( FALSE == tuya_get_cloud_stat() ) {
	    return;
      }
		  
      INT ret = msg_upload_proc(ty_msg.power,Count_timer);
	  
      if( (WM_SUCCESS == ret) && (Count_timer==0) ) {
	     sys_stop_timer(ty_msg.syn_timer);
      }

      PR_DEBUG("syn timer cb ...");
}
  
  
STATIC VOID count_timer_cb(UINT timerID,PVOID pTimerArg)
{
     if( (ty_msg.appt_sec == 0)) {
	   return;
     }

     INT cur_posix = wmtime_time_get_posix();
	 
     if( cur_posix < TIME_POSIX_2016 ) {
	    PR_DEBUG("cur_posix:%d", cur_posix);
	    return;
     }
	  
     if( cur_posix <= ty_msg.appt_sec) {
	  if( (ty_msg.appt_sec - cur_posix) % 5 == 0 ) {
	       PostSemaphore(ty_msg.sec_up_sem);
	  }
     }
     else if ( ty_msg.appt_sec != 0) {
	  ty_msg.appt_sec = 0;
	  ty_msg.power ^=1;
	  PostSemaphore(ty_msg.press_key_sem);
	  PostSemaphore(ty_msg.sec_up_sem);
     }	 
}
  
STATIC VOID syn_active_data(VOID)
{
     msg_upload_proc(ty_msg.power,0);
}

void uart_inial ()
{
     //print_port_init();
    print_port_full_init(UART0,BIT_RATE_115200,UART_WordLength_8b,USART_Parity_None,USART_StopBits_1); 
    // user_uart_raw_full_init(BIT_RATE_4800,UART_WordLength_8b,USART_Parity_Even,USART_StopBits_1);
}
  
STATIC OPERATE_RET get_power_stat()
{
     OPERATE_RET op_ret;
     cJSON *root = NULL, *json = NULL;
  
     UCHAR *buf = (UCHAR *)Malloc(256);
     if(NULL == buf) {
	  PR_ERR("malloc error");
	  return OPRT_MALLOC_FAILED;
     }
  
     op_ret = tuya_psm_get_single(DEVICE_MOD,POWER_STAT_KEY,buf,256);
     if(OPRT_OK != op_ret) {
	  PR_ERR("tuya_psm_get_single op_ret:%d",op_ret);
	  Free(buf);
	  return op_ret;
     }
  
     root = cJSON_Parse(buf);
     if(NULL == root) {
	  PR_ERR("cjson parse");
	  goto JSON_PARSE_ERR;
     }
  
     json = cJSON_GetObjectItem(root,"power0");
     if(NULL == json) {
 	  PR_ERR("cjson get power");
	  goto JSON_PARSE_ERR;
     }
	   
     ty_msg.power = json->valueint;
	
     cJSON_Delete(root);
     Free(buf);
	 
     return OPRT_OK;
  
     JSON_PARSE_ERR:
     cJSON_Delete(root);
     Free(buf);

     return OPRT_COM_ERROR;
}
  
STATIC OPERATE_RET set_power_stat(INT state)
{
     cJSON *root = cJSON_CreateObject();
     if(NULL == root) {
 	  PR_ERR("cJSON_CreateObject error");
	  return OPRT_CJSON_GET_ERR;
     }
	  
     cJSON_AddNumberToObject(root, "power0", ty_msg.power);
	
     UCHAR *buf = cJSON_PrintUnformatted(root);
     if(NULL == buf) {
	  PR_ERR("buf is NULL");
	  cJSON_Delete(root);
	  return OPRT_MALLOC_FAILED;
     }
     cJSON_Delete(root);
	  
     OPERATE_RET op_ret = tuya_psm_set_single(DEVICE_MOD,POWER_STAT_KEY,buf);
     Free(buf);
     if(OPRT_OK != op_ret) {
	  PR_ERR("tuya_psm_set_single op_ret:%d",op_ret);
	  return op_ret;
     }
	  
     return OPRT_OK;	 
}
  
STATIC VOID save_timer_cb(UINT timerID,PVOID pTimerArg)
{
      set_power_stat(ty_msg.power);
}

STATIC VOID init_power_stat(VOID)
{
      ty_msg.power = 0;
      ty_msg.appt_sec = 0;	 
      get_power_stat();
      PR_DEBUG("init_power_stat, ty_msg.power: %d",ty_msg.power);	  
      ctrl_power_led(ty_msg.power,0);
      //  tuya_set_led_type(power_ctrl,OL_LOW,0);
      // tuya_set_led_type(power_led,OL_HIGH,0);
}

STATIC VOID key_process_prod(INT gpio_no,PUSH_KEY_TYPE_E type,INT cnt)
{	 
	PR_DEBUG("gpio_no: %d",gpio_no);
	PR_DEBUG("type: %d",type);
	PR_DEBUG("cnt: %d",cnt);
      
	if(WF_RESET_KEY == gpio_no) {
		PR_DEBUG("++++++++++prod_cucotest, power on!++++++++++++");
		SystemSleep(1000);
		tuya_set_led_type(power_ctrl,OL_HIGH,0);
	    tuya_set_led_type(power_led,OL_LOW,0);	
        SystemSleep(1000);
		PR_DEBUG("++++++++++prod_cucotest, power off!++++++++++++");			
		tuya_set_led_type(power_ctrl,OL_LOW,0);
	    tuya_set_led_type(power_led,OL_HIGH,0);			 
	}
}

#if 1
u8 prod_cucotest(void)
{
  	OPERATE_RET op_ret;

	//uart_inial();
	op_ret = tuya_create_led_handle(WF_DIR_LED,&wf_light);
	if( OPRT_OK  != op_ret) {
	     PR_ERR("POWER_LED error");
	} 
       op_ret = tuya_create_led_handle(POWER_CTRL,&power_ctrl);
       if(OPRT_OK  != op_ret) {
	    return op_ret;
       }  
       op_ret = tuya_create_led_handle(POWER_LED,&power_led);
       if(OPRT_OK  != op_ret) {
	    return op_ret;
       }	
	   
	//set_led_light_type(wf_light,OL_FLASH_HIGH,300);
	tuya_set_led_type(wf_light,OL_FLASH_HIGH,1500);
	tuya_set_led_type(power_ctrl,OL_LOW,0);
    tuya_set_led_type(power_led,OL_HIGH,0);

	op_ret = tuya_kb_init();
	if(OPRT_OK  != op_ret) {
	      // return op_ret;
	      PR_ERR("tuya_kb_init error");		 
	}
	
	tuya_set_kb_seq_enable(FALSE);
        tuya_set_kb_trig_type(WF_RESET_KEY,KEY_UP_TRIG,FALSE);
	// register key to process
	op_ret = tuya_kb_reg_proc(WF_RESET_KEY,3000,key_process_prod);
	if(OPRT_OK  != op_ret) {
	     //return op_ret;
	     PR_ERR("tuya_kb_reg_proc error");		 		  
	}	  

	PR_DEBUG("+++++++++++++++come to prod_cucotest");	
  
        SystemSleep(1500);

	
}

#else
u8 prod_cucotest(BOOL flag, CHAR rssi)
{
  	OPERATE_RET op_ret;

	uart_inial();

	PR_DEBUG("rssi:%d", rssi);

	if( rssi < -60 ) {
		PR_DEBUG("wifi signal too low!");
		return;
	}	
	if( flag == FALSE) {
		PR_DEBUG("flag is false!");
		return;
	}
	
	op_ret = tuya_create_led_handle(WF_DIR_LED,&wf_light);
	if( OPRT_OK  != op_ret) {
	     PR_ERR("POWER_LED error");
	} 
       op_ret = tuya_create_led_handle(POWER_CTRL,&power_ctrl);
       if(OPRT_OK  != op_ret) {
	    return op_ret;
       }  
       op_ret = tuya_create_led_handle(POWER_LED,&power_led);
       if(OPRT_OK  != op_ret) {
	    return op_ret;
       }	
	   
	//set_led_light_type(wf_light,OL_FLASH_HIGH,300);
	tuya_set_led_type(wf_light,OL_FLASH_HIGH,1500);
	tuya_set_led_type(power_ctrl,OL_LOW,0);
        tuya_set_led_type(power_led,OL_HIGH,0);

	op_ret = tuya_kb_init();
	if(OPRT_OK  != op_ret) {
	      // return op_ret;
	      PR_ERR("tuya_kb_init error");		 
	}
	
	tuya_set_kb_seq_enable(FALSE);
        tuya_set_kb_trig_type(WF_RESET_KEY,KEY_UP_TRIG,FALSE);
	// register key to process
	op_ret = tuya_kb_reg_proc(WF_RESET_KEY,3000,key_process_prod);
	if(OPRT_OK  != op_ret) {
	     //return op_ret;
	     PR_ERR("tuya_kb_reg_proc error");		 		  
	}	  

	PR_DEBUG("+++++++++++++++come to prod_cucotest");	
  
        SystemSleep(1500);

	
}
#endif

#define ProdTest

VOID app_init(VOID)
{
     OPERATE_RET op_ret;
	 
     uart_inial();

     op_ret = msf_register_module(DEVICE_MOD,DEVICE_PART);
     if( op_ret != OPRT_OK && op_ret != OPRT_PSM_E_EXIST ) {
          PR_ERR("msf_register_module error:%d",op_ret);
          return;
     }
	 
      op_ret = tuya_create_led_handle(POWER_CTRL,&power_ctrl);
      if(OPRT_OK  != op_ret) {
	   PR_ERR("POWER_CTRL error");
      }
	  
      op_ret = tuya_create_led_handle(POWER_LED,&power_led);
      if(OPRT_OK  != op_ret) {
	   PR_ERR("POWER_LED error");
      }
	  
     op_ret = tuya_create_led_handle(WF_DIR_LED,&wf_light);
     if( OPRT_OK  != op_ret ) {
	   PR_ERR("POWER_LED error");
      }
	  
      tuya_set_led_type(wf_light,OL_HIGH,0);
      tuya_set_led_type(power_led,OL_HIGH,0);
	  
      PR_ERR("app_init 01");
      set_prod_ssid("tuya_mdev_test1");
      // PR_ERR("app_init");
      PR_ERR("app_init 02");
      PR_ERR("app_init 03");
	
      app_cfg_set(WCM_OLD_CPT, prod_cucotest);
      //app_cfg_set(WCM_LOW_POWER, prod_cucotest);
	  
      PR_ERR("app_init 04");
	 
      PR_ERR("app_init 05");
      set_ap_ssid("SmartLife");
}

 
/***********************************************************
*  Function: device_init
*  Input: 
*  Output: 
*  Return: 
***********************************************************/
OPERATE_RET device_init(VOID)
{
     char *ID;
     char i,*p;
     OPERATE_RET op_ret;
     char buf[4];
	  
     //app_init();
     uart_inial();
	 
     PR_DEBUG("fireware info name:%s version:%s",APP_BIN_NAME,USER_SW_VER);

     // p=tuya_get_sdk_ver();
	 
     op_ret = tuya_device_init(PRODECT_KEY,device_cb,USER_SW_VER);
     if( op_ret != OPRT_OK) {
	  return op_ret;
     }
	 
//#ifndef ProdTest
     op_ret = tuya_psm_register_module(DEVICE_MOD, DEVICE_PART);
     if( op_ret != OPRT_OK && op_ret != OPRT_PSM_E_EXIST) {
    	  PR_ERR("tuya_psm_register_module error:%d",op_ret);
	  return op_ret;
     }
//#endif
   
     ID = tuya_get_devid();
     PR_NOTICE("ID:%s",ID);
 
     op_ret = tuya_create_led_handle(POWER_CTRL,&power_ctrl);
     if(OPRT_OK  != op_ret) {
	  return op_ret;
     }
  
     op_ret = tuya_create_led_handle(POWER_LED,&power_led);
     if(OPRT_OK  != op_ret) {
	  return op_ret;
     }
	  
    op_ret = tuya_create_led_handle(WF_DIR_LED,&wf_light);
     if(OPRT_OK  != op_ret) {
	  return op_ret;
     }

     tuya_set_led_type(wf_light,OL_HIGH,0);
     tuya_set_led_type(power_led,OL_HIGH,0);
     init_power_stat();
     tuya_active_reg(syn_active_data); 
  
     ty_msg.press_key_sem = CreateSemaphore();
     if( NULL == ty_msg.press_key_sem ) {
	  return OPRT_MALLOC_FAILED;
     }
  
     op_ret = InitSemaphore(ty_msg.press_key_sem,0,1);
     if(OPRT_OK != op_ret) {
 	  return op_ret;
     }
    
     ty_msg.sec_up_sem = CreateSemaphore();
     if( NULL == ty_msg.sec_up_sem ) {
 	  return OPRT_MALLOC_FAILED;
     }
	  
     op_ret = InitSemaphore(ty_msg.sec_up_sem,0,1);
     if(OPRT_OK != op_ret) {
	  return op_ret;
     }
	 // 创建一个消息队列 led
	 op_ret = CreateMsgQueAndInit(&ty_msg.ledMsgQueenHandle);
	 if(op_ret!=OPRT_OK)
	 {
		return op_ret;
	 }
	  // 创建一个消息队列 rf
	 op_ret = CreateMsgQueAndInit(&ty_msg.rfMsgQueenHandle);
	 if(op_ret!=OPRT_OK)
	 {
		return op_ret;
	 }
	
     op_ret = CreateAndStart(&ty_msg.msg_thread,msg_proc,NULL,1024,TRD_PRIO_3,"ty_task");
     if(op_ret != OPRT_OK) {
 	  return op_ret;
     }
  
     op_ret = CreateAndStart(&ty_msg.sec_thread,sec_proc,NULL,1024,TRD_PRIO_3,"sec_task");
     if(op_ret != OPRT_OK) {
	  return op_ret;
     }
  
     op_ret = sys_add_timer(syn_timer_cb,NULL,&ty_msg.syn_timer);
     if(OPRT_OK != op_ret) {
	   PR_ERR("add syn_timer err");
	   return op_ret;
     }
     else {
	   sys_start_timer(ty_msg.syn_timer,SYN_TIME*1000,TIMER_CYCLE);
     }
  
     op_ret = sys_add_timer(count_timer_cb,NULL,&ty_msg.count_timer);
     if( OPRT_OK != op_ret) {
  	  PR_ERR("add syn_timer err");
	  return op_ret;
     }
     else {
	  sys_start_timer(ty_msg.count_timer,ONE_SEC*1000,TIMER_CYCLE);
     }
	  
     op_ret = sys_add_timer(save_timer_cb,NULL,&ty_msg.saver_timer);
     if( OPRT_OK != op_ret) {
 	  PR_ERR("add syn_timer err");
       	  return op_ret;
     }

     op_ret = CreateAndStart(&ty_msg.adc_thread,adc_proc,NULL,1024,TRD_PRIO_6,"adc_task");
     if( op_ret != OPRT_OK) {
	  return op_ret;
     }
	// 创建一个射频接收任务函数
	 op_ret = CreateAndStart(&ty_msg.rf_rec_thread,rf_recv_task,NULL,1024,TRD_PRIO_6,"rf_recv_task");
     if( op_ret != OPRT_OK) {
	  return op_ret;
     }
	// 创建一个射频数据处理任务函数
	op_ret = CreateAndStart(&ty_msg.rf_del_thread,rf_proc_task,NULL,1024,TRD_PRIO_6,"rf_proc_task");
     if( op_ret != OPRT_OK) {
	  return op_ret;
     }
	 
	// 创建一个led 任务函数
	 op_ret = CreateAndStart(&ty_msg.led_thread,led_proc_task,NULL,1024,TRD_PRIO_6,"led_proc_task");
     if( op_ret != OPRT_OK) {
	  return op_ret;
     }
	 
     op_ret = device_differ_init();
     if( op_ret != OPRT_OK) {
	  return op_ret;
     }
	 
      //user_uart_raw_full_init(BIT_RATE_4800,UART_WordLength_8b,USART_Parity_Even,USART_StopBits_1);
      uart_inial();

      return op_ret;
}
  
STATIC VOID key_process(INT gpio_no,PUSH_KEY_TYPE_E type,INT cnt)
{
      PR_DEBUG("gpio_no: %d",gpio_no);
      PR_DEBUG("type: %d",type);
      PR_DEBUG("cnt: %d",cnt);
	  
      MSG_ID id=msgLedId;
	  P_MSG_DATA data="led";
	  MSG_DATA_LEN len=sizeof(data);
	  
      if( WF_RESET_KEY == gpio_no) {
	  	
		if( LONG_KEY == type) {
			PR_DEBUG("###### LONG_KEY EVENT OCCUR");
			tuya_psm_set_single(RF_MOD,RF_USER_ID_KEY,"#");
		    tuya_dev_reset_factory();
		}
		else if(SEQ_KEY==type){
			PR_DEBUG("###### SEQ_KEY EVENT OCCUR ");
			if(cnt==3)
			{
				PR_DEBUG("###### send a message");
				rfStudy_flag = TRUE; // 进入学习状态
				PostMessage(ty_msg.ledMsgQueenHandle,id,data,len);
			}
		}
		else {
			PR_DEBUG("###### NORMAL_KEY EVENT OCCUR");
			if(rfStudy_flag)
				rfStudy_flag=FALSE;
		    ty_msg.power ^= 1;
		 	PostSemaphore(ty_msg.press_key_sem);
		}
      }
}

STATIC OPERATE_RET device_differ_init(VOID)
{
     OPERATE_RET op_ret;
  
     //set_key_detect_time(50);	  
     // key process init
     op_ret = tuya_kb_init();
     if( OPRT_OK  != op_ret) {
	  return op_ret;
     }
     //tuya_set_kb_seq_enable(FALSE); 
     //tuya_set_kb_trig_type(WF_RESET_KEY,KEY_UP_TRIG,FALSE);

	 tuya_set_kb_seq_enable(TRUE);  // XWH 2020/1/10
     tuya_set_kb_trig_type(WF_RESET_KEY,KEY_DOWN_TRIG,TRUE); // XWH 2020/1/10
	 
     // register key to process
     op_ret = tuya_kb_reg_proc(WF_RESET_KEY,3000,key_process);
     if( OPRT_OK  != op_ret) {
	  return op_ret;
     }
  
     // create led handle
     op_ret = tuya_create_led_handle(WF_DIR_LED,&wf_light);
     if( OPRT_OK  != op_ret) {
	  return op_ret;
     }
  
     TIMER_ID timer;
     op_ret = sys_add_timer(wfl_timer_cb,NULL,&timer);
     if( OPRT_OK != op_ret) {
	  return op_ret;
     }
     else {
	  sys_start_timer(timer,300,TIMER_CYCLE);
     }

     return OPRT_OK;
}
  
STATIC VOID wfl_timer_cb(UINT timerID,PVOID pTimerArg)
{
     STATIC UINT last_wf_stat = 0xffffffff;
     static u8 cnt = 0;
     CHAR rssi = 0;

     if (++cnt > 4) {
	  cnt = 0;
         rssi = wifi_station_get_rssi();
         //PR_DEBUG("+++++++++++++++wifi_station_get_rssi:%d", rssi);			
     }
	 

     GW_WIFI_STAT_E wf_stat = tuya_get_wf_status();
     //PR_DEBUG("wf_stat:%d",wf_stat);
     
     if(last_wf_stat != wf_stat) {
	   PR_DEBUG("wf_stat:%d",wf_stat);
	   switch(wf_stat) {
		  case STAT_UNPROVISION: 
			   //tuya_set_led_type(power_led,OL_HIGH,0);
			   tuya_set_led_type(wf_light,OL_FLASH_HIGH,150); 
			   break;
			  
		  case STAT_AP_STA_UNCONN:
		  case STAT_AP_STA_CONN: 
		 	   //tuya_set_led_type(power_led,OL_HIGH,0);
			   tuya_set_led_type(wf_light,OL_FLASH_HIGH,1000);
			   break;
  
		   case STAT_LOW_POWER:
		   case STAT_STA_UNCONN: 
			    //tuya_set_led_type(wf_light,OL_HIGH,0);
			    tuya_set_led_type(wf_light,OL_FLASH_HIGH,1500);				
#if 	0			
			    if ( ty_msg.power) 
	        		  tuya_set_led_type(power_led,OL_LOW,0);			
			    else
	     			  tuya_set_led_type(power_led,OL_HIGH,0);	
#endif				
			    break;
			  
		   case STAT_STA_CONN: 
			    tuya_set_led_type(wf_light,OL_HIGH,0);
	   		    sys_start_timer(ty_msg.syn_timer,SYN_TIME*1000,TIMER_CYCLE);
				
#if 	0			
			    if ( ty_msg.power) 
	        		  tuya_set_led_type(power_led,OL_LOW,0);			
			    else
	     			  tuya_set_led_type(power_led,OL_HIGH,0);	
#endif				
			    break;

		   default:
		   	    break;
	     }
     }
     last_wf_stat = wf_stat;
}
  
u16 temp_change(u16 resist)
{
	u16 ret = 0;

	if (resist >= TEMP_25)
		ret = 25;
	else if (resist >= TEMP_30)
		ret = 30;		
	else if (resist >= TEMP_35)
		ret = 35;
	else if (resist >= TEMP_40)
		ret = 40;	
	else if (resist >= TEMP_45)
		ret = 45;
	else if (resist >= TEMP_50)
		ret = 50;
	else if (resist >= TEMP_55)
		ret = 55;	
	else if (resist >= TEMP_60)
		ret = 60;
	else if (resist >= TEMP_65)
		ret = 65;
	else if (resist >= TEMP_70)
		ret = 70;	
	else if (resist >= TEMP_75)
		ret = 75;
	else if (resist >= TEMP_80)
		ret = 80;
	else if (resist >= TEMP_85)
		ret = 85;		
	else if (resist >= TEMP_90)
		ret = 90;
	else if (resist >= TEMP_95)
		ret = 95;
	else if (resist >= TEMP_100)
		ret = 100;	
	else if (resist >= TEMP_105)
		ret = 105;
	else if (resist >= TEMP_110)
		ret = 110;
	else if (resist >= TEMP_115)
		ret = 115;	
	else 
		ret = 120;	

	return ret;
}  

static void over_heat_protect(void)
{
	PR_DEBUG("------------------enter tem alarm-----------");
	tem_alarm = 1;	
	msg_upload_proc(ty_msg.power,0);		
	
	SystemSleep(20000);	  	
	ty_msg.power = 0;
	ctrl_power_led(ty_msg.power,0);
	if( ty_msg.appt_sec > 0 ) {
	    ty_msg.appt_sec = 0;
	}
	
	tem_alarm = 0;
	msg_upload_proc(ty_msg.power,0);		
	//sys_start_timer(ty_msg.saver_timer,3000,TIMER_ONCE);
}
	
static void adc_proc()
{
    static u8 cnt = 0;
	
    while(1) {
   	  SystemSleep(500);	  
	
	  adc_value = system_adc_read();
	  
	  tem_resist = (adc_value * TEM_PATI_RESIST * 10) / (3300 - adc_value);
	  //temperature = temp_change(tem_resist);
	  //PR_DEBUG("------------------adc_value: %d--------------", adc_value);
	  //PR_DEBUG("------------------tem_resist: %d--------", tem_resist);			 
	  //PR_DEBUG("------------------temperature: %d--------", temperature);
	  //PR_DEBUG("------------------cnt: %d--------------", cnt);

	  if (tem_resist < TEM_ALARM_RESIST) {
	  	if (++cnt > 5) {
		     if ( ty_msg.power ) {
		    	   over_heat_protect();
		     }
		     cnt  = 0;
	  	}
	  }
	  else {
	  	cnt = 0;
	  }
     }	  
}  
/***********************************************************
function_name: rf_data_valid_check
arg1: uid 
arg2: len
return: OPERATE_RET
note:检查数据是否有效数据
  	12-byte		id       key    preamble       syn 
包数据格式:  10 00 00 00   04   AA AA AA AA AA   2D D4
***********************************************************/
static OPERATE_RET rf_data_valid_check(char* uid,uint8 len)
{
	//PR_DEBUG("%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X ",uid[0],uid[1],uid[2],uid[3],uid[4],
							//uid[5],uid[6],uid[7],uid[8],uid[9],uid[10],uid[11]);
	if(uid[5]!=0XAA||uid[6]!=0XAA||uid[7]!=0XAA||uid[8]!=0XAA||uid[9]!=0XAA)
		return OPRT_INVALID_PARM;
	return OPRT_OK;
}

/***********************************************************
function_name: rf_id_check
arg: uid 
return: OPERATE_RET
note:查询该设备是否已经配对
***********************************************************/
static OPERATE_RET rf_id_check(char* uid)
{
	OPERATE_RET op_ret;
	char tmpBuff[9]={0};
	
	char* userId=(char*)malloc(sizeof(char)*256);
	if(!userId)
		return OPRT_MALLOC_FAILED;
	
	op_ret = tuya_psm_get_single(RF_MOD,RF_USER_ID_KEY,userId,256);
	if(op_ret!=OPRT_OK)
	{
		PR_DEBUG(">>>>> tuya_psm_get_single RF_USER_ID_KEY FAILED %d",op_ret);
		op_ret = tuya_psm_set_single(RF_MOD,RF_USER_ID_KEY,"#");
		if(op_ret!=OPRT_OK)
			goto END;
	}
	//PR_DEBUG(">>>>:userId:%s",userId);

	sprintf(tmpBuff,"%02X%02X%02X%02X",uid[0],uid[1],uid[2],uid[3]);
	//PR_DEBUG(">>>>>tmpBuff:%s ",tmpBuff);
	// 查找指定uid是否存在
	if(strstr(userId,tmpBuff)!=NULL)
	{
		free(userId);
		userId=NULL;
		return OPRT_OK;
	}
END:
	free(userId);
	userId=NULL;
	return OPRT_COM_ERROR;
}

/***********************************************************
function_name: rf_add_del_id
arg1: uid 
arg2: sel (true:增加uid  false: 删除uid)
return: OPERATE_RET
note:从flash 中增加或者删除指定uid
***********************************************************/
static OPERATE_RET rf_add_del_id(char* uid,bool sel)
{
	OPERATE_RET op_ret;
	char tmpBuff[10]={0};
	
	char* userId=(char*)malloc(sizeof(char)*256);
	if(!userId)
		return OPRT_MALLOC_FAILED;
	
	op_ret = tuya_psm_get_single(RF_MOD,RF_USER_ID_KEY,userId,256);
	if(op_ret!=OPRT_OK)
	{
		PR_DEBUG(">>>>> tuya_psm_get_single RF_USER_ID_KEY FAILED");
		goto END;
	}
	
	if(sel) // 增加uid
	{
		if(strlen(userId)+8>=256) // uid 4个字节
			goto END;
		sprintf(tmpBuff,"%02X%02X%02X%02X#",uid[0],uid[1],uid[2],uid[3]);
		strcat(userId,tmpBuff);
		PR_DEBUG(">>>>>>userId:%s ",userId);
		op_ret = tuya_psm_set_single(RF_MOD,RF_USER_ID_KEY,userId);
		if(op_ret!=OPRT_OK)
			goto END;
		else
			goto OPR_OK;
	}else{ // 删除uid
	
		sprintf(tmpBuff,"%02X%02X%02X%02X",uid[0],uid[1],uid[2],uid[3]);
		
		char *pUid = userId;
		char idBuf[256]={0};
				
		char *tmp = strstr(userId,tmpBuff);
		if(tmp==NULL)
			goto END;
		
		strncpy(idBuf,pUid,(tmp-pUid));
		
		tmp+=9;
	
		PR_DEBUG(">>>>>>tmp:%s userId:%s  idBuf:%s ",tmp,userId,idBuf);
		
		if(tmp!=NULL)
			strcat(idBuf,tmp);

		strcpy(userId,idBuf);
		PR_DEBUG(">>>>>>userId:%s ",userId);
		
		op_ret = tuya_psm_set_single(RF_MOD,RF_USER_ID_KEY,userId);
		if(op_ret==OPRT_OK)
			goto OPR_OK;
	}	
END:
	free(userId);
	userId=NULL;
	return OPRT_COM_ERROR;

OPR_OK:
	free(userId);
	userId=NULL;
	return OPRT_OK;
		
}

/***********************************************************
function_name: rf_event_proc
arg1: uid 
return: OPERATE_RET
note:解析射频数据，控制电源开关
                ID        KEY      preamble         syn
包数据格式:  10000000     XX   AA AA AA AA AA       2DD4 
***********************************************************/
static OPERATE_RET rf_event_proc(char* uid,uint32 len)
{

	BYTE keyId = uid[4]>>1;

	switch(keyId&0X0F){
		case 0x01:  // LBD
			{
				
			}
			break;
		case 0x02: //  当前为关，需要打开
			{
				 if(ty_msg.power ^1)
				 {
					ty_msg.power ^= 1;
				 	PostSemaphore(ty_msg.press_key_sem);
				 }
			}
			break;
		case 0x04: // 当前为开，需要关闭
			{
				if(ty_msg.power ^0)
				{
					ty_msg.power ^= 1;
		 		 	PostSemaphore(ty_msg.press_key_sem);
				}
			}
			break;
		case 0x08:
			{
				//if(ty_msg.power ^1) // 避免重复开或者关
				//{
					//ty_msg.power ^= 1;
		 		 	//PostSemaphore(ty_msg.press_key_sem);
				//}
			}
			break;
		default:
			break;

	}
	return OPRT_OK;
}

/***********************************************************
function_name: rf_recv_task
arg1: void 
return: void
note:检测射频数据任务函数
***********************************************************/
static void rf_recv_task()
{
	byte tmp,i;
	MSG_ID id=msgRfId;
	P_MSG_DATA data="rf";
	MSG_DATA_LEN len=sizeof(data);
	 int j=0;
	CMT2300_Init();
	setup_Rx();
	while(1){
		if(GPO3_H())
		{
		  radio.bGoStandby();
		  tmp = radio.bGetMessage(getstr);  //仿真到此能看到getstr收到的数据包
		  data = getstr;
		  len = tmp;
		  PostMessage(ty_msg.rfMsgQueenHandle,id,data,len);
		  radio.bIntSrcFlagClr();
		  radio.vClearFIFO(); 
		  radio.bGoRx();      
		}	
		 
	}
}


/***********************************************************
function_name: rf_proc_task
arg1: void 
return: void
note:射频数据处理任务函数
***********************************************************/

static void rf_proc_task()
{
	int num;
	OPERATE_RET op_ret;
	rfMsg = (MSG_LIST*)malloc(sizeof(MSG_LIST));
	if(!rfMsg)
		return ;
	while(1){
		if(GetMsgNodeNum(ty_msg.rfMsgQueenHandle,&num)==OPRT_OK&&num>0)
		{
			if(GetMsgNodeFromQueue(ty_msg.rfMsgQueenHandle,msgRfId,&rfMsg)==OPRT_OK)
			{
				op_ret = rf_data_valid_check(rfMsg->msg.pMsgData,rfMsg->msg.msgDataLen);
				if(op_ret!=OPRT_OK)
				{
					DelAndFreeMsgNodeFromQueue(ty_msg.rfMsgQueenHandle,rfMsg);
					continue;
				}
					
				op_ret = rf_id_check(rfMsg->msg.pMsgData);
				
				if(rfStudy_flag)
				{
					if(op_ret==OPRT_OK)// 从FLASH中删除对应的UID
					{
						op_ret = rf_add_del_id(rfMsg->msg.pMsgData,false);
						if(op_ret == OPRT_OK)
						{
							rfStudy_flag = FALSE;
							PR_DEBUG(">>>>>>> rf_add_del_id del succeed");
						}else{
							PR_DEBUG(">>>>>>> rf_add_del_id del failed");
						}
					}else{              // 添加UID到FLASH 中
						op_ret = rf_add_del_id(rfMsg->msg.pMsgData,true);
						if(op_ret == OPRT_OK){
							rfStudy_flag = FALSE;
							PR_DEBUG(">>>>>>> rf_add_del_id add succeed");
						}	
						else{
							PR_DEBUG(">>>>>>> rf_add_del_id add failed");
						}	
					}
				}else{
					if(op_ret==OPRT_OK) // 处理具体开关事件
					{
						rf_event_proc(rfMsg->msg.pMsgData,rfMsg->msg.msgDataLen);
					}	
				}

				DelAndFreeMsgNodeFromQueue(ty_msg.rfMsgQueenHandle,rfMsg);
			}
		}
	}
}
/***********************************************************
function_name: led_proc_task
arg1: void 
return: void
note:led状态处理任务函数
***********************************************************/

static void led_proc_task()
{
	OPERATE_RET op_ret;
	int num;
	ledMsg = (MSG_LIST*)malloc(sizeof(MSG_LIST));
	if(!ledMsg)
		return ;
	while(1){
		  if(GetMsgNodeNum(ty_msg.ledMsgQueenHandle,&num)==OPRT_OK&&num>0) 
		  {
			if(GetMsgNodeFromQueue(ty_msg.ledMsgQueenHandle,msgLedId,&ledMsg)==OPRT_OK)
			{		
				if(!rfStudy_flag) // 防止用户错误按键
				{
					DelAndFreeMsgNodeFromQueue(ty_msg.ledMsgQueenHandle,ledMsg);
					continue;
				}
				
				GPIO_OUTPUT_SET(WF_DIR_LED,0); // 两短一长
				SystemSleep(300);
				GPIO_OUTPUT_SET(WF_DIR_LED,1);
				SystemSleep(300);
				GPIO_OUTPUT_SET(WF_DIR_LED,0);
				SystemSleep(300);
				GPIO_OUTPUT_SET(WF_DIR_LED,1);
				SystemSleep(300);
				GPIO_OUTPUT_SET(WF_DIR_LED,0);
				SystemSleep(1000);
				GPIO_OUTPUT_SET(WF_DIR_LED,1);
				SystemSleep(100);
				
			}	
		  }
	}
}


