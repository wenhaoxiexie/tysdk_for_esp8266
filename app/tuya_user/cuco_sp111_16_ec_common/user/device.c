
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
#include "hlw8012.h"
#include "app_dltj.h"
  /***********************************************************
  *************************micro define***********************
  ***********************************************************/
#define SYN_TIME  2  //Í¬²½¿ª¹Ø×´Ì¬
#define ONE_SEC   1
#define WM_SUCCESS 0
#define WM_FAIL 1
  
#define DEVICE_MOD "device_mod"
#define DEVICE_PART "device_part"
#define APPT_POSIX_KEY "appt_posix_key"
#define POWER_STAT_KEY "power_stat_key"
  
#define TIME_POSIX_2016 1451577600 //2016ÄêÊ±¼ä´Á
  
  // reset key define
#define WF_RESET_KEY GPIO_ID_PIN(13)  //WIFI¸´Î»°´¼ü13
  // wifi direct led
#define WF_DIR_LED GPIO_ID_PIN(2)  //WiFiÖ¸Ê¾µÆ
#define POWER_LED  GPIO_ID_PIN(0)   //µçÔ´Ö¸Ê¾µÆ
  // power contrl
#define POWER_CTRL GPIO_ID_PIN(15)  //¼ÌµçÆ÷¿ØÖÆ
//#define POWER_CTRL1 GPIO_ID_PIN(13)  //¼ÌµçÆ÷¿ØÖÆ
//#define POWER_CTRL2 GPIO_ID_PIN(14)  //¼ÌµçÆ÷¿ØÖÆ
  //#define POWER_LED  GPIO_ID_PIN(10)// GPIO_ID_PIN(10)   //µçÔ´¿ª¹ØÖ¸Ê¾µÆ
//#define POWER_USB  GPIO_ID_PIN(5)   //µçÔ´¿ª¹ØÖ¸Ê¾µÆ
  
  typedef enum
  {
	  DP_SWITCH = 1,  //å¼€å…³
	  DP_SWITCH1 ,	//å¼€å…³
	  DP_SWITCH2 ,	//å¼€å…³
	  DP_SWITCH3 ,	//å¼€å…³
	  DP_SWITCH4 ,	//å¼€å…³
	  DP_SWITCH5,  //å¼€å…³
	  DP_SWITCH6 ,	//å¼€å…³
	  DP_SWITCH7 ,	//å¼€å…³
	  //DP_SWITCH8 ,	//å¼€å…³
	 // DP_SWITCH9 ,	//å¼€å…³
	  
	  DP_COUNT_DOWN,  //å€’è®¡æ—¶
	  DP_COUNT_DOWN1,  //å€’è®¡æ—¶
	  DP_COUNT_DOWN2,  //å€’è®¡æ—¶
	  DP_COUNT_DOWN3,  //å€’è®¡æ—¶
	  DP_COUNT_DOWN4,  //å€’è®¡æ—¶
	  DP_COUNT_DOWN5,
	  DP_COUNT_DOWN6,
	  DP_ALARM_USB,
  }DP_ID;

  
  
  typedef struct 
  {
	  INT power;
	  INT appt_sec;
	  TIMER_ID syn_timer;
	  TIMER_ID count_timer;
	  TIMER_ID saver_timer;
	  THREAD msg_thread;
	  THREAD sec_thread;   
	  SEM_HANDLE press_key_sem;
	  SEM_HANDLE sec_up_sem; 
  }TY_MSG_S;
  THREAD uart_ad;  
  SEM_HANDLE led_sem;
  
  /***********************************************************
  *************************variable define********************
  ***********************************************************/
  STATIC TY_MSG_S ty_msg;
  LED_HANDLE wf_light = NULL;
  LED_HANDLE power_ctrl = NULL;

  LED_HANDLE power_led = NULL;

#define KEY_NUM 4

  static uint8 state ;
  static uint8 sec_state ;


   static u8 power_on;
 static u8 poweron_led;
 static u8 pow_led;

  static uint8 power_power;
static  u8 over_current_alarm = 0;

  /***********************************************************
  *************************function define***********************
  ***********************************************************/
  STATIC VOID key_process(INT gpio_no,PUSH_KEY_TYPE_E type,INT cnt);
  STATIC VOID wfl_timer_cb(UINT timerID,PVOID pTimerArg);
  STATIC OPERATE_RET device_differ_init(VOID);
  STATIC OPERATE_RET set_power_stat(INT state);
STATIC VOID crtl_led(VOID);


  extern OPERATE_RET app_dltj_init(APP_DLTJ_MODE mode);
   void prod_cucotest(void);
  void over_usb_protect();

  
  KEY_ENTITY_S key_tbl[] = {
	  {WF_RESET_KEY,3000,key_process,0,0,0,0},
  };
  
  BOOL gpio_func_test(VOID)
  {
	  return TRUE;
  }
  
void pre_app_init()
{

}
  
  VOID set_firmware_tp(IN OUT CHAR *firm_name, IN OUT CHAR *firm_ver)
  {
	  strcpy(firm_name,APP_BIN_NAME);
	  strcpy(firm_ver,USER_SW_VER);
	  return;
  }
  
  STATIC INT msg_upload_proc(INT state,char i)
	  {
		  GW_WIFI_STAT_E wf_stat = tuya_get_wf_status();
		  if(STAT_UNPROVISION == wf_stat || \
			 STAT_STA_UNCONN == wf_stat || \
			 (tuya_get_gw_status() != STAT_WORK)) {
			  return WM_FAIL;
		  }
		  
		  cJSON *root = cJSON_CreateObject();
		  if(NULL == root) {
			  return WM_FAIL;
		  }
		 
		   cJSON_AddBoolToObject(root,"1",state);
		 // if(i == 1 ) {
			  cJSON_AddNumberToObject(root, "2", 0);
		//  }
		  cJSON_AddBoolToObject(root,"7",over_current_alarm);	
		
		  
		  CHAR *out = cJSON_PrintUnformatted(root);
		  cJSON_Delete(root);
		  if(NULL == out) {
			  PR_ERR("cJSON_PrintUnformatted err:");
			  return WM_FAIL;
		  }
		  
	  	PR_DEBUG("msg_upload_proc [%s]", out);
		  OPERATE_RET op_ret =tuya_obj_dp_report(out);// tuya_obj_dp_trans_report(out); 
		  PR_DEBUG("op_ret %d", op_ret);
		  Free(out);
		 
		  if( OPRT_OK == op_ret ) {
			  return WM_SUCCESS;
		  }else {
			  return WM_FAIL;
		  }
	  }
  
  
  STATIC INT msg_upload_sec(INT sec,char num)
	  {
		  GW_WIFI_STAT_E wf_stat = tuya_get_wf_status();
		  if(STAT_UNPROVISION == wf_stat || \
			 STAT_STA_UNCONN == wf_stat || \
			 (tuya_get_gw_status() != STAT_WORK)) {
			  return WM_FAIL;
		  }
		  
		  cJSON *root = cJSON_CreateObject();
		  if(NULL == root) {
			  return WM_FAIL;
		  }
		  cJSON_AddNumberToObject(root, "2", sec);
		  CHAR *out = cJSON_PrintUnformatted(root);
		  cJSON_Delete(root);
		  if(NULL == out) {
			  PR_ERR("cJSON_PrintUnformatted err:");
			  return WM_FAIL;
		  }
		  
	  //	PR_DEBUG("msg_upload_sec[%s]", out);
		  OPERATE_RET op_ret;
		 // if( sec == 0 ) {
			// op_ret = tuya_obj_dp_trans_report(out);
			// PR_DEBUG("op_ret=%d", op_ret);
	//	 }else {
			  op_ret = tuya_obj_dp_report(out);
			 PR_DEBUG("op_ret=%d", op_ret);
		//  }
		  Free(out);
	  
		  if( OPRT_OK == op_ret ) {
			  return WM_SUCCESS;
		  }else {
			  return WM_FAIL;
		  }
	  
	  }
  
  
  STATIC VOID msg_send(INT cmd,char i)
  {
	     static uint8 timer_one;
		if(cmd == ty_msg.power) {
			//
			timer_one++;
			if(!timer_one)
				{PR_DEBUG("msg_upload_proc");
			 //timer_one=0;
			 msg_upload_proc(cmd,0);
				}
			 
		}else {
		      if(timer_one>1)
		       timer_one--;
		  ty_msg.power=cmd;
		  // PR_DEBUG("PostSemaphore");
		 PostSemaphore(ty_msg.press_key_sem);
		}
	}
    STATIC VOID msg_send0(INT cmd,char i)
  {
	     static uint8 timer_one;
		if(cmd ==pow_led ) {//power_led
			//
			timer_one++;
			if(!timer_one)
				{PR_DEBUG("msg_upload_proc");
			 //timer_one=0;
			 msg_upload_proc(ty_msg.power,0);
				}
			 
		}else {
		      if(timer_one>1)
		       timer_one--;
		  pow_led=cmd;
		  // PR_DEBUG("PostSemaphore");
		 PostSemaphore(ty_msg.press_key_sem);
		}
	}
  
  
  STATIC VOID crtl_power(INT state,char i)
	  {
		  static char Power_On,Power_On1,Power_On2;
		  
		 
				  if( state ) 
			  tuya_set_led_type(power_ctrl,OL_HIGH,0);
				 else 
			  tuya_set_led_type(power_ctrl,OL_LOW,0);
				 
#if 	0			 	
		 if( poweron_led)
		 	{
			  if(ty_msg.power){
			  	   if( poweron_led)
				  tuya_set_led_type(power_led,OL_LOW,0);
				   else
				  tuya_set_led_type(power_led,OL_HIGH,0);
				   
					power_power=1;
	 				 }
	  			else
	  			{
				tuya_set_led_type(power_led,OL_HIGH,0);
				power_power=0;

	  			}
		 	}
		 else
		 	{
			 tuya_set_led_type(power_led,OL_HIGH,0);
		    }
#endif		 
		
	  }

STATIC VOID crtl_led(VOID)
{
		 if( poweron_led)
		 	{
			  if(ty_msg.power){
			  	   if( poweron_led)
				  tuya_set_led_type(power_led,OL_LOW,0);
				   else
				  tuya_set_led_type(power_led,OL_HIGH,0);
				   
					power_power=1;
	 				 }
	  			else
	  			{
				tuya_set_led_type(power_led,OL_HIGH,0);
				power_power=0;

	  			}
		 	}
		 else
		 	{
			 tuya_set_led_type(power_led,OL_HIGH,0);
		    }
		
}
  

  STATIC VOID msg_proc(PVOID pArg)
  {
	  while( 1 ) {
		  WaitSemaphore(ty_msg.press_key_sem);
		  //¸ù¾Ýpower×´Ì¬¿ØÖÆ¼ÌµçÆ÷ºÍµçÔ´Ö¸Ê¾µÆ
		   SystemSleep(10);
		  
		  crtl_power(ty_msg.power,0);
		  set_power_stat(ty_msg.power);		  
		  crtl_led();
		  
		  if( ty_msg.appt_sec > 0 ) {
			  ty_msg.appt_sec = 0;
		  }
		  //sys_start_timer(ty_msg.saver_timer,500*10,TIMER_ONCE);
		  SystemSleep(10);			  
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
		  }else if( ty_msg.appt_sec > sec ) {
			  msg_upload_sec(ty_msg.appt_sec - sec,0);
		  }else {
			  msg_upload_sec(0,0);
		  }
		  SystemSleep(20);
	  }
  }

  
  STATIC VOID sec1_proc(PVOID pArg)
  {
	  while( 1 ) {
		  WaitSemaphore(ty_msg.sec_up_sem);
		  INT sec = wmtime_time_get_posix();
		  if( ty_msg.appt_sec == 0 ) {
			  msg_upload_sec(0,1);
		  }else if( ty_msg.appt_sec > sec ) {
			  msg_upload_sec((ty_msg.appt_sec - sec),1);
		  }else {
			  msg_upload_sec(0,1);
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
		  switch(dps)
		  {
			  case DP_SWITCH:		
				  if(nxt->type == cJSON_False) {
					  msg_send(0,DP_SWITCH);
				  }else if(nxt->type == cJSON_True) {
					  msg_send(1,DP_SWITCH);
				  }
				  break;
			/* case DP_SWITCH6:
                
				 	if(nxt->type == cJSON_False) {
					   msg_send0(0,DP_SWITCH6);
				  }else if(nxt->type == cJSON_True) {
					 msg_send0(1,DP_SWITCH6);
				  }
				 // crtl_power();
				break;*/
			  case DP_SWITCH1:
				  //PR_ERR("DP_COUNT_DOWN");
				  if(nxt->type == cJSON_Number) {
					  if( nxt->valueint == 0 ) { //åœæ­¢å®šæ—¶
						  ty_msg.appt_sec = 0;
						  msg_upload_sec(0,DP_COUNT_DOWN);
					  }else {
						  ty_msg.appt_sec = wmtime_time_get_posix() + nxt->valueint;
						  msg_upload_sec(nxt->valueint,DP_COUNT_DOWN);
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
  //	PR_DEBUG("cmd:%d",cmd);
	  cJSON *nxt = root->child; 
       cJSON *last;
	  UCHAR dp_id = 0;
	  while(nxt) {
		  dp_id = atoi(nxt->string);
		   PR_ERR("dp_id=%d",dp_id);
		 if(dp_id)
		  	{
		  deal_dps_handle(dp_id,nxt);
		  nxt = cJSON_False;
		    }
		  SystemSleep(10);
		  
	  }
  }
  
  STATIC VOID syn_timer_cb(UINT timerID,PVOID pTimerArg)
	  {
		  static char Count_timer=0;
			 if(++Count_timer>KEY_NUM)
			  {
				 Count_timer=0;
	  
			   }
		  if( FALSE == tuya_get_cloud_stat() ) {
			  return;
		  }
		  
		  INT ret = msg_upload_proc(ty_msg.power,Count_timer);
		  if( (WM_SUCCESS == ret)&&(Count_timer==0) ) {
			 // if( ty_msg[Count_timer].appt_sec == 0 ) {
				 // ret = msg_upload_sec(ty_msg[Count_timer].appt_sec,Count_timer+DP_COUNT_DOWN);		 
				  //if( WM_SUCCESS == ret ) { 		   
					  sys_stop_timer(ty_msg.syn_timer);
					  // PR_DEBUG("sys_stop_timer");
				//  }
			//  }else {
				//  sys_stop_timer(ty_msg[0].syn_timer);
				 //  PR_DEBUG("sys_stop_timer");
			 // }
		  }
	    PR_DEBUG("syn timer cb ...");
	  }
  
  
  STATIC VOID count_timer_cb(UINT timerID,PVOID pTimerArg)
	  {
 
	  if( (ty_msg.appt_sec == 0))//&&(ty_msg[1].appt_sec == 0)&&(ty_msg[2].appt_sec == 0)&&(ty_msg[3].appt_sec == 0) ) {
		{  return;
	  }

	  INT cur_posix = wmtime_time_get_posix();
	  if( cur_posix < TIME_POSIX_2016 ) {
		  PR_DEBUG("cur_posix:%d", cur_posix);
		  return;
	  }
	  
		  if( cur_posix <= ty_msg.appt_sec){
			  if( (ty_msg.appt_sec - cur_posix)%5 == 0 ) {
			 
			  PostSemaphore(ty_msg.sec_up_sem);//5ç§’æ›´æ–°ä¸€æ¬¡
				  }
			 }
		  else if(ty_msg.appt_sec!=0)
		   {
		  ty_msg.appt_sec = 0;
		  ty_msg.power^=1;
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

         PR_DEBUG("-----------------come to set_power_stat------------------");
	  
	
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
  
  
         power_on=1;
	  pow_led=1;//power_led
	  ty_msg.power = 0;
	  ty_msg.appt_sec = 0;	 
	  get_power_stat();
         power_power=ty_msg.power;
	  crtl_power(ty_msg.power,0);
	  crtl_led();
	  
	//  tuya_set_led_type(power_ctrl,OL_LOW,0);
	 // tuya_set_led_type(power_led,OL_HIGH,0);
		
	  
  }
  void prod_cucotest(void)
  	{
  	OPERATE_RET op_ret;
	//uart_inial();
	 op_ret = tuya_create_led_handle(WF_DIR_LED,&wf_light);
	  if(OPRT_OK  != op_ret) {
		  PR_ERR("POWER_LED error");
	  } 
	//set_led_light_type(wf_light,OL_FLASH_HIGH,300);
	tuya_set_led_type(wf_light,OL_FLASH_HIGH,1500);
	 tuya_set_led_type(power_ctrl,OL_HIGH,0);
    tuya_set_led_type(power_led,OL_LOW,0);
     SystemSleep(1500);
	 
     op_ret= app_dltj_init(D_CAL_START_MODE);
     crtl_power(0,0);
     crtl_led();
	 
	  if(OPRT_OK != op_ret){
        PR_ERR("ele cnt test FAIL");
       tuya_set_led_type(wf_light,OL_HIGH,0);   
    }else{
        PR_DEBUG("ele cnt test OK");
        tuya_set_led_type(wf_light,OL_LOW,0);  
    }
	
    }
  #define ProdTest
    VOID app_init(VOID)
  {
     OPERATE_RET op_ret;
	 uart_inial();
	// PR_ERR("app_init");
	// #ifndef ProdTest
	// tuya_psm_register_module(DEVICE_MOD, DEVICE_PART);
   //  #endif
	op_ret = msf_register_module(DEVICE_MOD,DEVICE_PART);
	if(op_ret != OPRT_OK && op_ret != OPRT_PSM_E_EXIST) {
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
	  if(OPRT_OK  != op_ret) {
		  PR_ERR("POWER_LED error");
	  } /**/
	 tuya_set_led_type(wf_light,OL_HIGH,0);
     tuya_set_led_type(power_led,OL_HIGH,0);
	 PR_ERR("app_init");
	 set_prod_ssid("tuya_mdev_test");
	// PR_ERR("app_init");
	 PR_ERR("app_init");
	 PR_ERR("app_init");
	

	  app_cfg_set(WCM_OLD_CPT, prod_cucotest);
	  PR_ERR("app_init");
	 
	  PR_ERR("app_init");
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

      //app_init();
     uart_inial();
	  PR_DEBUG("fireware info name:%s version:%s",APP_BIN_NAME,USER_SW_VER);
	   char i,*p;
	  OPERATE_RET op_ret;
	 // p=tuya_get_sdk_ver();
	 
	  op_ret = tuya_device_init(PRODECT_KEY,device_cb,USER_SW_VER);
	  if(op_ret != OPRT_OK) {
		  return op_ret;
	  }
  //#ifndef ProdTest
	  op_ret = tuya_psm_register_module(DEVICE_MOD, DEVICE_PART);
	  if(op_ret != OPRT_OK && op_ret != OPRT_PSM_E_EXIST) {
		  PR_ERR("tuya_psm_register_module error:%d",op_ret);
		  return op_ret;
	  }
// #endif
      
  	 ID=tuya_get_devid();
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
	  char buf[4];
	
	
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
	  }else {
		  sys_start_timer(ty_msg.syn_timer,SYN_TIME*1000,TIMER_CYCLE);
	  }
  
	  op_ret = sys_add_timer(count_timer_cb,NULL,&ty_msg.count_timer);
	  if(OPRT_OK != op_ret) {
		  PR_ERR("add syn_timer err");
		  return op_ret;
	  }else {
		  sys_start_timer(ty_msg.count_timer,ONE_SEC*1000,TIMER_CYCLE);
	  }
	  
	  op_ret = sys_add_timer(save_timer_cb,NULL,&ty_msg.saver_timer);
	  if(OPRT_OK != op_ret) {
		 PR_ERR("add syn_timer err");
		 return op_ret;
		}
	 PR_DEBUG("app_dltj_init");
     op_ret= app_dltj_init(APP_DLTJ_NORMAL);//D_NORMAL_MODE
     if(OPRT_OK != op_ret) {
        PR_ERR("dltj init err!");
        return op_ret;
     }/* */
	 op_ret = device_differ_init();
	 if(op_ret != OPRT_OK) {
		return op_ret;
	 }
    //  user_uart_raw_full_init(BIT_RATE_4800,UART_WordLength_8b,USART_Parity_Even,USART_StopBits_1);
	  uart_inial();
	  return op_ret;
  }
  
  
  STATIC VOID key_process(INT gpio_no,PUSH_KEY_TYPE_E type,INT cnt)
  {
	  PR_DEBUG("gpio_no: %d",gpio_no);
	  PR_DEBUG("type: %d",type);
	  PR_DEBUG("cnt: %d",cnt);
      
	  if(WF_RESET_KEY == gpio_no) {
		  if(LONG_KEY == type) {
			  tuya_dev_reset_factory();
		  }
		  else
		  {
		  if(!power_on)
		  	{
			power_power^=1;
			ty_msg.power^=1;
		    PostSemaphore(ty_msg.press_key_sem);
		  	}
	      }
	  	}
  }

VOID over_protect()
{
	  PR_DEBUG("over protect,  turn off the power!");
	  
	  if(power_power) {
	  	  power_power=0;
		  ty_msg.power=0;
		  PostSemaphore(ty_msg.press_key_sem);
         }
}


  STATIC OPERATE_RET device_differ_init(VOID)
  {
	  OPERATE_RET op_ret;
  
  //	set_key_detect_time(50);
	  
	  // key process init
	  op_ret = tuya_kb_init();
	  if(OPRT_OK  != op_ret) {
		  return op_ret;
	  }
	  tuya_set_kb_seq_enable(FALSE);
      tuya_set_kb_trig_type(WF_RESET_KEY,KEY_UP_TRIG,FALSE);
	  // register key to process
	  op_ret = tuya_kb_reg_proc(WF_RESET_KEY,3000,key_process);
	  if(OPRT_OK  != op_ret) {
		  return op_ret;
	  }
  
	  // create led handle
	  op_ret = tuya_create_led_handle(WF_DIR_LED,&wf_light);
	  if(OPRT_OK  != op_ret) {
		  return op_ret;
	  }
  
	  TIMER_ID timer;
	  op_ret = sys_add_timer(wfl_timer_cb,NULL,&timer);
	  if(OPRT_OK != op_ret) {
		  return op_ret;
	  }else {
		  sys_start_timer(timer,300,TIMER_CYCLE);
	  }

  
	  return OPRT_OK;
  }
  
  STATIC VOID wfl_timer_cb(UINT timerID,PVOID pTimerArg)
  {
	  STATIC UINT last_wf_stat = 0xffffffff;
	  static u8 timer=0;

	  if(power_on)
	  	{
        if(++timer>10)
			{power_on=0;timer=0;
	       }
	    }
	  GW_WIFI_STAT_E wf_stat = tuya_get_wf_status();
	  //  PR_DEBUG("wf_stat:%d",wf_stat);
	  if(last_wf_stat != wf_stat) {
		  PR_DEBUG("wf_stat:%d",wf_stat);
		  switch(wf_stat) {
			  case STAT_UNPROVISION: {
			  	poweron_led=0;
				tuya_set_led_type(power_led,OL_HIGH,0);
				tuya_set_led_type(wf_light,OL_FLASH_HIGH,150);
			  }
			  break;
			  
			  case STAT_AP_STA_UNCONN:
			  case STAT_AP_STA_CONN: {
			  		poweron_led=0;
				  tuya_set_led_type(power_led,OL_HIGH,0);
				  tuya_set_led_type(wf_light,OL_FLASH_HIGH,1000);
			  }
			  break;
  
			  case STAT_LOW_POWER:
			  case STAT_STA_UNCONN: {
			     	if(!poweron_led) {
					poweron_led=1;
					crtl_power(ty_msg.power,0);
    				        crtl_led();

			  	}
				//tuya_set_led_type(wf_light,OL_HIGH,0);
			       tuya_set_led_type(wf_light,OL_FLASH_HIGH,1500);								  
			  }
			  break;
			  
			  case STAT_STA_CONN: {
			  		if(!poweron_led)
						{poweron_led=1;
					crtl_power(ty_msg.power,0);
   				        crtl_led();

			  			}
				  tuya_set_led_type(wf_light,OL_HIGH,0);
     	   		         sys_start_timer(ty_msg.syn_timer,SYN_TIME*1000,TIMER_CYCLE);

			  }
			  break;
		  }
  
		  last_wf_stat = wf_stat;
	  }
  }
  
  
void msg_upload_ext(void)
{
	over_current_alarm = 1;
	msg_upload_proc(ty_msg.power,0);	  
        SystemSleep(50);	
	over_current_alarm = 0;
	msg_upload_proc(ty_msg.power,0);	  
	

}
