 /***********************************************************
  *  File: device.c  
  *  Author: husai
  *  Date: 20151215
  ***********************************************************/
#define __DEVICE_GLOBALS

#include "device.h"
#include "tuya_smart_api.h"
#include "gpio.h"
#include "wf_sdk_adpt.h"
#include "apt8s10.h"
#include "ws100t10.h"

#define SYN_TIME  2  //同步开关状态
#define ONE_SEC   1
#define WM_SUCCESS 0
#define WM_FAIL 1
  
#define DEVICE_MOD "device_mod"
#define DEVICE_PART "device_part"
#define APPT_POSIX_KEY "appt_posix_key"
#define POWER_STAT_KEY "power_stat_key"
  
#define TIME_POSIX_2016 1451577600     //2016年时间戳
  
#define WF_RESET_KEY GPIO_ID_PIN(0)    //WIFI复位按键
#define WF_DIR_LED   GPIO_ID_PIN(15)   //WiFi指示灯
#define POWER_LED    GPIO_ID_PIN(13)   //电源指示灯
#define POWER_CTRL   GPIO_ID_PIN(12)   //继电器控制

//开关(可下发可上报)
//备注:
#define DPID_LAMP_SWITCH 1
//亮度(可下发可上报)
//备注:
#define DPID_BRIGHT_VALUE 3

#define SYN_NUM 4
 
typedef struct 
{
  INT power;
  TIMER_ID syn_timer;
  THREAD   msg_thread;
  THREAD   touch_thread;	  	  
  SEM_HANDLE press_key_sem;
}TY_MSG_S; 
  
STATIC TY_MSG_S ty_msg;
LED_HANDLE wf_light = NULL;  
LED_HANDLE power_led = NULL;

static u8 power_on;
static u8 g_power_ctrl_first = 1;
static uint32 pwm_vacancyRatio = 0; 
static bool    g_pwm_switch_on = false;

u8 power_led_status = 0;

UCHAR touch_key_read_data = 0;
UCHAR bright_level = 0;
UCHAR touch_led_ctrl_value = 0;
UCHAR app_bright_value = 0;

STATIC VOID key_process(INT gpio_no,PUSH_KEY_TYPE_E type,INT cnt);
STATIC VOID wfl_timer_cb(UINT timerID,PVOID pTimerArg);
STATIC OPERATE_RET device_differ_init(VOID);
void over_usb_protect();
STATIC VOID touch_proc(PVOID pArg);    

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

  cJSON_AddNumberToObject(root, "3", app_bright_value);		
  
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
	
  cJSON_AddNumberToObject(root, "9", sec);

  CHAR *out = cJSON_PrintUnformatted(root);
  cJSON_Delete(root);
  if(NULL == out) {
	  PR_ERR("cJSON_PrintUnformatted err:");
	  return WM_FAIL;
  }
  
	PR_DEBUG("msg_upload_sec[%s]", out);
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
  if(cmd == ty_msg.power) {
	  msg_upload_proc(cmd,0);
  }else {
  	ty_msg.power = cmd;
   PostSemaphore(ty_msg.press_key_sem);
  }
}
  
STATIC VOID ctrl_power_led(INT state,char i)
{		 
     if(state) {
	   if (g_power_ctrl_first) {
		g_power_ctrl_first = 0;
		bright_level = 7;
		app_bright_value = 255;
	   }
	   g_pwm_switch_on = true;	
	   power_led_status = 1;
	   //pwm_vacany_ratio_set(bright_level);
	  // touch_FluidLamp(bright_level, 1);
     }
     else {
	   g_pwm_switch_on = false;
	   power_led_status = 0;
	   //touch_ctrl_led_fun(0x01 << (bright_level - 1));
     }
}

STATIC VOID msg_proc(PVOID pArg)
{
  while( 1 ) {
	  WaitSemaphore(ty_msg.press_key_sem);
	  
	  //根据power状态控制继电器和电源指示灯
	  ctrl_power_led(ty_msg.power,0);
	  msg_upload_proc(ty_msg.power,0);
	  SystemSleep(100);
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
	  case DPID_LAMP_SWITCH:		
		  if(nxt->type == cJSON_False) {
			  msg_send(0,DPID_LAMP_SWITCH);
		  }else if(nxt->type == cJSON_True) {
			  msg_send(1,DPID_LAMP_SWITCH);
		  }
		  break;

	  case DPID_BRIGHT_VALUE:	
	  	   PR_DEBUG("json valueint: %d",nxt->valueint);
		   app_bright_value = nxt->valueint;
		   if (ty_msg.power) {	  	 
			bright_level = app_bright_level_change(app_bright_value);
			touch_led_ctrl_value = apt8s10_DirLed_ctrl_value_set(bright_level);
			//touch_ctrl_led_fun(touch_led_ctrl_value);					
			//pwm_vacany_ratio_set(bright_level);
			msg_upload_proc(1,0);
		  } 				   
		  break;				  
		
	  default:
		  break;
  }
}
  
VOID device_cb(SMART_CMD_E cmd,cJSON *root)
{  
  cJSON *nxt = root->child; 
  cJSON *last;
  UCHAR dp_id = 0;
  while(nxt) {
	  dp_id = atoi(nxt->string);
	   PR_DEBUG("dp_id=%d",dp_id);
	 if(dp_id)
	  	{
	  deal_dps_handle(dp_id,nxt);
	  nxt = cJSON_False;
	    }
	  SystemSleep(100); 
  }
}

STATIC VOID syn_timer_cb(UINT timerID,PVOID pTimerArg)
{
	static char Count_timer = 0;
	
	if(++Count_timer > SYN_NUM) 
		 Count_timer = 0;
	
	if( FALSE == tuya_get_cloud_stat()) 
	     return;
	
	INT ret = msg_upload_proc(ty_msg.power, Count_timer);
	
	if( (WM_SUCCESS == ret) && (Count_timer == 0) ) 
	      sys_stop_timer(ty_msg.syn_timer);

	PR_DEBUG("syn timer cb ...");
}
  
STATIC VOID syn_active_data(VOID)
{
  msg_upload_proc(ty_msg.power,0);
}

void uart_inial (void)
{
	print_port_init(UART0);
	//print_port_full_init(UART0,BIT_RATE_115200,UART_WordLength_8b,USART_Parity_None,USART_StopBits_1);
		   
	//user_uart_raw_full_init(BIT_RATE_4800,UART_WordLength_8b,USART_Parity_None,USART_StopBits_1_5);

	user_uart_raw_full_init(BIT_RATE_74880,UART_WordLength_8b,USART_Parity_None,USART_StopBits_1);
		   
}

VOID app_init(VOID)
{
 //tuya_set_wf_cfg(0);
 uart_inial();
 app_cfg_set(WCM_OLD,NULL);
}

STATIC VOID init_power_stat(VOID)
{
  power_on = 1;
  ty_msg.power = 0;
  bright_level = 0;
  app_bright_value = 25;	
  ctrl_power_led(ty_msg.power,0);
}

OPERATE_RET device_init(VOID)
{
  char *ID;
  char i,*p;
  OPERATE_RET op_ret;

  uart_inial();
  apt8s10_init();
  ws100t10_init();
  
  PR_NOTICE("fireware info name:%s version:%s",APP_BIN_NAME,USER_SW_VER);
 
  op_ret = tuya_device_init(PRODECT_KEY,device_cb,USER_SW_VER);
  if(op_ret != OPRT_OK) {
	  return op_ret;
  }

  op_ret = tuya_psm_register_module(DEVICE_MOD, DEVICE_PART);
  if(op_ret != OPRT_OK && op_ret != OPRT_PSM_E_EXIST) {
	  PR_ERR("tuya_psm_register_module error:%d",op_ret);
	  return op_ret;
  }
  
  ID = tuya_get_devid();
  PR_NOTICE("ID:%s",ID);

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
  
  op_ret = CreateAndStart(&ty_msg.msg_thread,msg_proc,NULL,1024,TRD_PRIO_3,"ty_task");
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
  
 op_ret = device_differ_init();
 if(op_ret != OPRT_OK) {
	return op_ret;
 }
  
  return op_ret;
}
  
STATIC VOID key_process(INT gpio_no,PUSH_KEY_TYPE_E type,INT cnt)
{
	PR_DEBUG("key gpio_no: %d",gpio_no);
	PR_DEBUG("type: %d",type);
	PR_DEBUG("cnt: %d",cnt);

	if(WF_RESET_KEY != gpio_no) 
		return;

        if (LONG_KEY == type) 
	       tuya_dev_reset_factory();
        else  {	
	       if(!power_on) {
		   	if(ty_msg.power)
				tuya_set_led_type(power_led,OL_LOW,0);
			else
				tuya_set_led_type(power_led,OL_HIGH,0);
			ty_msg.power ^= 1;
 		       PostSemaphore(ty_msg.press_key_sem);
	       }
        }
}

STATIC OPERATE_RET device_differ_init(VOID)
{
  OPERATE_RET op_ret;
   
  op_ret = CreateAndStart(&ty_msg.touch_thread,touch_proc,NULL,1024,TRD_PRIO_2,"cuco_task");
  if(op_ret != OPRT_OK) {
      return op_ret;
  }
  
  op_ret = tuya_kb_init();
  if(OPRT_OK  != op_ret) {
	  return op_ret;
  }
  
  tuya_set_kb_seq_enable(FALSE);
  tuya_set_kb_trig_type(WF_RESET_KEY,KEY_UP_TRIG,FALSE);
  
  op_ret = tuya_kb_reg_proc(WF_RESET_KEY,3000,key_process);
  if(OPRT_OK  != op_ret) {
	  return op_ret;
  }

  op_ret = tuya_create_led_handle(WF_DIR_LED,&wf_light);
  if(OPRT_OK  != op_ret) {
	  return op_ret;
  }

  op_ret = tuya_create_led_handle(POWER_LED,&power_led);
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
    if(++timer > 10)
	{
	    power_on = 0;
	    timer = 0;
    }
  }
  
  GW_WIFI_STAT_E wf_stat = tuya_get_wf_status();
  if(last_wf_stat != wf_stat) {
	  PR_DEBUG("wf_stat:%d",wf_stat);
	  switch(wf_stat) {
		  case STAT_UNPROVISION: 				
			   tuya_set_led_type(wf_light,OL_FLASH_HIGH,250);
		          break;
		  
		  case STAT_AP_STA_UNCONN:
		  case STAT_AP_STA_CONN: 				  
			   tuya_set_led_type(wf_light,OL_FLASH_HIGH,1500);
		          break;

		  case STAT_LOW_POWER:
		  case STAT_STA_UNCONN: 
			   tuya_set_led_type(wf_light,OL_HIGH,0);
		          break;
		  
		  case STAT_STA_CONN:
			   tuya_set_led_type(wf_light,OL_HIGH,0);
		          break;
	  }  
	  last_wf_stat = wf_stat;
  }
}

STATIC VOID touch_proc(PVOID pArg)
{
	static u8 g_last_key_read = 0;
	u8 i=0;
	
	apt8s10_DirLed_level(7, 0);
	apt8s10_DirLed_OnOff(0x00);

	while( 1 ) {  
		  //if (0 == ty_msg.power)
		  	 //continue;
		  if (0==INT_STATE()) {
			 apt8s10_readData(&touch_key_read_data, 1);
			 if (0 == touch_key_read_data) {
			      if (g_last_key_read > 0)
				   continue;
			 }
			 g_last_key_read = touch_key_read_data;
			  
			 SystemSleep(10);
			 
			 ws100t10_ControlLight(touch_key_read_data,0);
			 	 
			 bright_level = apt8s10_DirLed_level_change(touch_key_read_data);
			 app_bright_value = app_bright_value_back(bright_level);
			 touch_led_ctrl_value = apt8s10_DirLed_ctrl_value_set(bright_level);
			 apt8s10_DirLed_OnOff(touch_led_ctrl_value);
			 msg_upload_proc(1,0);
		  }
	}	

}


