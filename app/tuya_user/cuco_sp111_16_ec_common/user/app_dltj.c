/***********************************************************
*  File: app_dltj.c
*  Author: litao
*  Date: 170704
***********************************************************
*  Author: wym
*  Date: 180409
*  Description:上报线程重构，增加上报过滤，过流保护，修复电
   量存储一部分bug。
***********************************************************
*  Author: wym
*  Date: 180418
*  Description:调整了电量参数获取和上报顺序，删除了不必要的
   变量和处理。
***********************************************************
*  Author: wym
*  Date: 180504
*  Description:调整了上报线程的结构和初始化结构,动态调整电流
   电压功率上报间隔。
***********************************************************/
#define _APP_DLTJ_GLOBAL
#include "app_dltj.h"
#include "cJSON.h"
#include "tuya_ws_db.h"
#include "system/uni_time_queue.h"
#include "system/uni_thread.h"
#include "system/uni_semaphore.h"
#include "mem_pool.h"
#include "com_def.h"
#include "wf_sdk_adpt.h"
#include "smart_wf_frame.h"


/***********************************************************
*************************micro define***********************
***********************************************************/
#if _APP_DLTJ_DEBUG
#define APP_DLTJ_DEBUG  PR_DEBUG
#else
#define APP_DLTJ_DEBUG(...)
#endif



#define DEVICE_MOD "device_mod"
#define DEVICE_PART "device_part"
#define ELE_SAVE_KEY "ele_save_key"
#define TEM_ELE_SAVE_KEY "tem_ele_save_key"

#define TIME_POSIX_2016 1451577600

#define ADD_ELE_THRE 10//临时电量累加处理阈值
#define ADD_TIME_MIN_THRE 15//电量处理最小时间间隔
#define ADD_TIME_MAX_THRE 1800//电量处理最大时间间隔，单位S
#define PVI_REPORT_MIN_THRE 5 //电流电压功率数据上报最小间隔
#define PVI_REPORT_CHG_THRE 2 //电流电压功率数据上报最小间隔

#define PVI_REPORT_MAX_THRE 3600 //电流电压功率数据上报最大间隔

#define REPT_THRE_CURR 5//电流上报变化阈值，单位%
#define REPT_THRE_VOL 2//电压上报变化阈值，单位%
#define REPT_THRE_PWR 5//功率上报变化阈值，单位%
#define ELE_SLEEP_TIME 1//电量线程休眠间隔

#define _IS_OVER_PROTECT 1//是否存在过流保护功能
//#define LIMIT_CURRENT 12000//过流保护限值，单位1mA

#if 1
#define LIMIT_CURRENT 16000//过流保护限值，单位1mA
#define ALARM_CURRENT 15300//过流保护限值，单位1mA
#else
#define LIMIT_CURRENT 500//过流保护限值，单位1mA
#define ALARM_CURRENT 300//过流保护限值，单位1mA
#endif


#define _IF_REPT_COE   TRUE     //是否上报电量校准系数


/***********************************************************
*************************variable define********************
***********************************************************/


typedef struct{
    u8 edpid;
    u8 idpid;
    u8 pdpid;
    u8 vdpid;
}AppDltjInfo_S,*pAppDltjInfo_S;

typedef struct 
{
    UINT unix_time;
	UINT ele_energy;
}LOC_DATA;

typedef struct 
{
	UINT add_ele;//下层电量统计驱动中累加的电量
	UINT cur_posix;//当前时间戳
	UINT loc_data_cnt;//本地电量数据组的元素个数
	UINT cur_current;
	UINT cur_vol;
	UINT cur_power;
	UINT last_current;
	UINT last_vol;
	UINT last_power;
	UINT tem_ele_val;//存放已经累加的但还未上报或存储到local_data中的电量
	THREAD report_thread;
	LOC_DATA ele_sav_data[10];//本地电量数据组
}POWER_CNT;
STATIC POWER_CNT power_cnt;

typedef enum{
    ELE_NOT_ACTIVE,//上电后未配网激活
    ELE_NORMAL,//已配网成功且当前mqtt连接成功
    ELE_UNCONNECT,//已配网成功当当前mqtt连接失败
    ELE_SYS_OTA//系统正在进行在线升级
}ELE_THREAD_STATE;
STATIC ELE_THREAD_STATE ele_state;
STATIC uint32 ele_handle_time = 0;//距离上一次电量上报之后的时间，单位比1S略大一些
STATIC uint32 pvi_handle_time = 0;
STATIC uint32 pvi_report_min_time = PVI_REPORT_MIN_THRE;//当前上报间隔，可调


typedef struct{
    TIMER_ID                    app_dltj_timer_id;
    THREAD                      app_dltj_thread;
    SEM_HANDLE                  app_dltj_sem;
    AppDltjInfo_S	            app_dltj_info;
}AppDltj_S,*pAppDltj_S;
AppDltj_S mAppDltj;


/***********************************************************
*************************function define********************
***********************************************************/
STATIC VOID app_dltj_proc(PVOID pArg);
STATIC BOOL same_day_judge(UINT u_time1, UINT u_time2);
STATIC OPERATE_RET get_time_posix(UINT *curPosix);
STATIC OPERATE_RET set_tem_ele_val(INT val);
STATIC OPERATE_RET get_tem_ele_val(INT *val);
STATIC OPERATE_RET set_ele_data(VOID);
STATIC OPERATE_RET get_ele_data(VOID);
STATIC VOID addto_local_data(UINT time, UINT ele);
STATIC OPERATE_RET report_pvi_data(VOID);
STATIC OPERATE_RET update_ele_data(UINT time, UINT ele_value);
STATIC VOID report_local_data(VOID);
STATIC VOID save_reported_pvi_data();
STATIC BOOL value_range_judge(UINT JudgedValue, UINT TargetValue, UINT range);
STATIC VOID pvi_data_handle();
STATIC VOID add_ele_handle();
STATIC BOOL if_pvi_need_report();
STATIC OPERATE_RET dltj_driver_init(UCHAR mode);
extern VOID over_protect();
extern void msg_upload_ext(void);


STATIC OPERATE_RET dltj_driver_init(UCHAR mode)
{
    DLTJ_CONFIG dltj_config_data;
    dltj_config_data.epin = 5;
    //dltj_config_data.ivpin = 14;
    dltj_config_data.ivpin = 4;	
    dltj_config_data.ivcpin.pin = 12;
    dltj_config_data.ivcpin.type = IO_DRIVE_LEVEL_HIGH;
#if 0
    dltj_config_data.v_ref = 1700;
    dltj_config_data.i_ref = 19230;
    dltj_config_data.p_ref = 62000;
    dltj_config_data.e_ref = 342;
#else
    dltj_config_data.v_ref = 177;
    dltj_config_data.i_ref = 625;
    dltj_config_data.p_ref = 375;
    dltj_config_data.e_ref = 2698;
#endif
    dltj_config_data.v_def = 2200;
    dltj_config_data.i_def = 8650;
    dltj_config_data.p_def = 1870;
    dltj_config_data.if_have = TRUE;
    hlw8012_init(&(dltj_config_data));
    OPERATE_RET op_ret;
    op_ret = ele_cnt_init(mode);
    return op_ret;

}

OPERATE_RET app_dltj_init(APP_DLTJ_MODE mode)
{
    OPERATE_RET op_ret;
	PR_ERR("app_dltj_init");
    if(APP_DLTJ_NORMAL == mode){
        op_ret = dltj_driver_init(D_NORMAL_MODE);
        if(OPRT_OK != op_ret){
            PR_ERR("mode:%d,dltj driver init err!!!",mode);
            return op_ret;
        }
        mAppDltj.app_dltj_info.edpid = DP_ELE;
        mAppDltj.app_dltj_info.idpid = DP_CURRENT;
        mAppDltj.app_dltj_info.vdpid = DP_VOLTAGE;
        mAppDltj.app_dltj_info.pdpid = DP_POWER;
      //  if(OPRT_OK != get_ele_data()){
     //       APP_DLTJ_DEBUG("get ele data err...");
      //  }
       // if(OPRT_OK != get_tem_ele_val(&power_cnt.tem_ele_val)){
      //      APP_DLTJ_DEBUG("get tem ele data err...");
     //   }
        op_ret = CreateAndStart(&mAppDltj.app_dltj_thread,app_dltj_proc,\
        NULL,1024+1024,TRD_PRIO_2,"app_dltj_task");
        if(op_ret != OPRT_OK) {
            PR_ERR("creat ele thread err...");
            return op_ret;
        }
    }else{
        op_ret = dltj_driver_init(D_CAL_START_MODE);
        if(OPRT_OK != op_ret){
            PR_ERR("mode:%d,dltj prod test err!!!",mode);
            return op_ret;
        }
    }
    return OPRT_OK;
}
u32 Cal_number_reall(u32 *buf,u32 num)
{
 u8 j,k;
 u32 temp;
// APP_DLTJ_DEBUG("II:%d	cur_power[ii]:%d cur_vol[ii]:%d cur_current[ii]:%d",ii,cur_power[ii],cur_vol[ii],cur_current[ii]);
 //for(j=0;j<=num;j++)
// 	{
// APP_DLTJ_DEBUG("buf[%d]:%d",j,buf[j]);
 //	}
 for(j=0;j<2;j++)
 	{
     for(k=j+1;k<=num;k++)
     	{
		if(buf[j]>buf[k])
			{
			 temp=buf[k];
		     buf[j]=buf[k];
			 buf[k]=temp;
			}
			
	    }

	
    }

  for(j=2;j<4;j++)
 	{
     for(k=j+1;k<=num;k++)
     	{
		if(buf[j]<buf[k])
			{
			 temp=buf[k];
		     buf[j]=buf[k];
			 buf[k]=temp;
			}
			
	    }
    }
  buf[4]=(buf[4]+buf[5])/2;
  return(buf[4]);

}

u32 Cal_current_reall(u32 current,u32 vol ,u32 power,UINT old_curent)
{
u32 curent,curr ;
 if(power)
 	{
 	curent=(power*1000/vol);
	curr=curent>current?(curent-current):(current-curent);
	if(curr<(curent*15/1000))
		return (current+old_curent)/2;
	else
		return curent;
    
    }
 else
 	return current;

}

#if  0
STATIC VOID pvi_data_handle()
{
static u32 cur_power[6];
static u32 cur_vol[6];
static u32 cur_current[6];
static u8 ii=0;
static UINT oldcurrent=0;
#if 1
   
   get_ele_par(&cur_power[ii],&cur_vol[ii],&cur_current[ii]);


//APP_DLTJ_DEBUG("II:%d	cur_power[ii]:%d cur_vol[ii]:%d cur_current[ii]:%d",ii,cur_power[ii],cur_vol[ii],cur_current[ii]);

if(++ii>5)
{
	//APP_DLTJ_DEBUG("ii:%d",ii);
      power_cnt.cur_power=Cal_number_reall(cur_power,5);
	  power_cnt.cur_vol=Cal_number_reall(cur_vol,5);
	  power_cnt.cur_current=Cal_number_reall(cur_current,5);
	  power_cnt.cur_current=Cal_current_reall( power_cnt.cur_current,power_cnt.cur_vol,power_cnt.cur_power,oldcurrent);
	  oldcurrent=power_cnt.cur_current;
	ii=0;	
}
/*

get_ele_par( &power_cnt.cur_power,&power_cnt.cur_vol, &power_cnt.cur_current);
*/


#else
    power_cnt.cur_power++;
    power_cnt.cur_vol++;
    power_cnt.cur_current+= 1000;
#endif
    APP_DLTJ_DEBUG("cur:%d power:%d vol:%d",power_cnt.cur_current,power_cnt.cur_power,power_cnt.cur_vol);
  //  if(_IS_OVER_PROTECT){
        if(power_cnt.cur_current >= LIMIT_CURRENT){
            over_protect();
        }
   // }
}

#else
STATIC VOID pvi_data_handle()
{
static u32 cur_power[6];
static u32 cur_vol[6];
static u32 cur_current[6];
static u8 ii=0;
static UINT oldcurrent=0;
static u8 timers_alarm=0,timers_limit=0;

#if 1
   
   get_ele_par(&cur_power[ii],&cur_vol[ii],&cur_current[ii]);


//APP_DLTJ_DEBUG("II:%d	cur_power[ii]:%d cur_vol[ii]:%d cur_current[ii]:%d",ii,cur_power[ii],cur_vol[ii],cur_current[ii]);

if(++ii>5)
{
	//APP_DLTJ_DEBUG("ii:%d",ii);
      power_cnt.cur_power=Cal_number_reall(cur_power,5);
	  power_cnt.cur_vol=Cal_number_reall(cur_vol,5);
	  power_cnt.cur_current=Cal_number_reall(cur_current,5);
	  power_cnt.cur_current=Cal_current_reall( power_cnt.cur_current,power_cnt.cur_vol,power_cnt.cur_power,oldcurrent);
	  oldcurrent=power_cnt.cur_current;
	ii=0;	
}
/*

get_ele_par( &power_cnt.cur_power,&power_cnt.cur_vol, &power_cnt.cur_current);
*/


#else
    power_cnt.cur_power++;
    power_cnt.cur_vol++;
    power_cnt.cur_current+= 1000;
#endif
    APP_DLTJ_DEBUG("cur:%d power:%d vol:%d",power_cnt.cur_current,power_cnt.cur_power,power_cnt.cur_vol);

        if(power_cnt.cur_current >= LIMIT_CURRENT){
	     if (++timers_limit > 2) {
		  PR_DEBUG("---------------------------over current limit, cur: %d", power_cnt.cur_current);	
		  PR_DEBUG("---------------------------over current limit, timers_limit: %d", timers_limit);		  			  
		  msg_upload_ext();
		  over_protect();
		  power_cnt.cur_current = 0;
		  timers_limit = 0;			  
	     }
        }
	 else if(power_cnt.cur_current >= ALARM_CURRENT) {
	     timers_alarm++;	
	     if (timers_alarm == 3) {
		  PR_DEBUG("---------------------------over current alarm, cur: %d", power_cnt.cur_current);		
		  PR_DEBUG("---------------------------over current alarm, timers_alarm: %d", timers_alarm);		  			  
		  msg_upload_ext();
	     }
	     else if (timers_alarm > 22) {
            	  over_protect();
		  power_cnt.cur_current = 0;				  
		  timers_alarm	= 0;	  
	     }
	 }
	 else {
		timers_limit = 0;
		timers_alarm = 0;
		PR_DEBUG("--------------------------- cur: %d", power_cnt.cur_current);		  			
	 }
}
#endif
/*联网激活前，所有新增加的电量存储在power_cnt.tem_ele_val中，并存储在TEM_ELE_SAVE_KEY区，
 激活之后，将power_cnt.tem_ele_val上报并清空TEM_ELE_SAVE_KEY，此后对于新增加的电量，先尝试
 上报，上报失败则存在ele_sav_data数组中，联网状态下电量线程每次循环会尝试上报ele_sav_data数组*/
STATIC VOID add_ele_handle()
{
    get_ele(&power_cnt.add_ele);
    get_time_posix(&power_cnt.cur_posix);
    //power_cnt.add_ele = 10;
    power_cnt.tem_ele_val += power_cnt.add_ele;
    APP_DLTJ_DEBUG("add_ele = %d,tem_ele_val = %d",power_cnt.add_ele,power_cnt.tem_ele_val);
    power_cnt.add_ele = 0;
    //电量处理事件触发
    if((power_cnt.tem_ele_val >= ADD_ELE_THRE && ele_handle_time >= ADD_TIME_MIN_THRE ))
    //|| ele_handle_time >= ADD_TIME_MAX_THRE)
    {
        ele_handle_time = 0;
        if(power_cnt.tem_ele_val){
            if(ELE_NOT_ACTIVE == ele_state){
                //set_tem_ele_val(power_cnt.tem_ele_val);
                APP_DLTJ_DEBUG("set tem ele val :%d success...",power_cnt.tem_ele_val);
            }else if(ELE_NORMAL == ele_state){
                if(OPRT_OK != update_ele_data(power_cnt.cur_posix, power_cnt.tem_ele_val)){
                power_cnt.tem_ele_val = 0;
                }
				else
                power_cnt.tem_ele_val = 0;
            }else if(ELE_UNCONNECT == ele_state || ELE_SYS_OTA == ele_state){
               // addto_local_data(power_cnt.cur_posix, power_cnt.tem_ele_val);
               // power_cnt.tem_ele_val = 0;
            }
        }
    }
}


STATIC VOID app_dltj_proc(PVOID pArg)
{
    ele_state = ELE_NOT_ACTIVE;
    while(1){
        switch(ele_state){
        case ELE_NOT_ACTIVE:
            if(get_gw_mq_conn_stat() && OPRT_OK == get_time_posix(&power_cnt.cur_posix)){
                ele_state = ELE_NORMAL;//首次联网激活，将临时电量上报并清零临时电量内存
                pvi_handle_time = pvi_report_min_time;
                ele_handle_time = ADD_TIME_MAX_THRE;
                //set_tem_ele_val(0);
                if(_IF_REPT_COE){
                    if(OPRT_OK != report_coe_data()){
                        PR_DEBUG("report coe data fail!!!");
                    }
                }
            }
            break;
        case ELE_NORMAL:
        case ELE_UNCONNECT:
            if(UPGRADING == get_fw_ug_stat()){
                ele_state = ELE_SYS_OTA;
                break;
            }
            if(get_gw_mq_conn_stat() != TRUE){
                ele_state = ELE_UNCONNECT;
            }else{
                ele_state = ELE_NORMAL;
            }
            break;
        case ELE_SYS_OTA://OTA结束成功将重启,失败将继续工作
            if(UG_EXECPTION == get_fw_ug_stat()||UG_FIN == get_fw_ug_stat()){
                ele_state = ELE_NORMAL;
            }
            break;
        }
        pvi_data_handle();
        add_ele_handle();
        if(ele_state == ELE_NORMAL){//&& if_pvi_need_report()
            if((pvi_handle_time >= pvi_report_min_time )\
            || pvi_handle_time >= PVI_REPORT_MAX_THRE){
                pvi_handle_time = 0;
                if(OPRT_OK == report_pvi_data()){
                    //save_reported_pvi_data();
                }
            }
            pvi_handle_time += ELE_SLEEP_TIME;
           // if(power_cnt.loc_data_cnt != 0){
             //   report_local_data();
           // }
        }
        APP_DLTJ_DEBUG("remain size:%d",system_get_free_heap_size());
        SystemSleep(ELE_SLEEP_TIME *1000);
        ele_handle_time += ELE_SLEEP_TIME;
        APP_DLTJ_DEBUG(" ele state :%d,pvi time:%d, ele time:%d,cur_posix :%d",\
        ele_state,pvi_handle_time,ele_handle_time,power_cnt.cur_posix);
    }
}


STATIC BOOL same_day_judge(UINT u_time1, UINT u_time2)
{
	return (u_time1+28800)/86400 == (u_time2+28800)/86400 ? TRUE: FALSE;
}

STATIC OPERATE_RET get_time_posix(UINT *curPosix)
{
    UINT tmp_posix;
    tmp_posix = wmtime_time_get_posix();
    if(tmp_posix < TIME_POSIX_2016) {
        APP_DLTJ_DEBUG("get curPosix err!!!");
        return OPRT_INVALID_PARM;
    }
    *curPosix = tmp_posix;
    return OPRT_OK;
}


#if 0

STATIC OPERATE_RET set_tem_ele_val(INT val)
{
	OPERATE_RET op_ret;
    cJSON *root = NULL;
	UCHAR *buf = NULL;

    root = cJSON_CreateObject();
    if(NULL == root) {
		PR_ERR("cJSON_CreateObject error");
		return OPRT_CJSON_GET_ERR;
	}
    
    cJSON_AddNumberToObject(root, "tem_ele", val);
    buf = cJSON_PrintUnformatted(root);
    if(NULL == buf) {
        PR_ERR("buf is NULL");
        cJSON_Delete(root);
        return OPRT_COM_ERROR;
    }    
    cJSON_Delete(root);
    APP_DLTJ_DEBUG("-----------------------to set tem ele val:%s-----------------------",buf);
	op_ret = msf_set_single(DEVICE_MOD,TEM_ELE_SAVE_KEY,buf);
	if(OPRT_OK != op_ret) {
		APP_DLTJ_DEBUG("msf_get_single err:%d",op_ret);
        Free(buf);
		return op_ret;
	}
    
    Free(buf);
    return OPRT_OK;    
}



STATIC OPERATE_RET get_tem_ele_val(INT *val)
{
	OPERATE_RET op_ret;
    cJSON *root = NULL, *json = NULL;
	UCHAR *buf;

    buf = (UCHAR *)Malloc(256);
	if(NULL == buf) {
		PR_ERR("malloc error");
		*val = 0;
		return OPRT_MALLOC_FAILED;
	}

	op_ret = msf_get_single(DEVICE_MOD,TEM_ELE_SAVE_KEY,buf,256);
	if(OPRT_OK != op_ret) {
		APP_DLTJ_DEBUG("msf_get_single err:%d",op_ret);
        Free(buf);
		*val = 0;
		return op_ret;
	}
    APP_DLTJ_DEBUG("Get tem ele val:%s",buf);
	root = cJSON_Parse(buf);
	if(NULL == root) {
		PR_ERR("cjson parse");
        goto JSON_PARSE_ERR;
	}

    json = cJSON_GetObjectItem(root,"tem_ele");
    if(NULL == json) {
        PR_ERR("cjson get ");
        goto JSON_PARSE_ERR;
	}

    *val = json->valueint;
    cJSON_Delete(root);
    Free(buf);
    return OPRT_OK;

JSON_PARSE_ERR:
    cJSON_Delete(root);
    Free(buf);
	*val = 0;
    return OPRT_COM_ERROR;
}

#endif 

STATIC OPERATE_RET set_ele_data(VOID)
{
	OPERATE_RET op_ret;
    cJSON *arrayItem = NULL, *array = NULL;
	UCHAR *buf = NULL;
	INT i;

	array = cJSON_CreateArray();    
	if(NULL == array) {        
		PR_ERR("cJSON_CreateArray err");        
		return OPRT_CJSON_GET_ERR;    
	}

	for(i=0; i<power_cnt.loc_data_cnt; i++){
		arrayItem = cJSON_CreateObject();
	    if(NULL == arrayItem) {
			cJSON_Delete(array);
			PR_ERR("cJSON_CreateObject error");
			return OPRT_CJSON_GET_ERR;
		}
		cJSON_AddNumberToObject(arrayItem, "time", power_cnt.ele_sav_data[i].unix_time);
		cJSON_AddNumberToObject(arrayItem, "ele", power_cnt.ele_sav_data[i].ele_energy);
		cJSON_AddItemToArray(array,arrayItem);
	}
    buf = cJSON_PrintUnformatted(array);
    if(NULL == buf) {
        PR_ERR("buf is NULL");
        cJSON_Delete(array);
        return OPRT_COM_ERROR;
    }    
	APP_DLTJ_DEBUG("set ele data buf:%s",buf);
	
	op_ret = msf_set_single(DEVICE_MOD,ELE_SAVE_KEY,buf);
	if(OPRT_OK != op_ret) {
		APP_DLTJ_DEBUG("msf_set_ele data err:%d",op_ret);
        cJSON_Delete(array);
        Free(buf);
		return op_ret;
	}
	
    cJSON_Delete(array);
    Free(buf);
    return OPRT_OK;    
}

#if 0
STATIC OPERATE_RET get_ele_data(VOID)
{
	OPERATE_RET op_ret;
    cJSON *arrayItem = NULL, *array = NULL, *json = NULL;
	UCHAR *buf=NULL;
	INT i;

    buf = (UCHAR *)Malloc(1024);
	if(NULL == buf) {
		PR_ERR("malloc error");
		power_cnt.loc_data_cnt = 0;
		return OPRT_MALLOC_FAILED;
	}

	op_ret = msf_get_single(DEVICE_MOD,ELE_SAVE_KEY,buf,1024);
	if(OPRT_OK != op_ret) {
		APP_DLTJ_DEBUG("msf_get_ele_data err:%d",op_ret);
		power_cnt.loc_data_cnt = 0;
        Free(buf);
		return op_ret;
	}
	APP_DLTJ_DEBUG("get ele data buf:%s",buf);
	array = cJSON_Parse(buf);
	if(NULL == array) {
		PR_ERR("cjson parse err");
		power_cnt.loc_data_cnt = 0;
	    Free(buf);
	    return OPRT_COM_ERROR;
	}

	Free(buf);
	power_cnt.loc_data_cnt = cJSON_GetArraySize(array);

	for(i=0; i<power_cnt.loc_data_cnt; i++){
		arrayItem = cJSON_GetArrayItem(array, i);
		if(arrayItem){
			json = cJSON_GetObjectItem(arrayItem,"time");
			if(json == NULL){
				cJSON_Delete(array);
				return OPRT_COM_ERROR;
			}
			power_cnt.ele_sav_data[i].unix_time = json->valueint;
			json = cJSON_GetObjectItem(arrayItem,"ele");
			if(json == NULL){
				cJSON_Delete(array);
				return OPRT_COM_ERROR;
			}
				
			power_cnt.ele_sav_data[i].ele_energy = json->valueint;
			APP_DLTJ_DEBUG("i:%d,time:%d,ele:%d",i,power_cnt.ele_sav_data[i].unix_time,\
			power_cnt.ele_sav_data[i].ele_energy);
		}
	}
	cJSON_Delete(array);
    return OPRT_OK;
}


//最多存10组电量，按天分组
STATIC VOID addto_local_data(UINT time, UINT ele)
{
	INT i;
	APP_DLTJ_DEBUG("time:%d,ele:%d",time,ele);
	APP_DLTJ_DEBUG("try to add to local data...");

	if(power_cnt.loc_data_cnt != 0){
		if(time == 0){
			power_cnt.ele_sav_data[power_cnt.loc_data_cnt-1].ele_energy += ele;
			return;
		}
		//电量按天存在flash中
		if(same_day_judge(power_cnt.ele_sav_data[power_cnt.loc_data_cnt-1].unix_time,time)){
			power_cnt.ele_sav_data[power_cnt.loc_data_cnt-1].unix_time = time;
			power_cnt.ele_sav_data[power_cnt.loc_data_cnt-1].ele_energy += ele;
			APP_DLTJ_DEBUG("Same day ...%d %d",power_cnt.ele_sav_data[power_cnt.loc_data_cnt-1].ele_energy,ele);
		}else{
			if(power_cnt.loc_data_cnt == 10){
				//for(i=0; i<9; i++){
				//	power_cnt.ele_sav_data[i].unix_time = power_cnt.ele_sav_data[i+1].unix_time;
				//	power_cnt.ele_sav_data[i].ele_energy = power_cnt.ele_sav_data[i+1].ele_energy;
				//}
				//power_cnt.ele_sav_data[power_cnt.loc_data_cnt-1].unix_time = time;
				power_cnt.ele_sav_data[power_cnt.loc_data_cnt-1].ele_energy += ele;//达到最大存储数目之后，将后续电量存在最后一天
			}else{
				power_cnt.ele_sav_data[power_cnt.loc_data_cnt].unix_time = time;
				power_cnt.ele_sav_data[power_cnt.loc_data_cnt].ele_energy = ele;
				power_cnt.loc_data_cnt++;
			}

		}
	}else{
		power_cnt.ele_sav_data[0].unix_time = time;
		power_cnt.ele_sav_data[0].ele_energy = ele;
		power_cnt.loc_data_cnt++;
	}
	APP_DLTJ_DEBUG("set loc_data_cnt : %d",power_cnt.loc_data_cnt);
	if(OPRT_OK != set_ele_data()){
		APP_DLTJ_DEBUG("set ele to flash err ...");
	}
}


STATIC BOOL if_pvi_need_report()
{
    if(!value_range_judge(power_cnt.cur_current,power_cnt.last_current,REPT_THRE_CURR) ||\
       !value_range_judge(power_cnt.cur_power,power_cnt.last_power,REPT_THRE_PWR) ||\
       !value_range_judge(power_cnt.cur_vol,power_cnt.last_vol,REPT_THRE_VOL)){
        pvi_report_min_time = PVI_REPORT_CHG_THRE;
        return TRUE;
    }else{
        pvi_report_min_time = PVI_REPORT_MIN_THRE;
        return FALSE;
    }
}
#endif

STATIC OPERATE_RET report_pvi_data(VOID)
{
    cJSON *root = NULL;
    root = cJSON_CreateObject();
    if(NULL == root) {
        return OPRT_CR_CJSON_ERR;
    }
    char dpid_str[8];
    sprintf(dpid_str, "%d", mAppDltj.app_dltj_info.idpid);
    cJSON_AddNumberToObject(root,dpid_str,power_cnt.cur_current);
    sprintf(dpid_str, "%d", mAppDltj.app_dltj_info.pdpid);
	cJSON_AddNumberToObject(root,dpid_str,power_cnt.cur_power);
	sprintf(dpid_str, "%d", mAppDltj.app_dltj_info.vdpid);
	cJSON_AddNumberToObject(root,dpid_str,power_cnt.cur_vol); 
    CHAR *out;
    out=cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if(NULL == out) {
        PR_ERR("cJSON_PrintUnformatted err:");
        return OPRT_CJSON_GET_ERR;
    }

    APP_DLTJ_DEBUG("try to report:[%s]", out);
    OPERATE_RET op_ret = sf_obj_dp_report(get_single_wf_dev()->dev_if.id,\
                            out);
    if(OPRT_OK != op_ret) {
		Free(out);
        PR_ERR("report ele par err,op_ret:%d",op_ret);
        return OPRT_DP_REPORT_CLOUD_ERR;
    }
	Free(out);
    return OPRT_OK;
   
}

STATIC OPERATE_RET update_ele_data(UINT time, UINT ele_value)
{
	OPERATE_RET op_ret;
	STATS_DATA_S *stat_data;
	INT size = 0;
	size = sizeof(STATS_DATA_S) + 4;
	APP_DLTJ_DEBUG("size:%d",size);
	stat_data = (STATS_DATA_S *)Malloc(size);
	memset(stat_data, 0, size);
	
	stat_data->time = time;
	stat_data->type = PROP_VALUE;
	stat_data->len  = 4;
	stat_data->data.value = ele_value;
    APP_DLTJ_DEBUG("Upload tem ele time:%d, value:%d!!!!!",time,ele_value);
	op_ret = mq_client_publish_obj_data(mAppDltj.app_dltj_info.edpid, stat_data);
	if(OPRT_OK != op_ret){
		PR_ERR("Ele report fail %d!!!", op_ret);
		Free(stat_data);
		return OPRT_COM_ERROR;
	}
	Free(stat_data);
	APP_DLTJ_DEBUG("Ele report success...");
	return OPRT_OK;
}

#if 0

STATIC VOID report_local_data(VOID)
{
	INT i,j;
	UINT val_size = power_cnt.loc_data_cnt;
	INT cha_flag = 0;
	if(val_size == 0)
		return ;
	for(i=0; i<val_size; i++){
		if(update_ele_data(power_cnt.ele_sav_data[0].unix_time,power_cnt.ele_sav_data[0].ele_energy) != OPRT_OK){
			break;
		}
		APP_DLTJ_DEBUG("Upload cnt:%d,curr_i:%d,unix_time:%d,ele_energy:%d success...",power_cnt.loc_data_cnt,i,\
		        power_cnt.ele_sav_data[0].unix_time,power_cnt.ele_sav_data[0].ele_energy);
		cha_flag = 1;
		for(j=0; j<val_size-1; j++){
			power_cnt.ele_sav_data[j].unix_time = power_cnt.ele_sav_data[j+1].unix_time;
			power_cnt.ele_sav_data[j].ele_energy = power_cnt.ele_sav_data[j+1].ele_energy;
		}
		power_cnt.loc_data_cnt -= 1;
	}
	
	if(cha_flag == 1){
		if(OPRT_OK != set_ele_data()){
			PR_ERR("set ele to flash err ...");
		}
	}
}


STATIC VOID save_reported_pvi_data()
{
    power_cnt.last_current = power_cnt.cur_current;
    power_cnt.last_vol = power_cnt.cur_vol;
    power_cnt.last_power = power_cnt.cur_power;
}

#endif



//判断JudgedValue是否在TargetValue的正负range%范围之内，如果是返回1，如果否返回0
//JudgedValue为被判定的值，TargetValue为被判定值的目标值，range为浮动范围0-100(%).
STATIC BOOL value_range_judge(UINT JudgedValue, UINT TargetValue, UINT range)
{
    if((JudgedValue * 100 >= TargetValue * (100 - range)) && \
        (JudgedValue * 100 <= TargetValue * (100 + range))){
        return TRUE;
    }else{
        return FALSE;
    }
}






