/***********************************************************
*  File: app_dltj.h
*  Author: litao
*  Date: 170704
***********************************************************/
#ifndef  __APP_DLTJ_H__
#define  __APP_DLTJ_H__
#include "com_def.h"
#include "sys_adapter.h"
#include "hlw8012.h"



#ifdef __cplusplus
extern "C" {
#endif
#ifdef _APP_DLTJ_GLOBAL
    #define _APP_DLTJ_EXT
#else
    #define _APP_DLTJ_EXT extern
#endif




/***********************************************************
*************************micro define***********************
***********************************************************/
#define _APP_DLTJ_DEBUG 1

#define DP_ELE  3//增加电量上报
#define DP_CURRENT  4//电流上报
#define DP_POWER  5//功率上报
#define DP_VOLTAGE  6//电压上报




/***********************************************************
*************************variable define********************
***********************************************************/
typedef enum{
    APP_DLTJ_NORMAL,
    APP_DLTJ_PRODTEST
}APP_DLTJ_MODE;

/***********************************************************
*************************function define********************
***********************************************************/
_APP_DLTJ_EXT \
OPERATE_RET app_dltj_init(APP_DLTJ_MODE mode);



#ifdef __cplusplus
}
#endif
#endif

