/**
 * Copyright @ 深圳市谷米万物科技有限公司. 2009-2019. All rights reserved.
 * File name:        common_type_defines.h
 * Author:           王志华
 * Version:          1.0
 * Date:             2019-02-28
 * Description:      通用数据类型定义
 * Others:
 * Function List:
 * History:
    1. Date:         2019-02-28
       Author:       王志华
       Modification: 创建初始版本
    2. Date:
	   Author:
	   Modification:

 */

#ifndef __COMMON_TYPE_DEFINES_H__
#define __COMMON_TYPE_DEFINES_H__
//timer https://www.codeproject.com/articles/34982/create-multiple-independent-timers-using-a-linked
typedef unsigned char U8,u8;
typedef signed char S8,s8;
typedef unsigned short U16,u16;
typedef signed short S16,s16;
typedef unsigned int U32,u32;
typedef signed int S32,s32;
typedef unsigned long long U64,u64;
typedef long long S64,s64;

#ifndef NULL
#ifdef __cplusplus
#define NULL (0)
#else
#define NULL ((void *)0)
#endif
#endif
#ifndef null
#define null NULL
#endif

#ifndef __cplusplus
#ifndef bool
typedef U8 bool;
typedef U8 BOOL;
#define true 1
#define TRUE 1
#define false 0
#define FALSE 0
#endif
#endif

typedef unsigned int UINT;
typedef unsigned char kal_uint8;
typedef signed char kal_int8;
typedef char kal_char;
typedef unsigned short kal_wchar;
typedef unsigned short int kal_uint16;
typedef signed short int kal_int16;
typedef unsigned int kal_uint32;
typedef signed int kal_int32;

typedef U16 module_type;
typedef U16 msg_type;
typedef U16 sap_type;
typedef int MMI_BOOL;


#define GM_TICKS_10_MSEC           (2)         /* 10 msec */
#define GM_TICKS_50_MSEC           (10)        /* 50 msec */
#define GM_TICKS_100_MSEC          (21)        /* 100 msec */
#define GM_TICKS_500_MSEC          (108)       /* 500 msec */
#define GM_TICKS_1024_MSEC         (221)       /* 1024 msec */

#define GM_TICKS_1_SEC             (217)       /* 1 sec */
#define GM_TICKS_2_SEC_2           (433)       /* 2 sec */
#define GM_TICKS_3_SEC             (650)       /* 3 sec */
#define GM_TICKS_5_SEC             (1083)      /* 5 sec */
#define GM_TICKS_10_SEC            (2167)      /* 10 sec */
#define GM_TICKS_30_SEC            (6500)      /* 30 sec */
#define GM_TICKS_1_MIN             (13000)     /* 1 min */



typedef enum
{
    ALARM_NONE = 0,
    ALARM_POWER_OFF = 0x01,      //鐢垫簮鏂數鎶ヨ,
    ALARM_BATTERY_LOW = 0x03,    //鐢垫睜浣庣數鎶ヨ
    ALARM_SHOCK = 0x04,          //闇囧姩鎶ヨ
    ALARM_MOVE = 0x05,			 //杞﹁締绉诲姩鎶ヨ锛堜娇鐢ㄥ師浣嶇Щ鎶ヨ浣嶏級
    ALARM_SPEED = 0x0d,          //瓒呴€熸姤璀?
    ALARM_FAKE_CELL = 0x0f,      //浼熀绔欐姤璀?
    ALARM_POWER_HIGH = 0x11,     //鐢垫簮鐢靛帇杩囬珮鎶ヨ
    ALARM_COLLISION = 0x12,      //纰版挒鎶ヨ
    ALARM_SPEED_UP = 0x13,       //鎬ュ姞閫熸姤璀?
    ALARM_SPEED_DOWN = 0x14,     //鎬ュ噺閫熸姤璀?
    ALARM_TURN_OVER = 0x15,      //缈昏浆鎶ヨ
    ALARM_SHARP_TURN = 0x16,     //鎬ヨ浆寮姤璀?
    ALARM_REMOVE = 0x17,         //鎷嗗姩鎶ヨ
    ALARM_MAX
}AlarmTypeEnum;

typedef struct
{
    AlarmTypeEnum type;
    u16 info;
}AlarmInfo;

#define TIM_GEN_1MS               2
#define TIM_GEN_10MS              10 // 3  // 10
#define TIM_GEN_100MS             100
#define TIM_GEN_1SECOND           1000
#define TIM_GEN_1_MIN             60000


typedef enum 
{
    KAL_FALSE,
    KAL_TRUE
} kal_bool;
	
typedef void (*FuncPtr) (void);
typedef void (*PsFuncPtr) (void *);

#define GOOME_APN_MAX_LENGTH 30
#define GOOME_DNS_MAX_LENTH  50
#endif


