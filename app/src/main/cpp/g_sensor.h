/**
 * Copyright @ 深圳市谷米万物科技有限公司. 2009-2019. All rights reserved.
 * File name:        g_sensor.h
 * Author:           王志华       
 * Version:          1.0
 * Date:             2019-03-01
 * Description:      运动传感器数据处理,控制GPS休眠与唤醒、判断运动状态（用于控制GPS是否产生数据）、相关报警
 * Others:           唤醒与休眠逻辑:
					（1）任意1轴加速度差值大于40mg,传感器中断（任意一轴的差值大于16mg）,以上两种事件10s钟监测到3次以上唤醒
					（2）连续5分钟没有监测到唤醒的条件休眠
 * Function List:    
    1. 创建g_sensor模块
    2. 销毁g_sensor模块
    3. g_sensor模块定时处理入口
    4. 获取g_sensor阈值
    5. 设置g_sensor阈值
    6. 获取g-sensor状态
 * History: 
    1. Date:         2019-03-01
       Author:       王志华
       Modification: 创建初始版本
    2. Date: 		 
	   Author:		 
	   Modification: 

 */

#ifndef __G_SENSOR_H__
#define __G_SENSOR_H__

#include "gm_type.h"
#include "error_code.h"


//10秒钟内监测到3次以上震动唤醒
#define SHAKE_COUNT_TO_AWAKE 3
#define SHAKE_TIME_SECONDS 10


//运动传感器最大数值,需要根据传感器型号调整
#define G_SENSOR_MAX_VALUE 2047

//运动传感器最大量程,需要根据传感器型号进行调整
#define G_SENSOR_RANGE 2.0

//传感器采样频率（每秒次数）
#define G_SENSOR_SAMPLE_FREQ 40

#define MIN_CRUISE_SPEED 20

typedef enum
{
	GSENSOR_TYPE_UNKNOWN = 0,      //未知
	GSENSOR_TYPE_SC7A20 = 0x11,    //批量生产使用中
	GSENSOR_TYPE_BMA253 = 0xFA,    //已停产
	GSENSOR_TYPE_DA213 = 0x13,     //测试通过,未批量生产
}GSensorType;


typedef enum
{
    THRESHOLD_INDEX_FOR_RAPID_ACLR = 0,
    THRESHOLD_INDEX_FOR_EMERGENCY_BRAKE = 1,
    THRESHOLD_INDEX_FOR_SLIGHT_COLLISION = 2,
    THRESHOLD_INDEX_FOR_NORMAL_COLLISION = 3,
    THRESHOLD_INDEX_FOR_SERIOUS_COLLISION = 4,
    THRESHOLD_INDEX_FOR_LOWVOlTAGE_ALARM = 5,
    THRESHOLD_INDEX_FOR_QUAKE_ALARM = 6,
    THRESHOLD_INDEX_FOR_STATIC = 7,
    THRESHOLD_INDEX_FOR_RUN = 8,
    THRESHOLD_INDEX_FOR_SUDDEN_TURN = 9,
    THRESHOLD_ARRAY_LEN = 10,
}THRESHOLD_INDEX;
typedef struct
{
    S16 x;
    S16 y;
    S16 z;
} Aclr, *pAclr;

typedef struct
{
    U8 shake_level;
    U32 shake_threshold;
    float static_thr;        // 静止阈值
    float run_thr;           // 运动阈值
    float brake_thr;         // 急刹车阈值
    float rapid_aclr_thr;// 急加速阈值
    float rapid_inc_speed_thr;//急加速速度阈值
	float rapid_dec_speed_thr;//急减速速度阈值
    float sudden_turn_thr;   // 急转弯阈值
    float slight_crash_thr;  // 轻微碰撞阈值
    float normal_crash_thr;  // 一般碰撞阈值
    float serious_crash_thr; // 严重碰撞阈值
}GSensorThreshold;

typedef struct timefileter
{
	clock_t cstart;
	clock_t cends;
	int timethresd;
}AlarmTimefilter;


typedef enum
{
    NO_COLLISION = 0,
    SLIGHT_COLLISION = 0x2000,
    NORMAL_COLLISION = 0x4000,
    SERIOUS_COLLISION = 0x6000
}COLLISION_LEVEL;
/*
 * 急加速 harsh acceleration
车辆在纵向上的加速度大于2.5m/s2
3.15 的事件。
急减速 harsh deceleration
车辆在纵向上的加速度小于-4.5m/s2
3.16 的事件。
急转弯 harsh turn
车辆在横向上的加速度绝对值大于4m/s2
3.17 且卫星定位方向改变量大于45°的事件。
急变道 harsh lane-change
车辆在横向上的加速度绝对值大于4m/s2
3.18且卫星定位方向改变量小于20°的事件。
水平碰撞事故 horizontal collision
车辆在纵向或横向上的加速度绝对值大于20m/s2
3.19 ，且车辆的俯仰角及侧倾角的绝对值均不大于20°的事件。
翻转事故 rollover
车辆的俯仰角或侧倾角的绝对值大于70°的事件。
3.20车辆稳定性报警 vehicle stability warning
车辆在大于3s的时间内，持续以绝对值大于50°/s的角速度改变航向角的事件。
 */
//角度变化超过30度认为发生了突变
#define MIN_ANGLE_RANGE 30
	
//学习时长20秒
#define STUDY_TIME_SEC 20
	
//首次运行至少需要多少条数据
#define MIN_STUDY_GRAVITY_TIMES (STUDY_TIME_SEC*G_SENSOR_SAMPLE_FREQ)
	
//传感器存储数据长度
#define G_SENSOR_BUF_LEN MIN_STUDY_GRAVITY_TIMES

//传感器存储数据长度
#define G_SHAKE_BUF_LEN 50
/*
 *  * 急加速 harsh acceleration
*    车辆在纵向上的加速度大于2.5m/s2
 */
//急加速阈值0.4g->0.8
#define RAPID_ACLR_THRESHOLD 0.8
/*
 * 急减速 harsh deceleration
*  车辆在纵向上的加速度小于-4.5m/s2
 */
//急刹车阈值0.7g->1.5
#define EMERGENCY_BRAKE_THRESHOLD 1.5
/*
 * 急转弯 harsh turn
*  车辆在横向上的加速度绝对值大于4m/s2
 */
//急转弯阈值0.3g->0.6
#define SUDDEN_TURN_THRESHOLD 0.6

/*
 * 水平碰撞事故 horizontal collision
 * 车辆在纵向或横向上的加速度绝对值大于20m/s2
3.19 ，且车辆的俯仰角及侧倾角的绝对值均不大于20°的事件。
 */
	
//[3g,5g)轻微碰撞(人不会受伤);[5g,10g)中等碰撞(人受轻伤);[10g,+∞)严重碰撞(人受重伤)
#define SLIGHT_COLLISION_MIN_ACLR 4.0
#define NORMAL_COLLISION_MIN_ACLR 6.0
#define SERIOUS_COLLISION_MIN_ACLR 10.0
	
//角度记录条数(2分钟数据)
#define ANGLE_RECORD_NUM (SECONDS_PER_MIN)
#define TURN_OVER_ANGLE 45
#define REMOVE_ANGLE 30
	
//震动报警车辆垂直方向加速度阈值0.02g
#define QUAKE_ACLR_THRESHOLD 0.02
	
//运动转静止时车辆水平方向加速度阈值10mg
#define STATIC_ACLR_THRESHOLD 0.01
	
//静止转为运动状态的车辆水平方向加速度阈值20mg
#define RUN_ACLR_THRESHOLD 0.020
	
//连续加速度值小于阈值的次数,判断为静止:20秒,每秒钟100次
#define MIN_LOW_ACLR_NUM (STUDY_TIME_SEC*G_SENSOR_SAMPLE_FREQ)
	
//高加速度超过次数重置重力加速度:2分钟=120秒,每秒钟100次
#define RESET_GRAVTITY_NUM (2*60*G_SENSOR_SAMPLE_FREQ)
	
//连续MIN_CRUISE_SPEED_TIME秒匀速运动,即使加速度很小也不判为静止
#define MIN_CRUISE_SPEED_TIME 5

void set_sensor_thr(int thresd_index,double alarm_thr );
void setspeedtag(int speedtag);

void check_vehicle_state(Aclr a);

/**
 * Function:   创建g_sensor模块
 * Description:创建g_sensor模块
 * Input:	   无
 * Output:	   无
 * Return:	   GM_SUCCESS——成功；其它错误码——失败
 * Others:	   使用前必须调用,否则调用其它接口返回失败错误码
 */
GM_ERRCODE g_sensor_create(void);

/**
 * Function:   销毁g_sensor模块
 * Description:销毁g_sensor模块
 * Input:	   无
 * Output:	   无
 * Return:	   GM_SUCCESS——成功；其它错误码——失败
 * Others:	   
 */
GM_ERRCODE g_sensor_destroy(void);

/**
 * Function:   重置无振动时间
 * Description:外部打开GPS模块要调用这个函数，否则可能很快又休眠了
 * Input:	   无
 * Output:	   无
 * Return:	   无
 * Others:	   
 */
void g_sensor_reset_noshake_time(void);


/**
 * Function:   获取传感器与水平面的夹角
 * Description:单位为度,范围[0,180]
 * Input:	   无
 * Output:	   无
 * Return:	   传感器与水平面的夹角
 * Others:	   
 */
U8 g_sensor_get_angle(void);



/**
 * Function:   获取传感器错误次数
 * Description:
 * Input:	   无
 * Output:	   无
 * Return:	   错误次数
 * Others:	   
 */
U32 g_sensor_get_error_count(void);

/**
 * Function:   获取传震动事件次数（读取数据）
 * Description:
 * Input:	   无
 * Output:	   无
 * Return:	   震动事件次数
 * Others:	   
 */
U8 g_sensor_get_shake_event_count_when_read(void);

/**
 * Function:   获取传震动事件次数（中断）
 * Description:
 * Input:	   无
 * Output:	   无
 * Return:	   震动事件次数
 * Others:	   
 */
U8 g_sensor_get_shake_event_count_when_eint(void);
void report_rapid_aclr_alarm(void);
void report_sudden_turn_alarm(void);
void report_emergency_brake_alarm(void);

#endif

