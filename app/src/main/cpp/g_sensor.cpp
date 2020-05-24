/**
 * Copyright @ 深圳市谷米万物科技有限公司. 2009-2019. All rights reserved.
 * File name:        g_sensor.h
 * Author:           王志华       
 * Version:          1.0
 * Date:             2019-03-01
 * Description:      运动传感器数据处理,控制GPS休眠与唤醒、判断运动状态（用于控制GPS是否产生数据）、相关报警
 * Others:      
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
#include <jni.h>
#include <time.h>
#include <stdlib.h>
#include <math.h>
#include "alarmhandle.h"
#include "g_sensor.h"
#include "applied_math.h"
#include "circular_queue.h"
#include "systemstate.h"
#include "gps.h"
#include "utility.h"
#include "tyyslog.h"
#include "alarmhandle.h"




extern void report_alarm(int alarmid,double jindu,double weidu, double hanxian);//
int m_speedtag;//0 no ;speed 1 using gps speedv
typedef struct
{
	GSensorType type;

	//启动时间（毫秒）
	U64 start_time;

	//无振动时间毫秒
	U32 no_shake_time_ms;

	//唤醒时间（毫秒）
	U32 awake_time;

	//休眠时间（毫秒）
	U32 sleep_time;

	U32 rw_error_count;
	U32 read_data_count;

	//震动事件队列（存储的是震动时间）
	CircularQueue shake_event_time_queue_when_read;
	CircularQueue shake_event_time_queue_when_eint;
	U32 last_report_shake_alarm_time;
	U32 last_report_turnover_alarm_time;
	U32 last_report_remove_alarm_time;

	//存20秒传感器数据
	CircularQueue aclr_x_queue;
	CircularQueue aclr_y_queue;
	CircularQueue aclr_z_queue;

    CircularQueue sensor_angle_queue;

    Vector3D gravity;
    unsigned int cal_gravity_avg_len;
    Vector3D static_gravity;
    Vector2D vehicle_head_vector;

    //传感器加速度的滑动平均值,含重力加速度在内
    Vector3D senor_aclr_moving_avg;
    U32 cal_senor_aclr_avg_len;
    U32 cal_senor_aclr_avg_len_short_period;
    U64 study_aclr_count;
    U64 static_low_aclr_count;
    U64 super_low_aclr_count;

    COLLISION_LEVEL collision_level;
    U32 collision_aclr_count;
    GPSData last_collision_gps_info;
    Aclr last_collision_aclr;

	GSensorThreshold threshold;
}GSensor;

static GSensor s_gsensor;
//time filter
AlarmTimefilter sudden_turntmfilter;
AlarmTimefilter rapid_accelerationtmfilter;
AlarmTimefilter emergency_braketmfilter;


static void reset_sensor_data(void);
static GM_ERRCODE get_config(void);
static void init_gsensor_chip(void);
static void check_awake_sleep(void);
static void read_data_from_chip(void);
static void check_shake_event(Aclr current_aclr);
static void check_shake_alarm(void);
static void report_shake_alarm(void);
static void check_acc_and_report_shake_alarm(void);
static U8 get_data_addr(const GSensorType type);



static void check_collision(Vector3D sensor_aclr);
static bool check_emergency_brake(float vehicle_horizontal_aclr_magnitude);
static bool check_rapid_acceleration(float vehicle_horizontal_aclr_magnitude);
static bool check_sudden_turn_acceleration(float vehicle_horizontal_aclr_magnitude);
static void study_gravity(Aclr a);
static GM_ERRCODE get_vehicle_acceleration(Vector3D g, Vector3D sensor_aclr, Vector3D *vehicle_horizontal_aclr, Vector3D *vehicle_vertical_aclr);
static void calculate_moving_avg(Vector3D sensor_aclr, int num, PVector3D p_aclr_moving_avg, unsigned int *p_len);
static void check_static_or_run(float vehicle_horizontal_aclr_magnitude);
static void calc_sensor_angle(void);
static void check_angle_alarm(void);
static void init_sensorthresd();
static void   inittimerfilter();
#define TIMEfiltescale 20


/**
 * Function:   创建g_sensor模块
 * Description:创建g_sensor模块
 * Input:	   无
 * Output:	   无
 * Return:	   GM_SUCCESS——成功；其它错误码——失败
 * Others:	   使用前必须调用,否则调用其它接口返回失败错误码
 */
GM_ERRCODE g_sensor_create(void)
{
	GM_ERRCODE ret = GM_SUCCESS;
	s_gsensor.type = GSENSOR_TYPE_UNKNOWN;
	s_gsensor.start_time = 0;
	s_gsensor.no_shake_time_ms = 0;
	s_gsensor.awake_time = 0;
	s_gsensor.sleep_time = 0;
	s_gsensor.rw_error_count = 0;
	s_gsensor.read_data_count = 0;
	s_gsensor.last_report_shake_alarm_time = 0;
	s_gsensor.last_report_turnover_alarm_time = 0;
	s_gsensor.last_report_remove_alarm_time = 0;

	reset_sensor_data();

	ret = circular_queue_create(&s_gsensor.shake_event_time_queue_when_read,G_SHAKE_BUF_LEN,GM_QUEUE_TYPE_INT);
	if (GM_SUCCESS != ret)
	{
		//LOG(ERROR, "Failed to circular_queue_create()!");
		return ret;
	}

	ret = circular_queue_create(&s_gsensor.shake_event_time_queue_when_eint,G_SHAKE_BUF_LEN,GM_QUEUE_TYPE_INT);
	if (GM_SUCCESS != ret)
	{
		//LOG(ERROR, "Failed to circular_queue_create()!");
		return ret;
	}
	
	ret = circular_queue_create(&s_gsensor.aclr_x_queue,G_SENSOR_BUF_LEN,GM_QUEUE_TYPE_INT);
	if (GM_SUCCESS != ret)
	{
		//LOG(ERROR, "Failed to circular_queue_create()!");
		return ret;
	}
	
	ret = circular_queue_create(&s_gsensor.aclr_y_queue,G_SENSOR_BUF_LEN,GM_QUEUE_TYPE_INT);
	if (GM_SUCCESS != ret)
	{
		//LOG(ERROR, "Failed to circular_queue_create()!");
		return ret;
	}
	
	ret = circular_queue_create(&s_gsensor.aclr_z_queue,G_SENSOR_BUF_LEN,GM_QUEUE_TYPE_INT);
	if (GM_SUCCESS != ret)
	{
		//LOG(ERROR, "Failed to circular_queue_create()!");
		return ret;
	}
	
	ret = circular_queue_create(&s_gsensor.sensor_angle_queue, ANGLE_RECORD_NUM,GM_QUEUE_TYPE_INT);
	if (GM_SUCCESS != ret)
	{
		//LOG(ERROR, "Failed to circular_queue_create()!");
		return ret;
	}
    init_sensorthresd();
    inittimerfilter();
	return ret;
}

static void   inittimerfilter()
{
    sudden_turntmfilter.cstart=    clock();
    sudden_turntmfilter.cends=clock();
    rapid_accelerationtmfilter.cstart=    clock();
    rapid_accelerationtmfilter.cends=clock();
    emergency_braketmfilter.cstart=    clock();
    emergency_braketmfilter.cends=clock();
    sudden_turntmfilter.timethresd=100000*TIMEfiltescale;//3S
    rapid_accelerationtmfilter.timethresd=100000*TIMEfiltescale;
    emergency_braketmfilter.timethresd=100000*TIMEfiltescale;
}
static void init_sensorthresd()
{
    s_gsensor.threshold.static_thr=STATIC_ACLR_THRESHOLD;        // 静止阈值
    s_gsensor.threshold.run_thr=RUN_ACLR_THRESHOLD;           // 运动阈值
    s_gsensor.threshold.brake_thr=EMERGENCY_BRAKE_THRESHOLD;         // 急刹车阈值
    s_gsensor.threshold.rapid_aclr_thr=RAPID_ACLR_THRESHOLD;// 急加速阈值
   // float rapid_inc_speed_thr;//急加速速度阈值
   // float rapid_dec_speed_thr;//急减速速度阈值
    s_gsensor.threshold.rapid_inc_speed_thr=10;
    s_gsensor.threshold.rapid_dec_speed_thr=-20;
    s_gsensor.threshold.sudden_turn_thr=SUDDEN_TURN_THRESHOLD;   // 急转弯阈值
    s_gsensor.threshold.slight_crash_thr=SLIGHT_COLLISION_MIN_ACLR;  // 轻微碰撞阈值
    s_gsensor.threshold.normal_crash_thr=NORMAL_COLLISION_MIN_ACLR;  // 一般碰撞阈值
    s_gsensor.threshold.serious_crash_thr=SERIOUS_COLLISION_MIN_ACLR; // 严重碰撞阈值
}
static void reset_sensor_data(void)
{
    s_gsensor.gravity.x = 0;
    s_gsensor.gravity.y = 0;
    s_gsensor.gravity.z = 0;
    s_gsensor.cal_gravity_avg_len = 0;
    s_gsensor.cal_senor_aclr_avg_len = 0;
    s_gsensor.cal_senor_aclr_avg_len_short_period = 0;
    s_gsensor.static_gravity.x = 0;
    s_gsensor.static_gravity.y = 0;
    s_gsensor.static_gravity.z = 0;
    s_gsensor.vehicle_head_vector.x = 0;
    s_gsensor.vehicle_head_vector.y = 0;
    s_gsensor.study_aclr_count = 0;
    s_gsensor.static_low_aclr_count = 0;
    s_gsensor.super_low_aclr_count = 0;
    m_speedtag=0;//int m_speedtag;//0 no ;speed 1 using gps speed
}

void set_sensor_thr(int thresd_index,double alarm_thr )
{
    switch (thresd_index)
    {
        case 0:
            s_gsensor.threshold.static_thr=alarm_thr;        // 静止阈值
            break ;
        case 1:
            s_gsensor.threshold.run_thr=alarm_thr;           // 运动阈值
            break ;
        case 2:
            s_gsensor.threshold.brake_thr=alarm_thr;         // 急刹车阈值
            break ;
        case 3:
            s_gsensor.threshold.rapid_aclr_thr=alarm_thr;// 急加速阈值
            break ;
        case 4:
            s_gsensor.threshold.sudden_turn_thr=alarm_thr;   // 急转弯阈值
            break ;
        case 5:
             s_gsensor.threshold.slight_crash_thr=alarm_thr;  // 轻微碰撞阈值
            break ;
        case 6:
            s_gsensor.threshold.normal_crash_thr=alarm_thr;  // 一般碰撞阈值
            break ;
        case 7:
            s_gsensor.threshold.serious_crash_thr=alarm_thr; // 严重碰撞阈值
            break ;
        case 8:
             s_gsensor.threshold.rapid_inc_speed_thr=alarm_thr;////急加速速度阈值
            break;
        case 9:
            s_gsensor.threshold.rapid_dec_speed_thr=alarm_thr;//急减速速度阈值
        default:
            break;
    }
}
void setspeedtag(int speedtag)
{
    m_speedtag=speedtag;//int m_speedtag;//0 no ;speed 1 using gps speed
}

static GM_ERRCODE get_config(void)
{
	//zly
	//LOG(ERROR, "get_config*********ZLY*********");
	//zly
	return GM_SUCCESS;
}

/**
 * Function:   销毁g_sensor模块
 * Description:销毁g_sensor模块
 * Input:	   无
 * Output:	   无
 * Return:	   GM_SUCCESS——成功；其它错误码——失败
 * Others:	   
 */
GM_ERRCODE g_sensor_destroy(void)
{
	circular_queue_destroy(&s_gsensor.shake_event_time_queue_when_read,GM_QUEUE_TYPE_INT);

	circular_queue_destroy(&s_gsensor.shake_event_time_queue_when_eint,GM_QUEUE_TYPE_INT);
	
	circular_queue_destroy(&s_gsensor.aclr_x_queue,GM_QUEUE_TYPE_INT);

	circular_queue_destroy(&s_gsensor.aclr_y_queue,GM_QUEUE_TYPE_INT);
	
	circular_queue_destroy(&s_gsensor.aclr_z_queue,GM_QUEUE_TYPE_INT);
	
	circular_queue_destroy(&s_gsensor.sensor_angle_queue,GM_QUEUE_TYPE_INT);
	
	return GM_SUCCESS;
}

void g_sensor_reset_noshake_time(void)
{
	s_gsensor.no_shake_time_ms = 0;
}

U8 g_sensor_get_angle(void)
{
	S32 last_angle = 0;
	if(false == circular_queue_get_tail_i(&(s_gsensor.sensor_angle_queue),&last_angle))
	{
		return 0;
	}
	return (U8)last_angle;
}


U32 g_sensor_get_error_count(void)
{
	return s_gsensor.rw_error_count;
}

int statecnt=0;
//每10ms调用一次
void check_vehicle_state(Aclr a)
{
    s_gsensor.start_time += TIM_GEN_10MS;
    //实时加速度（含重力）
    Vector3D sensor_aclr = {0, 0, 0};
    //水平方向加速度（不含重力）
    Vector3D vehicle_horizontal_aclr = {0, 0, 0};
    //垂直方向加速度（不含重力）
    Vector3D vehicle_vertical_aclr = {0, 0, 0};
    float vehicle_horizontal_aclr_magnitude = 0;
    float vehicle_vertical_aclr_magnitude = 0;
	bool aclr_alarm_enable = true;

    //ALOGD("check_add data to queue\r\n");
    circular_queue_en_queue_i(&(s_gsensor.aclr_x_queue), a.x);
    circular_queue_en_queue_i(&(s_gsensor.aclr_y_queue), a.y);
    circular_queue_en_queue_i(&(s_gsensor.aclr_z_queue), a.z);

    sensor_aclr.x = a.x;
    sensor_aclr.y = a.y;
    sensor_aclr.z = a.z;
    statecnt++;
    /*debug timerzly
    if(statecnt==200)
    {
        GM_startsudden_turn_accelerationtimer();
        statecnt=0;

    }
     */

	//config_service_get(CFG_IS_ACLRALARM_ENABLE, TYPE_BOOL, &aclr_alarm_enable, sizeof(aclr_alarm_enable));
   // ALOGD("aclr_alarm_enable\r\n");
	if (aclr_alarm_enable)
	{
		check_collision(sensor_aclr);
	}
    calculate_moving_avg(sensor_aclr, G_SENSOR_SAMPLE_FREQ, &(s_gsensor.senor_aclr_moving_avg), &(s_gsensor.cal_senor_aclr_avg_len));
   // ALOGD("(s_gsensor.senor_aclr_moving_avg is x= %f y=%f,z=%f\r\n",s_gsensor.senor_aclr_moving_avg.x, s_gsensor.senor_aclr_moving_avg.y,s_gsensor.senor_aclr_moving_avg.z);

    //还没学好
    if (s_gsensor.study_aclr_count < MIN_STUDY_GRAVITY_TIMES)
    {
		//保持不变
		//system_state_set_vehicle_state(VEHICLE_STATE_RUN);
    }
    //学习完毕
    else
    {

    	get_vehicle_acceleration(s_gsensor.gravity, s_gsensor.senor_aclr_moving_avg, &vehicle_horizontal_aclr, &vehicle_vertical_aclr);
       // ALOGD("s_gsensor gravity is %f, %f %f\r\n",s_gsensor.gravity.x, s_gsensor.gravity.y,s_gsensor.gravity.z);
       // ALOGD("s_gsensor senor_aclr_moving_avg is %f, %f %f\r\n",s_gsensor.senor_aclr_moving_avg.x, s_gsensor.senor_aclr_moving_avg.y,s_gsensor.senor_aclr_moving_avg.z);
      // ALOGD("vehicle_horizontal_aclr is %f,vehicle_vertical_aclr is %f\r\n",vehicle_horizontal_aclr, vehicle_vertical_aclr);
        //检测是否由静止转为运动
        //zly 2019-11-15
        // sensor input has been convert ro real
         vehicle_horizontal_aclr_magnitude = applied_math_get_magnitude_3d(vehicle_horizontal_aclr) * G_SENSOR_RANGE / G_SENSOR_MAX_VALUE;
         ALOGD("vehicle_horizontal_aclr_magnitude is %f \r\n",vehicle_horizontal_aclr_magnitude);
        //endzly
        //
        //vehicle_horizontal_aclr_magnitude = applied_math_get_magnitude_3d(vehicle_horizontal_aclr) /9.8;
        check_static_or_run(vehicle_horizontal_aclr_magnitude);
		aclr_alarm_enable= true;
		if (aclr_alarm_enable)
		{
			check_rapid_acceleration(vehicle_horizontal_aclr_magnitude);

	        check_emergency_brake(vehicle_horizontal_aclr_magnitude);

	        check_sudden_turn_acceleration(vehicle_vertical_aclr_magnitude);
		}     
    }

    study_gravity(a);
	/*
	if (s_gsensor.start_time % TIM_GEN_1SECOND == 0)
	{
		calc_sensor_angle();
		if (aclr_alarm_enable)
		{
			check_angle_alarm();
		}
	}*/
}

static void calc_sensor_angle(void)
{
    U8 sensor_angle = 0;
	Vector3D z_axis = {0, 0, -1};
    sensor_angle = applied_math_get_angle_3d(s_gsensor.senor_aclr_moving_avg, z_axis);  
    circular_queue_en_queue_i(&s_gsensor.sensor_angle_queue, sensor_angle);
}

/**
 * Function:   检查角度变化引起的报警（翻车报警、拆动报警）
 * Description:1、默认关闭，由固件管理平台配置
 *             2、启动2分钟内不报警（包括重启）
 *             3、保存最近1分钟角度记录（每秒1条，共60条）
 *             4、当前角度与历史记录差大于TURN_OVER_ANGLE并且当前状态为静止，判定为翻车报警
 *             5、当前角度与历史记录差大于REMOVE_ANGLE并且当前状态为运动，判定为拆动报警
 *             6、以上两种报警在1分钟记录内只报一条（通过限定上报间隔大于ANGLE_RECORD_NUM实现）,防止一次大的动作上报多次报警
 * Input:	   无
 * Output:	   无
 * Return:	   无
 * 测试方法:	   1、拆动：启动2分钟后，将定位器变换超过30°（由于误差存在，操作时角度要大于判断角度20°以上），要触发拆动报警
 *             2、翻车：启动2分钟后，将定位器变换超过45°（由于误差存在，操作时角度要大于判断角度20°以上），然后保持完全静止1分钟以上，要触发翻车报警
 *             3、不误报：
 *                1）无论什么安装角度，启动后不动不能报警
 *                2）水平晃动和垂直震动不能报警（车辆行驶）
 *                3）激烈驾驶（急加速、急减速）不能报警
 */

static void check_angle_alarm(void)
{
	U8 index = 0;
	S32 new_angle = 0;
	S32 last_angle = 0;
	U8 angle_diff = 0;
	U8 max_diff = 0;
	AlarmInfo alarm_info;
	U32 now = util_clock();
    GPSData new_gpsinfo = {0};
    if(!gps_get_last_data(&new_gpsinfo))
    {
        return;
    }

	//启动4分钟内角度不准确不检查角度报警
	if (now < 2*ANGLE_RECORD_NUM)
	{
		return;
	}

	if(false == circular_queue_get_tail_i(&(s_gsensor.sensor_angle_queue),&new_angle))
	{
		return;
	}

	//LOG(DEBUG,"now(%d) angle:%d",util_clock(),new_angle);

	for(index = 1;index < ANGLE_RECORD_NUM;index++)
	{
		//如果找不到历史记录退出循环
		if(false == circular_queue_get_by_index_i(&(s_gsensor.sensor_angle_queue),index,&last_angle))
		{
			break;
		}
		angle_diff = applied_math_get_angle_diff(new_angle,last_angle);
		if(angle_diff > max_diff)
		{
			max_diff = angle_diff;
		}
	}

	if (max_diff > TURN_OVER_ANGLE && VEHICLE_STATE_STATIC == system_state_get_vehicle_state() 
		&& (now - s_gsensor.last_report_turnover_alarm_time) > ANGLE_RECORD_NUM)
	{
		//LOG(FATAL,"TURN_OVER_ANGLE:%d",max_diff);
		alarm_info.type = ALARM_TURN_OVER;
       // report_alarm(ALARM_TURN_OVER);
        report_alarm(ALARM_TURN_OVER,new_gpsinfo.lat,new_gpsinfo.lng,new_gpsinfo.course);


		//alarm_info.info = max_diff;
		//gps_service_push_alarm(&alarm_info);
		system_state_set_turn_over_alarm(true);
		s_gsensor.last_report_turnover_alarm_time = now;
	}
	else if(max_diff > REMOVE_ANGLE && VEHICLE_STATE_RUN == system_state_get_vehicle_state() 
		&& (now - s_gsensor.last_report_remove_alarm_time) > ANGLE_RECORD_NUM)
	{
		//LOG(FATAL,"ALARM_REMOVE:%d",max_diff);
		alarm_info.type = ALARM_REMOVE;
		alarm_info.info = max_diff;
		//gps_service_push_alarm(&alarm_info);
		system_state_set_remove_alarm(true);
		s_gsensor.last_report_remove_alarm_time = now;
	}
	else
	{
	}

}
static void check_collision(Vector3D sensor_aclr)
{
	GM_ERRCODE ret = GM_SUCCESS;
	AlarmInfo alarm_info;
    GPSData new_gpsinfo = {0};
    if(!gps_get_last_data(&new_gpsinfo))
    {
        return;
    }

  float sensor_alcr_mag = applied_math_get_magnitude_3d(sensor_aclr) * G_SENSOR_RANGE / (G_SENSOR_MAX_VALUE);
    //zly 2019-11-15sensor input has been convert ro real  //
////[3g,5g)轻微碰撞(人不会受伤);
/// [5g,10g)中等碰撞(人受轻伤);
/// [10g,+∞)严重碰撞(人受重伤)
    sensor_alcr_mag = sensor_alcr_mag/9.8;
    ALOGD("sensor_alcr_mag is %4f\r\n",sensor_alcr_mag);
    COLLISION_LEVEL level = NO_COLLISION;
    if (sensor_alcr_mag >= s_gsensor.threshold.serious_crash_thr)
    {
        s_gsensor.collision_aclr_count++;
        level = SERIOUS_COLLISION;
    }
    else if(sensor_alcr_mag >= s_gsensor.threshold.normal_crash_thr)
    {
        s_gsensor.collision_aclr_count++;
        level = NORMAL_COLLISION;
    }
    else if(sensor_alcr_mag >= s_gsensor.threshold.slight_crash_thr)
    {
        s_gsensor.collision_aclr_count++;
        level = SLIGHT_COLLISION;
    }
    else
    {
        s_gsensor.collision_aclr_count = 0;
        level = NO_COLLISION;
    }

	//记录最严重的一次碰撞
    if(level >= s_gsensor.collision_level)
    {
        s_gsensor.collision_level = level;
        gps_get_last_data(&(s_gsensor.last_collision_gps_info));
        s_gsensor.last_collision_aclr.x = sensor_aclr.x;
        s_gsensor.last_collision_aclr.y = sensor_aclr.y;
        s_gsensor.last_collision_aclr.z = sensor_aclr.z;
    }

	//3次（30ms)以上的高加速度才算碰撞,防止传感器突发错误数据误报
	if(s_gsensor.collision_aclr_count >= 3)
	{
		alarm_info.type = ALARM_COLLISION;
        ALOGD("check_collision\r\n");

      //  report_alarm(ALARM_COLLISION);
        report_alarm(ALARM_COLLISION,new_gpsinfo.lat,new_gpsinfo.lng,new_gpsinfo.course);
		alarm_info.info = s_gsensor.collision_level;
		s_gsensor.collision_level = NO_COLLISION;
		s_gsensor.collision_aclr_count = 0;
		//ret = gps_service_push_alarm(&alarm_info);

		system_state_set_collision_alarm(true);
		if (GM_SUCCESS != ret)
		{
		//	LOG(ERROR, "Failed to gps_service_push_alarm(ALARM_COLLISION),ret=%d", ret);
		}
	}
}



static bool check_emergency_brake(float vehicle_horizontal_aclr_magnitude)
{
    if (vehicle_horizontal_aclr_magnitude >= s_gsensor.threshold.brake_thr)
    {
		//GM_StartTimer(GM_TIMER_GSENSOR_CHECK_SPEED_AFTER_EMERGENCY_BRAKE, GM_TICKS_1_SEC * 5, report_emergency_brake_alarm);
        emergency_braketmfilter.cstart=clock();
        if(emergency_braketmfilter.cstart-emergency_braketmfilter.cends>=emergency_braketmfilter.timethresd) {
            GM_startemergency_braketimer();
            emergency_braketmfilter.cends=clock();
            return true;
        } else
        {
            //drop the input
            ALOGD( "Drop check_emergency_brake \n\n");
            return false;
        }
    }
    else
    {
        return false;
    }
}

//上报急刹车报警
void report_emergency_brake_alarm(void)
{
    GPSData new_gpsinfo = {0};
    GPSData last_five_second_gpsinfo = {0};
	AlarmInfo alarm_info;



	GM_ERRCODE ret = GM_SUCCESS;

    if(m_speedtag==0)//0 no speed ;speed 1 using gps speed
    {
        alarm_info.type = ALARM_SPEED_DOWN;
        ALOGD("ALARM_SPEED_DOWN\r\n");
        report_alarm(ALARM_SPEED_DOWN,last_five_second_gpsinfo.lat,last_five_second_gpsinfo.lng,last_five_second_gpsinfo.course);
        return;
    }
	
    if(!gps_get_last_data(&new_gpsinfo))
    {
        return;
    }
    if(!gps_get_last_n_senconds_data(5,&last_five_second_gpsinfo))
    {
        return;
    }
	
    //是急转弯,角度变化大于2倍MIN_ANGLE_RANGE
    if (last_five_second_gpsinfo.speed > MIN_CRUISE_SPEED && applied_math_get_angle_diff(new_gpsinfo.course,last_five_second_gpsinfo.course) > 2 * MIN_ANGLE_RANGE)
    {
        return;
    }
	
    //最近1秒的速度比5秒前的速度低20km/h以上,才判定为急减速
    if(new_gpsinfo.speed - last_five_second_gpsinfo.speed < s_gsensor.threshold.rapid_dec_speed_thr)
    {
        alarm_info.type = ALARM_SPEED_DOWN;
        ALOGD("ALARM_SPEED_DOWN\r\n");
        report_alarm(ALARM_SPEED_DOWN,last_five_second_gpsinfo.lat,last_five_second_gpsinfo.lng,last_five_second_gpsinfo.course);
		//ret = gps_service_push_alarm(&alarm_info);
		system_state_set_speed_down_alarm(true);
		if (GM_SUCCESS != ret)
		{
			//LOG(ERROR, "Failed to gps_service_push_alarm(ALARM_SPEED_DOWN),ret=%d", ret);
		}
    }
}


static bool check_rapid_acceleration(float vehicle_horizontal_aclr_magnitude)
{
    if (vehicle_horizontal_aclr_magnitude >= s_gsensor.threshold.rapid_aclr_thr)
    {
		//GM_StartTimer(GM_TIMER_GSENSOR_CHECK_SPEED_AFTER_RAPID_ACCERATION, GM_TICKS_1_SEC * 5, report_rapid_aclr_alarm);

        rapid_accelerationtmfilter.cstart=clock();
        if(rapid_accelerationtmfilter.cstart-rapid_accelerationtmfilter.cends>=rapid_accelerationtmfilter.timethresd) {
            GM_startrapid_accelerationtimer();
            rapid_accelerationtmfilter.cends=clock();
            return true;
        } else
        {
            //drop the input
            ALOGD( "Drop check_rapid_acceleration \n\n");
            return false;
        }

    }
    else
    {
        return false;
    }
}


//获取急加速记录
 void report_rapid_aclr_alarm(void)
{
    GPSData new_gpsinfo = {0};
    GPSData last_five_second_gpsinfo = {0};
	AlarmInfo alarm_info;
	GM_ERRCODE ret = GM_SUCCESS;
    if(m_speedtag==0)//0 no speed ;speed 1 using gps speed
    {
        alarm_info.type = ALARM_SPEED_UP;
        ALOGD("ALARM_SPEED_UP\r\n");
        report_alarm(ALARM_SPEED_UP,last_five_second_gpsinfo.lat,last_five_second_gpsinfo.lng,last_five_second_gpsinfo.course);
        return;
    }

    if(!gps_get_last_data(&new_gpsinfo))
    {
        return;
    }
    if(!gps_get_last_n_senconds_data(5,&last_five_second_gpsinfo))
    {
        return;
    }
	
    //是急转弯,角度变化大于2倍MIN_ANGLE_RANGE
    if (last_five_second_gpsinfo.speed > MIN_CRUISE_SPEED && applied_math_get_angle_diff(new_gpsinfo.course,last_five_second_gpsinfo.course) > 2*MIN_ANGLE_RANGE)
    {
        return;
    }

    //最近1秒的速度比5秒前的速度大10km/h以上,才判定为急加速
    if(new_gpsinfo.speed - last_five_second_gpsinfo.speed > s_gsensor.threshold.rapid_inc_speed_thr)
    {
        alarm_info.type = ALARM_SPEED_UP;
        ALOGD("ALARM_SPEED_UP\r\n");
        report_alarm(ALARM_SPEED_UP,last_five_second_gpsinfo.lat,last_five_second_gpsinfo.lng,last_five_second_gpsinfo.course);
		//ret = gps_service_push_alarm(&alarm_info);
		system_state_set_speed_up_alarm(true);
		if (GM_SUCCESS != ret)
		{
			//LOG(ERROR, "Failed to gps_service_push_alarm(ALARM_SPEED_UP),ret=%d", ret);
		}
    }
}

static bool check_sudden_turn_acceleration(float vehicle_horizontal_aclr_magnitude)
{
    if (vehicle_horizontal_aclr_magnitude >= s_gsensor.threshold.sudden_turn_thr)
    {
		//GM_StartTimer(GM_TIMER_GSENSOR_CHECK_ANGLE_AFTER_SUDDEN_TURN, GM_TICKS_1_SEC * 5, report_sudden_turn_alarm);

        sudden_turntmfilter.cstart=clock();
        if(sudden_turntmfilter.cstart-sudden_turntmfilter.cends>=sudden_turntmfilter.timethresd) {
            GM_startsudden_turn_accelerationtimer();
            sudden_turntmfilter.cends=clock();
            return true;
        } else
        {
            //drop the input
            ALOGD( "Drop check_sudden_turn \n\n");
            return false;
        }

    }
    else
    {
        return false;
    }
}

void report_sudden_turn_alarm(void)
{
    GPSData new_gpsinfo = {0};
    GPSData last_five_second_gpsinfo = {0};
    unsigned int index = 1;
    unsigned int direction_diff = 0;
	AlarmInfo alarm_info;
	GM_ERRCODE ret = GM_SUCCESS;
	//debugzly

    if(m_speedtag==0)//0 no speed ;speed 1 using gps speed
    {
        alarm_info.type = ALARM_SHARP_TURN;
        ALOGD("ALARM_SHARP_TURN\r\n");
        report_alarm(ALARM_SHARP_TURN,last_five_second_gpsinfo.lat,last_five_second_gpsinfo.lng,last_five_second_gpsinfo.course);
        return;
    }

   // report_alarm(ALARM_SHARP_TURN);
    //
    if(!gps_get_last_data(&new_gpsinfo))
    {
        return;
    }
	
    for(index = 1;index <= 5;index++)
    {
        if (!gps_get_last_n_senconds_data(index, &last_five_second_gpsinfo))
        {
            return;
        }
        direction_diff = applied_math_get_angle_diff(new_gpsinfo.course,last_five_second_gpsinfo.course);
        //角度变化是在转弯的时候发生,本来加速度就会加大,角度变化超过10度/秒
        if (last_five_second_gpsinfo.speed > MIN_CRUISE_SPEED && direction_diff > MIN_ANGLE_RANGE/3.0 * index)
        {
            alarm_info.type = ALARM_SHARP_TURN;
			//ret = gps_service_push_alarm(&alarm_info);
			system_state_set_sharp_turn_alarm(true);
            report_alarm(ALARM_SHARP_TURN,last_five_second_gpsinfo.lat,last_five_second_gpsinfo.lng,last_five_second_gpsinfo.course);
			if (GM_SUCCESS != ret)
			{
				//LOG(ERROR, "Failed to gps_service_push_alarm(ALARM_SHARP_TURN),ret=%d", ret);
			}
            break;
        }
        else
        {
            continue;
        }
    }
}


static void study_gravity(Aclr a)
{
    Vector3D sensor_aclr = {0,0,0};
    sensor_aclr.x = a.x;
    sensor_aclr.y = a.y;
    sensor_aclr.z = a.z;
    //1、首次执行
    //2、运行1年以上（计算滑动平均值有累积误差）
    if (s_gsensor.study_aclr_count == 0 || s_gsensor.study_aclr_count % ((U64)G_SENSOR_SAMPLE_FREQ * (U64)SECONDS_PER_YEAR) == 0)
    {
        //重置各项参数
        reset_sensor_data();
        //重置重力加速度初值
        s_gsensor.gravity.x = a.x;
        s_gsensor.gravity.y = a.y;
        s_gsensor.gravity.z = a.z;
        s_gsensor.senor_aclr_moving_avg.x = a.x;
        s_gsensor.senor_aclr_moving_avg.y = a.y;
        s_gsensor.senor_aclr_moving_avg.z = a.z;

        //清空计算滑动平均值的队列
        circular_queue_empty(&(s_gsensor.aclr_x_queue));
        circular_queue_empty(&(s_gsensor.aclr_y_queue));
        circular_queue_empty(&(s_gsensor.aclr_z_queue));

        //最新的加速度值入队列
        circular_queue_en_queue_i(&(s_gsensor.aclr_x_queue), a.x);
        circular_queue_en_queue_i(&(s_gsensor.aclr_y_queue), a.y);
        circular_queue_en_queue_i(&(s_gsensor.aclr_z_queue), a.z);
    }
    else
    {
        //长距离滑动平均得到重力和固定误差
        calculate_moving_avg(sensor_aclr, G_SENSOR_BUF_LEN - 1, &(s_gsensor.gravity), &(s_gsensor.cal_gravity_avg_len));
        if (VEHICLE_STATE_STATIC == system_state_get_vehicle_state())
        {
            s_gsensor.static_gravity = s_gsensor.gravity;
        }
    }
    s_gsensor.study_aclr_count++;
}


/*****************************************************************
功能:分解加速度到竖直方向和水平方向（传感器坐标系）
输入参数:g——滤波后重力加速度
输入参数:sensor_aclr——滤波后传感器加速度
输出参数:vehicle_horizontal_aclr——指向汽车水平方向加速度的指针
输出参数:vehicle_vertical_aclr——指向汽车垂直方向加速度的指针
返回值:0——成功；其它值错误
*****************************************************************/
static GM_ERRCODE get_vehicle_acceleration(Vector3D g, Vector3D sensor_aclr, Vector3D *vehicle_horizontal_aclr, Vector3D *vehicle_vertical_aclr)
{
    Vector3D B1 = {0, 0, 0};
    Vector3D B2 = {0, 0, 0};
    Vector3D B3 = {0, 0, 0};

    Vector3D B11 = {0, 0, 0};
    Vector3D B12 = {0, 0, 0};
    Vector3D B21 = {0, 0, 0};
    Vector3D B22 = {0, 0, 0};
    Vector3D B31 = {0, 0, 0};
    Vector3D B32 = {0, 0, 0};

    float s = 0;
    float t = 0;

    //判断参数合法性
    if (NULL == vehicle_horizontal_aclr || NULL == vehicle_vertical_aclr)
    {
        //ALOGD("get_vehicle_acceleration error\r\n");
        return GM_PARAM_ERROR;
    }
    //传感器加速度减去重力和固有误差
    sensor_aclr.x -= g.x;
    sensor_aclr.y -= g.y;
    sensor_aclr.z -= g.z;
   // ALOGD("sensor_aclr x y z  %f %f %f\r\n", sensor_aclr.x , sensor_aclr.y, sensor_aclr.z);

    //传感器加速度x分量在水平面的投影
    s = g.x * g.x + g.y * g.y + g.z * g.z;
    t = (g.x * sensor_aclr.x) / s;
    B11.x = sensor_aclr.x - g.x * t;
    B11.y = 0 - g.y * t;
    B11.z = 0 - g.z * t;
    B12.x = B1.x - B11.x;
    B12.y = B1.y - B11.y;
    B12.z = B1.z - B11.z;
  //  ALOGD("B11 x y z  %f %f %f B12  x y z  \r\n", B11.x , B11.y, B11.z,B12.x , B12.y, B12.z);
    //传感器加速度y分量在水平面的投影
    t = (g.y * sensor_aclr.y) / s;
    B21.x = 0 - g.x * t;
    B21.y = sensor_aclr.y - g.y * t;
    B21.z = 0 - g.z * t;
    B22.x = B2.x - B21.x;
    B22.y = B2.y - B21.y;
    B22.z = B2.z - B21.z;
   // ALOGD("B21 x y z  %f %f %f B22  x y z  %f %f %f \r\n", B21.x , B21.y, B21.z,B22.x , B22.y, B22.z);
    //传感器加速度z分量在水平面的投影
    t = (g.z * sensor_aclr.z) / s;
    B31.x = 0 - g.x * t;
    B31.y = 0 - g.y * t;
    B31.z = sensor_aclr.z - g.z * t;
    B32.x = B3.x - B31.x;
    B32.y = B3.y - B31.y;
    B32.z = B3.z - B31.z;
  //  ALOGD("B31 x y z  %f %f %f B32  x y z  %f %f %f \r\n", B31.x , B31.y, B31.z,B32.x , B32.y, B32.z);
    //车辆加速度的水平分量
    vehicle_horizontal_aclr->x = B11.x + B21.x + B31.x;
    vehicle_horizontal_aclr->y = B11.y + B21.y + B31.y;
    vehicle_horizontal_aclr->z = B11.z + B21.z + B31.z;
//    ALOGD("vehicle_horizontal_aclr x y z  %f %f %f\r\n", vehicle_horizontal_aclr->x , vehicle_horizontal_aclr->y, vehicle_horizontal_aclr->z);
    //车辆加速度的垂直分量
    vehicle_vertical_aclr->x = B12.x + B22.x + B32.x;
    vehicle_vertical_aclr->y = B12.y + B22.y + B32.y;
    vehicle_vertical_aclr->z = B12.z + B22.z + B32.z;
  //  ALOGD("vehicle_vertical_aclr x y z  %f %f %f\r\n", vehicle_vertical_aclr->x , vehicle_vertical_aclr->y, vehicle_vertical_aclr->z);
    return GM_SUCCESS;
}


//滑动平均
static void calculate_moving_avg(Vector3D sensor_aclr, int num, PVector3D p_aclr_moving_avg, unsigned int *p_len)
{
    int head_x = 0;
    int head_y = 0;
    int head_z = 0;
    int len = circular_queue_get_len(&(s_gsensor.aclr_x_queue));
    if (NULL == p_aclr_moving_avg || NULL == p_len || num <= 0 || num > circular_queue_get_capacity(&(s_gsensor.aclr_x_queue)))
    {
        return;
    }
    if (circular_queue_is_empty(&(s_gsensor.aclr_x_queue)))
    {
        *p_aclr_moving_avg = sensor_aclr;
        *p_len = 1;
    }
    //不满
    else if (num >= len)
    {
        p_aclr_moving_avg->x = (sensor_aclr.x + (len - 1) * p_aclr_moving_avg->x) / (len);
        p_aclr_moving_avg->y = (sensor_aclr.y + (len - 1) * p_aclr_moving_avg->y) / (len);
        p_aclr_moving_avg->z = (sensor_aclr.z + (len - 1) * p_aclr_moving_avg->z) / (len);
        *p_len = len;
        //printf("len = %d,x = %f,y=%f,z=%f\n",*p_len,p_aclr_moving_avg->x,p_aclr_moving_avg->y,p_aclr_moving_avg->z);
    }
    //满了
    else if (num < len)
    {
        circular_queue_get_by_index_i(&(s_gsensor.aclr_x_queue), num, &head_x);
        circular_queue_get_by_index_i(&(s_gsensor.aclr_y_queue), num, &head_y);
        circular_queue_get_by_index_i(&(s_gsensor.aclr_z_queue), num, &head_z);
        p_aclr_moving_avg->x = (sensor_aclr.x + (*p_len) * p_aclr_moving_avg->x - head_x) / num;
        p_aclr_moving_avg->y = (sensor_aclr.y + (*p_len) * p_aclr_moving_avg->y - head_y) / num;
        p_aclr_moving_avg->z = (sensor_aclr.z + (*p_len) * p_aclr_moving_avg->z - head_z) / num;
        *p_len = num;
        //printf("len = %d, head.x = %f, head.y = %f, head.z = %f,x = %f,y=%f,z=%f\n",*p_len,head_x,head_y,head_z,p_aclr_moving_avg->x,p_aclr_moving_avg->y,p_aclr_moving_avg->z);
    }
    else
    {
        //不存在这种情况
    }
}


static void check_static_or_run(float vehicle_horizontal_aclr_magnitude)
{
	//不过不休眠，也不判断静止	
	U16 sleep_time_minute_threshold = 20;
	//config_service_get(CFG_SLEEP_TIME, TYPE_SHORT, &sleep_time_minute_threshold, sizeof(sleep_time_minute_threshold));
	if(0 == sleep_time_minute_threshold)
	{
		system_state_set_vehicle_state(VEHICLE_STATE_RUN);
		return;
	}
	
    //1、正常行驶:加速度（传感器）超过阈值
    if (vehicle_horizontal_aclr_magnitude >= s_gsensor.threshold.run_thr)
    {
        s_gsensor.static_low_aclr_count = 0;
        s_gsensor.super_low_aclr_count = 0;
		if (VEHICLE_STATE_STATIC == system_state_get_vehicle_state())
		{
			bool move_alarm_enable = false;
			//config_service_get(CFG_IS_MOVEALARM_ENABLE, TYPE_BOOL, &move_alarm_enable, sizeof(move_alarm_enable));
			//自从休眠唤醒以后还没有上报过移动报警
			if (move_alarm_enable && !system_state_get_move_alarm())
			{
				AlarmInfo alarm_info;
				alarm_info.type = ALARM_MOVE;
	  			//gps_service_push_alarm(&alarm_info);
	  			system_state_set_move_alarm(true);
				//LOG(WARN,"MOVE ALARM");
			}
			system_state_set_vehicle_state(VEHICLE_STATE_RUN);

		}
		//LOG(DEBUG,"RUN:aclr=%f,thr=%f",vehicle_horizontal_aclr_magnitude,s_gsensor.threshold.run_thr);
  	}
    //2、运动转为静止
    else if (vehicle_horizontal_aclr_magnitude <= s_gsensor.threshold.static_thr)
    {
        s_gsensor.static_low_aclr_count++;
        //MIN_LOW_ACLR_NUM次加速度小于阈值才判定为静止了
        if (s_gsensor.static_low_aclr_count >= MIN_LOW_ACLR_NUM)
        {
			system_state_set_vehicle_state(VEHICLE_STATE_STATIC);
        }
        //如果加速度特别低,连续10秒就变静止
        if (vehicle_horizontal_aclr_magnitude <= (s_gsensor.threshold.static_thr / 5))
        {
            s_gsensor.super_low_aclr_count++;
            if (s_gsensor.super_low_aclr_count >= MIN_LOW_ACLR_NUM / 2)
            {
				system_state_set_vehicle_state(VEHICLE_STATE_STATIC);
            }
        }
        else
        {
            s_gsensor.super_low_aclr_count = 0;
        }
    }
    //3、中间态
    else
    {
    }

    //4、匀速行驶:速度超过阈值,并且加速度（通过速度计算）小
    if ((gps_get_constant_speed_time() >= MIN_CRUISE_SPEED_TIME && gps_get_aclr() < s_gsensor.threshold.rapid_aclr_thr * GRAVITY_CONSTANT))
    {
        s_gsensor.static_low_aclr_count = 0;
        s_gsensor.super_low_aclr_count = 0;
		if (VEHICLE_STATE_STATIC == system_state_get_vehicle_state())
		{
			//LOG(INFO,"RUN because of speed,time:%d,gps aclr:%f",gps_get_constant_speed_time(),gps_get_aclr());
			system_state_set_vehicle_state(VEHICLE_STATE_RUN);
			//GM_StartTimer(GM_TIMER_BMS_TRANSPRENT, 1000, bms_transprent_callback);
		}
    }
}


