/**
 * Copyright @ 深圳市谷米万物科技有限公司. 2009-2019. All rights reserved.
 * File name:        gps.h
 * Author:           王志华       
 * Version:          1.0
 * Date:             2019-03-19
 * Description:      使用agps_service、epo_service、config_service
 * Others:      
 * Function List:    
    1. 创建gps模块
    2. 销毁gps模块
    3. 模块定时处理入口
    4. 打开GPS模块
    5. 关闭GPS模块
    6. 获取最新GPS数据
 * History: 
    1. Date:         2019-03-19
       Author:       王志华
       Modification: 创建初始版本
    2. Date: 		 
	   Author:		 
	   Modification: 

 */
#include <jni.h>
#include <stdio.h>
#include <math.h>
#include "gps.h"
#include "applied_math.h"
#include "circular_queue.h"
#include "g_sensor.h"
#include "systemstate.h"
#include "utility.h"
#include "tyyslog.h"
#include <string.h>


#define GPS_BUFF_LEN 50


typedef enum 
{	//未初始化
	GM_GPS_STATE_NONE = 0,
		
	//初始化OK
	GM_GPS_STATE_INITED,

	//发送了版本号查询,等待版本号响应
	GM_GPS_STATE_WAIT_VERSION,

	//发送了AGPS时间请求,等待AGPS时间响应
	GM_GPS_STATE_WAIT_APGS_TIME,
	
	//发送了AGPS数据请求,等待AGPS数据响应
	GM_GPS_STATE_WAIT_APGS_DATA,

	//工作中
	GM_GPS_STATE_WORKING,
	
}GPSInternalState;

typedef struct
{
	//使用的波特率
	U32 baud_rate;

	//GPS芯片型号
	GPSChipType gpsdev_type;

	//内部状态
	GPSInternalState internal_state;

	//整体状态
	GPSState state;

	//定位状态时间记录
    StateRecord state_record;

	//存放定位数据的环形队列
	CircularQueue gps_time_queue;
	CircularQueue gps_lat_queue;
	CircularQueue gps_lng_queue;
	CircularQueue gps_speed_queue;
	CircularQueue gps_course_queue;

	//启动时间
	time_t power_on_time;

	time_t mtk_start_time;

	time_t last_rcv_time;

	bool push_data_enbale;

	//AGPS完成注入时间
	U16 agps_time;

	//定位时间
	time_t fix_time;

	//内存中保存时间（每秒一条）
	time_t save_time;

	//上报平台时间
	time_t report_time;

	time_t sleep_time;

	//从上次上报点到当前点的距离（曲线距离）
	float distance_since_last_report;

	//是否已打开VTG语句（泰斗芯片需要打开VTG语句）
	bool has_opened_td_vtg;
	


	//辅助定位经纬度
	float ref_lng;
	float ref_lat;

	//定位状态
	//GPSState的第0位表示是否定位定位,0——未定位；1——已定位
	bool is_fix;
	
	//GPSState的第1位表示2D/3D定位,0——2D；1——3D
	bool is_3d;

	//GPSState的第2位表示是否差分定位定位,0——非差分定位；1——差分定位
	bool is_diff;

	//信号强度等级（0-100,100最高）
	U8 signal_intensity_grade;

	//水平方向定位精度[0.5,99.9]
	float hdop;

    //可见卫星数
	U8 satellites_tracked;

	//天线高度（相对于平均海平面,单位米）
    float altitude; 

	//最大信噪比(1分钟更新一次)
	U8 max_snr;

	//信号最好的5颗星
	SNRInfo snr_array[SNR_INFO_NUM];

	//可见卫星数(1秒更新一次)
	U8 satellites_inview;

	//snr大于35的卫星数,即我们认为可用的卫星数(1秒更新一次)
	U8 satellites_good;

	float aclr_avg_calculate_by_speed;
	U32 constant_speed_time;
	U32 static_speed_time;
	StateRecord over_speed_alarm_state;
}Gps,*PGps;

static Gps s_gps;
	
//GPS串口超过多少秒没有接收到数据关闭重新打开
#define GM_REOPEN_GPS_PORT_TIME 30

static void clear_data(void);

static void check_has_received_data(void);

static void set_device_type(const GPSChipType dev_type);

static void write_mtk_config(void);

//查询MTK版本号
static void query_mtk_version(void);

//知道9600波特率能收到数据,查询泰斗和中科微版本号
static void query_td_and_at_version(void);

static void query_td_version(void);

static void query_at_version(void);
	
static GM_ERRCODE write_mtk_epo_time(const ST_Time utc_time);

static void write_mtk_epo_pos(void);

static void open_td_vtg(void);


static bool is_turn_point(const GPSData current_gps_data);

static void upload_gps_data(const GPSData current_gps_data);

static void check_fix_state(void);

static void check_fix_state_change(void);



static void check_over_speed_alarm(float speed);

static void reopen_gps(void);

static void request_write_time(void);





GM_ERRCODE gps_create(void)
{
	clear_data();
	s_gps.sleep_time = 0;
	circular_queue_create(&s_gps.gps_time_queue, GPS_BUFF_LEN, GM_QUEUE_TYPE_INT);
	circular_queue_create(&s_gps.gps_lat_queue, GPS_BUFF_LEN, GM_QUEUE_TYPE_FLOAT);
	circular_queue_create(&s_gps.gps_lng_queue, GPS_BUFF_LEN, GM_QUEUE_TYPE_FLOAT);
	circular_queue_create(&s_gps.gps_speed_queue, GPS_BUFF_LEN, GM_QUEUE_TYPE_FLOAT);
	circular_queue_create(&s_gps.gps_course_queue, GPS_BUFF_LEN, GM_QUEUE_TYPE_FLOAT);

	return GM_SUCCESS;
}

GM_ERRCODE gps_destroy(void)
{
	circular_queue_destroy(&s_gps.gps_time_queue, GM_QUEUE_TYPE_INT);
	circular_queue_destroy(&s_gps.gps_lat_queue, GM_QUEUE_TYPE_FLOAT);
	circular_queue_destroy(&s_gps.gps_lng_queue, GM_QUEUE_TYPE_FLOAT);
	circular_queue_destroy(&s_gps.gps_speed_queue, GM_QUEUE_TYPE_FLOAT);
	circular_queue_destroy(&s_gps.gps_course_queue, GM_QUEUE_TYPE_FLOAT);
	return GM_SUCCESS;
}


GM_ERRCODE gps_timer_proc(void)
{
	return GM_SUCCESS;
}

static void clear_data(void)
{
	s_gps.gpsdev_type = GM_GPS_TYPE_UNKNOWN;
	s_gps.internal_state = GM_GPS_STATE_INITED;
	s_gps.agps_time = 0;
	s_gps.state = GM_GPS_OFF;
	s_gps.state_record.state = false;
	s_gps.state_record.true_state_hold_seconds = 0;
	s_gps.state_record.false_state_hold_seconds = 0;
	s_gps.power_on_time = 0;
	s_gps.mtk_start_time = 0;
    s_gps.last_rcv_time = 0;
	s_gps.push_data_enbale = false;
	s_gps.fix_time = 0;
	s_gps.save_time = 0;
	s_gps.report_time = 0;
	s_gps.distance_since_last_report = 0;
	s_gps.has_opened_td_vtg = false;
	s_gps.ref_lng = 0;
	s_gps.ref_lat = 0;
	s_gps.is_fix = false;
	s_gps.is_3d = false;
	s_gps.is_diff = false;
	s_gps.signal_intensity_grade = 0;
	s_gps.hdop = 99.9;
	s_gps.satellites_tracked = 0;
	s_gps.max_snr = 0;
	memset(s_gps.snr_array, 0, sizeof(s_gps.snr_array));
	s_gps.satellites_inview = 0;
	s_gps.satellites_good = 0;
	s_gps.altitude = 0;
	s_gps.aclr_avg_calculate_by_speed = 0;
	s_gps.constant_speed_time = 0;
	s_gps.static_speed_time = 0;
	s_gps.static_speed_time = 0;
	s_gps.over_speed_alarm_state.state = system_state_get_overspeed_alarm();
	s_gps.over_speed_alarm_state.true_state_hold_seconds = 0;
	s_gps.over_speed_alarm_state.false_state_hold_seconds = 0;
	

	
}

GM_ERRCODE gps_power_on(bool push_data_enbale)
{

}

static void check_fix_state(void)
{


}



GM_ERRCODE gps_power_off(void)
{
	s_gps.state = GM_GPS_OFF;

	return GM_SUCCESS;
}

GPSState gps_get_state(void)
{
	return s_gps.state;
}

bool gps_is_fixed(void)
{
	if (GM_GPS_FIX_3D > gps_get_state() || GM_GPS_OFF == gps_get_state())
	{
		return false;
	}
	else
	{
		return true;
	}
}

U16 gps_get_fix_time(void)
{
	return s_gps.fix_time;
}

U8 gps_get_max_snr(void)
{
	return s_gps.max_snr;
}

U8 gps_get_satellites_tracked(void)
{
	return s_gps.satellites_tracked;
}

U8 gps_get_satellites_inview(void)
{
	return s_gps.satellites_inview;
}

U8 gps_get_satellites_good(void)
{
	return s_gps.satellites_good;
}
const SNRInfo* gps_get_snr_array(void)
{
	return s_gps.snr_array;
}

bool gps_get_last_data(GPSData* p_data)
{
	return gps_get_last_n_senconds_data(0,p_data);
}

/**
 * Function:   获取最近n秒的GPS数据
 * Description:
 * Input:	   seconds:第几秒,从0开始
 * Output:	   p_data:指向定位数据的指针
 * Return:	   GM_SUCCESS——成功；其它错误码——失败
 * Others:	   
 */
bool gps_get_last_n_senconds_data(U8 seconds,GPSData* p_data)
{
	if (NULL == p_data)
	{
		return false;
	}
	else
	{
		if(!circular_queue_get_by_index_i(&s_gps.gps_time_queue, seconds, (S32 *)&p_data->gps_time))
		{
				return false;
		}
		if(!circular_queue_get_by_index_f(&s_gps.gps_lat_queue, seconds, &p_data->lat))
		{
				return false;
		}
		if(!circular_queue_get_by_index_f(&s_gps.gps_lng_queue, seconds, &p_data->lng))
		{
				return false;
		}
		if(!circular_queue_get_by_index_f(&s_gps.gps_speed_queue, seconds, &p_data->speed))
		{
				return false;
		}
		if(!circular_queue_get_by_index_f(&s_gps.gps_course_queue, seconds, &p_data->course))
		{
				return false;
		}
        p_data->satellites = s_gps.satellites_tracked;
		p_data->precision = s_gps.hdop;
		p_data->signal_intensity_grade = s_gps.signal_intensity_grade;
		return true;
	}
}





void gps_on_rcv_uart_data(char* p_data, const U16 len)
{

}

//每1秒调用一次
static void calc_alcr_by_speed(GPSData gps_info)
{
    float last_second_speed = 0;
    float aclr_calculate_by_speed = 0;

    if (gps_info.speed >= MIN_CRUISE_SPEED)
    {
        s_gps.constant_speed_time++;
        s_gps.static_speed_time = 0;
        if (circular_queue_get_tail_f(&(s_gps.gps_speed_queue), &last_second_speed))
        {
            aclr_calculate_by_speed = (gps_info.speed - last_second_speed) * 1000.0 / SECONDS_PER_HOUR;
            s_gps.aclr_avg_calculate_by_speed = (s_gps.aclr_avg_calculate_by_speed * (s_gps.constant_speed_time - 1) + aclr_calculate_by_speed) / (s_gps.constant_speed_time);
        }
    }
    else
    {
        s_gps.constant_speed_time = 0;
        s_gps.static_speed_time++;
        s_gps.aclr_avg_calculate_by_speed = 0;
    }

}

static void check_over_speed_alarm(float speed)
{
	bool speed_alarm_enable = false;
	U8 speed_threshold = 0;
	U8 speed_check_time = 0;
	GM_CHANGE_ENUM state_change = GM_NO_CHANGE;
	AlarmInfo alarm_info;
	alarm_info.type = ALARM_SPEED;
	alarm_info.info = speed;
	
	//config_service_get(CFG_SPEED_ALARM_ENABLE, TYPE_BOOL, &speed_alarm_enable, sizeof(speed_alarm_enable));
	//config_service_get(CFG_SPEEDTHR, TYPE_BYTE, &speed_threshold, sizeof(speed_threshold));
	//config_service_get(CFG_SPEED_CHECK_TIME, TYPE_BYTE, &speed_check_time, sizeof(speed_check_time));
	
	if(!speed_alarm_enable)
	{
		return;
	}
	
	state_change = util_check_state_change(speed > speed_threshold, &s_gps.over_speed_alarm_state, speed_check_time, speed_check_time);
	if(GM_CHANGE_TRUE == state_change)
	{
		system_state_set_overspeed_alarm(true);
		//gps_service_push_alarm(&alarm_info);
	}
	else if(GM_CHANGE_FALSE == state_change)
	{
		system_state_set_overspeed_alarm(false);
	}
	else
	{
		
	}
}

U32 gps_get_constant_speed_time(void)
{
	return s_gps.constant_speed_time;
}

float gps_get_aclr(void)
{
	return s_gps.aclr_avg_calculate_by_speed;
}


static bool is_turn_point(const GPSData current_gps_data)
{
	U16 course_change_threshhold = 0;
    float course_change = 0;
	float current_course = current_gps_data.course;
	float last_second_course = 0;
	float last_speed = 0;
	GM_ERRCODE ret = GM_SUCCESS;
	U8 index = 0;

	//ret = config_service_get(CFG_TURN_ANGLE, TYPE_SHORT, &course_change_threshhold, sizeof(course_change_threshhold));
	if(GM_SUCCESS != ret)
	{
		//LOG(ERROR, "Failed to config_service_get(CFG_TURN_ANGLE),ret=%d", ret);
		return false;
	}
	
	if(!circular_queue_get_by_index_f(&s_gps.gps_course_queue, 1, &last_second_course))
	{
		return false;
	}

	course_change = applied_math_get_angle_diff(current_course,last_second_course);	
	//LOG(DEBUG,"[%s]:current course:%f,last second course:%f,course change:%f",__FUNCTION__,current_course,last_second_course,course_change);
	
    if (current_gps_data.speed >= 80.0f)
    {
        if (course_change >= 6)
        {   
            return true;
        }
        else if (course_change >= course_change_threshhold)
        {
            return true;
        }
		else
		{
			return false;
		}
    }
    else if (current_gps_data.speed >= 40.0f)
    {
        if (course_change >= 7)
        {
            return true;
        }
        else if (course_change >= course_change_threshhold)
        {
            return true;
        }
		else
		{
			return false;
		}
    }
    else if (current_gps_data.speed >= 25.0f)
    {
        if (course_change >= 8)
        {
            return true;
        }
        else if (course_change >= course_change_threshhold)
        {
            return true;
        }
		else
		{
			return false;
		}
    }
    else if (current_gps_data.speed >= 15.0f)
    {
        if (course_change >= (course_change_threshhold - 6))
        {
            return true;
        }
		else
		{
			return false;
		}
    }
    else if ((current_gps_data.speed >= 5.0f) && course_change >= 16 && s_gps.distance_since_last_report > 20.0f)
    {
        // 连续5秒速度大于5KMH,角度大于20度,里程超过20米,上传拐点
        for (index =0; index < 5; index++)
        {
			if(!circular_queue_get_by_index_f(&s_gps.gps_speed_queue, index, &last_speed))
			{
				return false;
			}
            if (last_speed <= 5.0f)
            {
                return false;
            }
        }
		return true;
    }
	else
	{
		return false;
	}
}

void on_received_gps(       double localatitude,
							double locallongtitude,
							double localcourse,
							float localgpstime,
							int localgpsspeed)
{
	GPSData gps_data = {0};
	U8 index = 0;
	float lat_avg = 0;
	float lng_avg = 0;
	float speed_avg = 0;
	bool if_smooth_track = true;
	// time_t seconds_from_reboot = util_clock();
	gps_data.lat = localatitude;
	gps_data.lng =locallongtitude;
	gps_data.speed = localgpsspeed;
	gps_data.gps_time=localgpstime;
	gps_data.course=localcourse;

	if (fabs(gps_data.lat) < 0.1 &&  fabs(gps_data.lng)  < 0.1)
	{
		return;
	}
	ALOGD("on_received_gps\r\n");
	calc_alcr_by_speed(gps_data);
	//check_over_speed_alarm(gps_data.speed);
	circular_queue_en_queue_i(&s_gps.gps_time_queue,gps_data.gps_time);
	circular_queue_en_queue_f(&s_gps.gps_lat_queue,gps_data.lat);
	circular_queue_en_queue_f(&s_gps.gps_lng_queue, gps_data.lng);
	circular_queue_en_queue_f(&s_gps.gps_speed_queue,gps_data.speed);
	circular_queue_en_queue_f(&s_gps.gps_course_queue,gps_data.course);
	//config_service_get(CFG_SMOOTH_TRACK, TYPE_BOOL, &if_smooth_track, sizeof(if_smooth_track));
	if(if_smooth_track)
	{
		//LOG(DEBUG,"smooth track");
		//鍙栧墠10绉掞紙鍖呮嫭褰撳墠鏃堕棿鐐癸級骞冲潎鍊?
		for(index = 0;index < 10;index++)
		{
			time_t last_n_time = 0;
			float last_n_lat = 0;
			float last_n_lng = 0;
			float last_n_speed = 0;

			if(false == circular_queue_get_by_index_i(&s_gps.gps_time_queue,index,(S32*)&last_n_time))
			{
				break;
			}
			if (gps_data.gps_time - last_n_time >= 10)
			{
				break;
			}
			circular_queue_get_by_index_f(&s_gps.gps_lat_queue,index,&last_n_lat);
			circular_queue_get_by_index_f(&s_gps.gps_lng_queue, index,&last_n_lng);
			circular_queue_get_by_index_f(&s_gps.gps_speed_queue,index,&last_n_speed);

			lat_avg = (lat_avg*index + last_n_lat)/(index + 1);
			lng_avg = (lng_avg*index + last_n_lng)/(index + 1);
			speed_avg = (speed_avg*index + last_n_speed)/(index + 1);
		}
		gps_data.lat = lat_avg;
		gps_data.lng = lng_avg;
		gps_data.speed = speed_avg;
	}
}