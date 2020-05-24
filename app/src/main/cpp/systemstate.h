//
// Created by Administrator on 2019/11/9 0009.
//

#ifndef MY_APPLICATION_SYSTEMSTATE_H
#define MY_APPLICATION_SYSTEMSTATE_H

#include "error_code.h"
#include "gm_type.h"
#include "gps.h"
typedef enum
{
    GM_SYSTEM_STATE_WORK = 0,
    GM_SYSTEM_STATE_SLEEP = 1
}SystemWorkState;

typedef enum
{
    VEHICLE_STATE_RUN = 0,
    VEHICLE_STATE_STATIC = 1
} VehicleState;

typedef struct
{
    //16姣旂壒CRC锛堜笉鍖呭惈鍓?涓瓧娈?浠巗tatus_bits寮€濮嬬畻璧凤級
    U16 crc;

    //鐗瑰緛鍊?
    U32 magic;

    /**********************************************************************
    bit0 鈥斺€斿惎鍔ㄧ被鍨?0-閲嶅惎,1鈥斺€斾笂鐢?
    bit1 鈥斺€斾笂鐢靛悗鏄惁宸蹭笂浼犺繃GPS瀹氫綅
    bit2 鈥斺€旀槸鍚﹀凡鍙戦€侀潤姝㈢偣
    bit3 鈥斺€斾慨鏀笽P鍚庢槸鍚﹀凡涓婁紶GPS鏁版嵁
    bit4 鈥斺€斿厖鐢垫槸鍚﹀紑鍚?杞欢鎺у埗)
    bit5 鈥斺€擜CC妫€娴嬫ā寮?0鈥斺€旈渿鍔ㄦ娴嬶紱  1鈥斺€擜CC绾挎娴?
    bit6 鈥斺€旇澶噐elay绔彛鐘舵€?           0鈥斺€斾綆鐢靛钩锛堟仮澶嶆补鐢碉級锛? 1鈥斺€旈珮鐢靛钩锛堟柇娌圭數锛?
    bit7 鈥斺€旂敤鎴疯缃畆elay绔彛鐘舵€?0鈥斺€斾綆鐢靛钩锛堟仮澶嶆补鐢碉級锛? 1鈥斺€旈珮鐢靛钩锛堟柇娌圭數锛?
    bit8 鈥斺€旇闃茬姸鎬?
    bit9 鈥斺€斾笂鐢靛悗鏄惁宸蹭笂浼犺繃LBS瀹氫綅
    bit10鈥斺€旀槸鍚﹀喎鍚姩
    bit11鈥斺€斿鐢?
    bit12鈥斺€斿鐢?
    bit13鈥斺€斿鐢?
    bit14鈥斺€斿鐢?
    bit15鈥斺€斿鐢?

    bit16鈥斺€旀柇鐢垫姤璀?
    bit17鈥斺€旂數姹犱綆鐢垫姤璀?
    bit18鈥斺€旂數婧愮數鍘嬭繃浣庢姤璀?
    bit19鈥斺€旂數婧愮數鍘嬭繃楂樻姤璀?
    bit20鈥斺€旈渿鍔ㄦ姤璀?
    bit21鈥斺€旇秴閫熸姤璀?
    bit22鈥斺€斾吉鍩虹珯鎶ヨ
    bit23鈥斺€旂鎾炴姤璀?
    bit24鈥斺€旀€ュ姞閫熸姤璀?
    bit25鈥斺€旀€ュ噺閫熸姤璀?
    bit26鈥斺€旂炕杞︽姤璀?
    bit27鈥斺€旀€ヨ浆寮姤璀?
    bit28鈥斺€旀媶鍔ㄦ姤璀?
    bit29鈥斺€旇溅杈嗙Щ鍔ㄦ姤璀?
    bit30鈥斺€斿鐢?
    bit31鈥斺€斿鐢?
    **********************************************************************/
    U32 status_bits;

    //绯荤粺鐘舵€侊紙宸ヤ綔,浼戠湢锛?
    SystemWorkState work_state;

    //杞﹁締鐘舵€侊紙杩愬姩,闈欐锛?
    VehicleState vehicle_state;


    //鍚敤鏃堕棿
    U32 start_time;

    //鍘嗙▼锛堝崟浣?绫筹級
    U64 mileage;

    //褰撳墠鍙墽琛屾枃浠剁殑鏍￠獙鍚?
    U32 check_sum;


    u32 last_good_time;  // 涓婃缃戠粶姝ｅ父鐨勬椂闂?
    u32 call_ok_count;   //CURRENT_GPRS_INIT->CURRENT_GPRS_CALL_OK 娆℃暟


    GPSData latest_gps;

}SystemState,*PSystemState;



static SystemState s_system_state;
GM_ERRCODE system_state_create(void);
static void init_para(void);
GM_ERRCODE system_state_timer_proc(void);
GM_ERRCODE system_state_set_work_state(const SystemWorkState work_state);
SystemWorkState system_state_get_work_state(void);
bool system_state_get_collision_alarm(void);
GM_ERRCODE system_state_set_collision_alarm(bool state);
bool system_state_get_speed_down_alarm(void);
GM_ERRCODE system_state_set_speed_down_alarm(bool state);
bool system_state_get_speed_up_alarm(void);
bool system_state_set_speed_up_alarm(bool state);
bool system_state_get_sharp_turn_alarm(void);
GM_ERRCODE system_state_set_sharp_turn_alarm(bool state);
GM_ERRCODE system_state_set_vehicle_state(const VehicleState vehicle_state);
VehicleState system_state_get_vehicle_state(void);
bool system_state_get_turn_over_alarm(void);
GM_ERRCODE system_state_set_turn_over_alarm(bool state);
bool system_state_get_remove_alarm(void);
GM_ERRCODE system_state_set_remove_alarm(bool state);
bool system_state_get_move_alarm(void);
GM_ERRCODE system_state_set_move_alarm(bool state);
bool system_state_get_overspeed_alarm(void);
GM_ERRCODE system_state_set_overspeed_alarm(bool state);
#endif //MY_APPLICATION_SYSTEMSTATE_H
