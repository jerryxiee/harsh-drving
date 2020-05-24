//
// Created by Administrator on 2019/11/9 0009.
//
#include <jni.h>
#include "systemstate.h"
#include "gm_type.h"

#include "error_code.h"
#include <string.h>
#include "utility.h"

#define SYSTEM_STATE_MAGIC_NUMBER 0xFEFEFEFE

GM_ERRCODE system_state_create(void)
{
    GM_ERRCODE ret = GM_SUCCESS;
    init_para();
    return ret;
}

static void init_para(void)
{
    char *addr = NULL;
    s_system_state.magic = SYSTEM_STATE_MAGIC_NUMBER;
    s_system_state.status_bits = 0;

    s_system_state.work_state = GM_SYSTEM_STATE_WORK;
    s_system_state.vehicle_state = VEHICLE_STATE_RUN;
    s_system_state.start_time = 0;

    s_system_state.mileage = 0;
    s_system_state.last_good_time = 0;

    memset(&s_system_state.latest_gps, 0, sizeof(s_system_state.latest_gps));

}

GM_ERRCODE system_state_timer_proc(void)
{
    s_system_state.start_time++;
    return GM_SUCCESS;
}

SystemWorkState system_state_get_work_state(void)
{
    return s_system_state.work_state;
}

GM_ERRCODE system_state_set_work_state(const SystemWorkState work_state)
{
    if(s_system_state.work_state != work_state)
    {
        s_system_state.work_state = work_state;

        //休眠或者唤醒，全部传感器报警状态清除，电压报警和其它状态不变
        s_system_state.status_bits &=0x0007ffff;
       // LOG(INFO,"system_state_set_work_state:%d",work_state);
        //状态发生变化，触发一次心跳
       // gps_service_heart_atonce();
        //return save_state_to_file();
    }
    return GM_SUCCESS;
}


bool system_state_get_collision_alarm(void)
{
    return GET_BIT23(s_system_state.status_bits);
}

GM_ERRCODE system_state_set_collision_alarm(bool state)
{
    if (state == system_state_get_collision_alarm())
    {
        return GM_SUCCESS;
    }



    if (state)
    {
        SET_BIT23(s_system_state.status_bits);
    }
    else
    {
        CLR_BIT23(s_system_state.status_bits);
    }
    //LOG(INFO,"system_state_set_collision_alarm");
    return GM_SUCCESS;
}

bool system_state_get_speed_down_alarm(void)
{
    return GET_BIT25(s_system_state.status_bits);
}

GM_ERRCODE system_state_set_speed_down_alarm(bool state)
{
    if (state == system_state_get_speed_down_alarm())
    {
        return GM_SUCCESS;
    }

    if (state)
    {
        SET_BIT25(s_system_state.status_bits);
    }
    else
    {
        CLR_BIT25(s_system_state.status_bits);
    }
    return GM_SUCCESS;
   // LOG(INFO,"system_state_set_speed_down_alarm");
}

bool system_state_get_speed_up_alarm(void)
{
    return GET_BIT24(s_system_state.status_bits);
}

bool system_state_set_speed_up_alarm(bool state)
{
    if (state == system_state_get_speed_up_alarm())
    {
        return GM_SUCCESS;
    }

    if (state)
    {
        SET_BIT24(s_system_state.status_bits);
    }
    else
    {
        CLR_BIT24(s_system_state.status_bits);
    }
    return GM_SUCCESS;
   // LOG(INFO,"system_state_set_speed_up_alarm");

}

bool system_state_get_sharp_turn_alarm(void)
{
    return GET_BIT27(s_system_state.status_bits);
}

GM_ERRCODE system_state_set_sharp_turn_alarm(bool state)
{
    if (state == system_state_get_sharp_turn_alarm())
    {
        return GM_SUCCESS;
    }


    if (state)
    {
        SET_BIT27(s_system_state.status_bits);
    }
    else
    {
        CLR_BIT27(s_system_state.status_bits);
    }
    return GM_SUCCESS;
   // LOG(INFO,"system_state_set_sharp_turn_alarm");
  //  return save_state_to_file();
}

GM_ERRCODE system_state_set_vehicle_state(const VehicleState vehicle_state)
{
    if(s_system_state.vehicle_state != vehicle_state)
    {
        //LOG(INFO, "vehicle_state from %d to %d", s_system_state.vehicle_state,vehicle_state);
        s_system_state.vehicle_state = vehicle_state;

    }//
    return GM_SUCCESS;
}

VehicleState system_state_get_vehicle_state(void)
{
    return s_system_state.vehicle_state;
}

bool system_state_get_turn_over_alarm(void)
{
    return GET_BIT26(s_system_state.status_bits);
}

GM_ERRCODE system_state_set_turn_over_alarm(bool state)
{
    if (state == system_state_get_turn_over_alarm())
    {
        return GM_SUCCESS;
    }

    if (state)
    {
        SET_BIT26(s_system_state.status_bits);
    }
    else
    {
        CLR_BIT26(s_system_state.status_bits);
    }
    return GM_SUCCESS;
}

bool system_state_get_remove_alarm(void)
{
    return GET_BIT28(s_system_state.status_bits);
}

GM_ERRCODE system_state_set_remove_alarm(bool state)
{
    if (state == system_state_get_remove_alarm())
    {
        return GM_SUCCESS;
    }

    if (state)
    {
        SET_BIT28(s_system_state.status_bits);
    }
    else
    {
        CLR_BIT28(s_system_state.status_bits);
    }
    return GM_SUCCESS;
}

bool system_state_get_move_alarm(void)
{
    return GET_BIT29(s_system_state.status_bits);
}

GM_ERRCODE system_state_set_move_alarm(bool state)
{
    if (state == system_state_get_move_alarm())
    {
        return GM_SUCCESS;
    }

    if (state)
    {
        SET_BIT29(s_system_state.status_bits);
    }
    else
    {
        CLR_BIT29(s_system_state.status_bits);
    }
    return GM_SUCCESS;
}

bool system_state_get_overspeed_alarm(void)
{
    return GET_BIT21(s_system_state.status_bits);
}

GM_ERRCODE system_state_set_overspeed_alarm(bool state)
{
    if (state == system_state_get_overspeed_alarm())
    {
        return GM_SUCCESS;
    }

    if (state)
    {
        SET_BIT21(s_system_state.status_bits);
    }
    else
    {
        CLR_BIT21(s_system_state.status_bits);
    }
    return GM_SUCCESS;
}