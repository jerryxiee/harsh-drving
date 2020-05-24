//
// Created by Administrator on 2019/11/13.
//

#include "alarmhandle.h"
#include <boost/chrono.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/asio.hpp>
#include "g_sensor.h"
#include "tyyslog.h"
#include "jni.h"



#define ALARMINTERVAL 5
extern boost::asio::deadline_timer sudden_turn_accelerationtimer;
extern boost::asio::deadline_timer rapid_accelerationtimer;
extern boost::asio::deadline_timer emergency_braketimer;
void  report_emergency_brake(const boost::system::error_code& /*e*/);
void report_sudden_turn_acceleration(const boost::system::error_code& /*e*/);
void  report_rapid_acceleration(const boost::system::error_code& /*e*/);

void tick3(const boost::system::error_code& /*e*/) {

    ALOGD("GM_startsudden_turn_accelerationtimer fired\r\n");

}
void GM_startemergency_braketimer()
{
    emergency_braketimer.cancel();
    emergency_braketimer.expires_from_now(boost::posix_time::seconds(ALARMINTERVAL));
    ALOGD("emergency_brake\r\n");
    // Posts the timer event
    emergency_braketimer.async_wait(report_emergency_brake);
}


void GM_startsudden_turn_accelerationtimer()
{
    sudden_turn_accelerationtimer.cancel();
    sudden_turn_accelerationtimer.expires_from_now(boost::posix_time::seconds(ALARMINTERVAL));
    ALOGD("sudden_turn_acceleration\r\n");
    // Posts the timer event
    sudden_turn_accelerationtimer.async_wait(report_sudden_turn_acceleration);
}

void GM_startrapid_accelerationtimer()
{
    rapid_accelerationtimer.cancel();
    rapid_accelerationtimer.expires_from_now(boost::posix_time::seconds(ALARMINTERVAL));
    ALOGD("rapid_acceleration\r\n");
    // Posts the timer event
    rapid_accelerationtimer.async_wait(report_rapid_acceleration);
}

void report_rapid_acceleration(const boost::system::error_code& /*e*/)
{
    ALOGD("rapid_acceleration timer fired\r\n");
    report_rapid_aclr_alarm();
}
void report_sudden_turn_acceleration(const boost::system::error_code& /*e*/)
{
    ALOGD("sudden_turn_acceleration timer fired\r\n");
    report_sudden_turn_alarm();
}
void report_emergency_brake(const boost::system::error_code& /*e*/)
{
    ALOGD("emergency_brake timer fired\r\n");
    report_emergency_brake_alarm();
}




