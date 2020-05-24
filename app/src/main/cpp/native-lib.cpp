#include <jni.h>
#include <string>

#include <iostream>
#include <time.h>
#include <cstdlib>

#include "tyyslog.h"
#include "pthread.h"

//GM
#include "utility.h"
#include "systemstate.h"
#include "gps.h"
#include "g_sensor.h"
//GM

char detectversion[]="Harsh DriverBehavior Detection Module 0.2.000\n\n";

#include <boost/chrono.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/asio.hpp>
// when building boost we persisted the NDK version used (BOOST_BUILT_WITH_NDK_VERSION) in this custom header file
#include <boost/version_ndk.hpp>

clock_t cstart,cends;
void inittime()
{
    cstart=clock();
    cends=clock();
}

using std::string;
pthread_t threadId;
void * threadFunc(void * arg);
boost::asio::io_service io_service;
boost::posix_time::seconds interval(100);  // 100 second
boost::asio::deadline_timer timer(io_service, interval);

 boost::asio::deadline_timer sudden_turn_accelerationtimer(io_service);
 boost::asio::deadline_timer rapid_accelerationtimer(io_service);
 boost::asio::deadline_timer emergency_braketimer(io_service);
void report_alarm(int alarmid,double jindu,double weidu,double hanxian);//
//add JNIEnv store for later use
// cached refs for later callbacks
JavaVM * g_vm;
jobject g_obj;
jmethodID g_mid;

void report_alarm(int alarmid);
//
void tick(const boost::system::error_code& /*e*/) {

    ALOGD("timer fired\r\n");

    // Reschedule the timer for 1 second in the future:
    timer.expires_at(timer.expires_at() + interval);
    // Posts the timer event
    timer.async_wait(tick);
}


extern "C" JNIEXPORT jstring
JNICALL
Java_thiagosalvadori_driverbehavior_MainActivity_stringFromJNI(
        JNIEnv *env,
        jobject obj/* this */) {


    string Str = "Hello from C++";
    env->GetJavaVM(&g_vm);
    bool returnValue = true;
    // convert local to global reference
    // (local will die after this method call)
    g_obj = env->NewGlobalRef(obj);

    // save refs for callback
    jclass g_clazz = env->GetObjectClass(g_obj);
    if (g_clazz == NULL) {
       // std::cout << "Failed to find class" << std::endl;
        ALOGD("Failed to find class\r\n");
    }

    g_mid = env->GetMethodID(g_clazz, "alarm_report", "(IDDD)V");
    if (g_mid == NULL) {
      //  std::cout << "Unable to get method ref" << std::endl;
    }
    inittime();
    // now you can store jvm somewhere

    //-------------------------------------
    boost::chrono::system_clock::time_point p  = boost::chrono::system_clock::now();
    std::time_t t = boost::chrono::system_clock::to_time_t(p);

    char buffer[26];
    ctime_r(&t, buffer);

    //  std::string tst = std::to_string(3);

    int ver = BOOST_VERSION;
    int ver_maj = ver/100000;
    int ver_min = ver / 100 %1000;
    int ver_pat = ver %100;

    string Ver_Maj = boost::lexical_cast<string>(ver_maj);
    string Ver_Min = boost::lexical_cast<string>(ver_min);
    string Ver_Pat = boost::lexical_cast<string>(ver_pat);

    Str += "\n Boost version: " + Ver_Maj + "." + Ver_Min + "." + Ver_Pat + "\n";
    Str += "... built with NDK version: " + string(BOOST_BUILT_WITH_NDK_VERSION) + "\n";
    Str += "... says time is " + std::string(buffer) + "\n\n";
    //--------------------------------------------

    // Schedule the timer for the first time:
    timer.async_wait(tick);
    // Enter IO loop. The timer will fire for the first time 1 second from now:
    //io_service.run();
    pthread_create(&threadId, NULL, &threadFunc, NULL);

    return env->NewStringUTF(Str.c_str());;

}

void * threadFunc(void * arg)
{
    ALOGD("Thread Start\r\n");
// Sleep for 2 seconds
//check call back
   // report_alarm(1);
    io_service.run();
    ALOGD("Thread sleep 2\r\n");
    pthread_exit(&threadId);
}

extern "C" JNIEXPORT void
JNICALL Java_thiagosalvadori_driverbehavior_MainActivity_startharshdetection(  JNIEnv *env,
                                                                               jobject /* this */)
{
    //基础部分
    util_create();
    system_state_create();
    gps_create();
    g_sensor_create();
    system_state_set_work_state(GM_SYSTEM_STATE_WORK);
    ALOGD("startharshdetection\r\n");
    ALOGD("%s\n\n\n\n",detectversion);
    //
    report_alarm(1,11,12,13);//
}
extern "C" JNIEXPORT void JNICALL Java_thiagosalvadori_driverbehavior_MainActivity_setspeedtag( JNIEnv *env,
                                                                                                jobject ,
                                                                                                jint  speedtag )
{
    setspeedtag(speedtag);
}
extern "C" JNIEXPORT void JNICALL Java_thiagosalvadori_driverbehavior_MainActivity_readaccrdata(
        JNIEnv *env,
        jobject ,
        jdouble accrx,
        jdouble accry,
        jdouble accrz        )
{
    std::string hello = "Hello from C++";
    Aclr aclr = {0};
    aclr.x = accrx*  G_SENSOR_MAX_VALUE/G_SENSOR_RANGE;
    aclr.y = accry* G_SENSOR_MAX_VALUE/G_SENSOR_RANGE;
    aclr.z = accrz *G_SENSOR_MAX_VALUE/G_SENSOR_RANGE;
    //G_SENSOR_MAX_VALUE/G_SENSOR_RANGE
    if(GM_SYSTEM_STATE_WORK == system_state_get_work_state())
    {
        check_vehicle_state(aclr);
        //ALOGD("check_vehicle_state\r\n");
    }
    // return env->NewStringUTF(hello.c_str());
}

extern "C" JNIEXPORT void JNICALL
Java_thiagosalvadori_driverbehavior_MainActivity_ongpsdata(
        JNIEnv *env,
        jobject ,
        jdouble localatitude, jdouble locallongtitude,  jdouble localcourse,  jfloat localgpstime,  jint localgpsspeed
)
{
       ALOGD("on_received_gps\r\n");
    on_received_gps(  localatitude,  locallongtitude, localcourse, localgpstime, localgpsspeed);


}

extern "C" JNIEXPORT void JNICALL
Java_thiagosalvadori_driverbehavior_MainActivity_setthreshold(       JNIEnv *env,
                                                                     jobject ,jint thresd_index,jdouble  alarm_thr)
{
    ALOGD("setthreshold\r\n");
    int m_thresd_index=thresd_index;
    double m_alarm_thr=alarm_thr;
    set_sensor_thr(m_thresd_index,m_alarm_thr);

}
void report_comm(int alarmid,double jindu,double weidu,double hanxian)//
{
    ALOGD("report alarm \r\n");
    JNIEnv * g_env;
    // double check it's all ok
    int getEnvStat = g_vm->GetEnv((void **)&g_env, JNI_VERSION_1_6);
    if (getEnvStat == JNI_EDETACHED) {
        //std::cout << "GetEnv: not attached" << std::endl;
        if (g_vm->AttachCurrentThread((JNIEnv **) &g_env, NULL) != 0) {
            //  std::cout << "Failed to attach" << std::endl;
        }
    } else if (getEnvStat == JNI_OK) {
        //
    } else if (getEnvStat == JNI_EVERSION) {
        // std::cout << "GetEnv: version not supported" << std::endl;
    }
    ALOGD("call  alarmid \r\n");
    g_env->CallVoidMethod(g_obj, g_mid, alarmid,1.0,2.0,3.0);//

    if (g_env->ExceptionCheck()) {
        g_env->ExceptionDescribe();
    }

    // g_vm->DetachCurrentThread();
}


void report_alarm(int alarmid,double jindu,double weidu,double hanxian)//
{
    report_comm( alarmid, jindu, weidu, hanxian);//
}

