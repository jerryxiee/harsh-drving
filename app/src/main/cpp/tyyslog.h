//
// Created by Administrator on 2019/11/12.
//

#ifndef DRIVERBEHAVIOR_MASTER_TYYSLOG_H
#define DRIVERBEHAVIOR_MASTER_TYYSLOG_H

#ifndef TYYS_DEBUG
#define TYYS_DEBUG 1
#endif

#if TYYS_DEBUG
#include <android/log.h>
#define LOG_TAG "TYYS_JNI"
#define ALOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)
#define ALOGV(...) __android_log_print(ANDROID_LOG_VERBOSE, LOG_TAG, __VA_ARGS__)
#define ALOGD(...) __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, __VA_ARGS__)
#else
#define ALOGE(...)
#define ALOGD(...)
#define ALOGV(...)
#endif

#include <jni.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#endif //DRIVERBEHAVIOR_MASTER_TYYSLOG_H
