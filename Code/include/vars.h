#ifndef VARS_h
#define VARS_h

#include <Arduino.h>

#include "debug.h"
#include "structs.h"
#include "settings.h"

#ifdef USE_MULTI_THREAD
    extern SemaphoreHandle_t semaphoreData;

    extern TaskHandle_t t_core1_http;
    extern TaskHandle_t t_core1_ntp;
    extern TaskHandle_t t_core1_loop;

    extern bool setTime();
    extern bool getData();
    extern void updateLastUpdate(const char *str, uint32_t color);
    extern bool valuesNeedsUpdate;
    extern bool ntpIsOk;

#endif

extern Debug *debug;
extern Settings *mySettings;

#endif