#ifndef VARS_h
#define VARS_h

#include <Arduino.h>

#include <MyDebug.h>
#include "structs.h"
#include "settings.h"
#include "openWeather.h"
#include "uiManager.h"
#include "myClock.h"
#include <MyWiFi.h>

extern void createTasks();

extern MyDebug *myDebug;
extern Settings *mySettings;
extern UIManager *uiManager;
extern MyClock *myClock;
extern MyWiFi *myWiFi;
#ifdef USE_OPEN_WEATHER
extern OpenWeather *openWeather;
extern TaskHandle_t t_core1_openWeather;
#endif

#endif