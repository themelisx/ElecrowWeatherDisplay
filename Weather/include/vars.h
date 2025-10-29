#ifndef VARS_h
#define VARS_h

#include <Arduino.h>

#include "debug.h"
#include "structs.h"
#include "settings.h"
#include "openWeather.h"
#include "uiManager.h"
#include "myClock.h"
#include "myWiFi.h"

extern void createTasks();

extern Debug *debug;
extern Settings *mySettings;
extern UIManager *uiManager;
extern MyClock *myClock;
extern MyWiFi *myWiFi;
#ifdef USE_OPEN_WEATHER
extern OpenWeather *openWeather;
extern TaskHandle_t t_core1_openWeather;
#endif

#endif