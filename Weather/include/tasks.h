#ifndef TASKS_h
#define TASKS_h

#include <Arduino.h>
#include "defines.h"
#include "debug.h"
#include "vars.h"

// Tasks
TaskHandle_t t_core1_lvgl;
TaskHandle_t t_core1_clock;  
#ifdef USE_OPEN_WEATHER
TaskHandle_t t_core1_openWeather;
#endif

void lvgl_task(void *pvParameters);
void clock_task(void *pvParameters);
void ntp_task(void *pvParameters);
#ifdef USE_OPEN_WEATHER
void openWeather_task(void *pvParameters);
#endif

#endif