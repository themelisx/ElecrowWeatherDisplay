#ifndef TASKS_h
#define TASKS_h

#include <Arduino.h>

#ifdef USE_MULTI_THREAD
void loop_task(void *pvParameters);
void clock_task(void *pvParameters);
void http_task(void *pvParameters);
void ntp_task(void *pvParameters);
#endif

#endif