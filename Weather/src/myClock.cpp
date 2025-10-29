#include <Arduino.h>

#include "defines.h"
#include "vars.h"
#include "debug.h"
#include "myClock.h"
#include "user_setup.h"

MyClock::MyClock() {
  debug->println(DEBUG_LEVEL_DEBUG, "[Clock]");
  
}

void MyClock::init() {
  debug->println(DEBUG_LEVEL_DEBUG, "Initializing Clock");  
  // time zone  
  zone = USER_TimeZone;
  // NTP
  ntpServer = "pool.ntp.org";  

  //tmp_buf = (char*)malloc(128);
  semaphoreData = xSemaphoreCreateMutex();
  xSemaphoreGive(semaphoreData);
}

bool MyClock::setTimeFromNTP() {
  debug->println(DEBUG_LEVEL_DEBUG, "Getting time from NTP...");

  xSemaphoreTake(semaphoreData, portMAX_DELAY);
  if (mySettings->IsDayLight()) {
    configTime(3600 * zone, 3600, ntpServer); // Daylight  
  } else {
    configTime(3600 * zone, 0, ntpServer); // Winter
  }
  
  if (getLocalTime(&timeinfo)) {
    rtc.setTimeStruct(timeinfo);
    xSemaphoreGive(semaphoreData);
    debug->println(DEBUG_LEVEL_DEBUG, "NTP Ok");
    return true;
  } else {
    xSemaphoreGive(semaphoreData);
    debug->println(DEBUG_LEVEL_ERROR, "NTP Failed");
    return false;
  }
}

struct tm MyClock::getTimeStruct() {
  xSemaphoreTake(semaphoreData, portMAX_DELAY);
  struct tm ret = rtc.getTimeStruct();
  xSemaphoreGive(semaphoreData);
  return ret;
}

String MyClock::getTime() {
  xSemaphoreTake(semaphoreData, portMAX_DELAY);
  String ret = rtc.getTime();
  xSemaphoreGive(semaphoreData);
  return ret;
}