#include <Arduino.h>

#include "defines.h"
#include <MyDebug.h>
#include "vars.h"

extern void ensureWiFiConnected(bool forceCheck);

void clock_task(void *pvParameters) {

  vTaskSuspend(NULL);

  myDebug->println(DEBUG_LEVEL_INFO, "Clock manager: Task running on core %d", xPortGetCoreID());

  bool ntpResult = false;
  
  uiManager->clearDateTime();

  while (!ntpResult) {

    myWiFi->ensureConnection();
    if (myWiFi->isConnected()) {
      ntpResult = myClock->setTimeFromNTP();
    
      if (!ntpResult) {
        vTaskDelay(DELAY_NTP_TASK / portTICK_PERIOD_MS);
      } else {
        uiManager->updateInfo("", COLOR_WHITE);
      }      
    } else {
      myDebug->println(DEBUG_LEVEL_DEBUG, "No internet connection");
      vTaskDelay(DELAY_WIFI_RECONNECT_TASK / portTICK_PERIOD_MS);
    }
    
  }

  #ifdef USE_OPEN_WEATHER
    vTaskResume(t_core1_openWeather);
  #endif

  while (1) {
    uiManager->updateDateTime(
      myClock->getTimeStruct()
    );
    uiManager->updateValues();
    vTaskDelay(DELAY_CLOCK_TASK / portTICK_PERIOD_MS);
  }
}