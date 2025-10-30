#include <Arduino.h>

#include "defines.h"

#ifdef USE_OPEN_WEATHER
#include <MyDebug.h>
#include "vars.h"

void openWeather_task(void *pvParameters) {
  
  vTaskSuspend(NULL);

  myDebug->print(DEBUG_LEVEL_INFO, "openWeather manager: Task running on core ");
  myDebug->println(DEBUG_LEVEL_INFO, xPortGetCoreID());

  int errors = 0;

  for (;;) {
    myWiFi->ensureConnection();
    if (myWiFi->isConnected()) {    
      if (openWeather->fetchData()) {
        vTaskDelay(DELAY_OPEN_WEATHER_TASK / portTICK_PERIOD_MS);
      } else {
        vTaskDelay(DELAY_OPEN_WEATHER_SHORT_TASK / portTICK_PERIOD_MS);
      }
    } else {
      myDebug->println(DEBUG_LEVEL_DEBUG, "No internet connection");
      vTaskDelay(DELAY_WIFI_RECONNECT_TASK / portTICK_PERIOD_MS);
    }
  }
}

#endif