#include <Arduino.h>
#include <WiFi.h>

#include "defines.h"
#include "debug.h"
#include "vars.h"


void ntp_task(void *pvParameters) {
  debug->print(DEBUG_LEVEL_INFO, "NTP manager: Task running on core ");
  debug->println(DEBUG_LEVEL_INFO, xPortGetCoreID());

  bool ntpResult = false;
  bool needsUpdate = false;

  while (!ntpIsOk) {

    ntpResult = setTime();
    
    if (ntpResult) {
      xSemaphoreTake(semaphoreData, portMAX_DELAY);
      ntpIsOk = ntpResult;
      xSemaphoreGive(semaphoreData);
      
      updateLastUpdate("Updating data...", COLOR_WHITE);      
      needsUpdate = getData();
      if (needsUpdate) {        
        debug->println(DEBUG_LEVEL_DEBUG, "Data updated ok");
        xSemaphoreTake(semaphoreData, portMAX_DELAY);
        valuesNeedsUpdate = true;
        xSemaphoreGive(semaphoreData);        
      }
    } else {
      vTaskDelay(30000 / portTICK_PERIOD_MS);
    }
  }
  debug->println(DEBUG_LEVEL_INFO, "Terminating NTP manager");
  vTaskDelete(NULL);
}
  