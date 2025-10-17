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
      xSemaphoreTake(semaphoreData, portMAX_DELAY);
      needsUpdate = getData();
      if (needsUpdate) {        
        valuesNeedsUpdate = true;
        xSemaphoreGive(semaphoreData);
        debug->println(DEBUG_LEVEL_DEBUG, "Data updated ok");
      }
    } else {
      xSemaphoreGive(semaphoreData);
      vTaskDelay(30000 / portTICK_PERIOD_MS);
    }
  }
  debug->println(DEBUG_LEVEL_INFO, "Terminating NTP manager");
  vTaskDelete(NULL);
}
  