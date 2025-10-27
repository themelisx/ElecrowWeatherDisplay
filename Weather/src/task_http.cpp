#include <Arduino.h>
#include <WiFi.h>

#include "defines.h"
#include "debug.h"
#include "vars.h"


void http_task(void *pvParameters) {
  debug->print(DEBUG_LEVEL_INFO, "Http manager: Task running on core ");
  debug->println(DEBUG_LEVEL_INFO, xPortGetCoreID());

  bool needsUpdate = false;
  int errors = 0;

  lv_timer_handler();

  vTaskResume(t_core1_ntp);

  for (;;) {
    debug->println(DEBUG_LEVEL_DEBUG, "Updating data...");
    #if defined(LANG_EN)
      updateLastUpdate("Updating data...", COLOR_WHITE);    
    #elif defined(LANG_GR)
      updateLastUpdate("Ανανέωση δεδομένων", COLOR_WHITE);    
    #endif
    needsUpdate = getData();
    if (needsUpdate) {        
      xSemaphoreTake(semaphoreData, portMAX_DELAY);
      valuesNeedsUpdate = true;
      xSemaphoreGive(semaphoreData);
      debug->println(DEBUG_LEVEL_DEBUG, "Data updated ok");
      errors = 0;
      vTaskDelay(300000 / portTICK_PERIOD_MS);
    } else {
      debug->println(DEBUG_LEVEL_DEBUG, "Error updating data");
      #if defined(LANG_EN)
        updateLastUpdate("Error updating data", COLOR_RED);
      #elif defined(LANG_GR)
        updateLastUpdate("Σφάλμα ανανέωσης", COLOR_RED);
      #endif
      errors++;
      if (errors > 4) {
        esp_restart();
      }
      vTaskDelay(60000 / portTICK_PERIOD_MS);
    }
  }
}