#include <Arduino.h>
#include <WiFi.h>

#include "defines.h"
#include "debug.h"
#include "vars.h"


void http_task(void *pvParameters) {
  debug->print(DEBUG_LEVEL_INFO, "Http manager: Task running on core ");
  debug->println(DEBUG_LEVEL_INFO, xPortGetCoreID());

  //long weatherLastUpdate = 0;
  //long now = millis();
  bool needsUpdate = false;
  int errors = 0;

  // WiFi.mode(WIFI_STA);
  
  // updateLastUpdate("Connecting to WiFi ...", COLOR_WHITE);
  lv_timer_handler();

  // WiFi.begin("JefNet2", "J3fN3tH0m3");
  // while (WiFi.status() != WL_CONNECTED) {
  //   delay(1000);
  // }
  // updateLastUpdate("WiFi connected", COLOR_WHITE);

  vTaskResume(t_core1_ntp);

  for (;;) {

    //if (now > (weatherLastUpdate + (1000 * 60 * 5))) {
      //weatherLastUpdate = now;
      debug->println(DEBUG_LEVEL_DEBUG, "Updating data...");
      updateLastUpdate("Updating data...", COLOR_WHITE);
      xSemaphoreTake(semaphoreData, portMAX_DELAY);
      needsUpdate = getData();
      if (needsUpdate) {        
        valuesNeedsUpdate = true;
        xSemaphoreGive(semaphoreData);
        debug->println(DEBUG_LEVEL_DEBUG, "Data updated ok");
        errors = 0;
        vTaskDelay(300000 / portTICK_PERIOD_MS);
      } else {
        xSemaphoreGive(semaphoreData);
        debug->println(DEBUG_LEVEL_DEBUG, "Error updating data");
        updateLastUpdate("Error updating data", COLOR_RED);
        //weatherLastUpdate = now - (1000 * 60);
        errors++;
        if (errors > 4) {
          esp_restart();
        }
        vTaskDelay(60000 / portTICK_PERIOD_MS);
      }
    //}
  }
}
  