#include <Arduino.h>

#include "defines.h"
#include "debug.h"
#include "vars.h"

void lvgl_task(void *pvParameters) {

  vTaskSuspend(NULL);

  debug->print(DEBUG_LEVEL_INFO, "UI manager: Task running on core ");
  debug->println(DEBUG_LEVEL_INFO, xPortGetCoreID());

  while (1) {    
    lv_timer_handler();
    vTaskDelay(DELAY_LVGL_TASK / portTICK_PERIOD_MS);
  }
  // debug->println(DEBUG_LEVEL_INFO, "Terminating UI manager");
  // vTaskDelete(NULL);
}