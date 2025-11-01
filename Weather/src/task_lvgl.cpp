#include <Arduino.h>

#include "defines.h"
#include <MyDebug.h>
#include "vars.h"

void lvgl_task(void *pvParameters) {

  vTaskSuspend(NULL);

  myDebug->println(DEBUG_LEVEL_INFO, "UI manager: Task running on core %d", xPortGetCoreID());

  while (1) {    
    lv_timer_handler();
    vTaskDelay(DELAY_LVGL_TASK / portTICK_PERIOD_MS);
  }
  // myDebug->println(DEBUG_LEVEL_INFO, "Terminating UI manager");
  // vTaskDelete(NULL);
}