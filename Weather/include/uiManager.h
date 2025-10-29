#ifndef UI_MANAGER_h
#define UI_MANAGER_h

#include <Arduino.h>
#include "defines.h"

class UIManager {
  private:
    // functions 
    void format_datetime(char *buf, size_t size, const struct tm *timeinfo);
    void timestampToTime(time_t timestamp, char *buffer, size_t buffer_size);
    const char* convertDegreesToDirection(int degrees);
    int windSpeedToBeaufort(float speed);

    // Calendar days and months  
    #if defined(LANG_GR)
      static const char *days[7];
      static const char *months[12];
    #elif defined(LANG_EN)
      static const char *days[7];
      static const char *months[12];
    #endif 

    // vars
    char time_str[9];
    String lastUpdate = "";
    char* tmp_buf;

  public:
    UIManager();

    void updateDateTime(const struct tm timeinfo);
    void updateInfo(const char *str, uint32_t color);
    void clearDateTime();
    void updateValues();

};

#endif