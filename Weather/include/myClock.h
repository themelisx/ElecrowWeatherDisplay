#ifndef CLOCK_h
#define CLOCK_h

#include <Arduino.h>
#include <ESP32Time.h>
#include "defines.h"

class MyClock {
private:
    // functions    

    //Vars
    int zone;
    char* ntpServer;
    struct tm timeinfo;

    ESP32Time rtc;
    SemaphoreHandle_t semaphoreData;

    char* tmp_buf;
public:
    MyClock();
    // functions
    void init();
    bool setTimeFromNTP();
    struct tm getTimeStruct();
    String getTime();
};
  
#endif