#ifndef WIFI_h
#define WIFI_h

#include <Arduino.h>
#include "defines.h"

class MyWiFi {
private:
    // functions
    void notInitialized();

    //Vars
    uint8_t WiFiChannel;
    bool initDone;

    SemaphoreHandle_t semaphoreData;

    char *tmp_buf;
    char *savedSSID;
    char *savedPassword;
public:
    MyWiFi();
    // functions
    void init(wifi_mode_t mode, char *ssid, char *password);
    void stop();
    void connect();
    void disconnect();
    void ensureConnection();
    bool isConnected();
    wl_status_t getStatus();
    uint8_t getChannel();
};
  
#endif