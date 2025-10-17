#ifndef SETTINGS_h
#define SETTINGS_h

#include <Arduino.h>

#include "defines.h"

#ifdef ENABLE_EEPROM
  #include "myEEPROM.h"    
#endif

class Settings {
  private:

    #ifdef ENABLE_EEPROM
      MyEEPROM *myEEPROM;
    #endif

    byte isDayLight;
  

  public:
    Settings();

    void load();
    void save();
    void setDefaults();

    bool IsDayLight();
    void setDayLight(bool isOn);

};

#endif