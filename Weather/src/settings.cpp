#include <Arduino.h>

#include "settings.h"
#include "defines.h"
#include "vars.h"

#include "myDebug.h"

#ifdef ENABLE_EEPROM
  #include "myEEPROM.h"    
#endif

Settings::Settings() {

  myDebug->println(DEBUG_LEVEL_DEBUG, "[Settings]");

  #ifdef ENABLE_EEPROM
    myEEPROM = new MyEEPROM(512);
    myEEPROM->start();
  #endif
}

void Settings::load() {
  myDebug->println(DEBUG_LEVEL_INFO, "Loading settings...");

  #ifdef ENABLE_EEPROM    
    if (myEEPROM->hasSignature()) {
        myDebug->println(DEBUG_LEVEL_INFO, "Signature OK");

        this->isDayLight = myEEPROM->readByte(EEPROM_DAYLIGHT);

    } else {
      myDebug->println(DEBUG_LEVEL_INFO, "No signature");
      myEEPROM->createSignature();
      setDefaults();
      save();
    }
  #else
    setDefaults();
  #endif
  myDebug->println(DEBUG_LEVEL_DEBUG, "Done");
}

void Settings::save() {
  
  myDebug->println(DEBUG_LEVEL_DEBUG, "Saving settings...");
  #ifdef ENABLE_EEPROM
    if (!myEEPROM->hasSignature()) {
      myDebug->println(DEBUG_LEVEL_DEBUG, "No signature");
      myEEPROM->createSignature();
    }
    myDebug->println(DEBUG_LEVEL_DEBUG, "Writing data to EEPROM");

    myEEPROM->writeByte(EEPROM_DAYLIGHT, this->isDayLight);

  #endif
  myDebug->println(DEBUG_LEVEL_DEBUG, "[OK] Saving settings");
}

void Settings::setDefaults() {
    
    myDebug->println(DEBUG_LEVEL_INFO, "Setting default values");

    this->isDayLight = 0;

    myDebug->println(DEBUG_LEVEL_DEBUG, "[OK] Setting default values");
}

void Settings::setDayLight(bool isOn) {
    if (isOn) {
      this->isDayLight = 1;
    } else {
      this->isDayLight = 0;
    }
}

bool Settings::IsDayLight() {
    return this->isDayLight == 1;
}

