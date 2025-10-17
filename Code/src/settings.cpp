#include <Arduino.h>

#include "settings.h"
#include "defines.h"
#include "vars.h"

#include "debug.h"

#ifdef ENABLE_EEPROM
  #include "myEEPROM.h"    
#endif

Settings::Settings() {

  debug->println(DEBUG_LEVEL_DEBUG, "[Settings]");

  #ifdef ENABLE_EEPROM
    myEEPROM = new MyEEPROM(512);
    myEEPROM->start();
    debug->println(DEBUG_LEVEL_DEBUG, "[OK]");
  #endif
}

void Settings::load() {
  debug->println(DEBUG_LEVEL_INFO, "Loading settings...");

  #ifdef ENABLE_EEPROM    
    if (myEEPROM->hasSignature()) {
        debug->println(DEBUG_LEVEL_INFO, "Signature OK");

        this->isDayLight = myEEPROM->readByte(EEPROM_DAYLIGHT);

    } else {
      debug->println(DEBUG_LEVEL_INFO, "No signature");
      myEEPROM->createSignature();
      setDefaults();
      save();
    }
  #else
    setDefaults();
  #endif
  debug->println(DEBUG_LEVEL_DEBUG, "Done");
}

void Settings::save() {
  
  debug->println(DEBUG_LEVEL_DEBUG, "Saving settings...");
  #ifdef ENABLE_EEPROM
    if (!myEEPROM->hasSignature()) {
      debug->println(DEBUG_LEVEL_DEBUG, "No signature");
      myEEPROM->createSignature();
    }
    debug->println(DEBUG_LEVEL_DEBUG, "Writing data to EEPROM");

    myEEPROM->writeByte(EEPROM_DAYLIGHT, this->isDayLight);

  #endif
  debug->println(DEBUG_LEVEL_DEBUG, "[OK] Saving settings");
}

void Settings::setDefaults() {
    
    debug->println(DEBUG_LEVEL_INFO, "Setting default values");

    this->isDayLight = 0;

    debug->println(DEBUG_LEVEL_DEBUG, "[OK] Setting default values");
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

