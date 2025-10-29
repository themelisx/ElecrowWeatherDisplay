#include <Arduino.h>

#include "UI/ui.h"
#include "defines.h"
#include "vars.h"

#include "debug.h"
#include "uiManager.h"

#if defined(LANG_GR)
const char *UIManager::days[7] = {"Κυρ", "Δευ", "Τρι", "Τετ", "Πεμ", "Παρ", "Σαβ"};
const char *UIManager::months[12] = {"Ιαν", "Φεβ", "Μαρ", "Απρ", "Μαι", "Ιουν",
                                     "Ιουλ", "Αυγ", "Σεπ", "Οκτ", "Νοε", "Δεκ"};
#elif defined(LANG_EN)
const char *UIManager::days[7] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
const char *UIManager::months[12] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun",
                                     "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
#endif

UIManager::UIManager() {

  debug->println(DEBUG_LEVEL_DEBUG, "[UIManager]");
  tmp_buf = (char*)malloc(128);

}

void UIManager::format_datetime(char *buf, size_t size, const struct tm *timeinfo) {
    char tmp[64];
    strftime(tmp, sizeof(tmp), "%a, %d %b %Y", timeinfo);

    int wday = timeinfo->tm_wday; // 0=Κυρ ... 6=Σαβ
    int mon  = timeinfo->tm_mon;  // 0=Ιαν ... 11=Δεκ

    // replace %a and %b with selected language
    snprintf(buf, size, "%s, %02d %s %d", days[wday], timeinfo->tm_mday, months[mon], 1900 + timeinfo->tm_year);
}

void UIManager::updateDateTime(const struct tm timeinfo) {
  // TODO: Add to settings "Date format"
  char date_str[50];
  format_datetime(date_str, sizeof(date_str), &timeinfo);
  lv_label_set_text(ui_ValueDate, date_str);

  // TODO: Add to settings "Hour format"
  strftime(tmp_buf, 50, "%H:%M", &timeinfo);      // 24h format
  //strftime(tmp_buf, 50, "%I:%M %p", &timeinfo); // 12h format
  lv_label_set_text(ui_ValueTime, tmp_buf);
}

void UIManager::clearDateTime() {
  #if defined(LANG_EN)
    uiManager->updateInfo("Clock sync...", COLOR_WHITE);
  #elif defined(LANG_GR)
    uiManager->updateInfo("Συγχρονισμός ώρας...", COLOR_WHITE);
  #endif
  lv_label_set_text(ui_ValueDate, "--- --/--/----");
  lv_label_set_text(ui_ValueTime, "--:--");
}

void UIManager::timestampToTime(time_t timestamp, char *buffer, size_t buffer_size) {
    struct tm *time_info;
    time_info = localtime(&timestamp);
    strftime(buffer, buffer_size, "%H:%M", time_info);
}

const char* UIManager::convertDegreesToDirection(int degrees) {
    // Normalize degrees to the range [0, 360)
    degrees = degrees % 360;

    if (degrees < 0) degrees += 360;

    #if defined(LANG_EN)
      if (degrees >= 337.5 || degrees < 22.5)  return "N";
      if (degrees >= 22.5 && degrees < 67.5)   return "NE";
      if (degrees >= 67.5 && degrees < 112.5)  return "E";
      if (degrees >= 112.5 && degrees < 157.5) return "SE";
      if (degrees >= 157.5 && degrees < 202.5) return "S";
      if (degrees >= 202.5 && degrees < 247.5) return "SW";
      if (degrees >= 247.5 && degrees < 292.5) return "W";
      if (degrees >= 292.5 && degrees < 337.5) return "NW";
    #elif defined(LANG_GR)
      if (degrees >= 337.5 || degrees < 22.5)  return "Β";
      if (degrees >= 22.5 && degrees < 67.5)   return "ΒΑ";
      if (degrees >= 67.5 && degrees < 112.5)  return "Α";
      if (degrees >= 112.5 && degrees < 157.5) return "ΝΑ";
      if (degrees >= 157.5 && degrees < 202.5) return "Ν";
      if (degrees >= 202.5 && degrees < 247.5) return "ΝΔ";
      if (degrees >= 247.5 && degrees < 292.5) return "Δ";
      if (degrees >= 292.5 && degrees < 337.5) return "ΒΔ";
    #else
        #error "No Language defined!"
    #endif
    

    return "Unknown"; // In case something unexpected happens
}

int UIManager::windSpeedToBeaufort(float speed) {
    if (speed < 0.5)
        return 0;
    else if (speed < 1.5)
        return 1;
    else if (speed < 3.3)
        return 2;
    else if (speed < 5.5)
        return 3;
    else if (speed < 7.9)
        return 4;
    else if (speed < 10.7)
        return 5;
    else if (speed < 13.8)
        return 6;
    else if (speed < 17.1)
        return 7;
    else if (speed < 20.7)
        return 8;
    else if (speed < 24.4)
        return 9;
    else if (speed < 28.4)
        return 10;
    else if (speed < 32.6)
        return 11;
    else
        return 12;
}

void UIManager::updateValues() {

  #if defined(USE_OPEN_WEATHER)
    if (openWeather->isDataUpdated()) {
      debug->println(DEBUG_LEVEL_DEBUG, "Updating openWeather UI values");
      s_openWeatherData data = openWeather->getData();
      openWeather->setDataUpdated(false);

      sprintf(tmp_buf, "%0.1f °C", data.temperature[0]);
      lv_label_set_text(ui_ValueTemperature, tmp_buf);

      sprintf(tmp_buf, "%0.1f °C", data.temperature[1]);
      lv_label_set_text(ui_ValueFeelsLike, tmp_buf);

      timestampToTime(data.sun[0], time_str, sizeof(time_str));
      lv_label_set_text(ui_ValueSunrise, time_str);

      timestampToTime(data.sun[1], time_str, sizeof(time_str));
      lv_label_set_text(ui_ValueSunset, time_str);

      sprintf(tmp_buf, "%d %%", data.humidity);
      lv_label_set_text(ui_ValueHumidity, tmp_buf);

      sprintf(tmp_buf, "%d hPa", data.pressure);
      lv_label_set_text(ui_ValuePressure, tmp_buf);

      #if defined(LANG_EN)
        sprintf(tmp_buf, "%0.1f m/s", data.windSpeed);
        lv_label_set_text(ui_ValueWindSpeed, tmp_buf);
        sprintf(tmp_buf, "Wind: %d Bf", windSpeedToBeaufort(data..windSpeed));
        lv_label_set_text(ui_Label2, tmp_buf);

        sprintf(tmp_buf, "Direction: %s", convertDegreesToDirection(data..windDirection));
        lv_label_set_text(ui_ValueWindDirection, tmp_buf);

        sprintf(tmp_buf, "Updated: %s", myClock->getTime());
        lv_label_set_text(ui_ValueLastUpdate, tmp_buf);
      #elif defined(LANG_GR)
        lv_label_set_text(ui_Label2, "Άνεμος");
        sprintf(tmp_buf, "%d Bf", windSpeedToBeaufort(data.windSpeed));
        lv_label_set_text(ui_ValueWindSpeed, tmp_buf);

        sprintf(tmp_buf, "Κατεύθυνση: %s", convertDegreesToDirection(data.windDirection));
        lv_label_set_text(ui_ValueWindDirection, tmp_buf);

        sprintf(tmp_buf, "Ενημερώθηκε: %s", myClock->getTime());
        updateInfo(tmp_buf, COLOR_WHITE);
      #endif  
    }
  #endif
}

void UIManager::updateInfo(const char *str, uint32_t color) {
  lv_label_set_text(ui_ValueLastUpdate, str);  
  lv_obj_set_style_text_color(ui_ValueLastUpdate, lv_color_hex(color), LV_PART_MAIN | LV_STATE_DEFAULT);
}
