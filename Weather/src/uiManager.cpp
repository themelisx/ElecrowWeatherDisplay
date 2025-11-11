#include <Arduino.h>

#include "UI/ui.h"
#include "defines.h"
#include "vars.h"

#include <MyDebug.h>
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

  myDebug->println(DEBUG_LEVEL_DEBUG, "[UIManager]");
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

  // TODO: Add to settings "dim at night"
  // TODO: Add to settings "dim hours"
  // TODO: Add to settings "dim percentage"
  if (timeinfo.tm_hour > 21 || timeinfo.tm_hour < 7) {
    setNightMode(true);
  } else {
    setNightMode(false);
  }
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
    // Normalize degrees to [0, 360)
    degrees = degrees % 360;
    if (degrees < 0) degrees += 360;

#if defined(LANG_EN)
    static constexpr const char* dirs[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
#elif defined(LANG_GR)
    static constexpr const char* dirs[] = {"Β", "ΒΑ", "Α", "ΝΑ", "Ν", "ΝΔ", "Δ", "ΒΔ"};
#else
    #error "No Language defined!"
#endif

    // Each direction covers 45°, starting at N = 0°
    int index = static_cast<int>((degrees + 22.5) / 45.0) % 8;
    return dirs[index];
}


int UIManager::windSpeedToBeaufort(float speed) {
    static const float limits[] = {
        0.5, 1.5, 3.3, 5.5, 7.9, 10.7,
        13.8, 17.1, 20.7, 24.4, 28.4, 32.6
    };

    for (int i = 0; i < 12; ++i)
        if (speed < limits[i])
            return i;
    return 12;
}

void UIManager::updateValues() {

  #if defined(USE_OPEN_WEATHER)
    if (openWeather->isDataUpdated()) {
      myDebug->println(DEBUG_LEVEL_DEBUG, "Updating openWeather UI values");
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
