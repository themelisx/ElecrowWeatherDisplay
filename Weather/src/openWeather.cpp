#include <Arduino.h>
#include <HttpClient.h>
#include <ArduinoJson.h> 
#include <PNGdec.h>

#include "defines.h"
#include "vars.h"
#include "UI/ui.h"

#include <MyDebug.h>
#include "user_setup.h"
#include "myClock.h"
#include "openWeather.h"

#ifdef USE_OPEN_WEATHER

OpenWeather::OpenWeather() {
  myDebug->println(DEBUG_LEVEL_DEBUG, "[OpenWeather]");  
}

void OpenWeather::init() {
  myDebug->println(DEBUG_LEVEL_DEBUG, "Initializing OpenWeather");  
  dataUpdated = false;
  bufferSize = IMG_WIDTH * IMG_HEIGHT * 2;
  openWeatherServer = "https://api.openweathermap.org/data/2.5/weather?q=" + town + "&appid=" + myAPI + "&units=" + units;
  oldIconUrl = "";
  tmp_buf = (char*)malloc(128);  
  // Allocate memory for the buffer
  imageBuffer = (uint8_t *)malloc(bufferSize + 1000);
  // Allocate RGB565 buffer
  rgb565_buffer = (uint16_t *)malloc(bufferSize + 1000);

  semaphoreData = xSemaphoreCreateMutex();
  xSemaphoreGive(semaphoreData);    
}

bool OpenWeather::isDataUpdated() {
  bool ret;
  xSemaphoreTake(semaphoreData, portMAX_DELAY);
  ret = dataUpdated;
  xSemaphoreGive(semaphoreData);
  return ret;
}

void OpenWeather::setDataUpdated(bool updated) {
  xSemaphoreTake(semaphoreData, portMAX_DELAY);
  dataUpdated = updated;
  xSemaphoreGive(semaphoreData);
}

s_openWeatherData OpenWeather::getData() {
  s_openWeatherData ret;
  xSemaphoreTake(semaphoreData, portMAX_DELAY);
  ret = data;
  xSemaphoreGive(semaphoreData);
  return ret;
}

bool OpenWeather::fetchData() {
  myDebug->println(DEBUG_LEVEL_DEBUG, "Updating OpenWeather data...");

  #if defined(LANG_EN)
    uiManager->updateInfo("Updating data...", COLOR_WHITE);    
  #elif defined(LANG_GR)
    uiManager->updateInfo("Ανανέωση δεδομένων", COLOR_WHITE);    
  #endif

  http.begin(openWeatherServer);
  int httpResponseCode = http.GET();

  if (httpResponseCode > 0) {
      String payload = http.getString();
      myDebug->println(DEBUG_LEVEL_DEBUG2, "server response");
      myDebug->println(DEBUG_LEVEL_DEBUG2, payload);

      JsonDocument doc;
      DeserializationError error = deserializeJson(doc, payload);

      if (!error) {
          xSemaphoreTake(semaphoreData, portMAX_DELAY);
          data.temperature[0] = doc["main"]["temp"];
          data.temperature[1] = doc["main"]["feels_like"];
          data.temperature[2] = doc["main"]["temp_min"];
          data.temperature[3] = doc["main"]["temp_max"];
          data.humidity = doc["main"]["humidity"];
          data.pressure = doc["main"]["pressure"];
          data.windSpeed = doc["wind"]["speed"];
          data.windDirection = doc["wind"]["deg"];
          data.sun[0] = doc["sys"]["sunrise"];
          data.sun[1] = doc["sys"]["sunset"];
          //data.visibility = doc["visibility"];
          //data.description = doc["weather"][0]["description"].as<String>();
          dataUpdated = true;
          lastUpdate = myClock->getTime();
          xSemaphoreGive(semaphoreData);

          String iconUrl =
              "https://openweathermap.org/img/wn/" +
              doc["weather"][0]["icon"].as<String>() +
              "@4x.png";
          iconUrl.replace("https://", "http://");

          http.end();

          if (imageBuffer != nullptr) {
              myDebug->println(DEBUG_LEVEL_DEBUG, iconUrl);
              if (!iconUrl.equals(oldIconUrl)) {
                  myDebug->println(DEBUG_LEVEL_DEBUG, "icon is different");
                  oldIconUrl = iconUrl;
                  downloadImageToMemory(iconUrl.c_str());
              }
          }
          return true;
      } else {
          http.end();
          myDebug->println(DEBUG_LEVEL_ERROR, "ERROR JSON: ");
          myDebug->println(DEBUG_LEVEL_ERROR, error.c_str());
          return false;
      }
  } else {
      http.end();
      myDebug->println(DEBUG_LEVEL_ERROR, "HTTP ERROR %d", httpResponseCode);
      return false;
  }
}

int OpenWeather::renderPNGToBuffer(PNGDRAW *pDraw) {
    // take "this" from pUser
    OpenWeather *self = (OpenWeather *)pDraw->pUser;
    return self->renderPNGToBufferImpl(pDraw);
}

int OpenWeather::renderPNGToBufferImpl(PNGDRAW *pDraw) {
    uint16_t lineBuffer[IMG_WIDTH];
    png.getLineAsRGB565(pDraw, lineBuffer, PNG_RGB565_LITTLE_ENDIAN, 0x0000);

    for (int x = 0; x < pDraw->iWidth; x++) {
        int dst_index = (pDraw->y * IMG_WIDTH + x);
        rgb565_buffer[dst_index] = lineBuffer[x];
    }

    return 1;
}

bool OpenWeather::decodePngToRgb565(uint8_t *png_data, int png_size) {
    int rc = png.openRAM(png_data, png_size, renderPNGToBuffer);
    if (rc == PNG_SUCCESS) {
        rc = png.decode(this, 0); // <<--- pass "this" as pUser
        png.close();
        return true;
    } else {
        myDebug->println(DEBUG_LEVEL_ERROR, "PNG decode failed, code = %d", rc);
        return false;
    }
}

void OpenWeather::downloadImageToMemory(const char *url) {
  myDebug->println(DEBUG_LEVEL_DEBUG, "image url: %s", url);

  WiFiClient client;
  HTTPClient http;
  if (http.begin(client, url)) {
    int httpCode = http.GET();

    if (httpCode == HTTP_CODE_OK) {
      int totalSize = http.getSize();
      if (totalSize > bufferSize) {
        myDebug->println(DEBUG_LEVEL_ERROR, "Image size exceeds buffer size");
      } else {
        WiFiClient *stream = http.getStreamPtr();
        int bytesRead = 0;

        while (http.connected() && (bytesRead < totalSize || totalSize == -1)) {
            int len = stream->available();
            if (len > 0) {
                int toRead = min(len, totalSize - bytesRead);
                int readNow = stream->readBytes(imageBuffer + bytesRead, toRead);
                bytesRead += readNow;
            }
            delay(1);
        }

        myDebug->println(DEBUG_LEVEL_DEBUG, "%d bytes downloaded", bytesRead);

        if (bytesRead != -1) {
              myDebug->println(DEBUG_LEVEL_DEBUG, "updating icon...");
              if (decodePngToRgb565(imageBuffer, bytesRead)) {
                  int w = png.getWidth();
                  int h = png.getHeight();
                  img_dsc.header.always_zero = 0;
                  img_dsc.header.w = w;
                  img_dsc.header.h = h;
                  img_dsc.data_size = bytesRead;
                  img_dsc.header.cf = LV_IMG_CF_TRUE_COLOR;
                  img_dsc.data = (const uint8_t *)rgb565_buffer;

                  lv_img_set_src(ui_Image1, &img_dsc);
              }
            }
      }
    } else {
      myDebug->println(DEBUG_LEVEL_ERROR, "Failed to download image");
      myDebug->println(DEBUG_LEVEL_ERROR, http.errorToString(httpCode).c_str());
    }

    http.end();
  }
}
#endif