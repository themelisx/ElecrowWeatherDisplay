#ifndef OPEN_WEATHER_h
#define OPEN_WEATHER_h

#include <Arduino.h>
#include "defines.h"

#ifdef USE_OPEN_WEATHER

    #include <HttpClient.h>
    #include <ArduinoJson.h>
    #include <PNGdec.h>
    #include "user_setup.h"

    // Weather Image
    #define IMG_WIDTH 200
    #define IMG_HEIGHT 200
    #define PNG_PIXELS_PER_LINE  2048

    class OpenWeather {
    private:
        // functions
        static int renderPNGToBuffer(PNGDRAW *pDraw); // static callback
        int renderPNGToBufferImpl(PNGDRAW *pDraw);    // normal method
        void downloadImageToMemory(const char *url);

        // User setup
        String town = USER_WeatherTown;
        String myAPI = USER_WeatherAPI;
        String units = USER_WeatherUnits; 

        // vars
        PNG png;
        uint8_t *imageBuffer = nullptr;
        uint16_t *rgb565_buffer = nullptr;
        int bufferSize;
        lv_img_dsc_t img_dsc;

        String openWeatherServer;
        s_openWeatherData data;
        String oldIconUrl;

        SemaphoreHandle_t semaphoreData;
        bool dataUpdated;
        String lastUpdate;
        HTTPClient http;
        char* tmp_buf;
    public:
        OpenWeather();
        // functions
        void init();
        bool fetchData();        
        bool isDataUpdated();
        void setDataUpdated(bool updated);
        s_openWeatherData getData();
        bool decodePngToRgb565(uint8_t *png_data, int png_size);
    };
    
#endif

#endif