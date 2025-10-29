#ifndef structs_h
#define structs_h

#include <Arduino.h>
#include "../lvgl/lvgl.h"

#ifdef USE_OPEN_WEATHER
typedef struct s_openWeatherData {
    float temperature[4];
    int humidity;
    int pressure;
    float windSpeed;
    int windDirection;
    long sun[2];
    int visibility;
} s_openWeatherData;
#endif

typedef struct s_widget {
    int id;
    String title;
    String description;
    String strFormat;
    float value;
} s_widget;

#endif