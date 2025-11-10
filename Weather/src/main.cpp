#include <ArduinoJson.h> 

#include <lvgl.h>
#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_SSD1306.h> 
#include <Adafruit_GFX.h>
#include <PNGdec.h>

#include <LovyanGFX.hpp>
#include <lgfx/v1/platforms/esp32s3/Panel_RGB.hpp>
#include <lgfx/v1/platforms/esp32s3/Bus_RGB.hpp>

#include "UI/ui.h"

#include "../lvgl/lvgl.h"

#include "../include/defines.h"
#include "MyDebug.h"
#include "../include/structs.h"
#include "../include/openWeather.h"
#include "../include/uiManager.h"
#include "../include/myClock.h"
#include "MyWiFi.h"
#include "mySettings.h"
#include "../include/externals.h"

////////////////
// User setup // 
////////////////
#include "../include/user_setup.h"
char WiFiSSID[] = USER_WiFiSSID;
char WiFiPassword[] = USER_WiFiPassword;
/////////////////////////////////



// Common vars
bool isDayLight;

// TFT
TwoWire I2Cone = TwoWire(0);
Adafruit_SSD1306 display = Adafruit_SSD1306(128, 64, &I2Cone,OLED_RESET);

SPIClass& spi = SPI;
uint16_t touchCalibration_x0 = 300, touchCalibration_x1 = 3600, touchCalibration_y0 = 300, touchCalibration_y1 = 3600;
uint8_t  touchCalibration_rotate = 1, touchCalibration_invert_x = 2, touchCalibration_invert_y = 0;
static int val = 100;

static uint32_t screenWidth;
static uint32_t screenHeight;
static lv_color_t disp_draw_buf[800 * 480 / 10];

MyDebug *myDebug;
MyClock *myClock;
MySettings *mySettings;
UIManager *uiManager;
MyWiFi *myWiFi;
#ifdef USE_OPEN_WEATHER
OpenWeather *openWeather;
#endif

#include "../include/lgfx.h"
LGFX lcd;
#include "touch.h"

void my_disp_flush(lv_display_t * disp, const lv_area_t * area, uint8_t * px_map)
{
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

  // px_map είναι raw pixel map (here we assume LV_COLOR_DEPTH == 16 => RGB565)
  // Cast to the type LGFX expects for pushImageDMA:
  lcd.pushImageDMA(area->x1, area->y1, w, h, (lgfx::rgb565_t*)px_map);

  /* Tell LVGL we're done */
  lv_display_flush_ready(disp);
}


/* Input read callback — LVGL v9 signature */
void my_touchpad_read(lv_indev_t * indev, lv_indev_data_t * data)
{
  if (touch_has_signal())
  {
    if (touch_touched())
    {
      data->state = LV_INDEV_STATE_PR;
      data->point.x = touch_last_x;
      data->point.y = touch_last_y;
      #ifndef MODE_RELEASE
        myDebug->println(DEBUG_LEVEL_DEBUG2, "Data x: %d", touch_last_x);
        myDebug->println(DEBUG_LEVEL_DEBUG2, "Data y: %d", touch_last_y );
      #endif
    }
    else if (touch_released())
    {
      data->state = LV_INDEV_STATE_REL;
    }
  }
  else
  {
    data->state = LV_INDEV_STATE_REL;
  }
  delay(15);
}

void initializeUI() {  

  myDebug->println(DEBUG_LEVEL_INFO, "initialize UI...");  
  ui_init();

  uiManager = new UIManager();
  
}

void configureDisplay() {
  myDebug->println(DEBUG_LEVEL_INFO, "Configuring display...");

  screenWidth = lcd.width();
  screenHeight = lcd.height();

  lv_display_t * disp = lv_display_create(screenWidth, screenHeight);

  static lv_color_t *disp_draw_buf = (lv_color_t *)malloc(screenWidth * 10 * sizeof(lv_color_t));

  lv_display_set_buffers(disp,
                        disp_draw_buf,
                        NULL,
                        screenWidth * 10 * sizeof(lv_color_t),
                        LV_DISPLAY_RENDER_MODE_PARTIAL);

  lv_display_set_flush_cb(disp, my_disp_flush);

  lv_indev_t * indev = lv_indev_create();
  lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(indev, my_touchpad_read);

#ifdef TFT_BL
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
#endif

  lcd.fillScreen(0x000000u);
}

void initializeDisplay() {
  myDebug->println(DEBUG_LEVEL_INFO, "Initializing display...");
  lcd.begin();
  lcd.fillScreen(0x000000u);
  lcd.setTextSize(2); 
  //lcd.setBrightness(127);
}

void initializeSettings() {
  mySettings = new MySettings();
  mySettings->start();
}

void initializeDebug() {
  // Initialize Serial and set debug level
  myDebug = new MyDebug();
  #if defined(MODE_DEBUG_FULL)
    myDebug->start(115200, DEBUG_LEVEL_DEBUG2);
  #elif defined(MODE_DEBUG)
    myDebug->start(115200, DEBUG_LEVEL_DEBUG);
  #elif defined(MODE_RELEASE_INFO)
    myDebug->start(115200, DEBUG_LEVEL_INFO);
  #elif defined(MODE_RELEASE)
    myDebug->start(115200, DEBUG_LEVEL_NONE);
  #else
    #error "Select build mode for logs"
  #endif  
}

void initializeTouchScreen() {
  myDebug->println(DEBUG_LEVEL_INFO, "Initializing touch screen...");
  touch_init();
}

void initializeLVGL() {
  myDebug->println(DEBUG_LEVEL_INFO, "Initializing LVGL...");
  lv_init();
}

//  WiFi Auto-Reconnect


void initializeClock() {
  myClock = new MyClock();
  myClock->init();
}

void initializeOpenWeather() {
  #ifdef USE_OPEN_WEATHER
    openWeather = new OpenWeather();
    openWeather->init();
  #endif
}

void initializeWiFi() {
  myWiFi = new MyWiFi();
  myWiFi->init(WIFI_STA, USER_WiFiSSID, USER_WiFiPassword);
  myWiFi->connect();
}

void setup() {

  initializeDebug(); 
  myDebug->println(DEBUG_LEVEL_INFO, "Staring up...");

  initializeWiFi();
  initializeDisplay();  
  delay(200);

  initializeLVGL();
  initializeTouchScreen();
  configureDisplay();

  initializeSettings();  
  initializeUI();
  initializeClock();
  initializeOpenWeather();

  createTasks();
  
  myDebug->println(DEBUG_LEVEL_INFO, "Setup completed");

  if (mySettings->readBool(PREF_DAYLIGHT)) {
    lv_obj_add_state(ui_DayLight, LV_STATE_CHECKED);
  } else {
    lv_obj_add_state(ui_DayLight, LV_STATE_DEFAULT);
  }
}

void onDayLightPressed(bool pressed) {
  mySettings->writeBool(PREF_DAYLIGHT, pressed);
}

void loop() {
   vTaskDelete(NULL);
}
