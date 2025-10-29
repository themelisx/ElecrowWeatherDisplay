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
#include "../include/debug.h"
#include "../include/structs.h"
#include "../include/settings.h"
#include "../include/openWeather.h"
#include "../include/uiManager.h"
#include "../include/myClock.h"
#include "../include/myWiFi.h"
#include "../include/externals.h"

////////////////
// User setup // 
////////////////
#include "../include/user_setup.h"
char WiFiSSID[] = USER_WiFiSSID;
char WiFiPassword[] = USER_WiFiPassword;
/////////////////////////////////



// Common vars


// TFT
TwoWire I2Cone = TwoWire(0);
Adafruit_SSD1306 display = Adafruit_SSD1306(128, 64, &I2Cone,OLED_RESET);

SPIClass& spi = SPI;
uint16_t touchCalibration_x0 = 300, touchCalibration_x1 = 3600, touchCalibration_y0 = 300, touchCalibration_y1 = 3600;
uint8_t  touchCalibration_rotate = 1, touchCalibration_invert_x = 2, touchCalibration_invert_y = 0;
static int val = 100;

static uint32_t screenWidth;
static uint32_t screenHeight;
static lv_disp_draw_buf_t draw_buf;
static lv_color_t disp_draw_buf[800 * 480 / 10];
static lv_disp_drv_t disp_drv;

Debug *debug;
MyClock *myClock;
Settings *mySettings;
UIManager *uiManager;
MyWiFi *myWiFi;
#ifdef USE_OPEN_WEATHER
OpenWeather *openWeather;
#endif

#include "../include/lgfx.h"
LGFX lcd;
#include "touch.h"

/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);  
  lcd.pushImageDMA(area->x1, area->y1, w, h,(lgfx::rgb565_t*)&color_p->full);//
  lv_disp_flush_ready(disp);

}

void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data)
{
  if (touch_has_signal())  {
    if (touch_touched()) {
      data->state = LV_INDEV_STATE_PR;

      /*Set the coordinates*/
      data->point.x = touch_last_x;
      data->point.y = touch_last_y;
      #ifndef MODE_RELEASE
        debug->print(DEBUG_LEVEL_DEBUG2, "Data x :" );
        debug->println(DEBUG_LEVEL_DEBUG2, touch_last_x );
        debug->print(DEBUG_LEVEL_DEBUG2, "Data y :" );
        debug->println(DEBUG_LEVEL_DEBUG2, touch_last_y );
      #endif
    }
    else if (touch_released()) {
      data->state = LV_INDEV_STATE_REL;
    }
  }
  else {
    data->state = LV_INDEV_STATE_REL;
  }
  delay(15);
}

void initializeUI() {  

  debug->println(DEBUG_LEVEL_INFO, "initialize UI...");  
  ui_init();

  uiManager = new UIManager();
  
}

void configureDisplay() {
  debug->println(DEBUG_LEVEL_INFO, "Configuring display...");

  screenWidth = lcd.width();
  screenHeight = lcd.height();

  lv_disp_draw_buf_init(&draw_buf, disp_draw_buf, NULL, screenWidth * screenHeight / 10);

  /* Initialize the display */
  lv_disp_drv_init(&disp_drv);
  /* Change the following line to your display resolution */
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  /* Initialize the (dummy) input device driver */
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read;
  lv_indev_drv_register(&indev_drv);
#ifdef TFT_BL
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
#endif

  lcd.fillScreen(0x000000u);
}

void initializeDisplay() {
  debug->println(DEBUG_LEVEL_INFO, "Initializing display...");
  lcd.begin();
  lcd.fillScreen(0x000000u);
  lcd.setTextSize(2); 
  //lcd.setBrightness(127);
}

void initializeSettings() {
  mySettings = new Settings();
  #ifdef CLEAR_SETTINGS
    mySettings->setDefaults();
    mySettings->save();
  #else
    mySettings->load();    
  #endif
}

void initializeDebug() {
  // Initialize Serial and set debug level
  debug = new Debug();
  #if defined(MODE_DEBUG_FULL)
    debug->start(115200, DEBUG_LEVEL_DEBUG2);
  #elif defined(MODE_DEBUG)
    debug->start(115200, DEBUG_LEVEL_DEBUG);
  #elif defined(MODE_RELEASE_INFO)
    debug->start(115200, DEBUG_LEVEL_INFO);
  #elif defined(MODE_RELEASE)
    debug->start(115200, DEBUG_LEVEL_NONE);
  #else
    #error "Select build mode for logs"
  #endif  
}

void initializeTouchScreen() {
  debug->println(DEBUG_LEVEL_INFO, "Initializing touch screen...");
  touch_init();
}

void initializeLVGL() {
  debug->println(DEBUG_LEVEL_INFO, "Initializing LVGL...");
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
  debug->println(DEBUG_LEVEL_INFO, "Staring up...");

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
  
  debug->println(DEBUG_LEVEL_INFO, "Setup completed");

  if (mySettings->IsDayLight()) {
    lv_obj_add_state(ui_DayLight, LV_STATE_CHECKED);
  } else {
    lv_obj_add_state(ui_DayLight, LV_STATE_DEFAULT);
  }
}

void onDayLightPressed(bool pressed) {
  mySettings->setDayLight(pressed);
  mySettings->save();
}

void loop() {
   vTaskDelete(NULL);
}
