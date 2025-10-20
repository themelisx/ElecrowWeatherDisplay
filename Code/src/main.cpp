#include <WiFi.h>
#include "esp_wifi.h"
#include <HttpClient.h>
#include <WiFiClient.h>
#include <ArduinoJson.h> 

#include <lvgl.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h> 
#include <Adafruit_GFX.h>
#include <ESP32Time.h>
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

#ifdef USE_MULTI_THREAD
  #include "../include/tasks.h"
#endif

/////////////////////////////////
// User setup 
/////////////////////////////////
#include "../include/user_setup.h"

char WiFiSSID[] = USER_WiFiSSID;
char WiFiPassword[] = USER_WiFiPassword;
// time zone  
int zone = USER_TimeZone;
// Open Weather
String town = USER_WeatherTown;
String myAPI = USER_WeatherAPI;
String units = USER_WeatherUnits; 
/////////////////////////////////

#define IMG_WIDTH 200
#define IMG_HEIGHT 200
#define PNG_PIXELS_PER_LINE  2048
PNG png;
uint8_t *imageBuffer = nullptr;
uint16_t *rgb565_buffer = nullptr;

const char* ntpServer = "pool.ntp.org";
String server = "https://api.openweathermap.org/data/2.5/weather?q=" + town + "&appid=" + myAPI + "&units=" + units;

ESP32Time rtc(0);
char time_str[9];

uint8_t WiFiChannel;

float windSpeed;
int windDirection;
long sun[2];
float temperature[4];
int humidity;
int pressure;
int visibility;
bool SpiMounted = false;
String description;
String lastUpdate = "";
String oldIconUrl = "";
int bufferSize = IMG_WIDTH * IMG_HEIGHT * 2;
lv_img_dsc_t img_dsc;

bool ntpIsOk = false;
long valuesNeedsUpdateCheck;
bool valuesNeedsUpdate = false;
bool radioNeedsUpdate;
bool multimediaNeedsUpdate;

long now;

#ifdef USE_MULTI_THREAD
  // Tasks
  TaskHandle_t t_core1_loop;
  TaskHandle_t t_core1_clock;
  TaskHandle_t t_core1_http;
  TaskHandle_t t_core1_ntp;
  // Semaphores
  SemaphoreHandle_t semaphoreData;
#else
  
#endif

unsigned long previousBlinkTime;
int blinkInterval = 15000;

char tmp_buf[100];
unsigned long timerDelay = 5000;

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
Settings *mySettings;

class LGFX : public lgfx::LGFX_Device
{
public:

  lgfx::Bus_RGB     _bus_instance;
  lgfx::Panel_RGB   _panel_instance;
  #if defined(ELECROW_DISPLAY_35)
  LGFX(void)
  {
    {                                      
      auto cfg = _bus_instance.config(); 

      cfg.port = 0;              
      cfg.freq_write = 80000000; 
      // cfg.pin_wr = GPIO_NUM_18;
      // cfg.pin_rd = GPIO_NUM_48;
      // cfg.pin_rs = GPIO_NUM_45;

      cfg.pin_d0 = GPIO_NUM_47;
      cfg.pin_d1 = GPIO_NUM_21;
      cfg.pin_d2 = GPIO_NUM_14;
      cfg.pin_d3 = GPIO_NUM_13;
      cfg.pin_d4 = GPIO_NUM_12;
      cfg.pin_d5 = GPIO_NUM_11;
      cfg.pin_d6 = GPIO_NUM_10;
      cfg.pin_d7 = GPIO_NUM_9;
      cfg.pin_d8 = GPIO_NUM_3;
      cfg.pin_d9 = GPIO_NUM_8;
      cfg.pin_d10 = GPIO_NUM_16;
      cfg.pin_d11 = GPIO_NUM_15;
      cfg.pin_d12 = GPIO_NUM_7;
      cfg.pin_d13 = GPIO_NUM_6;
      cfg.pin_d14 = GPIO_NUM_5;
      cfg.pin_d15 = GPIO_NUM_4;

      _bus_instance.config(cfg);              
      _panel_instance.setBus(&_bus_instance); 
    }
    {                                        
      auto cfg = _panel_instance.config();

      cfg.pin_cs = -1;   
      cfg.pin_rst = -1;  
      cfg.pin_busy = -1; 

      cfg.memory_width = 320;   
      cfg.memory_height = 480;  
      cfg.panel_width = 320;    
      cfg.panel_height = 480;   
      cfg.offset_x = 0;         
      cfg.offset_y = 0;         
      cfg.offset_rotation = 0;  
      cfg.dummy_read_pixel = 8; 
      cfg.dummy_read_bits = 1;  
      cfg.readable = true;      
      cfg.invert = false;     
      cfg.rgb_order = false;    
      cfg.dlen_16bit = true;    
      cfg.bus_shared = true;    

      _panel_instance.config(cfg);
    }
    setPanel(&_panel_instance);
  }

  #elif defined(ELECROW_DISPLAY_50)
  LGFX(void)
  {
    {
      auto cfg = _bus_instance.config();
      cfg.panel = &_panel_instance;
      
      cfg.pin_d0  = GPIO_NUM_8; // B0
      cfg.pin_d1  = GPIO_NUM_3;  // B1
      cfg.pin_d2  = GPIO_NUM_46;  // B2
      cfg.pin_d3  = GPIO_NUM_9;  // B3
      cfg.pin_d4  = GPIO_NUM_1;  // B4
      
      cfg.pin_d5  = GPIO_NUM_5;  // G0
      cfg.pin_d6  = GPIO_NUM_6; // G1
      cfg.pin_d7  = GPIO_NUM_7;  // G2
      cfg.pin_d8  = GPIO_NUM_15;  // G3
      cfg.pin_d9  = GPIO_NUM_16; // G4
      cfg.pin_d10 = GPIO_NUM_4;  // G5
      
      cfg.pin_d11 = GPIO_NUM_45; // R0
      cfg.pin_d12 = GPIO_NUM_48; // R1
      cfg.pin_d13 = GPIO_NUM_47; // R2
      cfg.pin_d14 = GPIO_NUM_21; // R3
      cfg.pin_d15 = GPIO_NUM_14; // R4

      cfg.pin_henable = GPIO_NUM_40;
      cfg.pin_vsync   = GPIO_NUM_41;
      cfg.pin_hsync   = GPIO_NUM_39;
      cfg.pin_pclk    = GPIO_NUM_0;
      cfg.freq_write  = 15000000;

      cfg.hsync_polarity    = 0;
      cfg.hsync_front_porch = 8;
      cfg.hsync_pulse_width = 4;
      cfg.hsync_back_porch  = 43;
      
      cfg.vsync_polarity    = 0;
      cfg.vsync_front_porch = 8;
      cfg.vsync_pulse_width = 4;
      cfg.vsync_back_porch  = 12;

      cfg.pclk_active_neg   = 1;
      cfg.de_idle_high      = 0;
      cfg.pclk_idle_high    = 0;

      _bus_instance.config(cfg);
    }
    {
      auto cfg = _panel_instance.config();
      cfg.memory_width  = 800;
      cfg.memory_height = 480;
      cfg.panel_width  = 800;
      cfg.panel_height = 480;
      cfg.offset_x = 0;
      cfg.offset_y = 0;
      _panel_instance.config(cfg);
    }
    _panel_instance.setBus(&_bus_instance);
    setPanel(&_panel_instance);
  }
  #elif defined(ELECROW_DISPLAY_70)
  // Constructor for the LGFX class.
  LGFX(void) {
    // Configure the RGB bus.
    {
      auto cfg = _bus_instance.config();
      cfg.panel = &_panel_instance;

      // Configure data pins.
      cfg.pin_d0  = GPIO_NUM_15; // B0
      cfg.pin_d1  = GPIO_NUM_7;  // B1
      cfg.pin_d2  = GPIO_NUM_6;  // B2
      cfg.pin_d3  = GPIO_NUM_5;  // B3
      cfg.pin_d4  = GPIO_NUM_4;  // B4
      
      cfg.pin_d5  = GPIO_NUM_9;  // G0
      cfg.pin_d6  = GPIO_NUM_46; // G1
      cfg.pin_d7  = GPIO_NUM_3;  // G2
      cfg.pin_d8  = GPIO_NUM_8;  // G3
      cfg.pin_d9  = GPIO_NUM_16; // G4
      cfg.pin_d10 = GPIO_NUM_1;  // G5
      
      cfg.pin_d11 = GPIO_NUM_14; // R0
      cfg.pin_d12 = GPIO_NUM_21; // R1
      cfg.pin_d13 = GPIO_NUM_47; // R2
      cfg.pin_d14 = GPIO_NUM_48; // R3
      cfg.pin_d15 = GPIO_NUM_45; // R4

      // Configure sync and clock pins.
      cfg.pin_henable = GPIO_NUM_41;
      cfg.pin_vsync   = GPIO_NUM_40;
      cfg.pin_hsync   = GPIO_NUM_39;
      cfg.pin_pclk    = GPIO_NUM_0;
      cfg.freq_write  = 15000000;

      // Configure timing parameters for horizontal and vertical sync.
      cfg.hsync_polarity    = 0;
      cfg.hsync_front_porch = 40;
      cfg.hsync_pulse_width = 48;
      cfg.hsync_back_porch  = 40;
      
      cfg.vsync_polarity    = 0;
      cfg.vsync_front_porch = 1;
      cfg.vsync_pulse_width = 31;
      cfg.vsync_back_porch  = 13;

      // Configure polarity for clock and data transmission.
      cfg.pclk_active_neg   = 1;
      cfg.de_idle_high      = 0;
      cfg.pclk_idle_high    = 0;

      // Apply configuration to the RGB bus instance.
      _bus_instance.config(cfg);
    }

    // Configure the panel.
    {
      auto cfg = _panel_instance.config();
      cfg.memory_width  = 800;
      cfg.memory_height = 480;
      cfg.panel_width   = 800;
      cfg.panel_height  = 480;
      cfg.offset_x      = 0;
      cfg.offset_y      = 0;

      // Apply configuration to the panel instance.
      _panel_instance.config(cfg);
    }

    // Set the RGB bus and panel instances.
    _panel_instance.setBus(&_bus_instance);
    setPanel(&_panel_instance);
  }
  #else
    #error "No Display size defined!"
  #endif  
  
};

LGFX lcd;
#include "touch.h"

void createTasks() {
  #ifdef USE_MULTI_THREAD

  debug->println(DEBUG_LEVEL_INFO, "Creating Tasks...");

  debug->println(DEBUG_LEVEL_INFO, "Staring up Http client...");

  xTaskCreatePinnedToCore(
    loop_task,       // Task function.
    "Loop_Manager",  // Name of task.
    10000,          // Stack size of task
    NULL,           // Parameter of the task
    0,              // Priority of the task
    &t_core1_loop,  // Task handle to keep track of created task
    0);             // Pin task to core 0

  vTaskSuspend(t_core1_loop);

  xTaskCreatePinnedToCore(
    clock_task,       // Task function.
    "CLOCK_Manager",  // Name of task.
    10000,          // Stack size of task
    NULL,           // Parameter of the task
    1,              // Priority of the task
    &t_core1_clock,  // Task handle to keep track of created task
    0);             // Pin task to core 0

  vTaskSuspend(t_core1_clock);

  xTaskCreatePinnedToCore(
    http_task,       // Task function.
    "HTTP_Manager",  // Name of task.
    10000,          // Stack size of task
    NULL,           // Parameter of the task
    2,              // Priority of the task
    &t_core1_http,  // Task handle to keep track of created task
    0);             // Pin task to core 0

  vTaskSuspend(t_core1_http);

  xTaskCreatePinnedToCore(
    ntp_task,       // Task function.
    "NTP_Manager",  // Name of task.
    10000,          // Stack size of task
    NULL,           // Parameter of the task
    2,              // Priority of the task
    &t_core1_ntp,  // Task handle to keep track of created task
    0);             // Pin task to core 0

  vTaskSuspend(t_core1_ntp);

  debug->println(DEBUG_LEVEL_INFO, "All tasks created\nStarting tasks...");

  vTaskResume(t_core1_loop);
  vTaskResume(t_core1_http);
  vTaskResume(t_core1_clock);  

#endif
}

/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);  

  //lcd.fillScreen(TFT_WHITE);
#if (LV_COLOR_16_SWAP != 0)
 lcd.pushImageDMA(area->x1, area->y1, w, h,(lgfx::rgb565_t*)&color_p->full);
#else
  lcd.pushImageDMA(area->x1, area->y1, w, h,(lgfx::rgb565_t*)&color_p->full);//
#endif

  lv_disp_flush_ready(disp);

}

void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data)
{
  if (touch_has_signal())
  {
    if (touch_touched())
    {
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

  debug->println(DEBUG_LEVEL_INFO, "initialize UI...");  
  ui_init();
  
}

void setupDisplay() {
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

void createSemaphores() {
#ifdef USE_MULTI_THREAD
  semaphoreData = xSemaphoreCreateMutex();
  xSemaphoreGive(semaphoreData);
#endif
}

void loadSettings() {
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

void updateLastUpdate(const char *str, uint32_t color) {
  if (color == COLOR_RED) { // Error
    sprintf(tmp_buf, "Updated: %s", lastUpdate);
    lv_label_set_text(ui_ValueLastUpdate, tmp_buf);
  } else {
    lv_label_set_text(ui_ValueLastUpdate, str);  
  }
  lv_obj_set_style_text_color(ui_ValueLastUpdate, lv_color_hex(color), LV_PART_MAIN | LV_STATE_DEFAULT);
  
}

bool setTime() {
  debug->println(DEBUG_LEVEL_DEBUG, "Getting time from NTP...");

  if (mySettings->IsDayLight()) {
    configTime(3600 * zone, 3600, ntpServer); // Daylight  
  } else {
    configTime(3600 * zone, 0, ntpServer); // Winter
  }
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    rtc.setTimeStruct(timeinfo);
    debug->println(DEBUG_LEVEL_DEBUG, "NTP Ok");
    return true;
  } else {
    debug->println(DEBUG_LEVEL_DEBUG, "NTP Failed");
    return false;
  }
}

const char* convertDegreesToDirection(int degrees) {
    // Normalize degrees to the range [0, 360)
    degrees = degrees % 360;

    if (degrees < 0) degrees += 360;

    if (degrees >= 337.5 || degrees < 22.5)  return "N";
    if (degrees >= 22.5 && degrees < 67.5)   return "NE";
    if (degrees >= 67.5 && degrees < 112.5)  return "E";
    if (degrees >= 112.5 && degrees < 157.5) return "SE";
    if (degrees >= 157.5 && degrees < 202.5) return "S";
    if (degrees >= 202.5 && degrees < 247.5) return "SW";
    if (degrees >= 247.5 && degrees < 292.5) return "W";
    if (degrees >= 292.5 && degrees < 337.5) return "NW";

    return "Unknown"; // In case something unexpected happens
}

int windSpeedToBeaufort(float speed) {
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

void timestampToTime(time_t timestamp, char *buffer, size_t buffer_size) {
    struct tm *time_info;
    time_info = localtime(&timestamp);
    strftime(buffer, buffer_size, "%H:%M", time_info);
}

void updateValues() {

  debug->println(DEBUG_LEVEL_DEBUG, "Updating values");

  sprintf(tmp_buf, "%0.1f °C", temperature[0]);
  lv_label_set_text(ui_ValueTemperature, tmp_buf);

  sprintf(tmp_buf, "%0.1f °C", temperature[1]);
  lv_label_set_text(ui_ValueFeelsLike, tmp_buf);

  //sprintf(tmp_buf, "Min : %0.1f °C", temperature[2]);
  timestampToTime(sun[0], time_str, sizeof(time_str));
  lv_label_set_text(ui_ValueSunrise, time_str);

  timestampToTime(sun[1], time_str, sizeof(time_str));
  lv_label_set_text(ui_ValueSunset, time_str);

  sprintf(tmp_buf, "%d %%", humidity);
  lv_label_set_text(ui_ValueHumidity, tmp_buf);

  sprintf(tmp_buf, "%d hPa", pressure);
  lv_label_set_text(ui_ValuePressure, tmp_buf);

  sprintf(tmp_buf, "%0.1f m/s", windSpeed);
  lv_label_set_text(ui_ValueWindSpeed, tmp_buf);
  sprintf(tmp_buf, "Wind: %d Bf", windSpeedToBeaufort(windSpeed));
  lv_label_set_text(ui_Label2, tmp_buf);

  sprintf(tmp_buf, "Direction: %s", convertDegreesToDirection(windDirection));
  lv_label_set_text(ui_ValueWindDirection, tmp_buf);

  sprintf(tmp_buf, "Updated: %s", rtc.getTime());
  lv_label_set_text(ui_ValueLastUpdate, tmp_buf);
  
  debug->println(DEBUG_LEVEL_DEBUG, "Done");
}

int renderPNGToBuffer(PNGDRAW *pDraw) {
    uint16_t lineBuffer[IMG_WIDTH]; 
    png.getLineAsRGB565(pDraw, lineBuffer, PNG_RGB565_LITTLE_ENDIAN, 0x0000);

    for (int x = 0; x < pDraw->iWidth; x++) {
        int dst_index = (pDraw->y * IMG_WIDTH + x);
        rgb565_buffer[dst_index] = lineBuffer[x];
    }

    return 1; // 1 σημαίνει OK, 0 σημαίνει stop
}

bool decodePngToRgb565(uint8_t *png_data, int png_size) {    
    int rc = png.openRAM(png_data, png_size, renderPNGToBuffer);
    if (rc == PNG_SUCCESS) {
      rc = png.decode(NULL, 0);
      png.close();
      return true;
    } else {
      debug->print(DEBUG_LEVEL_ERROR, "PNG decode failed, code = ");
      debug->println(DEBUG_LEVEL_ERROR, rc);
      return false;
    }    
}

void downloadImageToMemory(const char *url) {
  debug->print(DEBUG_LEVEL_DEBUG, "image url: ");
  debug->println(DEBUG_LEVEL_DEBUG, url);

  WiFiClient client;
  HTTPClient http;
  if (http.begin(client, url)) {
    int httpCode = http.GET();

    if (httpCode == HTTP_CODE_OK) {
      int totalSize = http.getSize();
      if (totalSize > bufferSize) {
        debug->println(DEBUG_LEVEL_ERROR, "Image size exceeds buffer size");
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

        debug->print(DEBUG_LEVEL_DEBUG, bytesRead);
        debug->println(DEBUG_LEVEL_DEBUG, " bytes downloaded");

        if (bytesRead != -1) {
              debug->println(DEBUG_LEVEL_DEBUG, "updating icon...");
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
      debug->println(DEBUG_LEVEL_ERROR, "Failed to download image");
      debug->println(DEBUG_LEVEL_ERROR, http.errorToString(httpCode).c_str());
    }

    http.end();
  }
}

bool getData() {
    debug->println(DEBUG_LEVEL_DEBUG, "Getting data...");

    bool ret = false;

    HTTPClient http;
    http.begin(server);
    int httpResponseCode = http.GET();

    if (httpResponseCode > 0) {
        String payload = http.getString();
        debug->println(DEBUG_LEVEL_DEBUG, "server response");
        debug->println(DEBUG_LEVEL_DEBUG, payload);

        // Νέος τρόπος με JsonDocument (ArduinoJson v7+)
        JsonDocument doc;

        DeserializationError error = deserializeJson(doc, payload);

        if (!error) {
            xSemaphoreTake(semaphoreData, portMAX_DELAY);
            temperature[0] = doc["main"]["temp"];
            temperature[1] = doc["main"]["feels_like"];
            temperature[2] = doc["main"]["temp_min"];
            temperature[3] = doc["main"]["temp_max"];
            humidity = doc["main"]["humidity"];
            pressure = doc["main"]["pressure"];
            windSpeed = doc["wind"]["speed"];
            windDirection = doc["wind"]["deg"];
            sun[0] = doc["sys"]["sunrise"];
            sun[1] = doc["sys"]["sunset"];

            visibility = doc["visibility"];
            description = doc["weather"][0]["description"].as<String>();

            String iconUrl =
                "https://openweathermap.org/img/wn/" +
                doc["weather"][0]["icon"].as<String>() +
                "@4x.png";
            iconUrl.replace("https://", "http://");
            
            xSemaphoreGive(semaphoreData);

            http.end();

            if (imageBuffer != nullptr) {
                debug->println(DEBUG_LEVEL_DEBUG, iconUrl);
                if (!iconUrl.equals(oldIconUrl)) {
                    debug->println(DEBUG_LEVEL_DEBUG, "icon is different");
                    oldIconUrl = iconUrl;
                    downloadImageToMemory(iconUrl.c_str());
                }
            }

            lastUpdate = rtc.getTime();
            ret = true;
        } else {
            http.end();
            debug->println(DEBUG_LEVEL_ERROR, "ERROR JSON: ");
            debug->println(DEBUG_LEVEL_ERROR, error.c_str());
        }
    } else {
        http.end();
        debug->println(DEBUG_LEVEL_ERROR, "HTTP ERROR ");
        debug->println(DEBUG_LEVEL_ERROR, httpResponseCode);
    }

    return ret;
}

void setup() {

  radioNeedsUpdate = false;

  // Allocate memory for the buffer
  imageBuffer = (uint8_t *)malloc(bufferSize + 1000);
  // Allocate RGB565 buffer
  rgb565_buffer = (uint16_t *)malloc(bufferSize + 1000);

  initializeDebug(); 

  debug->println(DEBUG_LEVEL_INFO, "Staring up...");

  WiFi.mode(WIFI_STA);

  debug->print(DEBUG_LEVEL_INFO, "MAC Address: ");
  debug->println(DEBUG_LEVEL_INFO, WiFi.macAddress());
  
  debug->print(DEBUG_LEVEL_DEBUG, "Connecting to WiFi");
  WiFi.begin(WiFiSSID, WiFiPassword);

  int count = 1;
  while (WiFi.status() != WL_CONNECTED) {
    debug->print(DEBUG_LEVEL_DEBUG, ".");
    delay(1000);
    count++;
    if (count > 10) {
      debug->println(DEBUG_LEVEL_DEBUG, "\nTime out");
      break;
    }
  }
  wifi_ap_record_t ap_info;
  if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
    WiFiChannel = ap_info.primary;
    debug->println(DEBUG_LEVEL_DEBUG, "\nWiFi connected");
  } else {
    WiFiChannel = 0;
    debug->println(DEBUG_LEVEL_DEBUG, "Not connected to any WiFi network.");    
  }

  debug->print(DEBUG_LEVEL_DEBUG, "WiFi channel: ");
  debug->println(DEBUG_LEVEL_DEBUG, WiFiChannel);    

  initializeDisplay();
  delay(200);

  initializeLVGL();
  initializeTouchScreen();
  setupDisplay();

  createSemaphores();
  loadSettings();
  
  initializeUI();

  createTasks();

  debug->println(DEBUG_LEVEL_INFO, "Setup completed");

  if (mySettings->IsDayLight()) {
    lv_obj_add_state(ui_DayLight, LV_STATE_CHECKED);
  } else {
    lv_obj_add_state(ui_DayLight, LV_STATE_DEFAULT);
  }

}

void loop_task(void *pvParameters) {

  debug->print(DEBUG_LEVEL_INFO, "Loop manager: Task running on core ");
  debug->println(DEBUG_LEVEL_INFO, xPortGetCoreID());

  while (1) {
    vTaskDelay(5 / portTICK_PERIOD_MS);
    lv_timer_handler();
  }
  debug->println(DEBUG_LEVEL_INFO, "Terminating Loop manager");
  vTaskDelete(NULL);
}

void clock_task(void *pvParameters) {

  debug->print(DEBUG_LEVEL_INFO, "Clock manager: Task running on core ");
  debug->println(DEBUG_LEVEL_INFO, xPortGetCoreID());

  while (1) {
    vTaskDelay(DELAY_MAIN_TASK / portTICK_PERIOD_MS);

    now = millis();

    if (now > valuesNeedsUpdateCheck + 1000) {
      
      //debug->println(DEBUG_LEVEL_DEBUG, "Checking if we need update");
      valuesNeedsUpdateCheck = now;
      xSemaphoreTake(semaphoreData, portMAX_DELAY);
      bool ntpOk = ntpIsOk;
      xSemaphoreGive(semaphoreData);
      if (ntpOk) {
        struct tm timeinfo = rtc.getTimeStruct();
        // TODO: Add to settings "Date format"
        strftime(tmp_buf, 50, "%a, %d %b %Y", &timeinfo);
        lv_label_set_text(ui_ValueDate, tmp_buf);

        // TODO: Add to settings "Hour format"
        strftime(tmp_buf, 50, "%H:%M", &timeinfo);      // 24h format
        //strftime(tmp_buf, 50, "%I:%M %p", &timeinfo); // 12h format
        lv_label_set_text(ui_ValueTime, tmp_buf);
      }
      if (valuesNeedsUpdate) {
        debug->println(DEBUG_LEVEL_DEBUG, "We need update");
        xSemaphoreTake(semaphoreData, portMAX_DELAY);
        valuesNeedsUpdate = false;
        updateValues();
        xSemaphoreGive(semaphoreData);
      } else {
        //debug->println(DEBUG_LEVEL_DEBUG, "No need to update");
      }
    }
  }
  debug->println(DEBUG_LEVEL_INFO, "Terminating Clock manager");
  vTaskDelete(NULL);
}

void onDayLightPressed(bool pressed) {
  mySettings->setDayLight(pressed);
  mySettings->save();
}

void loop() {

   vTaskDelete(NULL);

}
