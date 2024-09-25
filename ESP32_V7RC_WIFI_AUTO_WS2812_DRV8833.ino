/*

History 

1. 加入Fail Safe 功能 ( 先用不動作解決.... 也就是1500,1500,1500,1500 )
2. 加入隨選WIFI CH 的功能 (CH 3 ~ CH 10 來選)
3. 加入利用LED 燈來切換是否使用FIXER ... 


2024 0921 

改用 DRV8833 來做為驅動

保留原來的PWM Servo 驅動 PIN #2 

使用 PIN 12 13 14 15  來做為驅動 DRV8833 兩路馬達的腳位
本來用於控制WS2812 的 15 腳 換成 PIN 4  (不能用PIN 16 的樣子，衝到 CAM (實驗得知..))
 

*/

/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-cam-video-streaming-web-server-camera-home-assistant/
  
  IMPORTANT!!! 
   - Select Board "AI Thinker ESP32-CAM"
   - GPIO 0 must be connected to GND to upload a sketch
   - After connecting GPIO 0 to GND, press the ESP32-CAM on-board RESET button to put your board in flashing mode
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/
/*
UDP Code Here

https://www.alejandrowurts.com/projects/bb9e-v1-update-7-esp32-bilateral-coms/


*/

/*
RoboTW   


FB:  RoboTW 機器人論壇
FB:  https://www.facebook.com/groups/540271770146161/

這個程式是用來配合V7RC 手機APP 的，可以同時利用BLE 控制載具 並利用WIFI 進行即時影像接收。
This Program is used with V7RC app @ Apple Store and Google Play. It can be used to control Servos and DIOS with BLE and get real time Video via Wi-fi link.

 V7RC 是非常好用的手機遙控工具軟體，希望有機會也可以找嵐奕科技有限公司合作
 https://apps.apple.com/tw/app/v7rc/id1390983964
 https://play.google.com/store/apps/details?id=com.v7idea.v7rcliteandroidsdkversion&hl=zh_TW
 

 影像部分的程式是從下列網址copy 來用，感謝原作者
 https://randomnerdtutorials.com/esp32-cam-video-streaming-web-server-camera-home-assistant/

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

 
   如果喜歡，記得來FB 群組跟我們分享
   If you like this work, please come to our FB Group, and tell us what you made.

 allen54a0@gmail.com
  
 */

#include "esp_camera.h"
#include <WiFi.h>
#include "esp_timer.h"
#include "img_converters.h"
#include "Arduino.h"
#include "fb_gfx.h"
#include "soc/soc.h"          //disable brownout problems
#include "soc/rtc_cntl_reg.h" //disable brownout problems
#include "esp_http_server.h"


#include <Adafruit_NeoPixel.h>


//// 看看是要給坦克用，還是給甲蟲、螞蟻使用
#define _TANK_
//#define _BUG_


// Which pin on the Arduino is connected to the NeoPixels?
// On a Trinket or Gemma we suggest changing this to 1:
#define LED_PIN    4
#define LED_COUNT 3
//NEOPIXEL color... 
uint32_t color;


// Declare our NeoPixel pixels object:
Adafruit_NeoPixel pixels(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
 



// Replace with your network credentials

#define DEVICE_NAME "QBEAR_CAM001"
#define BLE_NAME DEVICE_NAME

const char *ssid = DEVICE_NAME;
const char *password = "a1234567";

///PWM  PIN 2  12 13  14 (4 LCD)
///LCDC CH  2  3   4  7

WiFiUDP Udp; // Creation of wifi Udp instance

char packetBuffer[255];

unsigned int localPort = 6188;

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

static const int servosPins[4] = {2, 12, 14, 13};

static const int PWM_CH_0 = 2;
static const int PWM_CH_1 = 4;
static const int PWM_CH_3 = 3;
static const int PWM_CH_2 = 7;


void SetServoPos(int ch, int pos)
{
  uint32_t duty = ((((float)pos / 180.0) * 2000) / 20000.0 * 65536.0) + 1634;

  ledcWrite(ch, duty);
  // set channel to pos
}


/// 留一路Servo 備用
/// 2024 0921 
void initServo()
{

  //channel, freq, resolution
  ledcSetup(PWM_CH_2, 50, 16);            ////LCD.ch2 attached to
  ledcAttachPin(servosPins[0], PWM_CH_2); //ESP32.P2

 
  SetServoPos(PWM_CH_2,90);

 
  
}


/// DRV8833 的宣告，GPT 生成 ------------------------------------------------------------->
/// 2024 0921 

// 定義兩路馬達控制的GPIO腳位
#define MOTOR1_PIN_1 12  // AIN1 for motor 1
#define MOTOR1_PIN_2 13  // AIN2 for motor 1
#define MOTOR2_PIN_1 14  // AIN1 for motor 2
#define MOTOR2_PIN_2 15  // AIN2 for motor 2

// 定義 LED PWM 相關參數
#define PWM_FREQ 5000      // PWM 頻率
#define PWM_RESOLUTION 8   // PWM 解析度，0-255
#define PWM_CHANNEL_1 1    // PWM 通道1，用於 MOTOR1_PIN_1
#define PWM_CHANNEL_2 2    // PWM 通道2，用於 MOTOR1_PIN_2
#define PWM_CHANNEL_3 3    // PWM 通道3，用於 MOTOR2_PIN_1
#define PWM_CHANNEL_4 4    // PWM 通道4，用於 MOTOR2_PIN_2




// 控制兩路馬達的正反轉與速度
// motorNo: 1 或 2 來選擇要控制的馬達
// direction: 1 正轉, -1 反轉, 0 停止
// speed: 速度範圍為 0-255 (使用 PWM)
void setMotor(int motorNo, int direction, int speed) {
  // 限制速度的範圍
  speed = constrain(speed, 0, 255);

  if (motorNo == 1) {
    // 控制馬達1的正反轉或停止
    if (direction == 1) {
      // 正轉
      ledcWrite(PWM_CHANNEL_1, speed);  // 給 MOTOR1_PIN_1 輸出 PWM
      ledcWrite(PWM_CHANNEL_2, 0);      // MOTOR1_PIN_2 設為低電平
    } else if (direction == -1) {
      // 反轉
      ledcWrite(PWM_CHANNEL_1, 0);      // MOTOR1_PIN_1 設為低電平
      ledcWrite(PWM_CHANNEL_2, speed);  // 給 MOTOR1_PIN_2 輸出 PWM
    } else {
      // 停止
      ledcWrite(PWM_CHANNEL_1, 0);
      ledcWrite(PWM_CHANNEL_2, 0);
    }
  } else if (motorNo == 2) {
    // 控制馬達2的正反轉或停止
    if (direction == 1) {
      // 正轉
      ledcWrite(PWM_CHANNEL_3, speed);  // 給 MOTOR2_PIN_1 輸出 PWM
      ledcWrite(PWM_CHANNEL_4, 0);      // MOTOR2_PIN_2 設為低電平
    } else if (direction == -1) {
      // 反轉
      ledcWrite(PWM_CHANNEL_3, 0);      // MOTOR2_PIN_1 設為低電平
      ledcWrite(PWM_CHANNEL_4, speed);  // 給 MOTOR2_PIN_2 輸出 PWM
    } else {
      // 停止
      ledcWrite(PWM_CHANNEL_3, 0);
      ledcWrite(PWM_CHANNEL_4, 0);
    }
  }
}

void drv8833Setup() {
  // 設置 GPIO 為輸出模式
  pinMode(MOTOR1_PIN_1, OUTPUT);
  pinMode(MOTOR1_PIN_2, OUTPUT);
  pinMode(MOTOR2_PIN_1, OUTPUT);
  pinMode(MOTOR2_PIN_2, OUTPUT);
  
  // 設置 PWM 通道和頻率、解析度
  ledcSetup(PWM_CHANNEL_1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_2, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_3, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_4, PWM_FREQ, PWM_RESOLUTION);
  
  // 將 GPIO 腳位綁定到 PWM 通道
  ledcAttachPin(MOTOR1_PIN_1, PWM_CHANNEL_1);
  ledcAttachPin(MOTOR1_PIN_2, PWM_CHANNEL_2);
  ledcAttachPin(MOTOR2_PIN_1, PWM_CHANNEL_3);
  ledcAttachPin(MOTOR2_PIN_2, PWM_CHANNEL_4);

   setMotor(1,0,0);
   setMotor(2,0,0);
    
 
}



/// DRV8833 的宣告，GPT 生成 ------------------------------------------------------------<<
/// 2024 0921 

//HZ Control
#define HZ_SETTING 100
int mainLoop_count;
unsigned long fast_loopTimer; // Time in miliseconds of main control loop
const int hzCount = (1000 / HZ_SETTING) - 1;
const int timeRemind = 1000 / HZ_SETTING;

///////////////////////BLE --------------------------->

int datafromV7RC[8] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
int faliSafefromV7RC[8] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
int ledfromV7RC[8] = {0, 0, 0, 0, 0, 0, 0, 0};
BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t txValue = 0;

 

void parseCommand();
int flagShowControl = 0;

//2020 0926 for Mixer
int mixerL = 1500;
int mixerR = 1500;
int mandible = 1500;

////////2020 1223 add funstion
// 用來判斷輸出的模式 1 = MIXER ON ,  0= MIXER OFf , 100 = Fail Safe ...
byte isMixerOn = 1;

/// 判斷是否LOST LINK
byte isLostLink = 0;
byte lostLinkCount = 0;
int linkSN = 0;
int lastLinkSN = 0;

#define WIFI_CH_MIN 3
#define WIFI_CH_MAX 10

 

int hexConvert2int(char highByte, char lowByte)
{

  int val = 0;
  int highB;
  int lowB;

  if (highByte >= 'A')
  {
    highB = highByte - 'A' + 10;
  }
  else if (highByte >= '0' && highByte <= '9')
  {
    highB = highByte - 0x30;
  }else {
  highB = 0; // 或其他處理
}

  if (lowByte >= 'A')
  {
    lowB = lowByte - 'A' + 10;
  }
  else if (lowByte >= '0' && lowByte <= '9')
  {
    lowB = lowByte - 0x30;
  }else {
  lowB = 0; // 或其他處理
}

  val = highB * 16 + lowB;
  val = val * 10;
  return val;
}

 
 


void dumpespLoraData()
{

  for (int i = 0; i < 8; i++)
  {

    Serial.print(map(datafromV7RC[i], 1000, 2000, -255, 255));
    Serial.print(",");
  }
  for (int i = 0; i < 6; i++)
  {

    Serial.print(map(datafromV7RC[i], 1000, 2000, -255, 255));
    Serial.print(",");
  }
  Serial.println("");
}


void parseCommand()
{
  char cmd = Serial.read();
  switch (cmd)
  {
  case 'D':

    dumpespLoraData();
    break;

    case '2':
    flagShowControl = 2;
    break;


  case '1':
    flagShowControl = 1;
    break;

  case '0':
    flagShowControl = 0;
    break;
  }
}

///////////////////////Servo --------------------------->

#define PART_BOUNDARY "123456789000000000000987654321"

// This project was tested with the AI Thinker Model, M5STACK PSRAM Model and M5STACK WITHOUT PSRAM
#define CAMERA_MODEL_AI_THINKER
//#define CAMERA_MODEL_M5STACK_PSRAM
//#define CAMERA_MODEL_M5STACK_WITHOUT_PSRAM

// Not tested with this model
//#define CAMERA_MODEL_WROVER_KIT

#if defined(CAMERA_MODEL_WROVER_KIT)
#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 21
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27

#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 19
#define Y4_GPIO_NUM 18
#define Y3_GPIO_NUM 5
#define Y2_GPIO_NUM 4
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

#elif defined(CAMERA_MODEL_M5STACK_PSRAM)
#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM 15
#define XCLK_GPIO_NUM 27
#define SIOD_GPIO_NUM 25
#define SIOC_GPIO_NUM 23

#define Y9_GPIO_NUM 19
#define Y8_GPIO_NUM 36
#define Y7_GPIO_NUM 18
#define Y6_GPIO_NUM 39
#define Y5_GPIO_NUM 5
#define Y4_GPIO_NUM 34
#define Y3_GPIO_NUM 35
#define Y2_GPIO_NUM 32
#define VSYNC_GPIO_NUM 22
#define HREF_GPIO_NUM 26
#define PCLK_GPIO_NUM 21

#elif defined(CAMERA_MODEL_M5STACK_WITHOUT_PSRAM)
#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM 15
#define XCLK_GPIO_NUM 27
#define SIOD_GPIO_NUM 25
#define SIOC_GPIO_NUM 23

#define Y9_GPIO_NUM 19
#define Y8_GPIO_NUM 36
#define Y7_GPIO_NUM 18
#define Y6_GPIO_NUM 39
#define Y5_GPIO_NUM 5
#define Y4_GPIO_NUM 34
#define Y3_GPIO_NUM 35
#define Y2_GPIO_NUM 17
#define VSYNC_GPIO_NUM 22
#define HREF_GPIO_NUM 26
#define PCLK_GPIO_NUM 21

#elif defined(CAMERA_MODEL_AI_THINKER)
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27

#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22
#else
#error "Camera model not selected"
#endif

static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t stream_httpd = NULL;

static esp_err_t stream_handler(httpd_req_t *req)
{
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t *_jpg_buf = NULL;
  char *part_buf[64];

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK)
  {
    return res;
  }

  while (true)
  {

    long startters = millis();
    fb = esp_camera_fb_get();

    if (!fb)
    {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
    }
    else
    {
      if (fb->width > 400)
      {
        if (fb->format != PIXFORMAT_JPEG)
        {
          bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
          esp_camera_fb_return(fb);
          fb = NULL;
          if (!jpeg_converted)
          {
            Serial.println("JPEG compression failed");
            res = ESP_FAIL;
          }
        }
        else
        {
          _jpg_buf_len = fb->len;
          _jpg_buf = fb->buf;
        }
      }
    }
    if (res == ESP_OK)
    {
      size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    }
    if (res == ESP_OK)
    {
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }
    if (res == ESP_OK)
    {
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    if (fb)
    {


      esp_camera_fb_return(fb);

       if (flagShowControl == 2){
          Serial.print(1000/ (millis() - startters));
          Serial.println(" FPS ");
       }
      fb = NULL;
      _jpg_buf = NULL;
    }
    else if (_jpg_buf)
    {
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    if (res != ESP_OK)
    {
      break;
    }
    //Serial.printf("MJPG: %uB\n",(uint32_t)(_jpg_buf_len));
  }
  return res;
}

void startCameraServer()
{
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;

  httpd_uri_t index_uri = {
      .uri = "/",
      .method = HTTP_GET,
      .handler = stream_handler,
      .user_ctx = NULL};

  //Serial.printf("Starting web server on port: '%d'\n", config.server_port);
  if (httpd_start(&stream_httpd, &config) == ESP_OK)
  {
    httpd_register_uri_handler(stream_httpd, &index_uri);
  }
}

int wifiStrangth[15]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

int getfreeWifiCH(){

  int bestRssiCH =0;
  int compareRssi;
  int compare2;
  

   Serial.println("Start scan");
  int n = WiFi.scanNetworks();

   if (n == 0) {
    Serial.println("no networks found");
  } else {
    Serial.print(n);
    Serial.println(" networks found");
    for (int i = 0; i < n; ++i) {
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(WiFi.SSID(i));
      Serial.print(" (");
      Serial.print(WiFi.RSSI(i));
      Serial.print(") ");
      Serial.print(" [");
      Serial.print(WiFi.channel(i));
      wifiStrangth[WiFi.channel(i)] += WiFi.RSSI(i);

      Serial.println("] ");
     
      delay(10);
    }


  }
  Serial.println("Scan done");
  Serial.println("");

  for(int i=0;i<15;i++){

    Serial.print(" CH  ");
    Serial.print(i);
    Serial.print(" = ");
    Serial.println(wifiStrangth[i]);

  }

  //check if there are channel available... 

  for(int i=1;i<11;i++){

    if(wifiStrangth[i]==0){
        Serial.print("\n\n\n\n we got cleaer ch : ");
        Serial.println(i);
        return i;
    }



  }

    ///find lowest RSSI Value... 
    bestRssiCH = 1; 
    compareRssi  =wifiStrangth[bestRssiCH] ;

  for(int i=1;i<11;i++){

      if(compareRssi<=wifiStrangth[i] ) {
          bestRssiCH =i;
          compareRssi = wifiStrangth[i];
      }


  }
        Serial.print("\n\n\n\n we got best RSSI  ch : ");
        Serial.println(bestRssiCH);
        return bestRssiCH;


 

}


void showCHcolor(int wifiCH, int delayTime){

/*
   strip.Color(255,   0,   0)     ; // Red
   strip.Color(  0, 255,   0)    ; // Green
   strip.Color(  0,   0, 255)    ; // Blue
   strip.Color(  0,   0,   0, 255) ; // True white (not RGB white)
*/


  pixels.clear();/// clear LED

  switch(wifiCH){


    case 0:{

          color = pixels.Color(0,   0,   0);   

    }break;


   case 1:{

          color = pixels.Color(100,   50,   0); // 棕

    }break;

   case 2:{

           color = pixels.Color(255,   0,   0);   // 紅

   }break;


   case 3:{

           color = pixels.Color(200,   100,   0);   //橙

   }break;

   
   case 4:{

           color = pixels.Color(255,   255,   0); // 黃

   }break;

   
   case 5:{

           color = pixels.Color(0,   255,   0); // 綠

   }break;

   
   case 6:{

           color = pixels.Color(0,   0,   255); // 藍

   }break;

   
   case 7:{

           color = pixels.Color(255,   0,   255); // 紫色

   }break;

   
   case 8:{

           color = pixels.Color(192,   192,   192); //灰

   }break;

   
   case 9:{

           color = pixels.Color(0,   0,   0,255); // 白

   }break;

   
   case 10:{

             color = pixels.Color(255,   0,   0);

   }break;

   
   case 11:{

             color = pixels.Color(100,   100,   100);

   }break;

   
   case 12:{

             color = pixels.Color(100,   0,   0);

   }break;

   
   case 13:{

           color = pixels.Color( 0 ,   100,   0);

   }break;

   
   case 14:{

           color = pixels.Color( 100,   100,   0);

   }break;

   
   case 15:{

             color = pixels.Color(0,   0,   100);

   }break;

   default:

              color = pixels.Color(100,   0,   100);


  }

            pixels.fill(color, 0, 3);
            pixels.show();  
            delay(delayTime);


}
 


void setup()
{
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector

  Serial.begin(115100);
  Serial.setDebugOutput(false);

  //Servo Init
  initServo();
  drv8833Setup();

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 10000000; // 20000000 for 12fps
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound())
  {

    Serial.println("psramFound FRAMESIZE_VGA ");
 
     config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 12;
    config.fb_count = 2;
  }
  else
  {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // Camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK)
  {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  //drop down frame size for higher initial frame rate
///  s->set_framesize(s, FRAMESIZE_QVGA);
 s->set_framesize(s, FRAMESIZE_VGA);


  pixels.begin();           // INITIALIZE NeoPixel pixels object (REQUIRED)
  
  
  for(int i=0;i<15;i++){

      showCHcolor(i,300);

  }



  // Connect to Wi-Fi network with SSID and password
  Serial.print("Setting AP (Access Point)… CH : ");




  // Remove the password parameter, if you want the AP (Access Point) to be open
  int WIFI_CH = getfreeWifiCH(); 
 
  Serial.println(WIFI_CH);
  WiFi.softAP(ssid, password, WIFI_CH, false, 2);
  //WiFi.softAP(ssid );

  showCHcolor(WIFI_CH,50);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("Camera Stream Ready! Connect to the ESP32 AP and go to: http://");
  Serial.println(IP);

  ////UDP Here ...
  Udp.begin(localPort);

  // Start streaming web server
  startCameraServer();

 
}

void loop()
{
  if (Serial.available())
    parseCommand();

  int packetSize = Udp.parsePacket();
  if (packetSize)
  {
    String rxData;
    String data;
    int len = Udp.read(packetBuffer, 255);
    if (len > 0)
      packetBuffer[len - 1] = 0;
    // Serial.println(packetBuffer);
    //Serial.println( len);

    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.printf("received: ");
    Udp.printf(packetBuffer);
    Udp.printf("\r\n");
    Udp.endPacket();

    if (len > 0)
    {
      //  Serial.println("*********");
      //   Serial.print("Received Value: ");
      for (int i = 0; i < len; i++)
      {
        //   Serial.print(rxValue[i]);
        rxData += packetBuffer[i];
      }

      //   Serial.println();
      // Serial.println("*********");

      /// 增加linkSN ，用來檢查資料是否持續更新
      linkSN++;
    }

    ///// V7RC Code ---------------------------------------------------------------->>>
    if (packetBuffer[1] == 'R')
    {

      for (int i = 0; i < 4; i++)
      {
        data = rxData.substring(i * 4 + 3, i * 4 + 7);
        datafromV7RC[i] = data.toInt();
      }
    }
    else if (packetBuffer[1] == 'E')
    {

      for (int i = 0; i < 8; i++)
      {

        ledfromV7RC[i] = hexConvert2int(packetBuffer[i * 2 + 3], packetBuffer[i * 2 + 4]);
      }

      if (ledfromV7RC[7] == 0)
      {
        isMixerOn = 1;
      }
      else
      {
        isMixerOn = 0;
      }

            pixels.clear(); 

            if(packetBuffer[3]=='F'){
               Serial.println("LED 1");  
              color = pixels.Color(255,   0,   0);

            }else if(packetBuffer[7]=='F'){
                 Serial.println("LED 2");  
               color = pixels.Color(0,   255,   0);
  

            }else if(packetBuffer[11]=='F'){
                Serial.println("LED 3");   
              color = pixels.Color(0,   0,   255);

            }else if(packetBuffer[15]=='F'){
               Serial.println("LED 4");    
              color = pixels.Color(255,   255,   0);

            }else{
               Serial.println("LED X");   
               color = pixels.Color(10,   10,   10);
            }
 

            pixels.fill(color, 0, 3);
            pixels.show();  


      
    }
    else
    { //for SS8   CMD  (8 Servo)   //SS8 96 96 96 96 96 96 96 96#

      for (int i = 0; i < 8; i++)
      {

        datafromV7RC[i] = hexConvert2int(packetBuffer[i * 2 + 3], packetBuffer[i * 2 + 4]);
      }
    }

    ////debug Only, send to Vrep....

    if (flagShowControl == 1)
    {
      Serial.print(packetBuffer[2]); /// should be V / T / 8 (2 ch, 4 ch , 8 ch )
      Serial.print(",");

      for (int i = 0; i < 8; i++)
      {
        Serial.print(datafromV7RC[i]);
        Serial.print(",");
      }

      Serial.println(",");
    }
  }

  ///// V7RC Code ----------------------------------------------------------------<<<<<

  if (millis() - fast_loopTimer > hzCount) //100 HZ
  {
    fast_loopTimer = millis();
    mainLoop_count++;

    //////Servo  Loop --------------------------------------------------------------------->

    if (mainLoop_count % 5 == 0) ///20 HZ
    {

      /// 2020 1223 確認資料有一直近來
      if (lastLinkSN != linkSN)
      { /// 如果Link SN 有改變的話

        lostLinkCount = 0;
        lastLinkSN = linkSN;
        isLostLink = 0;
      }
      else
      { /// 如果linkSN 跟上次一樣，表示資料沒有更新

        lostLinkCount++;

        if (lostLinkCount > 20)
        {
          isLostLink = 1;
        }
      }

      if (isLostLink == 0)
      { /// 如果沒有進入Fial Safe Mode

        
 
          SetServoPos(PWM_CH_2, map(datafromV7RC[0], 1000, 2000, 0, 180));

         
        if(datafromV7RC[1]>1503){

          setMotor(1, -1   , map(datafromV7RC[1], 1500, 2000, 0, 255) ) ;
        
        }else if(datafromV7RC[1]<1497) {
            
          setMotor(1, 1  , map(datafromV7RC[1], 1000, 1500, 255, 0) ) ;  

        }else{
           setMotor(1, 0  ,0 ) ;  

        }

         if(datafromV7RC[2]>1503){

          setMotor(2, -1   , map(datafromV7RC[2], 1500, 2000, 0, 255) ) ;
        
        }else if(datafromV7RC[2]<1497){
            
          setMotor(2,  1   , map(datafromV7RC[2], 1000, 1500, 255, 0) ) ;  

        }else{

          setMotor(2, 0  , 0) ;  

        }
         
 

       
  
      } else
      { // 進入Fail Safe ...

       // SetServoPos(PWM_CH_0, map(faliSafefromV7RC[0], 1000, 2000, 0, 180)); //PWM #1  =ESP32.  PIN 2
       // SetServoPos(PWM_CH_1, map(faliSafefromV7RC[1], 1000, 2000, 0, 180)); //PWM #2  =ESP32.  PIN 12
        SetServoPos(PWM_CH_2, map(faliSafefromV7RC[2], 1000, 2000, 0, 180)); //PWM #3  =ESP32.  PIN 14
        setMotor(1, 0   , 0 ) ;  
        setMotor(2, 0   ,  0 ) ;  
                

        //SetServoPos(PWM_CH_3, map(faliSafefromV7RC[3], 1000, 2000, 0, 180)); //PWM #4  =ESP32.  PIN 13

        if (mainLoop_count % 100 == 0)
          Serial.println("Fail_Safe_");
      }



    }
  }
}
