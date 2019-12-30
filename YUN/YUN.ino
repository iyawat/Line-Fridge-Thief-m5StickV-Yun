#include <M5StickC.h>
#include <WiFi.h>
#include <ssl_client.h>
#include <WiFiClientSecure.h>
#include <Adafruit_BMP280.h>
#include "SHT20.h"
#include "yunBoard.h"
#include <math.h>
#include "display.h"

const char* ssid = "xxx";
const char* passwd = "yyy";
const char* host = "notify-api.line.me";
const char* token = "zzz";

SHT20 sht20;
Adafruit_BMP280 bmp;

HardwareSerial serial_ext(2);

// Clock Parameters
uint32_t targetTime = 0;                    // for next 1 second timeout
static uint8_t conv2d(const char* p); // Forward declaration needed for IDE 1.6.x
uint8_t hh = conv2d(__TIME__), mm = conv2d(__TIME__ + 3), ss = conv2d(__TIME__ + 6); // Get H, M, S from compile time
byte omm = 99, oss = 99;
byte xcolon = 0, xsecs = 0;
unsigned int colour = 0;
bool led_on;

// YUN Parameters
uint32_t update_time = 0;
float tmp, hum;
float pressure;
uint16_t light;
extern uint8_t  lightR;
extern uint8_t  lightG;
uint8_t light_flag=0;

// IMU Parameters
float accX = 0.0F;
float accY = 0.0F;
float accZ = 0.0F;

float gyroX = 0.0F;
float gyroY = 0.0F;
float gyroZ = 0.0F;

typedef struct {
  uint32_t length;
  uint8_t *buf;
} jpeg_data_t;

jpeg_data_t jpeg_data;
static const int RX_BUF_SIZE = 20000;
static const uint8_t packet_begin[3] = { 0xFF, 0xD8, 0xEA };

/*
sendLineNotify関数は、以下参照。
https://github.com/anoken/purin_wo_mimamoru_gijutsu/blob/master/2_6_M5Camera_Send_LineNotify/2_6_M5Camera_Send_LineNotify.ino
*/
//Line通知
void sendLineNotify(uint8_t* image_data, size_t image_sz) {
  WiFiClientSecure client;
  if (!client.connect(host, 443))   return;
  int httpCode = 404;
  size_t image_size = image_sz;
  String boundary = "------------";
  String body = "--" + boundary + "\r\n";
  String message = "Fridge Thieve Detected !!!";
  body += "Content-Disposition: form-data; name=\"message\"\r\n\r\n" + message + " \r\n";
  if (image_data != NULL && image_sz > 0 ) {
    image_size = image_sz;
    body += "--" + boundary + "\r\n";
    body += "Content-Disposition: form-data; name=\"imageFile\"; filename=\"image.jpg\"\r\n";
    body += "Content-Type: image/jpeg\r\n\r\n";
  }
  String body_end = "--" + boundary + "--\r\n";
  size_t body_length = body.length() + image_size + body_end.length();
  String header = "POST /api/notify HTTP/1.1\r\n";
  header += "Host: notify-api.line.me\r\n";
  header += "Authorization: Bearer " + String(token) + "\r\n";
  header += "User-Agent: " + String("M5Stack") + "\r\n";
  header += "Connection: close\r\n";
  header += "Cache-Control: no-cache\r\n";
  header += "Content-Length: " + String(body_length) + "\r\n";
  header += "Content-Type: multipart/form-data; boundary=" + boundary + "\r\n\r\n";
  client.print(header + body);
  Serial.print(header + body);

  bool Success_h = false;
  uint8_t line_try = 3;
  while (!Success_h && line_try-- > 0) {
    if (image_size > 0) {
      size_t BUF_SIZE = 1024;
      if ( image_data != NULL) {
        uint8_t *p = image_data;
        size_t sz = image_size;
        while ( p != NULL && sz) {
          if ( sz >= BUF_SIZE) {
            client.write( p, BUF_SIZE);
            p += BUF_SIZE; sz -= BUF_SIZE;
          } else {
            client.write( p, sz);
            p += sz; sz = 0;
          }
        }
      }
      client.print("\r\n" + body_end);
      Serial.print("\r\n" + body_end);

      while ( client.connected() && !client.available()) delay(10);
      if ( client.connected() && client.available() ) {
        String resp = client.readStringUntil('\n');
        httpCode    = resp.substring(resp.indexOf(" ") + 1, resp.indexOf(" ", resp.indexOf(" ") + 1)).toInt();
        Success_h   = (httpCode == 200);
        Serial.println(resp);
      }
      delay(10);
    }
  }
  client.stop();
}

void sendTextLineNotify(String message) {
  WiFiClientSecure client;
  if (!client.connect(host, 443))   return;
  int httpCode = 404;
  String boundary = "------------";
  String body = "--" + boundary + "\r\n";
  body += "Content-Disposition: form-data; name=\"message\"\r\n\r\n" + message + " \r\n";
  String body_end = "--" + boundary + "--\r\n";
  size_t body_length = body.length() + body_end.length();
  String header = "POST /api/notify HTTP/1.1\r\n";
  header += "Host: notify-api.line.me\r\n";
  header += "Authorization: Bearer " + String(token) + "\r\n";
  header += "User-Agent: " + String("M5Stack") + "\r\n";
  header += "Connection: close\r\n";
  header += "Cache-Control: no-cache\r\n";
  header += "Content-Length: " + String(body_length) + "\r\n";
  header += "Content-Type: multipart/form-data; boundary=" + boundary + "\r\n\r\n";
  client.print(header + body);
  Serial.print(header + body);

  bool Success_h = false;
  uint8_t line_try = 3;
  while (!Success_h && line_try-- > 0) {
      client.print("\r\n" + body_end);
      Serial.print("\r\n" + body_end);

      while ( client.connected() && !client.available()) delay(10);
      if ( client.connected() && client.available() ) {
        String resp = client.readStringUntil('\n');
        httpCode    = resp.substring(resp.indexOf(" ") + 1, resp.indexOf(" ", resp.indexOf(" ") + 1)).toInt();
        Success_h   = (httpCode == 200);
        Serial.println(resp);
      }
      delay(10);
    }
  client.stop();
}

void setup() {
  M5.begin();
  M5.IMU.Init();

  int8_t i,j;
  Wire.begin(0, 26, 100000);
  M5.Lcd.setRotation(1);
  
  if (!bmp.begin(0x76)) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1000); /* Standby time. */
   light_flag==0;
   led_on = false;
    
  setup_wifi();

  jpeg_data.buf = (uint8_t *) malloc(sizeof(uint8_t) * RX_BUF_SIZE);
  jpeg_data.length = 0;
  if (jpeg_data.buf == NULL) {
    Serial.println("malloc jpeg buffer 1 error");
  }

  serial_ext.begin(115200, SERIAL_8N1, 32, 33);
}

void loop() {
  M5.update();

  M5.Lcd.setTextSize(2);
  if(millis() > update_time) {
    update_time = millis() + 1000;
    tmp = sht20.read_temperature();
    hum = sht20.read_humidity();
    pressure = bmp.readPressure();
    M5.Lcd.setCursor(2, 20);
    M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
    M5.Lcd.printf("%.2fC\r\n", tmp);
    M5.Lcd.setCursor(2, 40);
    M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
    M5.Lcd.printf("%d", int(hum));
    M5.Lcd.print("%\r\n");
    M5.Lcd.setCursor(2, 60);
    M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
    M5.Lcd.printf("%d Pa\r\n", int(pressure));
    light = light_get();

    // Set next update for 1 second later
    targetTime = millis() + 1000;

    // Adjust the time values by adding 1 second
    ss++;              // Advance second
    if (ss == 60) {    // Check for roll-over
      ss = 0;          // Reset seconds to zero
      omm = mm;        // Save last minute time for display update
      mm++;            // Advance minute
      if (mm > 59) {   // Check for roll-over
        mm = 0;
        hh++;          // Advance hour
        if (hh > 23) { // Check for 24hr roll-over (could roll-over on 13)
          hh = 0;      // 0 for 24 hour clock, set to 1 for 12 hour clock
        }
      }
    }

    // Update digital time
    int xpos = 2;
    int ypos = 2; // Top left corner ot clock text, about half way down
    int ysecs = ypos;
    M5.Lcd.setTextColor(TFT_MAGENTA, TFT_BLACK); 
    if (omm != mm) { // Redraw hours and minutes time every minute
      omm = mm;
      // Draw hours and minutes
      if (hh < 10) xpos += M5.Lcd.drawChar('0', xpos, ypos, 1); // Add hours leading zero for 24 hr clock
      xpos += M5.Lcd.drawNumber(hh, xpos, ypos, 1);             // Draw hours
      xcolon = xpos; // Save colon coord for later to flash on/off later
      xpos += M5.Lcd.drawChar(':', xpos, ypos, 1);
      if (mm < 10) xpos += M5.Lcd.drawChar('0', xpos, ypos, 1); // Add minutes leading zero
      xpos += M5.Lcd.drawNumber(mm, xpos, ypos, 1);             // Draw minutes
      xsecs = xpos; // Sae seconds 'x' position for later display updates
    }
    if (oss != ss) { // Redraw seconds time every second
      oss = ss;
      xpos = xsecs;

      if (ss % 2) { // Flash the colons on/off
        M5.Lcd.setTextColor(0x39C4, TFT_BLACK);        // Set colour to grey to dim colon
        M5.Lcd.drawChar(':', xcolon, ypos, 1);     // Hour:minute colon
        xpos += M5.Lcd.drawChar(':', xsecs, ysecs, 1); // Seconds colon
        M5.Lcd.setTextColor(TFT_MAGENTA, TFT_BLACK);    
      }
      else {
        M5.Lcd.drawChar(':', xcolon, ypos, 1);     // Hour:minute colon
        xpos += M5.Lcd.drawChar(':', xsecs, ysecs, 1); // Seconds colon
        M5.Lcd.setTextColor(TFT_MAGENTA, TFT_BLACK);    
      }

      //Draw seconds
      if (ss < 10) xpos += M5.Lcd.drawChar('0', xpos, ysecs, 1); // Add leading zero
      M5.Lcd.drawNumber(ss, xpos, ysecs, 1);                     // Draw seconds
    }
    
  }
  
  if(M5.BtnA.wasPressed()) {
    esp_restart();  
  }

  delay(10);
  // put your main code here, to run repeatedly:

  if (serial_ext.available()) {
    uint8_t rx_buffer[10];
    int rx_size = serial_ext.readBytes(rx_buffer, 10);
    if (rx_size == 10) {   //packet receive of packet_begin
      if ((rx_buffer[0] == packet_begin[0]) && (rx_buffer[1] == packet_begin[1]) && (rx_buffer[2] == packet_begin[2])) {
        //image size receive of packet_begin
        jpeg_data.length = (uint32_t)(rx_buffer[4] << 16) | (rx_buffer[5] << 8) | rx_buffer[6];
        int rx_size = serial_ext.readBytes(jpeg_data.buf, jpeg_data.length);
        Serial.println("Receiving Data..");
        //image processing, for example, line notify send image
        sendLineNotify(jpeg_data.buf, jpeg_data.length);
        //image processing end
        led_on = true;
      }
    }
  }
  
  if (led_on) {
    display_light();
    if(light>1500)
      {
        if(light_flag==0)
        {
          light_flag=1;
           lightR=40;
           lightG=40;
          
        }
        led_breath();
      }
      else 
      {
        led_off();
        led_on = false;
        light_flag=0;
      }
  }

    M5.Lcd.setTextSize(1);
  M5.IMU.getGyroData(&gyroX,&gyroY,&gyroZ);
  M5.IMU.getAccelData(&accX,&accY,&accZ);
//  M5.Lcd.setCursor(2, 60);
//  M5.Lcd.printf("%6.2f  %6.2f  %6.2f", gyroX, gyroY, gyroZ);
//  M5.Lcd.setCursor(2, 70);
//  M5.Lcd.printf("%5.2f  %5.2f  %5.2f", accX, accY, accZ);

  if (gyroX > 40) {
    String message = "Fridge Opened !! ";
    sendTextLineNotify(message);
    message = "";
  } 
  if (gyroX < -40) {
    String message = "Fridge Closed !! ";
    sendTextLineNotify(message);
     message = "";
  } 

  vTaskDelay(10 / portTICK_RATE_MS);
}

void setup_wifi() {
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, passwd);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

// Function to extract numbers from compile time string
static uint8_t conv2d(const char* p) {
  uint8_t v = 0;
  if ('0' <= *p && *p <= '9')
    v = *p - '0';
  return 10 * v + *++p - '0';
}
