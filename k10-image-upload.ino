// Arduino IDE code for Unihiker K10 to post images to http server using php
// This code is adapted for Unihiker K10 from:
// https://randomnerdtutorials.com/esp32-cam-post-image-photo-server
// Read this article for full instructions!

include <Arduino.h>
#include <WiFi.h>
#include "esp_camera.h"

const char* ssid = "xxxxxx";
const char* password = "xxxxxx";

String serverName = "xx.xx.xx.xx";   // REPLACE WITH YOUR Server IP ADDRESS
//String serverName = "example.com";   // OR REPLACE WITH YOUR DOMAIN NAME

String serverPath = "/upload.php";     // The default serverPath should be upload.php

const int serverPort = 80;

WiFiClient client;

// CAMERA_MODEL_UNIHIKER_K10
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      7
#define SIOD_GPIO_NUM     47
#define SIOC_GPIO_NUM     48

#define Y9_GPIO_NUM        6
#define Y8_GPIO_NUM       15
#define Y7_GPIO_NUM       16
#define Y6_GPIO_NUM       18
#define Y5_GPIO_NUM        9
#define Y4_GPIO_NUM       11
#define Y3_GPIO_NUM       10
#define Y2_GPIO_NUM        8
#define VSYNC_GPIO_NUM     4
#define HREF_GPIO_NUM      5
#define PCLK_GPIO_NUM     17

camera_fb_t * fb = NULL; // frame buffer pointer
size_t _jpg_buf_len = 0; // jpeg buffer length
uint8_t * _jpg_buf = NULL; // jpeg buffer pointer
uint32_t _jpg_width = NULL; // frame width
uint32_t _jpg_height = NULL; // frame height

const int timerInterval = 30000;    // time between each HTTP POST image
unsigned long previousMillis = 0;   // last time image was sent

void setup() {
  Serial.begin(115200);
  Serial.println("***Start K10 Uploader***");
  WiFi.mode(WIFI_STA);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);  
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.print("ESP32-CAM IP Address: ");
  Serial.println(WiFi.localIP());

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
  config.xclk_freq_hz = 20000000;
//  config.pixel_format = PIXFORMAT_RGB565;
  config.pixel_format = PIXFORMAT_YUV422;
  // These are framesize values I found for Unihiker K10
  // FRAMESIZE_QVGA => 240x320,
  // FRAMESIZE_SVGA => 640x480,
  //  FRAMESIZE_XGA => 800x600,
  // FRAMESIZE_SXGA => 1280x720,
  // FRAMESIZE_UXGA => 1280x1024
  config.frame_size = FRAMESIZE_XGA;
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_count = 1;
  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    delay(1000);
    ESP.restart();
  }
  // set camera sensor settings 
  sensor_t * s = esp_camera_sensor_get();
  s->set_hmirror(s, 1);  // 0 = disable , 1 = enable       // required to get colors right

  sendPhoto(); 
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= timerInterval) {
    sendPhoto();
    previousMillis = currentMillis;
  }
}

String sendPhoto() {
  String getAll;
  String getBody;
  Serial.println("***sendPhoto***");

// K10 camera doesn't seem to be able to generate jpg format
// so we have to use fmt2jpg to get jpeg format
  fb = esp_camera_fb_get();
  if (fb) {
    // convert to jpg format
    bool jpeg_converted = fmt2jpg(fb->buf, fb->len, fb->width, fb->height,
                fb->format, 32, &_jpg_buf, &_jpg_buf_len);
    _jpg_width=fb->width;
    _jpg_height=fb->height;
    // delete frame buffer
    esp_camera_fb_return(fb);
    fb = NULL;
  }

  Serial.println("Connecting to server: " + serverName);

  if (client.connect(serverName.c_str(), serverPort)) {
    Serial.println("Connection successful!");    
    String head = "--RandomNerdTutorials\r\nContent-Disposition: form-data; name=\"imageFile\"; filename=\"esp32-cam.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
    String tail = "\r\n--RandomNerdTutorials--\r\n";

//    uint32_t imageLen = fb->len;
    uint32_t imageLen = _jpg_buf_len;
    uint32_t extraLen = head.length() + tail.length();
    uint32_t totalLen = imageLen + extraLen;
    Serial.println(imageLen);
    client.println("POST " + serverPath + " HTTP/1.1");
    client.println("Host: " + serverName);
    client.println("Content-Length: " + String(totalLen));
    client.println("Content-Type: multipart/form-data; boundary=RandomNerdTutorials");
    client.println();
    client.print(head);
  
//    uint8_t *fbBuf = fb->buf;
//    size_t fbLen = fb->len;
    uint8_t *fbBuf = _jpg_buf;
    size_t fbLen = _jpg_buf_len;
    for (size_t n=0; n<fbLen; n=n+1024) {
      if (n+1024 < fbLen) {
        client.write(fbBuf, 1024);
        fbBuf += 1024;
      }
      else if (fbLen%1024>0) {
        size_t remainder = fbLen%1024;
        client.write(fbBuf, remainder);
      }
    }   
    client.print(tail);
    
    esp_camera_fb_return(fb);
    
    int timoutTimer = 10000;
    long startTimer = millis();
    boolean state = false;
    
    while ((startTimer + timoutTimer) > millis()) {
      Serial.print(".");
      delay(100);      
      while (client.available()) {
        char c = client.read();
        if (c == '\n') {
          if (getAll.length()==0) { state=true; }
          getAll = "";
        }
        else if (c != '\r') { getAll += String(c); }
        if (state==true) { getBody += String(c); }
        startTimer = millis();
      }
      if (getBody.length()>0) { break; }
    }
    Serial.println();
    client.stop();
    Serial.println(getBody);
  }
  else {
    getBody = "Connection to " + serverName +  " failed.";
    Serial.println(getBody);
  }
  return getBody;
}
