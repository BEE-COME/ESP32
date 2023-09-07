#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <ESPAsyncWiFiManager.h> 
#include <DNSServer.h>

#define led 2
//pwmµÄled¿ØÖÆ
const int pwmChannel = 0;
const int pwmPin = led;
const int pwmFreq = 5000;
const int pwmResolution = 8;

void setup() {
  ledcSetup(pwmChannel, pwmFreq, pwmResolution);
  ledcAttachPin(pwmPin, pwmChannel);
}

void loop() {
  for (int brightness = 0; brightness <= 255; brightness++) {
    ledcWrite(pwmChannel, brightness);
    delay(5);
  }
  for (int brightness = 255; brightness >= 0; brightness--) {
    ledcWrite(pwmChannel, brightness);
    delay(5);
  }
}
