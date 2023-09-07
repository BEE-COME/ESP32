#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <ESPAsyncWiFiManager.h> 
#include <DNSServer.h>



#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <Update.h>

//Í¨¹ýwifi webÔ¶³Ìota
const char* ssid = "1F-RD";

const char* password = "sate1111";

AsyncWebServer server(80);

void setup() {
  Serial.begin(115200);
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  Serial.println(WiFi.localIP());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", "Hello, World!");
  });

  server.on("/update", HTTP_POST, [](AsyncWebServerRequest *request){
    AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", (Update.hasError()) ? "Update failed" : "Update successful");
    response->addHeader("Connection", "close");
    request->send(response);
  }, [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final){
    if (!index) {
      Serial.println("Updating...");
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
        Update.printError(Serial);
      }
    }
    if (Update.write(data, len) != len) {
      Update.printError(Serial);
    }
    if (final) {
      if (Update.end(true)) {
        Serial.println("Update successful");
      } else {
        Update.printError(Serial);
      }
    }
  });

  server.begin();
}

void loop() {
  
}
