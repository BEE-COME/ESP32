#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
// #include <SPIFFS.h>
#include <ESPAsyncWebServer.h>
#include <EEPROM.h>

// #include <BLEDevice.h>
// #include <BLEServer.h>
// #include <BLEUtils.h>
// #include <BLE2902.h>
// #include <BLEScan.h>
// #include <BLEAdvertisedDevice.h>

// const char* ssid = "5G";     // �滻Ϊ����WiFi��������
// const char* password = "12345678"; // �滻Ϊ����WiFi����
const char *serverIP = "82.156.198.56"; // �滻ΪĿ���������IP��ַ
const int serverPort = 8000;            // �滻ΪĿ��������Ķ˿ں�
// int send_server(const char* a);

// #define SERVICE_UUID        "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
// #define CHARACTERISTIC_UUID "6e400002-b5a3-f393-e0a9-e50e24dcca9e"

#define TARGET_SERVICE_UUID "6e400001-b5a3-f393-e0a9-e50e24dcca9e"

// �Զ�����������UUID
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

const int ssidAddr = 4;      // WiFi���ƵĴ洢��ַ
const int passwordAddr = 36; // WiFi����Ĵ洢��ַ
u16_t eepromSize=512;
const char *ssid1 = "ESP32Hotspot";
const char *password1 = "12345678";
const u16_t Fg = 0x5AA5;

IPAddress ip(192, 168, 1, 1); // ���ù̶���IP��ַ
IPAddress subnet(255, 255, 255, 0);

AsyncWebServer server(80);

void saveWiFiCredentials(const char *ssid, const char *password);
void loadWiFiCredentials(char *ssid, char *password);
String getWebPage();
int send_server(const char *a);

#define WIFI_EEPROM_START_ADDRESS 0

void setup()
{
    // ����WiFiƾ��
    char ssid[32];
    char password[32];

    // ��ʼ������ͨ��
    Serial.begin(115200);
    delay(2000);

    // ��ʼ��EEPROM
    EEPROM.begin(512);

    // ��ȡEEPROM�еı�־��
    char flag[2];
    for (int i = 0; i < 2; i++)
    {
        flag[i] = EEPROM.read(WIFI_EEPROM_START_ADDRESS + i);
    }    
    EEPROM.end();    // ����EEPROM

    // ����־���Ƿ����

    u16_t t_a = (flag[0] << 8 | flag[1]);
    if (t_a == Fg)
    {        
        loadWiFiCredentials(ssid, password);
        Serial.println("���ҵ�WiFi����");
        // Serial.println(ssid);
        Serial.println(String(ssid));
        Serial.println(String(password));
        // ���ӵ�WiFi����
        WiFi.begin(ssid, password);
    }
    else
    {
        Serial.println("�������ѱ����Wi-Fi�˻�������");
        // ������������
        // ...        
        u16_t tmp = (flag[0] << 8 | flag[1]);
        Serial.print(String(tmp, HEX));       

        Serial.println("δ�ҵ�WiFi����");

        // �����ȵ�WiFi
        WiFi.softAP(ssid1, password1);
        WiFi.softAPConfig(ip, ip, subnet);

        Serial.print("�ȵ�WiFi���ƣ�");
        Serial.println(ssid1);
        Serial.print("�ȵ�WiFi���룺");
        Serial.println(password1);

        // ��ȡ�ȵ�WiFi��IP��ַ
        IPAddress ip = WiFi.softAPIP();
        Serial.print("�ȵ�WiFi IP��ַ��");
        Serial.println(ip);

        // ����Web������·�ɺʹ������
        server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
                  { request->send(200, "text/html", getWebPage()); });

        server.on("/config", HTTP_POST, [](AsyncWebServerRequest *request)
                  {        

    String ssid2 = request->arg("ssid");
    String password2 = request->arg("password");
    // ��WiFi���ƺ�����洢�ڱ�����
    saveWiFiCredentials(ssid2.c_str(),password2.c_str());

    EEPROM.begin(512);
    char flag[2];
    flag[0] = u8_t(Fg >> 8);
    flag[1] = u8_t(Fg);
    for (int i = 0; i < 2; i++)
    {
        EEPROM.write(WIFI_EEPROM_START_ADDRESS + i, flag[i]);
    }
    EEPROM.commit(); // �ύ����
    EEPROM.end();    // ����EEPROM

    // ���ӵ�WiFi����
    WiFi.begin(ssid2.c_str(), password2.c_str());       

    request->send(200, "text/plain", "WiFi���óɹ���"); 
    server.end();
    });

        // ����Web������
        server.begin();
    }

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    


// ����EEPROM
EEPROM.end();
}

void loop()
{
    // ִ����������...
      if (WiFi.status() == WL_CONNECTED)
      {
          const char *a = "hello";
          send_server(a);
          sleep(5);
      }
}

void saveWiFiCredentials(const char *ssid, const char *password)
{
   EEPROM.begin(eepromSize);

  // �洢 WiFi ����
  for (int i = 0; i < strlen(ssid) + 1; i++) {
    EEPROM.write(ssidAddr + i, ssid[i]);
  }
  EEPROM.write(ssidAddr + strlen(ssid), '\0');  // ��ӿ��ַ� '\0'

  // �洢 WiFi ����
  for (int i = 0; i < strlen(password) + 1; i++) {
    EEPROM.write(passwordAddr + i, password[i]);
  }
  EEPROM.write(passwordAddr + strlen(password), '\0');  // ��ӿ��ַ� '\0'

  EEPROM.commit();
  EEPROM.end();
}

void loadWiFiCredentials(char* ssid, char* password) {
  EEPROM.begin(eepromSize);

  // ��ȡ WiFi ����
  int i = 0;
  char ch = EEPROM.read(ssidAddr + i);
  while (ch != '\0' && i < 32) {
    ssid[i] = ch;
    i++;
    ch = EEPROM.read(ssidAddr + i);
  }
  ssid[i] = '\0';

  // ��ȡ WiFi ����
  i = 0;
  ch = EEPROM.read(passwordAddr + i);
  while (ch != '\0' && i < 32) {
    password[i] = ch;
    i++;
    ch = EEPROM.read(passwordAddr + i);
  }
  password[i] = '\0';

  EEPROM.end();
}

String getWebPage()
{
    String page = "<html><body>";
    page += "<h1>WiFi����</h1>";
    page += "<form method='post' action='/config'>";
    page += "WiFi���ƣ�<input type='text' name='ssid'><br>";
    page += "WiFi���룺<input type='password' name='password'><br>";
    page += "<input type='submit' value='�ύ'>";
    page += "</form>";
    page += "</body></html>";

    return page;
}

// //wifi���� TCP����
// void setup() {
//   delay(5000);
//   Serial.begin(115200);
//   WiFi.begin(ssid, password);

//   while (WiFi.status() != WL_CONNECTED) {
//     delay(1000);
//     Serial.println("Connecting to WiFi...");
//   }

//   Serial.println("Connected to WiFi");
//   Serial.print("IP address: ");
//   Serial.println(WiFi.localIP());
// }

// void loop() {
//   const char *a = "hello";
//   send_server(a);
//   sleep(5);
// }

int send_server(const char *a)
{
    // ����TCP����
    WiFiClient client;
    if (client.connect(serverIP, serverPort))
    {
        Serial.println("Connected to server!");

        // ������Ϣ
        client.println(a);

        // �ر�����
        client.stop();
        Serial.println("Disconnected from server!");
    }
    else
    {
        Serial.println("Connection failed!");
    }
    return 0;
}