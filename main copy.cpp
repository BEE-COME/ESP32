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

// const char* ssid = "5G";     // 替换为您的WiFi网络名称
// const char* password = "12345678"; // 替换为您的WiFi密码
const char *serverIP = "82.156.198.56"; // 替换为目标服务器的IP地址
const int serverPort = 8000;            // 替换为目标服务器的端口号
// int send_server(const char* a);

// #define SERVICE_UUID        "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
// #define CHARACTERISTIC_UUID "6e400002-b5a3-f393-e0a9-e50e24dcca9e"

#define TARGET_SERVICE_UUID "6e400001-b5a3-f393-e0a9-e50e24dcca9e"

// 自定义服务和特征UUID
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

const int ssidAddr = 4;      // WiFi名称的存储地址
const int passwordAddr = 36; // WiFi密码的存储地址
u16_t eepromSize=512;
const char *ssid1 = "ESP32Hotspot";
const char *password1 = "12345678";
const u16_t Fg = 0x5AA5;

IPAddress ip(192, 168, 1, 1); // 设置固定的IP地址
IPAddress subnet(255, 255, 255, 0);

AsyncWebServer server(80);

void saveWiFiCredentials(const char *ssid, const char *password);
void loadWiFiCredentials(char *ssid, char *password);
String getWebPage();
int send_server(const char *a);

#define WIFI_EEPROM_START_ADDRESS 0

void setup()
{
    // 加载WiFi凭据
    char ssid[32];
    char password[32];

    // 初始化串口通信
    Serial.begin(115200);
    delay(2000);

    // 初始化EEPROM
    EEPROM.begin(512);

    // 读取EEPROM中的标志符
    char flag[2];
    for (int i = 0; i < 2; i++)
    {
        flag[i] = EEPROM.read(WIFI_EEPROM_START_ADDRESS + i);
    }    
    EEPROM.end();    // 结束EEPROM

    // 检查标志符是否存在

    u16_t t_a = (flag[0] << 8 | flag[1]);
    if (t_a == Fg)
    {        
        loadWiFiCredentials(ssid, password);
        Serial.println("已找到WiFi密码");
        // Serial.println(ssid);
        Serial.println(String(ssid));
        Serial.println(String(password));
        // 连接到WiFi网络
        WiFi.begin(ssid, password);
    }
    else
    {
        Serial.println("不存在已保存的Wi-Fi账户和密码");
        // 进行其他操作
        // ...        
        u16_t tmp = (flag[0] << 8 | flag[1]);
        Serial.print(String(tmp, HEX));       

        Serial.println("未找到WiFi密码");

        // 创建热点WiFi
        WiFi.softAP(ssid1, password1);
        WiFi.softAPConfig(ip, ip, subnet);

        Serial.print("热点WiFi名称：");
        Serial.println(ssid1);
        Serial.print("热点WiFi密码：");
        Serial.println(password1);

        // 获取热点WiFi的IP地址
        IPAddress ip = WiFi.softAPIP();
        Serial.print("热点WiFi IP地址：");
        Serial.println(ip);

        // 设置Web服务器路由和处理程序
        server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
                  { request->send(200, "text/html", getWebPage()); });

        server.on("/config", HTTP_POST, [](AsyncWebServerRequest *request)
                  {        

    String ssid2 = request->arg("ssid");
    String password2 = request->arg("password");
    // 将WiFi名称和密码存储在变量中
    saveWiFiCredentials(ssid2.c_str(),password2.c_str());

    EEPROM.begin(512);
    char flag[2];
    flag[0] = u8_t(Fg >> 8);
    flag[1] = u8_t(Fg);
    for (int i = 0; i < 2; i++)
    {
        EEPROM.write(WIFI_EEPROM_START_ADDRESS + i, flag[i]);
    }
    EEPROM.commit(); // 提交更改
    EEPROM.end();    // 结束EEPROM

    // 连接到WiFi网络
    WiFi.begin(ssid2.c_str(), password2.c_str());       

    request->send(200, "text/plain", "WiFi配置成功！"); 
    server.end();
    });

        // 启动Web服务器
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
    


// 结束EEPROM
EEPROM.end();
}

void loop()
{
    // 执行其他操作...
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

  // 存储 WiFi 名称
  for (int i = 0; i < strlen(ssid) + 1; i++) {
    EEPROM.write(ssidAddr + i, ssid[i]);
  }
  EEPROM.write(ssidAddr + strlen(ssid), '\0');  // 添加空字符 '\0'

  // 存储 WiFi 密码
  for (int i = 0; i < strlen(password) + 1; i++) {
    EEPROM.write(passwordAddr + i, password[i]);
  }
  EEPROM.write(passwordAddr + strlen(password), '\0');  // 添加空字符 '\0'

  EEPROM.commit();
  EEPROM.end();
}

void loadWiFiCredentials(char* ssid, char* password) {
  EEPROM.begin(eepromSize);

  // 读取 WiFi 名称
  int i = 0;
  char ch = EEPROM.read(ssidAddr + i);
  while (ch != '\0' && i < 32) {
    ssid[i] = ch;
    i++;
    ch = EEPROM.read(ssidAddr + i);
  }
  ssid[i] = '\0';

  // 读取 WiFi 密码
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
    page += "<h1>WiFi配置</h1>";
    page += "<form method='post' action='/config'>";
    page += "WiFi名称：<input type='text' name='ssid'><br>";
    page += "WiFi密码：<input type='password' name='password'><br>";
    page += "<input type='submit' value='提交'>";
    page += "</form>";
    page += "</body></html>";

    return page;
}

// //wifi进程 TCP连接
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
    // 建立TCP连接
    WiFiClient client;
    if (client.connect(serverIP, serverPort))
    {
        Serial.println("Connected to server!");

        // 发送消息
        client.println(a);

        // 关闭连接
        client.stop();
        Serial.println("Disconnected from server!");
    }
    else
    {
        Serial.println("Connection failed!");
    }
    return 0;
}