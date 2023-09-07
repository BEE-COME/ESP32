#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
// #include <SPIFFS.h>
#include <ESPAsyncWebServer.h>
#include <EEPROM.h>
const char* ssid = "ESP32";
const char* password = "12345678";
const char* uplinkSsid = "1F-RD";
const char* uplinkPassword = "sate1111";
const char *serverIP = "82.156.198.56"; // 替换为目标服务器的IP地址
const int serverPort = 8000;            // 替换为目标服务器的端口号

#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>
#include <WiFiMulti.h>
#include <ESPAsyncWebServer.h>
#include <ESPAsyncWiFiManager.h> 
#include <DNSServer.h>

void ser_read();
int send_server(const char *a);
String client_read(int ms);

AsyncWebServer server(80);
DNSServer dns;

AsyncWiFiManager wifiManager(&server, &dns);

static String data,data_r,cl_data,cl_data_r;  // 用于保存接收到的数据
int data_fg=0;
WiFiClient client;

void setup() {
  Serial.begin(115200);

  // 初始化Wi-Fi管理器,自动连接wifi
wifiManager.autoConnect(ssid);
Serial.println("connected...yeey :)");



}

void loop()
{
    ser_read();
    
        //   const char *a = "hello";
    if (data_fg)
    {
        if (WiFi.status() == WL_CONNECTED)
        {
            // const char *a = "hello";
            send_server(data_r.c_str());
            data_fg = 0;           
        }
        else
        {
            Serial.println("Connection status failed!");
        }
    }
    delay(10);

// 其他代码
}

int send_server(const char *a)
{
    // 建立TCP连接
    
    String tmp_data;
    if (client.connect(serverIP, serverPort))
    {
        // 发送消息
        client.println(a);
        delay(10);

        tmp_data=client_read(500);//读取500ms内的服务器回复信息

        
        if(tmp_data.length())Serial.println(tmp_data.c_str());
        

        // 关闭连接
        client.stop();
        // Serial.println("Disconnected from server!");
    }
    else
    {
        Serial.println("Connection failed!");
    }
    return 0;
}

void ser_read()
{
    if (Serial.available())
    {                           // 检查是否有可用的串口数据
        char c = Serial.read(); // 读取一个字符

        if (c == '\n')
        { // 判断是否接收到完整的一行数据
            // 处理接收到的完整一行数据
            //   Serial.print("Received data: ");
            Serial.println(data);
            data_r=data;
            data_fg=1;
            data = ""; // 清空数据，准备接收下一行数据
            delay(10);
        }
        else
        {
            data += c; // 将字符添加到数据字符串中
        }
    }
}

String client_read(int ms)
{
    unsigned long startTime = millis();
    int tmp_fg=0;
    while (millis() - startTime < ms)
    {
        if (client.available())
        {
            // 读取数据
            char c = client.read();
            if (c == '\n')
            { // 判断是否接收到完整的一行数据
                // 处理接收到的完整一行数据
                //   Serial.print("Received data: ");
                cl_data_r=cl_data;
                tmp_fg=1;
                cl_data = ""; // 清空数据，准备接收下一行数据
                break;
            }
            else
            {
                cl_data += c; // 将字符添加到数据字符串中
            }
        }
    }
    if(tmp_fg)
    {
        return cl_data_r;
    }  
    else
    {
        return "";
    }
}


