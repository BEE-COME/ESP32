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
const char *serverIP = "82.156.198.56"; // �滻ΪĿ���������IP��ַ
const int serverPort = 8000;            // �滻ΪĿ��������Ķ˿ں�

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

static String data,data_r,cl_data,cl_data_r;  // ���ڱ�����յ�������
int data_fg=0;
WiFiClient client;

void setup() {
  Serial.begin(115200);

  // ��ʼ��Wi-Fi������,�Զ�����wifi
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

// ��������
}

int send_server(const char *a)
{
    // ����TCP����
    
    String tmp_data;
    if (client.connect(serverIP, serverPort))
    {
        // ������Ϣ
        client.println(a);
        delay(10);

        tmp_data=client_read(500);//��ȡ500ms�ڵķ������ظ���Ϣ

        
        if(tmp_data.length())Serial.println(tmp_data.c_str());
        

        // �ر�����
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
    {                           // ����Ƿ��п��õĴ�������
        char c = Serial.read(); // ��ȡһ���ַ�

        if (c == '\n')
        { // �ж��Ƿ���յ�������һ������
            // ������յ�������һ������
            //   Serial.print("Received data: ");
            Serial.println(data);
            data_r=data;
            data_fg=1;
            data = ""; // ������ݣ�׼��������һ������
            delay(10);
        }
        else
        {
            data += c; // ���ַ���ӵ������ַ�����
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
            // ��ȡ����
            char c = client.read();
            if (c == '\n')
            { // �ж��Ƿ���յ�������һ������
                // ������յ�������һ������
                //   Serial.print("Received data: ");
                cl_data_r=cl_data;
                tmp_fg=1;
                cl_data = ""; // ������ݣ�׼��������һ������
                break;
            }
            else
            {
                cl_data += c; // ���ַ���ӵ������ַ�����
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


