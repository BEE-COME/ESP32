#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <ESPAsyncWiFiManager.h> 
#include <DNSServer.h>
// #include "driver/timer.h"
#include <Wire.h>


#include <U8g2lib.h>

#include <NimBLEDevice.h>

static NimBLEClient* pClient;
static NimBLERemoteCharacteristic* pCharacteristic;
static NimBLERemoteService* pService;
static NimBLERemoteCharacteristic* pCh_send;
static boolean doConnect = false;

char* rec_data;
boolean rec_flag=false;

// void notifyCB(BLERemoteCharacteristic* pRemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify){
//     std::string str = (isNotify == true) ? "Notification" : "Indication";
//     str += " from ";
//     /** NimBLEAddress and NimBLEUUID have std::string operators */
//     str += std::string(pRemoteCharacteristic->getRemoteService()->getClient()->getPeerAddress());
//     str += ": Service = " + std::string(pRemoteCharacteristic->getRemoteService()->getUUID());
//     str += ", Characteristic = " + std::string(pRemoteCharacteristic->getUUID());
//     str += ", Value = " + std::string((char*)pData, length);
//     Serial.println(str.c_str());
// }

static void notifyCallback(
  NimBLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
    // Serial.print("Notify callback for characteristic ");
    // Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
    // Serial.print(" of data length ");
    // Serial.println(length);
    // Serial.print("data: ");
    Serial.println((char*)pData);
}

class  MyClientCallbacks : public NimBLEClientCallbacks {
    void onConnect(NimBLEClient* pClient) {
        doConnect=true;
        Serial.println("Connected to server");
        // NimBLERemoteService *pService= pClient->getService(serviceUuid);
        // NimBLERemoteCharacteristic *pCharacteristic = pService->getCharacteristic("F2FF");
        // 获取远程服务
        pService = pClient->getService(NimBLEUUID("F0FF"));
        if (pService == nullptr) {
            Serial.println("NO1");
            return;
        }

        // 获取远程特征
        pCharacteristic = pService->getCharacteristic(NimBLEUUID("F2FF"));
        if (pCharacteristic == nullptr) {
            Serial.println("NO2");
            return;
        }
        pCh_send= pService->getCharacteristic(NimBLEUUID("F1FF"));
        if (pCharacteristic == nullptr) {
            Serial.println("NO3");
            return;
        }

        

        if (pCharacteristic->canRead())
        {
            Serial.print(pCharacteristic->getUUID().toString().c_str());
            Serial.print(" canRead");
            Serial.println(pCharacteristic->readValue().c_str());
        }

        // if (pCharacteristic->canNotify())
        // {
        //     Serial.print(" canNotify ");
        //     pCharacteristic->subscribe(true, notifyCB);
        // }

        if(pCharacteristic->canNotify())
        {
            pCharacteristic->registerForNotify(notifyCallback);
        }
      
        if (pCh_send->canWriteNoResponse())
        {
            // Serial.print(" canWrite");
            // if (pCh_send->writeValue("No tip!"))
            // {
            //     // Serial.print("Wrote new value to: ");
            //     // Serial.println(pCharacteristic->getUUID().toString().c_str());
            // }
        }

        if (pCharacteristic->canWriteNoResponse())
        {
            Serial.print(" canWrite");
            if (pCharacteristic->writeValue("No tip!"))
            {
                Serial.print("Wrote new value to: ");
                Serial.println(pCharacteristic->getUUID().toString().c_str());
            }
        }
        // NimBLERemoteDescriptor* pDsc = pCharacteristic->getDescriptor(NimBLEUUID("2902"));
        // if(pDsc) {   /** make sure it's not null */
        //     Serial.print("Descriptor: ");
        //     Serial.print(pDsc->getUUID().toString().c_str());
        //     Serial.print(" Value: ");
        //     Serial.println(pDsc->readValue().c_str());
        // }
    }  

    void onDisconnect(NimBLEClient* pClient) {
        doConnect=false;
        Serial.println("Disconnected from server");
    }
};

void onReceiveFunction(void) {
  // This is a callback function that will be activated on UART RX events
  size_t available = Serial.available();
//   received_bytes += available;
//   Serial1.printf("onReceive Callback:: There are %d bytes available: ", available);
  rec_data="";
  while (available --) {
    // Serial.print((char)Serial.read());
    rec_data+=(char)Serial.read();
  }
  rec_flag=true;

//   Serial1.println();
}

String read_outtime(unsigned long  timeout)
{
    // unsigned long timeout ; // 读取超时时间，单位为毫秒
    String data = "";
    if (Serial.available()) 
    {
        unsigned long startTime = millis(); // 记录开始时间
        
        // 读取数据直到超时或没有更多数据可读
        while (millis() - startTime < timeout) 
        {
            if (Serial.available()) 
            {
                char c = Serial.read();
                data += c;
                // delay(1); // 等待一段时间以允许更多数据到达
                startTime = millis();
            }                
        }     
    }
    return (data); // 将数据发送到软串口
}

void setup() {
    Serial.begin(115200);
    NimBLEDevice::init("ESP32");
    pClient = NimBLEDevice::createClient();
    pClient->setClientCallbacks(new MyClientCallbacks());

    NimBLEAddress serverAddress("A9:1B:FE:06:22:C0"); // 替换为你的服务端的蓝牙地址
    pClient->connect(serverAddress);
}

void loop() {
    if(doConnect)
    {
        if (pCh_send->canWriteNoResponse())
        {
            // if(rec_flag)
            // {
            //     pCh_send->writeValue(rec_data);
            //     rec_flag=false;
            // }
            String td=read_outtime(50);
            if(!td.isEmpty())pCh_send->writeValue(td); // 将数据发送到软串口

            // if (Serial.available()) 
            // {
            //     unsigned long startTime = millis(); // 记录开始时间
            //     String data = "";
            //     // 读取数据直到超时或没有更多数据可读
            //     while (millis() - startTime < timeout) 
            //     {
            //         if (Serial.available()) 
            //         {
            //             char c = Serial.read();
            //             data += c;
            //             // delay(1); // 等待一段时间以允许更多数据到达
            //             startTime = millis();
            //         }                
            //     }
            //     // 发送数据到软串口
            //     pCh_send->writeValue(data); // 将数据发送到软串口
            // }
            
            // Serial.print(" canWrite");
            // String newValue = "Time since boot: " + String(millis()/1000);
            // Serial.println("Setting new characteristic value to \"" + newValue + "\"");
            // if (pCh_send->writeValue(newValue.c_str(), newValue.length()))
            // {
            //     Serial.print("Wrote new value to: ");
            // }
        }    
        
        // Set the characteristic's value to be the array of bytes that is actually a string.
        /*** Note: write / read value now returns true if successful, false otherwise - try again or disconnect ***/
        
    }
    // Do nothing
    //  delay(1000); // Delay a second between loops.
}
