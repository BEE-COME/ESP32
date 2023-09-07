#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <ESPAsyncWiFiManager.h> 
#include <DNSServer.h>

//用管脚来模拟uart通讯，完成接收和发送工作
//
//存在问题：不能发太长和接收太长的数据，不然后面会错码


#define TX_PIN 23  // 发送方的GPIO管脚
#define RX_PIN 22  // 接收方的GPIO管脚

#define bps 104     //bps间隔 104us
#define hbps bps/2
#define outtime 50  //超时时间50ms
String rec_data;
byte recData[100];
int cnt=0;
void setup() {
  Serial.begin(115200);  // 用于调试输出
  pinMode(TX_PIN, OUTPUT);
  pinMode(RX_PIN, INPUT);
}

void send_str(const char *a)
{
  digitalWrite(TX_PIN, HIGH);  // 发送空闲位（逻辑0）
  delayMicroseconds(bps);     // 根据波特率设置适当的延时
   // 发送数据
  for (int i = 0; i < strlen(a); i++) {
    // byte currentByte =0x05;
    digitalWrite(TX_PIN, LOW);  // 发送起始位（逻辑1）
    delayMicroseconds(bps);      // 根据波特率设置适当的延时
    for (int j = 0; j <8; j++) {
      digitalWrite(TX_PIN, (a[i] >> j) & 1);  // 发送数据位
      delayMicroseconds(bps);                        // 根据波特率设置适当的延时
    }
    digitalWrite(TX_PIN, HIGH);  // 发送停止位（逻辑0）
    delayMicroseconds(bps);     // 根据波特率设置适当的延时
  }
}

void send_by(byte a)
{
  digitalWrite(TX_PIN, HIGH);  // 发送空闲位（逻辑0）
  delayMicroseconds(bps);     // 根据波特率设置适当的延时
   // 发送数据

    byte currentByte = a;
    // byte currentByte =0x05;
    digitalWrite(TX_PIN, LOW);  // 发送起始位（逻辑1）
    delayMicroseconds(bps);      // 根据波特率设置适当的延时
    for (int j = 0; j <8; j++) {
      digitalWrite(TX_PIN, (currentByte >> j) & 1);  // 发送数据位
      delayMicroseconds(bps);                        // 根据波特率设置适当的延时
    }
    digitalWrite(TX_PIN, HIGH);  // 发送停止位（逻辑0）
    delayMicroseconds(bps);     // 根据波特率设置适当的延时
  
}

// 接收by函数
//参数，接收by数据
//返回接收长度
int rec_by_Data(byte* recData)
{
    int len=0;
    while (1)
    {
       // 接收数据   
        while (digitalRead(RX_PIN) == HIGH) {            
            if (len > 0)
            {
                unsigned long startTime = millis();
                while (millis() - startTime < outtime)
                {

                }              
                 return  len;             
            }
        }  // 等待起始位（逻辑1）
        delayMicroseconds(bps);                  // 根据波特率设置适当的延时
        // delayMicroseconds(hbps);
        byte receivedData = 0;
        for (int i = 0; i < 8; i++) {
            receivedData |= (digitalRead(RX_PIN) << i);  // 读取数据位
            delayMicroseconds(bps);                      // 根据波特率设置适当的延时
        }
        while (digitalRead(RX_PIN) == LOW) {}  // 等待停止位（逻辑0）
        delayMicroseconds(bps);                 // 根据波特率设置适当的延时
        recData[len]=receivedData;
        len++; 
    }
}



void loop() {
  // 发送数据
//   String dataToSend = "123456";
    // send_str(dataToSend.c_str());
  
   cnt=rec_by_Data(recData);

    if(cnt>0)
    {
        for (int i = 0; i < cnt; i++)
        {
            send_by(recData[i]);
            // Serial.write(recData[i]);
        }
    }
   

//   delay(1000);  // 等待一段时间后重复发送和接收
}

// 硬件模拟的做法

/*
volatile size_t sent_bytes = 0, received_bytes = 0;

void onReceiveFunction(void) {
  // This is a callback function that will be activated on UART RX events
  size_t available = Serial1.available();
//   received_bytes += available;
//   Serial1.printf("onReceive Callback:: There are %d bytes available: ", available);
  while (available --) {
    Serial1.print((char)Serial1.read());
  }
//   Serial1.println();
}

void setup() {
  Serial.begin(115200);  // 初始化串口通信
  Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);  // 初始化UART通信
  Serial1.setRxTimeout(3);
  Serial1.onReceive(onReceiveFunction, true);
}

void loop() {

}
*/