#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <ESPAsyncWiFiManager.h> 
#include <DNSServer.h>

//�ùܽ���ģ��uartͨѶ����ɽ��պͷ��͹���
//
//�������⣺���ܷ�̫���ͽ���̫�������ݣ���Ȼ��������


#define TX_PIN 23  // ���ͷ���GPIO�ܽ�
#define RX_PIN 22  // ���շ���GPIO�ܽ�

#define bps 104     //bps��� 104us
#define hbps bps/2
#define outtime 50  //��ʱʱ��50ms
String rec_data;
byte recData[100];
int cnt=0;
void setup() {
  Serial.begin(115200);  // ���ڵ������
  pinMode(TX_PIN, OUTPUT);
  pinMode(RX_PIN, INPUT);
}

void send_str(const char *a)
{
  digitalWrite(TX_PIN, HIGH);  // ���Ϳ���λ���߼�0��
  delayMicroseconds(bps);     // ���ݲ����������ʵ�����ʱ
   // ��������
  for (int i = 0; i < strlen(a); i++) {
    // byte currentByte =0x05;
    digitalWrite(TX_PIN, LOW);  // ������ʼλ���߼�1��
    delayMicroseconds(bps);      // ���ݲ����������ʵ�����ʱ
    for (int j = 0; j <8; j++) {
      digitalWrite(TX_PIN, (a[i] >> j) & 1);  // ��������λ
      delayMicroseconds(bps);                        // ���ݲ����������ʵ�����ʱ
    }
    digitalWrite(TX_PIN, HIGH);  // ����ֹͣλ���߼�0��
    delayMicroseconds(bps);     // ���ݲ����������ʵ�����ʱ
  }
}

void send_by(byte a)
{
  digitalWrite(TX_PIN, HIGH);  // ���Ϳ���λ���߼�0��
  delayMicroseconds(bps);     // ���ݲ����������ʵ�����ʱ
   // ��������

    byte currentByte = a;
    // byte currentByte =0x05;
    digitalWrite(TX_PIN, LOW);  // ������ʼλ���߼�1��
    delayMicroseconds(bps);      // ���ݲ����������ʵ�����ʱ
    for (int j = 0; j <8; j++) {
      digitalWrite(TX_PIN, (currentByte >> j) & 1);  // ��������λ
      delayMicroseconds(bps);                        // ���ݲ����������ʵ�����ʱ
    }
    digitalWrite(TX_PIN, HIGH);  // ����ֹͣλ���߼�0��
    delayMicroseconds(bps);     // ���ݲ����������ʵ�����ʱ
  
}

// ����by����
//����������by����
//���ؽ��ճ���
int rec_by_Data(byte* recData)
{
    int len=0;
    while (1)
    {
       // ��������   
        while (digitalRead(RX_PIN) == HIGH) {            
            if (len > 0)
            {
                unsigned long startTime = millis();
                while (millis() - startTime < outtime)
                {

                }              
                 return  len;             
            }
        }  // �ȴ���ʼλ���߼�1��
        delayMicroseconds(bps);                  // ���ݲ����������ʵ�����ʱ
        // delayMicroseconds(hbps);
        byte receivedData = 0;
        for (int i = 0; i < 8; i++) {
            receivedData |= (digitalRead(RX_PIN) << i);  // ��ȡ����λ
            delayMicroseconds(bps);                      // ���ݲ����������ʵ�����ʱ
        }
        while (digitalRead(RX_PIN) == LOW) {}  // �ȴ�ֹͣλ���߼�0��
        delayMicroseconds(bps);                 // ���ݲ����������ʵ�����ʱ
        recData[len]=receivedData;
        len++; 
    }
}



void loop() {
  // ��������
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
   

//   delay(1000);  // �ȴ�һ��ʱ����ظ����ͺͽ���
}

// Ӳ��ģ�������

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
  Serial.begin(115200);  // ��ʼ������ͨ��
  Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);  // ��ʼ��UARTͨ��
  Serial1.setRxTimeout(3);
  Serial1.onReceive(onReceiveFunction, true);
}

void loop() {

}
*/