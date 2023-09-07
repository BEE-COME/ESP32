#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <ESPAsyncWiFiManager.h> 
#include <DNSServer.h>
#include <Wire.h>

// ���ٶȼƺ������ǵ���������
float accelScaleFactor = 16384.0; // ���������̷�Χѡ���Ӧֵ
float gyroScaleFactor = 131.0; // ���������̷�Χѡ���Ӧֵ

// �������˲����Ĳ���
float Q_angle = 0.001;
float Q_gyro = 0.003;
float R_angle = 0.03;

// �������˲�����״̬����
float angle = 0.0;
float bias = 0.0;
float angle2 = 0.0;
float bias2 = 0.0;
float P1[2][2] = {{0.0, 0.0}, {0.0, 0.0}};
float P2[2][2] = {{0.0, 0.0}, {0.0, 0.0}};
unsigned long startTime;

const int MPU6050_ADDR = 0x68; // MPU6050��I2C��ַ

float accel_x_filtered, accel_y_filtered, accel_z_filtered;
float gyro_x_filtered, gyro_y_filtered, gyro_z_filtered;
const char* uplinkSsid = "1F-RD";
const char* uplinkPassword = "sate1111";
const char *serverIP = "82.156.198.56"; // �滻ΪĿ���������IP��ַ
const int serverPort = 800;            // �滻ΪĿ��������Ķ˿ں�
// const char *serverIP = "192.168.1.102"; // �滻ΪĿ���������IP��ַ
// const int serverPort = 777;            // �滻ΪĿ��������Ķ˿ں�

#include <ArduinoJson.h>
AsyncWebServer server(80);
DNSServer dns;
AsyncWiFiManager wifiManager(&server, &dns);
DynamicJsonDocument json(1024);


// #include <FreeRTOS.h>
// #include <task.h>

// #define led 2
// //pwm��led����
// const int pwmChannel = 0;
// const int pwmPin = led;
// const int pwmFreq = 5000;
// const int pwmResolution = 8;

// void ledTask(void *pvParameters) {
//   int pwmChannel = 0;

//   for (int brightness = 0; brightness <= 255; brightness++) {
//     ledcWrite(pwmChannel, brightness);
//     vTaskDelay(pdMS_TO_TICKS(5)); // �ӳ�5����
//   }

//   for (int brightness = 255; brightness >= 0; brightness--) {
//     ledcWrite(pwmChannel, brightness);
//     vTaskDelay(pdMS_TO_TICKS(5)); // �ӳ�5����
//   }

//   vTaskDelete(NULL); // ɾ������
// }

void setup() {
  Wire.begin(); // ��ʼ��I2C����
  Serial.begin(115200); // ��ʼ������ͨ��
  delay(2000); // �ȴ�2���ӣ��Ա����ӵ����ڼ�����
    // ��ʼ��Wi-Fi������,�Զ�����wifi
  // wifiManager.autoConnect();
  // Serial.println("connected...yeey :)");
  // ledcSetup(pwmChannel, pwmFreq, pwmResolution);
  // ledcAttachPin(pwmPin, pwmChannel);

  // ��ʼ��MPU6050
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1�Ĵ�����ַ
  Wire.write(0); // ���Ĵ���ֵ����Ϊ0������MPU6050
  Wire.endTransmission();

  // Wire.beginTransmission(MPU6050_ADDR);
  // Wire.write(0x19); // PWR_MGMT_1�Ĵ�����ַ
  // Wire.write(0x13); // ���Ĵ���ֵ����Ϊ0������MPU6050
  // Wire.endTransmission();
  startTime = millis();
}

void kalmanFilter0(float newAngle, float newRate, float dt, int axis) {
  // ���¿������˲�����״̬
  angle += dt * (newRate - bias);

  float Pdot[4] = {
    Q_angle - P1[0][1] - P1[1][0],
    -P1[1][1],
    -P1[1][1],
    Q_gyro
  };

  P1[0][0] += dt * Pdot[0];
  P1[0][1] += dt * Pdot[1];
  P1[1][0] += dt * Pdot[2];
  P1[1][1] += dt * Pdot[3];

  // ���㿨��������
  float S = P1[0][0] + R_angle;
  float K[2] = {
    P1[0][0] / S,
    P1[0][1] / S
  };

  // ���½ǶȺ�ƫ��
  float y = newAngle - angle;
  angle += K[0] * y;
  bias += K[1] * y;

  // ����Э�������
  float P00_temp = P1[0][0];
  float P01_temp = P1[0][1];

  P1[0][0] -= K[0] * P00_temp;
  P1[0][1] -= K[0] * P01_temp;
  P1[1][0] -= K[1] * P00_temp;
  P1[1][1] -= K[1] * P01_temp;
}

void kalmanFilter00(float newAngle, float newRate, float dt, int axis) {
  // ���¿������˲�����״̬
  angle2 += dt * (newRate - bias2);

  float Pdot[4] = {
    Q_angle - P2[0][1] - P2[1][0],
    -P2[1][1],
    -P2[1][1],
    Q_gyro
  };

  P2[0][0] += dt * Pdot[0];
  P2[0][1] += dt * Pdot[1];
  P2[1][0] += dt * Pdot[2];
  P2[1][1] += dt * Pdot[3];

  // ���㿨��������
  float S = P2[0][0] + R_angle;
  float K[2] = {
    P2[0][0] / S,
    P2[0][1] / S
  };

  // ���½ǶȺ�ƫ��
  float y = newAngle - angle2;
  angle2 += K[0] * y;
  bias2 += K[1] * y;

  // ����Э�������
  float P00_temp = P2[0][0];
  float P01_temp = P2[0][1];

  P2[0][0] -= K[0] * P00_temp;
  P2[0][1] -= K[0] * P01_temp;
  P2[1][0] -= K[1] * P00_temp;
  P2[1][1] -= K[1] * P01_temp;
}

WiFiClient client;

int send_server(const char *a)
{
    // ����TCP����
    
    String tmp_data;
    if (client.connect(serverIP, serverPort))
    {
        // ������Ϣ
        client.println(a);
        // delay(10);

        // tmp_data=client_read(500);//��ȡ500ms�ڵķ������ظ���Ϣ

        
        // if(tmp_data.length())Serial.println(tmp_data.c_str());
        

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



void loop() {
  // ��ȡ���ٶȼƺ�����������
  // for (int brightness = 0; brightness <= 255; brightness++) {
  //   ledcWrite(pwmChannel, brightness);
  //   delay(5);
  // }
  // for (int brightness = 255; brightness >= 0; brightness--) {
  //   ledcWrite(pwmChannel, brightness);
  //   delay(5);
  // }

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B); // ACCEL_XOUT_H�Ĵ�����ַ
  Wire.endTransmission(false); // �������������ź�

  Wire.requestFrom(MPU6050_ADDR, 14); // �����MPU6050��ȡ14���ֽڵ�����

  int16_t accel_x = Wire.read() << 8 | Wire.read();
  int16_t accel_y = Wire.read() << 8 | Wire.read();
  int16_t accel_z = Wire.read() << 8 | Wire.read();
  int16_t temp = Wire.read() << 8 | Wire.read();
  int16_t gyro_x = Wire.read() << 8 | Wire.read();
  int16_t gyro_y = Wire.read() << 8 | Wire.read();
  int16_t gyro_z = Wire.read() << 8 | Wire.read();

  // accel_x_filtered = kalmanFilter.updateEstimate(accel_x);
  // accel_y_filtered = kalmanFilter.updateEstimate(accel_y);
  // accel_z_filtered = kalmanFilter.updateEstimate(accel_z);
  // gyro_x_filtered = kalmanFilter.updateEstimate(gyro_x);
  // gyro_y_filtered = kalmanFilter.updateEstimate(gyro_y);
  // gyro_z_filtered = kalmanFilter.updateEstimate(gyro_z);

    // ������ٶȼƵĽǶ�
  float accelAngleX = atan2(accel_y, accel_z) * (180.0 / PI);
  float accelAngleY = atan2(-accel_x, accel_z) * (180.0 / PI);

  // ���������ǵĽ��ٶ�
  float gyroRateX = gyro_x / gyroScaleFactor;
  float gyroRateY = gyro_y / gyroScaleFactor;

  // ʹ�ÿ������˲���������̬����
  float dt = 0.01; // ����ʱ��������λΪ��
  kalmanFilter0(accelAngleX, gyroRateX, dt,0);
  float pitch = angle;

  kalmanFilter00(accelAngleY, gyroRateY, dt,1);
  float roll = angle2;

  //   float pitch = atan2(accel_y, sqrt(accel_x * accel_x + accel_z * accel_z)) * 180 / PI;
  //   float roll = atan2(-accel_x, accel_z) * 180 / PI;
  float yaw = atan2(gyro_y, gyro_x) * 180 / PI;
  // pitch=kalmanFilter.updateEstimate(pitch);
  // roll=kalmanFilter1.updateEstimate(roll);
  //   yaw=kalmanFilter2.updateEstimate(yaw);

  // float pitch = atan2(accel_y_filtered, sqrt(pow(accel_x_filtered, 2) + pow(accel_z_filtered, 2)));
  // float roll = atan2(-accel_x_filtered, accel_z_filtered);
  // float yaw = atan2(-gyro_y_filtered, sqrt(pow(gyro_x_filtered, 2) + pow(gyro_z_filtered, 2)));

  // ������ת��Ϊ����
  // pitch = pitch * 180.0 / PI;
  // roll = roll * 180.0 / PI;
  // yaw = yaw * 180.0 / PI;



    // Ӧ��ƫ����У׼
  // accel_x -= accel_offset_x;
  // accel_y -= accel_offset_y;
  // accel_z -= accel_offset_z;
  // gyro_x -= gyro_offset_x;
  // gyro_y -= gyro_offset_y;
  // gyro_z -= gyro_offset_z;

  // ���㸩���ǡ�����Ǻͺ����
  



  delay(10); // ����50ms

  // ��ӡ��̬��Ϣ
  
  if (millis() - startTime > 1000)//500ms��ӡ
  {
    startTime = millis();
      // ��ӡ����
  //     Serial.println(P[0][0] );
  //     Serial.println(P[0][1] );
  //     Serial.println(P[1][0] );
  //     Serial.println(P[1][1] );
  // Serial.print("AX: ");
  // Serial.println(accel_x);
  // Serial.print("AY: ");
  // Serial.println(accel_y);
  // Serial.print("AZ: ");
  // Serial.println(accel_z);
  // Serial.print("Temp: ");
  // Serial.println(temp / 340.00 + 36.53); // ����MPU6050���¶ȼ��㹫ʽת��Ϊ���϶�
  // Serial.print("GX: ");
  // Serial.println(gyro_x);
  // Serial.print("GY: ");
  // Serial.println(gyro_y);
  // Serial.print("GZ: ");
  // Serial.println(gyro_z);

    Serial.print("Pitch: ");
    Serial.println(pitch);
    Serial.print("Roll: ");
    Serial.println(roll);
    Serial.print("Yaw: ");
    Serial.println(yaw);
    
    json["Pitch"] = pitch;
    json["Roll"] = roll;
    json["Yaw"] = yaw;

  // ��JSON����ת��Ϊ�ַ���
  String jsonString;
  serializeJson(json, jsonString);

    

  //   if (WiFi.status() == WL_CONNECTED)
  //   {
  //       // String tmp_data;
  //     if (client.connect(serverIP, serverPort))
  //     {
  //         // ������Ϣ
  //         client.print(jsonString);
  //         // �ر�����
  //         client.stop();
  //     }    
  //   }
  }

}
