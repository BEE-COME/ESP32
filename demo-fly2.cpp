#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <ESPAsyncWiFiManager.h> 
#include <DNSServer.h>
#include <Wire.h>

// 加速度计和陀螺仪的缩放因子
float accelScaleFactor = 16384.0; // 根据满量程范围选择对应值
float gyroScaleFactor = 131.0; // 根据满量程范围选择对应值

// 卡尔曼滤波器的参数
float Q_angle = 0.001;
float Q_gyro = 0.003;
float R_angle = 0.03;

// 卡尔曼滤波器的状态变量
float angle = 0.0;
float bias = 0.0;
float angle2 = 0.0;
float bias2 = 0.0;
float P1[2][2] = {{0.0, 0.0}, {0.0, 0.0}};
float P2[2][2] = {{0.0, 0.0}, {0.0, 0.0}};
unsigned long startTime;

const int MPU6050_ADDR = 0x68; // MPU6050的I2C地址

float accel_x_filtered, accel_y_filtered, accel_z_filtered;
float gyro_x_filtered, gyro_y_filtered, gyro_z_filtered;
const char* uplinkSsid = "1F-RD";
const char* uplinkPassword = "sate1111";
const char *serverIP = "82.156.198.56"; // 替换为目标服务器的IP地址
const int serverPort = 800;            // 替换为目标服务器的端口号
// const char *serverIP = "192.168.1.102"; // 替换为目标服务器的IP地址
// const int serverPort = 777;            // 替换为目标服务器的端口号

#include <ArduinoJson.h>
AsyncWebServer server(80);
DNSServer dns;
AsyncWiFiManager wifiManager(&server, &dns);
DynamicJsonDocument json(1024);


// #include <FreeRTOS.h>
// #include <task.h>

// #define led 2
// //pwm的led控制
// const int pwmChannel = 0;
// const int pwmPin = led;
// const int pwmFreq = 5000;
// const int pwmResolution = 8;

// void ledTask(void *pvParameters) {
//   int pwmChannel = 0;

//   for (int brightness = 0; brightness <= 255; brightness++) {
//     ledcWrite(pwmChannel, brightness);
//     vTaskDelay(pdMS_TO_TICKS(5)); // 延迟5毫秒
//   }

//   for (int brightness = 255; brightness >= 0; brightness--) {
//     ledcWrite(pwmChannel, brightness);
//     vTaskDelay(pdMS_TO_TICKS(5)); // 延迟5毫秒
//   }

//   vTaskDelete(NULL); // 删除任务
// }

void setup() {
  Wire.begin(); // 初始化I2C总线
  Serial.begin(115200); // 初始化串口通信
  delay(2000); // 等待2秒钟，以便连接到串口监视器
    // 初始化Wi-Fi管理器,自动连接wifi
  // wifiManager.autoConnect();
  // Serial.println("connected...yeey :)");
  // ledcSetup(pwmChannel, pwmFreq, pwmResolution);
  // ledcAttachPin(pwmPin, pwmChannel);

  // 初始化MPU6050
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1寄存器地址
  Wire.write(0); // 将寄存器值设置为0，唤醒MPU6050
  Wire.endTransmission();

  // Wire.beginTransmission(MPU6050_ADDR);
  // Wire.write(0x19); // PWR_MGMT_1寄存器地址
  // Wire.write(0x13); // 将寄存器值设置为0，唤醒MPU6050
  // Wire.endTransmission();
  startTime = millis();
}

void kalmanFilter0(float newAngle, float newRate, float dt, int axis) {
  // 更新卡尔曼滤波器的状态
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

  // 计算卡尔曼增益
  float S = P1[0][0] + R_angle;
  float K[2] = {
    P1[0][0] / S,
    P1[0][1] / S
  };

  // 更新角度和偏差
  float y = newAngle - angle;
  angle += K[0] * y;
  bias += K[1] * y;

  // 更新协方差矩阵
  float P00_temp = P1[0][0];
  float P01_temp = P1[0][1];

  P1[0][0] -= K[0] * P00_temp;
  P1[0][1] -= K[0] * P01_temp;
  P1[1][0] -= K[1] * P00_temp;
  P1[1][1] -= K[1] * P01_temp;
}

void kalmanFilter00(float newAngle, float newRate, float dt, int axis) {
  // 更新卡尔曼滤波器的状态
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

  // 计算卡尔曼增益
  float S = P2[0][0] + R_angle;
  float K[2] = {
    P2[0][0] / S,
    P2[0][1] / S
  };

  // 更新角度和偏差
  float y = newAngle - angle2;
  angle2 += K[0] * y;
  bias2 += K[1] * y;

  // 更新协方差矩阵
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
    // 建立TCP连接
    
    String tmp_data;
    if (client.connect(serverIP, serverPort))
    {
        // 发送消息
        client.println(a);
        // delay(10);

        // tmp_data=client_read(500);//读取500ms内的服务器回复信息

        
        // if(tmp_data.length())Serial.println(tmp_data.c_str());
        

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



void loop() {
  // 读取加速度计和陀螺仪数据
  // for (int brightness = 0; brightness <= 255; brightness++) {
  //   ledcWrite(pwmChannel, brightness);
  //   delay(5);
  // }
  // for (int brightness = 255; brightness >= 0; brightness--) {
  //   ledcWrite(pwmChannel, brightness);
  //   delay(5);
  // }

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B); // ACCEL_XOUT_H寄存器地址
  Wire.endTransmission(false); // 发送重启传输信号

  Wire.requestFrom(MPU6050_ADDR, 14); // 请求从MPU6050读取14个字节的数据

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

    // 计算加速度计的角度
  float accelAngleX = atan2(accel_y, accel_z) * (180.0 / PI);
  float accelAngleY = atan2(-accel_x, accel_z) * (180.0 / PI);

  // 计算陀螺仪的角速度
  float gyroRateX = gyro_x / gyroScaleFactor;
  float gyroRateY = gyro_y / gyroScaleFactor;

  // 使用卡尔曼滤波器进行姿态估计
  float dt = 0.01; // 采样时间间隔，单位为秒
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

  // 将弧度转换为度数
  // pitch = pitch * 180.0 / PI;
  // roll = roll * 180.0 / PI;
  // yaw = yaw * 180.0 / PI;



    // 应用偏移量校准
  // accel_x -= accel_offset_x;
  // accel_y -= accel_offset_y;
  // accel_z -= accel_offset_z;
  // gyro_x -= gyro_offset_x;
  // gyro_y -= gyro_offset_y;
  // gyro_z -= gyro_offset_z;

  // 计算俯仰角、横滚角和航向角
  



  delay(10); // 采样50ms

  // 打印姿态信息
  
  if (millis() - startTime > 1000)//500ms打印
  {
    startTime = millis();
      // 打印数据
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
  // Serial.println(temp / 340.00 + 36.53); // 根据MPU6050的温度计算公式转换为摄氏度
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

  // 将JSON对象转换为字符串
  String jsonString;
  serializeJson(json, jsonString);

    

  //   if (WiFi.status() == WL_CONNECTED)
  //   {
  //       // String tmp_data;
  //     if (client.connect(serverIP, serverPort))
  //     {
  //         // 发送消息
  //         client.print(jsonString);
  //         // 关闭连接
  //         client.stop();
  //     }    
  //   }
  }

}
