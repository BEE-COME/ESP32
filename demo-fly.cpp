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
float P[2][2] = {{0.0, 0.0}, {0.0, 0.0}};
unsigned long startTime;

const int MPU6050_ADDR = 0x68; // MPU6050的I2C地址
#include <SimpleKalmanFilter.h>
SimpleKalmanFilter kalmanFilter(0.001, 0.001,0.001);
SimpleKalmanFilter kalmanFilter1(0.001, 0.001,0.001);
SimpleKalmanFilter kalmanFilter2(0.001, 0.001,0.001);
float accel_x_filtered, accel_y_filtered, accel_z_filtered;
float gyro_x_filtered, gyro_y_filtered, gyro_z_filtered;

void setup() {
  Wire.begin(); // 初始化I2C总线
  Serial.begin(115200); // 初始化串口通信
  delay(2000); // 等待2秒钟，以便连接到串口监视器

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


void loop() {
  // 读取加速度计和陀螺仪数据
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

  //   // 计算加速度计的角度
  // float accelAngleX = atan2(accel_y, accel_z) * (180.0 / PI);
  // float accelAngleY = atan2(-accel_x, accel_z) * (180.0 / PI);

  // // 计算陀螺仪的角速度
  // float gyroRateX = gyro_x / gyroScaleFactor;
  // float gyroRateY = gyro_y / gyroScaleFactor;

  // // 使用卡尔曼滤波器进行姿态估计
  // float dt = 0.01; // 采样时间间隔，单位为秒
  // kalmanFilter(accelAngleX, gyroRateX, dt,0);
  // float pitch = angle;

  // kalmanFilter(accelAngleY, gyroRateY, dt,1);
  // float roll = angle;



    // 将数据转换为俯仰角


    float pitch = atan2(accel_y, sqrt(accel_x * accel_x + accel_z * accel_z)) * 180 / PI;
    float roll = atan2(-accel_x, accel_z) * 180 / PI;
  float yaw = atan2(gyro_y, gyro_x) * 180 / PI;
  pitch=kalmanFilter.updateEstimate(pitch);
  roll=kalmanFilter1.updateEstimate(roll);
    yaw=kalmanFilter2.updateEstimate(yaw);
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
  
  if (millis() - startTime > 500)//500ms打印
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
  }

}
