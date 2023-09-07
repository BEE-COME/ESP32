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
float P[2][2] = {{0.0, 0.0}, {0.0, 0.0}};
unsigned long startTime;

const int MPU6050_ADDR = 0x68; // MPU6050��I2C��ַ
#include <SimpleKalmanFilter.h>
SimpleKalmanFilter kalmanFilter(0.001, 0.001,0.001);
SimpleKalmanFilter kalmanFilter1(0.001, 0.001,0.001);
SimpleKalmanFilter kalmanFilter2(0.001, 0.001,0.001);
float accel_x_filtered, accel_y_filtered, accel_z_filtered;
float gyro_x_filtered, gyro_y_filtered, gyro_z_filtered;

void setup() {
  Wire.begin(); // ��ʼ��I2C����
  Serial.begin(115200); // ��ʼ������ͨ��
  delay(2000); // �ȴ�2���ӣ��Ա����ӵ����ڼ�����

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


void loop() {
  // ��ȡ���ٶȼƺ�����������
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

  //   // ������ٶȼƵĽǶ�
  // float accelAngleX = atan2(accel_y, accel_z) * (180.0 / PI);
  // float accelAngleY = atan2(-accel_x, accel_z) * (180.0 / PI);

  // // ���������ǵĽ��ٶ�
  // float gyroRateX = gyro_x / gyroScaleFactor;
  // float gyroRateY = gyro_y / gyroScaleFactor;

  // // ʹ�ÿ������˲���������̬����
  // float dt = 0.01; // ����ʱ��������λΪ��
  // kalmanFilter(accelAngleX, gyroRateX, dt,0);
  // float pitch = angle;

  // kalmanFilter(accelAngleY, gyroRateY, dt,1);
  // float roll = angle;



    // ������ת��Ϊ������


    float pitch = atan2(accel_y, sqrt(accel_x * accel_x + accel_z * accel_z)) * 180 / PI;
    float roll = atan2(-accel_x, accel_z) * 180 / PI;
  float yaw = atan2(gyro_y, gyro_x) * 180 / PI;
  pitch=kalmanFilter.updateEstimate(pitch);
  roll=kalmanFilter1.updateEstimate(roll);
    yaw=kalmanFilter2.updateEstimate(yaw);
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
  
  if (millis() - startTime > 500)//500ms��ӡ
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
  }

}
