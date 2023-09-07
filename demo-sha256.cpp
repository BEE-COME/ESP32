#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <ESPAsyncWiFiManager.h> 
#include <DNSServer.h>


//SHA256计算
#include <Arduino.h>
#include <mbedtls/md.h>

#define SHA256_DIGEST_LENGTH 32

const mbedtls_md_info_t *sha_info;
mbedtls_md_context_t sha_ctx;
String data;
void onReceiveFunction(void) {
  // This is a callback function that will be activated on UART RX events
  size_t available = Serial.available();
//   received_bytes += available;
//   Serial1.printf("onReceive Callback:: There are %d bytes available: ", available);
  // data.clear();
  while (available --) {
    data+=(char)Serial.read();
  }
  Serial.print(data.c_str());
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.setRxTimeout(3);
  Serial.onReceive(onReceiveFunction, true);
    
}

void count_sha(const char* d)
{
  // 使用ESP32内置的SHA加速器进行SHA256初始化
  sha_info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
  mbedtls_md_init(&sha_ctx);
  mbedtls_md_setup(&sha_ctx, sha_info, 1);
  mbedtls_md_starts(&sha_ctx);
  
  // 输入数据
  // const char *input = "1234";
  size_t input_len = strlen(d);
  // Serial.write(d,HEX);
  
  // 计算SHA256哈希值
  mbedtls_md_update(&sha_ctx, (const unsigned char *)d, input_len);
  unsigned char output[SHA256_DIGEST_LENGTH];
  mbedtls_md_finish(&sha_ctx, output);
  
  // 打印结果
  Serial.print("SHA256 Hash: ");
  for (int i = 0; i < SHA256_DIGEST_LENGTH; i++) {
    Serial.printf("%02x", output[i]);
  }
  
  // 清理资源
  mbedtls_md_free(&sha_ctx);
}

void loop() {

  
  // ser_read(data.c_str());
  
  // if(strlen(a))Serial.print(a);
  // if(a.length())
  // {
  //   count_sha(a.c_str());
  //   a.clear();
    
  // }
  if(data.length())
  {
    count_sha(data.c_str());
    data.clear();
  } 
  delay(1000);
  




}

