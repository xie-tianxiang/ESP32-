//发送板代码
int ASignal = A5;//土壤湿度传感器引脚

#include <WiFi.h>//调用WIFI函数库
#include <esp_now.h>//调用ESP-NOW函数库
uint8_t broadcastAddress1[] = {0x60,0x55,0xF9,0x7B,0x8B,0x40};//ESP开发板接收板的MAC地址60:55:F9:7B:90:F0
typedef struct Tdata {//发送数据的结构体，必须匹配接收的结构
    int id; //每个发送板ID都不一致
    float x;//X整数变量
    float y;
} Tdata;
Tdata myData;//创建一个结构体的名称
esp_now_peer_info_t peerInfo;//创建对等接口
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {//数据发送时回调
  Serial.print("\r\n最后一个数据包发送状态:\t");
   Serial.println(status == ESP_NOW_SEND_SUCCESS ? "传送成功" : "传送失败");//判断是否传送成功
} 
void setup() {
  Serial.begin(115200);//打开串口波特率设置为115200
  pinMode(ASignal, INPUT);//土壤湿度传感器引脚初始化
  WiFi.mode(WIFI_STA);//将设备设置为 Wi-Fi“STA”(站模式)
  if (esp_now_init() != ESP_OK) {//初始化esp_now
    Serial.println("初始化ESP-NOW时出错");
    return;
    
  }
  esp_now_register_send_cb(OnDataSent);//ESPNow成功初始化，我们将注册发送包，获取已发送数据包的状态。 
  memcpy(peerInfo.peer_addr, broadcastAddress1, 6);//对端点设备地址。我们将其设置为广播地址。
  peerInfo.channel = 0; //对端点用于发送数据 Wi-Fi通道我们将其设置为0。    
  peerInfo.encrypt = false;//表示对端点发送的数据是否加密对于本示例，我们将其设置为 false(假)。 
  if (esp_now_add_peer(&peerInfo) != ESP_OK){//添加“对等点”到列表中
    Serial.println("添加对等点失败");
    return;//该函数返回一个esp_err_t类型的值，我们也将使用该值进行错误检查
  }
} 
void loop() {
  myData.id = 2;//开发板的ID每个都不一样
  float sensorValue = analogRead(ASignal);
  Serial.print("soil exploration=");
  Serial.println(sensorValue);
  myData.x = sensorValue;//温度变量传入
  myData.y = 0;
 //esp_err_t该函数,将返回一个类型为esp_err_t的值，我们将其存储在一个变量中并用于错误检查
  esp_err_t result = esp_now_send(broadcastAddress1, (uint8_t *) &myData, sizeof(myData));//通过ESP-NOW发送数据,0作为参数，它将向所有注册的对等方发送相同的消息 
  if (result == ESP_OK) {//判断发送返回的值是否成功
    Serial.println("发送成功");
  }
  else {
    Serial.println("发送数据时出错");
  }
  delay(2000);//延时2秒
}