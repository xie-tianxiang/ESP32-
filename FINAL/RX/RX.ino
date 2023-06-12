//接收板代码
#include <WiFi.h>//调用WIFI函数库
#include <esp_now.h>//调用ESP-NOW函数库
//接收数据的结构体，必须匹配发送的结构体
#include "driver/temp_sensor.h"//调用温度库
#include <stdlib.h>          
#include <Adafruit_GFX.h>    //核心图形库
#include <Adafruit_ST7735.h> //为ST7735硬件专用库
#include "Adafruit_ST77xx.h" //提供屏幕旋转功能
#include <SPI.h>



//最大字节数
#define MAX_PACKETSIZE 512

boolean UP, DOWN, LEFT, RIGHT, CENTER_FLAG; //全局标志位FLAG_VALUE----测试用

//屏幕引脚定义
#define TFT_CS          7
#define TFT_RST         10 //或者设置为-1，并连接到Arduino的RESET引脚
#define TFT_DC          6
#define TFT_MOSI        3 //数据输出
#define TFT_SCLK        2 //时钟输出
#define beep_pin  12
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS,TFT_DC,TFT_MOSI,TFT_SCLK,TFT_RST);
//定义圆周率
#define pi 3.1415926



typedef struct Rdata {
  int id;//本开发板的自身ID存储变量，方便接收板识别那块板是发送的数据
  float x;//整型变量用于存放发送板的X数据
  int y;//整型变量用于存放发送板的Y数据
}Rdata;
Rdata myData;//创建一个结构体的名称
// 创建一个结构来保存每个板的读数
Rdata board1;//用于存储开发板1的值
Rdata board2;//用于存储开发板2的值
Rdata board3;//用于存储开发板3的值
Rdata board4;
Rdata board5;
Rdata Shuju[5] = {board1, board2, board3,board4,board5};// 创建一个包含所有结构的数组方便调用读写数据

void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {//接收数据时执行的回调函数
  char macStr[18];//定义字符串数组
  Serial.print("data flow received: ");//  Serial.print("收到的数据包: ");
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
  mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);//获取板卡MAC地址
  Serial.println(macStr);//输出MAC地址
  memcpy(&myData, incomingData, sizeof(myData));//我们将接收到内存数据，都存储到结构体的变量中
  Serial.printf("bored ID %u: %u 字节\n", myData.id, len);//串口输出ID名称和接收到的字节数
  Shuju[myData.id-1].x = myData.x;//接收到的值送给对应的结构体X变量中
  Shuju[myData.id-1].y = myData.y;//接收到的值送给对应的结构体Y变量中
  Serial.printf("temperature: %f \n", Shuju[myData.id-1].x);//串口输出X值
  Serial.printf("y值: %d \n", Shuju[myData.id-1].y);//串口输出Y值
  Serial.println("successfully received");
}
 
void setup() {
Serial.begin(115200);//打开串口波特率设置为115200
  WiFi.mode(WIFI_STA);//将设备设置为 Wi-Fi“STA”(站模式)
  pinMode(beep_pin, OUTPUT);//设置引脚为输出模式
  temp_sensor_config_t temp_sensor = {
    .dac_offset = TSENS_DAC_L2,
    .clk_div = 6,
  };
  temp_sensor_set_config(temp_sensor);
  temp_sensor_start();
  if (esp_now_init() != ESP_OK) {//初始化esp_now
    Serial.println("初始化ESP-NOW出错！");
    return;//返回值用于错误检查
  }
  esp_now_register_recv_cb(OnDataRecv);//ESPNow成功初始化，我们将注册接收以获取接收包数据
  Serial.print(F("Hello! ST77xx TFT Test"));

  //如果使用0.96“160x80 TFT，请使用此初始值设定项（取消注释）：
  tft.initR(INITR_MINI160x80); // Init ST7735S迷你显示器
  Serial.println(F("Initialized"));
  tft.setRotation(3);  //屏幕方向设定
} 
void loop() {
  float tsens_out;
  temp_sensor_read_celsius(&tsens_out);
  Shuju[0].x = tsens_out;//温度变量传入
  Drawtext(0, 5, 2,Shuju[0].x, ST77XX_WHITE);
  Drawtext(0, 25, 2,Shuju[1].x, ST77XX_WHITE);
  Drawtext(0, 45, 2,Shuju[2].x, ST77XX_WHITE); 
  Drawtext(0, 65, 2,Shuju[3].x, ST77XX_WHITE); 
  Drawtext(80, 5, 2,Shuju[4].x, ST77XX_WHITE); 
  if(Shuju[1].x>=30)
  {
    digitalWrite(beep_pin,1);//引脚输出高电平
  }
  else
  {
    digitalWrite(beep_pin,0);
  }
  delay(1000);//延时1秒 
  tft.fillScreen(ST77XX_BLACK);  //填充背景颜色（必须填充，否则之前的图像，将不会消失） 

//  Serial.println("循环一圈啦"); 
}

void Drawtext(char x, char y, char size, float text, uint16_t color) {
  tft.setCursor(x, y);
  tft.setTextColor(color);
  tft.setTextSize(size);
  tft.setTextWrap(false);
  tft.print(text);
}