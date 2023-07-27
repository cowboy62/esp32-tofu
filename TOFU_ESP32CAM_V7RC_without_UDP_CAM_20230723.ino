/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-cam-video-streaming-web-server-camera-home-assistant/
  
  IMPORTANT!!! 
   - Select Board "AI Thinker ESP32-CAM"
   - GPIO 0 must be connected to GND to upload a sketch
   - After connecting GPIO 0 to GND, press the ESP32-CAM on-board RESET button to put your board in flashing mode
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/
/*
UDP Code Here

https://www.alejandrowurts.com/projects/bb9e-v1-update-7-esp32-bilateral-coms/


*/

/*
RoboTW   


FB:  RoboTW 機器人論壇
FB:  https://www.facebook.com/groups/540271770146161/

這個程式是用來配合V7RC 手機APP 的，可以利用WIFI UDP 控制載具 並利用WIFI 進行即時影像接收。
This Program is used with V7RC app @ Apple Store and Google Play. It can be used to control Servos and DIOS with WIFI and get real time Video via WIFI link.

 V7RC 是非常好用的手機遙控工具軟體，也感謝嵐奕科技有限公司
 https://apps.apple.com/tw/app/v7rc/id1390983964
 https://play.google.com/store/apps/details?id=com.v7idea.v7rcliteandroidsdkversion&hl=zh_TW
 

 影像部分的程式是從下列網址copy 來用，感謝
 原作者
 https://randomnerdtutorials.com/esp32-cam-video-streaming-web-server-camera-home-assistant/

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

 
   如果喜歡，記得來FB 群組跟我們分享
   If you like this work, please come to our FB Group, and tell us what you made.

 allen54a0@gmail.com


 
  
 */

 /*
  * 
  * 由PinHouse 改為豆腐機器人測試平台用,
  * 移除CAM功能
  * */

#include <WiFi.h>
#include "esp_timer.h"
#include "Arduino.h"
#include "soc/soc.h"          //disable brownout problems
#include "soc/rtc_cntl_reg.h" //disable brownout problems
#include "esp_http_server.h"



int CH1PwmCH  = 4  ;
int CH1Pin    = 12 ;
int CH2PwmCH = 5  ;
int CH2Pin   = 13 ;
int CH3PwmCH = 6  ;
int CH3Pin   = 14 ;
int CH4PwmCH = 7  ;
int CH4Pin   = 15 ;

void servo_angle(int channel, int angle)  // 使用 PWM 控制馬達轉動角度

{

 // regarding the datasheet of sg90 servo, pwm period is 20 ms and duty is 1->2ms

  int SERVO_RESOLUTION = 16;

  float range = (pow(2,SERVO_RESOLUTION)-1)/10;  

  float minDuty = (pow(2,SERVO_RESOLUTION)-1)/40;

  

  uint32_t duty = (range)*(float)angle/180.0 + minDuty;

  ledcWrite(channel, duty);

  delay(10);

}

// Replace with your network credentials

#define DEVICE_NAME "PinHouse V7RC"
#define BLE_NAME DEVICE_NAME

const char *ssid = DEVICE_NAME;
const char *password = "12345678";

///PWM  PIN 2  12 13  14 (4 LCD)
///LCDC CH  2  4   3   7

WiFiUDP Udp; // Creation of wifi Udp instance

char packetBuffer[255];

unsigned int localPort = 6188;

 

static const int servosPins[5] = {12, 13, 14, 15};

void initServo()
{

  // Ai-Thinker: pins  12 , 13, 14 15 for PWM Servo


   ledcSetup(CH1PwmCH, 50, 16);
   ledcAttachPin(CH1Pin, CH1PwmCH);  // gpio, channel 

   ledcSetup(CH2PwmCH, 50, 16);
   ledcAttachPin(CH2Pin, CH2PwmCH);

   ledcSetup(CH3PwmCH , 50, 16);
   ledcAttachPin(CH3Pin, CH3PwmCH );

   ledcSetup(CH4PwmCH, 50, 16);
   ledcAttachPin(CH4Pin, CH4PwmCH);
}

void SetServoPos(int ch, int pos)
{
  uint32_t duty = ((((float)pos / 180.0) * 2000) / 20000.0 * 65536.0) + 1634;

  ledcWrite(ch, duty);
  // set channel to pos
}

//HZ Control
#define HZ_SETTING 100
int mainLoop_count;
unsigned long fast_loopTimer; // Time in miliseconds of main control loop
const int hzCount = (1000 / HZ_SETTING) - 1;
const int timeRemind = 1000 / HZ_SETTING;

///////////////////////BLE --------------------------->

int datafromV7RC[8] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
 
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t txValue = 0;

 

void parseCommand();
int flagShowControl = 0;

 

int hexConvert2int(char highByte, char lowByte)
{

  int val = 0;
  int highB;
  int lowB;

  if (highByte >= 'A')
  {
    highB = highByte - 'A' + 10;
  }
  else if (highByte >= '0' && highByte <= '9')
  {
    highB = highByte - 0x30;
  }

  if (lowByte >= 'A')
  {
    lowB = lowByte - 'A' + 10;
  }
  else if (lowByte >= '0' && lowByte <= '9')
  {
    lowB = lowByte - 0x30;
  }

  val = highB * 16 + lowB;
  val = val * 10;
  return val;
}

///// V7RC Code
 
 

void parseCommand()
{
  char cmd = Serial.read();
  switch (cmd)
  {
  case 'D':

    dumpespLoraData();
    break;

  case '1':
    flagShowControl = 1;
    break;

  case '0':
    flagShowControl = 0;
    break;
  }
}

void dumpespLoraData()
{

  for (int i = 0; i < 8; i++)
  {

    Serial.print(map(datafromV7RC[i], 1000, 2000, -255, 255));
    Serial.print(",");
  }
  for (int i = 0; i < 6; i++)
  {

    Serial.print(map(datafromV7RC[i], 1000, 2000, -255, 255));
    Serial.print(",");
  }
  Serial.println("");
}

void setup()
{
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector

  Serial.begin(115200);
  Serial.setDebugOutput(false);
  initServo();
  servo_angle(CH1PwmCH, 90);   
  servo_angle(CH2PwmCH, 90);   
  servo_angle(CH3PwmCH, 90);   
  servo_angle(CH4PwmCH, 90);  

  

  // Connect to Wi-Fi network with SSID and password
  Serial.print("Setting AP (Access Point)…");
  // Remove the password parameter, if you want the AP (Access Point) to be open
  WiFi.softAP(ssid, password);
  //WiFi.softAP(ssid );

  IPAddress IP = WiFi.softAPIP();
  Serial.print("Camera Stream Ready! Connect to the ESP32 AP and go to: http://");
  Serial.println(IP);

  ////UDP Here ...
  Udp.begin(localPort);

  // Start streaming web server

}
int SentDelayCnt = 0 ;
int SentCnt = 0 ;
void loop()
{
  int CH1angle = 90 ;   
  int CH2angle = 90 ;   
  int CH3angle = 90 ;   
  int CH4angle = 90 ;   
  
  if (Serial.available())
    parseCommand();

  int packetSize = Udp.parsePacket();
  if (packetSize)
  {
    String rxData;
    String data;
    int len = Udp.read(packetBuffer, 255);
    if (len > 0)
      packetBuffer[len - 1] = 0;
    // Serial.println(packetBuffer);
    //Serial.println( len);

    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.printf("received: ");
    Udp.printf(packetBuffer);
    Udp.printf("\r\n");
    Udp.endPacket();

    if (len > 0)
    {
       // Serial.println("*********");//0724
        // Serial.print("Received Value: ");//0724
      for (int i = 0; i < len; i++)
      {
        rxData += packetBuffer[i];
          // Serial.print(packetBuffer[i]);//0724
      }

       //  Serial.println();//0724
      // Serial.println("*********");//0724
    }

    ///// V7RC Code ---------------------------------------------------------------->>>
    if (packetBuffer[1] == 'R')
    {

      for (int i = 0; i < 4; i++)
      {
        data = rxData.substring(i * 4 + 3, i * 4 + 7);
        datafromV7RC[i] = data.toInt();
      }
    }
    else
    { //for SS8   CMD  (8 Servo)   //SS8 96 96 96 96 96 96 96 96#

      for (int i = 0; i < 8; i++)
      {

        datafromV7RC[i] = hexConvert2int(packetBuffer[i * 2 + 3], packetBuffer[i * 2 + 4]);
      }
    }

    ////debug Only, send to Vrep....

    if (flagShowControl == 1)
    {
      Serial.print(packetBuffer[2]); /// should be V / T / 8 (2 ch, 4 ch , 8 ch )
      Serial.print(",");

      for (int i = 0; i < 8; i++)
      {
        Serial.print(datafromV7RC[i]);
        Serial.print(",");
      }

    //  Serial.println(",");
    }
  }

  ///// V7RC Code ----------------------------------------------------------------<<<<<

  if (millis() - fast_loopTimer > hzCount) //100 HZ
  {
    fast_loopTimer = millis();
    mainLoop_count++;
    SentDelayCnt ++ ;
    if (SentDelayCnt > 99 ) {
      SentDelayCnt = 0 ;
        SentCnt ++ ;
      Serial.println(SentCnt);
    }



    //////Servo  Loop --------------------------------------------------------------------->
   //Serial.print(",");
    if (mainLoop_count % 2 == 0)
    {
      //Serial.print("  ");

      CH1angle = map(datafromV7RC[0], 1000, 2000, 40, 140) ;
      CH2angle = map(datafromV7RC[1], 1000, 2000, 40, 140) ;
      CH3angle = map(datafromV7RC[2], 1000, 2000, 40, 140) ;
      CH4angle = map(datafromV7RC[3], 1000, 2000, 40, 140) ;

      servo_angle(CH1PwmCH,CH1angle );  
      servo_angle(CH2PwmCH,CH2angle );  
      servo_angle(CH3PwmCH,CH3angle );  
      servo_angle(CH4PwmCH,CH4angle );  




      
      
      Serial.print(" CH1:");
      Serial.print(CH1angle);
      Serial.print(" CH2:");
      Serial.print(CH2angle);
      Serial.print(" CH3:");
      Serial.print(CH3angle);
      Serial.print(" CH4:");
      Serial.print(CH4angle);
      Serial.println(" ");

      
    }
  }
}
