/*
   -- wemosD1 --
   
   This source code of graphical user interface 
   has been generated automatically by RemoteXY editor.
   To compile this code using RemoteXY library 2.3.3 or later version 
   download by link http://remotexy.com/en/library/
   To connect using RemoteXY mobile app by link http://remotexy.com/en/download/                   
     - for ANDROID 4.1.1 or later version;
     - for iOS 1.2.1 or later version;
    
   This source code is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.    
*/

//////////////////////////////////////////////
//        RemoteXY include library          //
//////////////////////////////////////////////

// определение режима соединения и подключение библиотеки RemoteXY 
#define REMOTEXY__DEBUGLOGS Serial
#define REMOTEXY_MODE__ESP8266WIFI_LIB_CLOUD
#include <ESP8266WiFi.h>

#include <RemoteXY.h>

// настройки соединения 
#define REMOTEXY_WIFI_SSID "Keenetic" //"viacheslavk"
#define REMOTEXY_WIFI_PASSWORD "9217424259" //"756235D394"
#define REMOTEXY_CLOUD_SERVER "cloud.remotexy.com"
#define REMOTEXY_CLOUD_PORT 6376
#define REMOTEXY_CLOUD_TOKEN "b5d31bcb691d09e52ea3d3fc58c541ed"


// конфигурация интерфейса  
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] = 
 { 255,5,0,45,0,67,0,10,13,1,
  66,132,19,16,29,23,2,24,2,0,
  21,56,24,13,2,26,31,31,79,78,
  0,79,70,70,0,7,44,21,44,23,
  7,2,26,2,2,65,36,6,22,13,
  13,66,0,49,18,9,21,2,26,67,
  5,3,92,57,4,2,26,41,65,34,
  31,28,5,5 };
   
// структура определяет все переменные вашего интерфейса управления  
struct { 

    // input variables
  uint8_t switch_1; // =1 если переключатель включен и =0 если отключен 
  float T_min;

    // output variables
  int8_t level_T; // =0..100 положение уровня 
  uint8_t led_1_r; // =0..255 яркость красного цвета индикатора 
  int8_t level_H; // =0..100 положение уровня 
  char text_1[41];  // =строка UTF8 оканчивающаяся нулем 
  uint8_t led_2_g; // =0..255 яркость зеленого цвета индикатора 

    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0 

} RemoteXY; 
#pragma pack(pop)

/////////////////////////////////////////////
//           END RemoteXY include          //
/////////////////////////////////////////////
#define PIN_SWITCH_1 D6 // relay
#define PIN_DIOD D0
#define PIN_BUTTON D8
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include "Adafruit_SSD1306.h"
#include <EEPROM.h>
#include <DHT.h>
DHT dht(D5, DHT21); // 2

#define OLED_RESET -1
Adafruit_SSD1306 display(OLED_RESET);

uint32_t dhtTimeOut;
uint32_t diodTimeOut;
uint32_t displayTimeOut;
uint32_t diodBlinkOn = 0;
uint32_t diodBlinkOff = 0;
uint32_t dhtReadTime;
float tem;
float hum;
float set_tem;
int tem_last;
int hum_last;
uint8_t power_on;
uint8_t display_show = 0; // display temperatur
boolean relay = false;    // relay off, true - on
boolean light = true;     // diod lighting, false - pause
unsigned long sec_;
unsigned int day_;
unsigned long rem_;
unsigned int hou_;
unsigned int min_;
String strDay;             // day works 
String strHou;
String strMin;
String strSec;


void setup() 
{
  
  RemoteXY_Init (); 

  // TODO you setup code
   pinMode(PIN_BUTTON, INPUT);
   pinMode (PIN_SWITCH_1, OUTPUT);
   pinMode(PIN_DIOD, OUTPUT);  
  
   dht.begin(); 
   hum = dht.readHumidity();
   tem = dht.readTemperature();
   if (isnan(hum) && isnan(tem)){
      hum_last = 0;
      tem_last = 0;
      RemoteXY.led_2_g = 0;
   } else {
      hum_last = round(hum);
      tem_last = round(tem);
      RemoteXY.led_2_g = 128;
   }
   RemoteXY.level_H = hum_last;
   RemoteXY.level_T = tem_last + 50;
    
   dhtReadTime = dhtTimeOut = displayTimeOut = millis();
    
   EEPROM.begin(4);
   set_tem = EEPROM.read(0)*10 + EEPROM.read(1);
   set_tem /= 10;
   power_on = EEPROM.read(2);
   RemoteXY.T_min = set_tem;
   RemoteXY.switch_1 = power_on;

   display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
   display.display();
   
   relay = power(-1);     
}

void loop() 
{ 
  
  
  RemoteXY_Handler ();
  
  
  // TODO you loop code
  // используйте структуру RemoteXY для передачи данных
  if (RemoteXY.switch_1 != power_on){
    power_on = RemoteXY.switch_1;
    EEPROM.write(2, power_on);
    EEPROM.commit();
  }
  if (RemoteXY.T_min != set_tem){
    set_tem = RemoteXY.T_min;
    EEPROM.write(0, (int)set_tem);
    EEPROM.write(1, (set_tem-(int)set_tem)*10);   
    EEPROM.commit();
  }
  
  if ((millis() - dhtTimeOut) > 15000){ // 15 sec.
    sec_ = millis() / 1000;
    day_ = sec_ / 86400;
    rem_ = sec_ % 86400;
    hou_ = rem_ / 3600;
    rem_ = rem_ % 3600;
    min_ = rem_ / 60;
    rem_ = rem_ % 60;
    strDay = String(day_, DEC) + "d";
    strHou = String(hou_, DEC) + "h";
    strMin = String(min_, DEC) + "m"; 
    strSec = String(rem_, DEC) + "s";
    String str = strDay + " " + strHou + " " + strMin;
    str.toCharArray(RemoteXY.text_1,40);
    
    REMOTEXY__DEBUGLOGS.println("");
    REMOTEXY__DEBUGLOGS.println(strDay+strHou+strMin);
    
    hum = dht.readHumidity();    
    tem = dht.readTemperature();
    if (isnan(hum) && isnan(tem)) {
      RemoteXY.led_2_g = 0;
		  diodTimeOut = millis();
		  if (relay){
        diodBlinkOn = 1000;
        diodBlinkOff = 50;
		  } else {
        diodBlinkOn = 50;
        diodBlinkOff = 3000;
      } 
      relay = power(RemoteXY.T_min - tem_last);
            
	  } else {
	    RemoteXY.led_2_g = 128;
	    dhtReadTime = millis();
      hum_last = round(hum);
      tem_last = round(tem);
	     
      diodBlinkOn = 0;  
		  relay = power(RemoteXY.T_min - tem);     
      
  		REMOTEXY__DEBUGLOGS.print("Temperature: ");
  		REMOTEXY__DEBUGLOGS.print(tem,1);
  		REMOTEXY__DEBUGLOGS.print(" Humidity: ");
  		REMOTEXY__DEBUGLOGS.println(hum,1);
  		  
	  }
   
    RemoteXY.level_H = hum_last;
    RemoteXY.level_T = tem_last + 50;
    
	  dhtTimeOut= millis();	  
  }
  
  if (diodBlinkOn > 0){
    if ((millis() - diodTimeOut) > ((light)? diodBlinkOn : diodBlinkOff)){
        light = !light;
        digitalWrite(PIN_DIOD, light);
        diodTimeOut = millis();        
    }          
  }
  
  if ((millis() - displayTimeOut) > 3000){
    display.clearDisplay();
    if (digitalRead(PIN_BUTTON)){      
      display.setTextSize(5);
      display.setTextColor(WHITE);
      display.setCursor(0,0);
      switch (display_show) {
        case 0:
          display.println(tem_last);
          display.setTextSize(1);
          display.println("c");
          break;
        case 1:
          display.println(hum_last);
          display.setTextSize(1);
          display.println("%");
          break;
        case 2:
          display.setTextSize(2);
          if (relay)
            display.println("Pover ON!");
          else
            display.println(" OFF");     
          break;
        default:          
          display.setTextSize(2);
          display.println(strDay);
          display.println(strHou);
          display.println(strMin);          
      }
    }
    display.display();
    if (++display_show > 3)
      display_show = 0;      
    displayTimeOut = millis();
  }  
  	
}

boolean power(float diff){
  int result;
  if (diff < 0 || RemoteXY.switch_1 == 0){
    digitalWrite(PIN_SWITCH_1, LOW);
    light = false;
    digitalWrite(PIN_DIOD, light);
    RemoteXY.led_1_r = 0;
    result = false;
	REMOTEXY__DEBUGLOGS.println("Power off.");
  }
  if (diff > 0 && RemoteXY.switch_1 != 0){
    digitalWrite(PIN_SWITCH_1, HIGH);
    light = true;
    digitalWrite(PIN_DIOD, light);
    RemoteXY.led_1_r = 196;
    result = true;
	REMOTEXY__DEBUGLOGS.println("Power on!");
  }  
  return result;  
}
