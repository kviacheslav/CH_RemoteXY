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
#define REMOTEXY_WIFI_SSID "viacheslavk"
#define REMOTEXY_WIFI_PASSWORD "756235D394"
#define REMOTEXY_CLOUD_SERVER "cloud.remotexy.com"
#define REMOTEXY_CLOUD_PORT 6376
#define REMOTEXY_CLOUD_TOKEN "b5d31bcb691d09e52ea3d3fc58c541ed"


// конфигурация интерфейса  
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] = 
  { 255,5,0,3,0,52,0,8,13,1,
  66,132,25,17,13,10,2,24,2,0,
  20,46,22,11,2,26,31,31,79,78,
  0,79,70,70,0,7,44,21,35,20,
  5,2,26,2,2,66,0,41,16,7,
  16,2,26,65,36,13,20,9,9 }; 
   
// структура определяет все переменные вашего интерфейса управления  
struct { 

    // input variable
  uint8_t switch_1; // =1 если переключатель включен и =0 если отключен 
  float T_min;

    // output variable
  int8_t level_T; // =0..100 положение уровня 
  int8_t level_H; // =0..100 положение уровня 
  uint8_t led_1_r; // =0..255 яркость красного цвета индикатора 

    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0  

} RemoteXY; 
#pragma pack(pop)

/////////////////////////////////////////////
//           END RemoteXY include          //
/////////////////////////////////////////////
#define PIN_SWITCH_1 D8
#define PIN_DIOD D0
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include "Adafruit_SSD1306.h"
#include <EEPROM.h>
#include <DHT.h>
DHT dht(D4, DHT21); // 2
DHT dht7(D7, DHT22); // 2
DHT dht6(D6, DHT22); // 2
DHT dht5(D5, DHT22); // 2

#define OLED_RESET -1
Adafruit_SSD1306 display(OLED_RESET);

uint32_t dhtTimeOut;
uint32_t diodTimeOut;
uint32_t diodBlinkOn;
uint32_t diodBlinkOff;
float tem;
float hum;
float tem5;
float hum5;
float tem6;
float hum6;
float tem7;
float hum7;
float set_tem;
uint8_t power_on;
boolean relay = false; //  - off true - on
boolean light = true;


void setup() 
{
  
  RemoteXY_Init (); 
  
  pinMode (PIN_SWITCH_1, OUTPUT);
  pinMode(PIN_DIOD, OUTPUT);   
  
  // TODO you setup code
   dht5.begin();
   dht6.begin();
   dht7.begin();
   dht.begin(); 
   hum = dht.readHumidity();
   tem = dht.readTemperature();   
   dhtTimeOut = millis();
   
   EEPROM.begin(4);
   set_tem = EEPROM.read(0) + EEPROM.read(1)/10;
   power_on = EEPROM.read(2);
   RemoteXY.T_min = set_tem;
   RemoteXY.switch_1 = power_on;

   display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
   display.display();
   
   relay = power(-1);
   
   diodBlinkOn = 0; // 0 раз в секунду
   light = false;
     
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
  
  if ((millis() - dhtTimeOut) > 20000){
	  hum = dht.readHumidity();
	  RemoteXY.level_H = (isnan(hum))? 0 : hum;
		
	  tem = dht.readTemperature();
	  RemoteXY.level_T = (isnan(tem))? 0 : tem + 50;
    
    hum5 = dht5.readHumidity();    
    tem5 = dht5.readTemperature();    
    REMOTEXY__DEBUGLOGS.println("");
    REMOTEXY__DEBUGLOGS.print("Temp5: ");
    REMOTEXY__DEBUGLOGS.print((isnan(tem5))? 0 : tem5,1);
    REMOTEXY__DEBUGLOGS.print(" Hum5: ");
    REMOTEXY__DEBUGLOGS.println((isnan(hum5))? 0 : hum5,1);

    hum6 = dht6.readHumidity();    
    tem6 = dht6.readTemperature();    
    REMOTEXY__DEBUGLOGS.print("Temp6: ");
    REMOTEXY__DEBUGLOGS.print((isnan(tem6))? 0 : tem6,1);
    REMOTEXY__DEBUGLOGS.print(" Hum6: ");
    REMOTEXY__DEBUGLOGS.println((isnan(hum6))? 0 : hum6,1);

    hum7 = dht7.readHumidity();    
    tem7 = dht7.readTemperature();    
    REMOTEXY__DEBUGLOGS.print("Temp7: ");
    REMOTEXY__DEBUGLOGS.print((isnan(tem7))? 0 : tem7,1);
    REMOTEXY__DEBUGLOGS.print(" Hum7: ");
    REMOTEXY__DEBUGLOGS.println((isnan(hum7))? 0 : hum7,1);
    
	  if (isnan(hum) && isnan(tem)) 
	  {
		  RemoteXY.level_H = 0;
		  RemoteXY.level_T = 0;
		  diodTimeOut = millis();
		  if (relay){
        diodBlinkOn = 1000;
        diodBlinkOff = 250;
		  }
      else{
        diodBlinkOn = 100;
        diodBlinkOff = 2000;
      }    
        
		  display.clearDisplay();
      display.setCursor(0,0);
      display.println("DHT21");
      display.println("error");
      display.display();
		  REMOTEXY__DEBUGLOGS.println("Failed to read from DHT sensor!");
	  }
	  else
	  {
	    RemoteXY.level_H = hum;
		  RemoteXY.level_T = tem + 50;
      diodBlinkOn = 0;  
		  relay = power(RemoteXY.T_min - tem);

      display.clearDisplay();
      display.setTextSize(2);
      display.setTextColor(WHITE);
      display.setCursor(0,0);
      // Temperature in Celcius
      //display.println("Temp.");
      display.println(tem);
      //display.println(" c");
      // Humidity in % 
      //display.println("Humidity");
      display.println(hum);
      //display.println(" %");
      if (relay)
        display.println("On!");
      else
        display.println("Off");
      display.display();
		  
  		REMOTEXY__DEBUGLOGS.print("Temperature: ");
  		REMOTEXY__DEBUGLOGS.print(tem,1);
  		REMOTEXY__DEBUGLOGS.print(" Humidity: ");
  		REMOTEXY__DEBUGLOGS.println(hum,1);
  		  
	  }
	  dhtTimeOut= millis();	  
  }
  if (diodBlinkOn > 0){
    if ((millis() - diodTimeOut) > ((light)? diodBlinkOn : diodBlinkOff)){
        light = !light;
        digitalWrite(PIN_DIOD, light);
        diodTimeOut = millis();
        
    }
          
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
    RemoteXY.led_1_r = 128;
    result = true;
	REMOTEXY__DEBUGLOGS.println("Power on!");
  }  
  return result;  
}
