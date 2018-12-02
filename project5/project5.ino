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
#define PIN_SWITCH_1 5 // D1

#include <EEPROM.h>
#include <DHT.h>
DHT dht(2, DHT22);
uint32_t dhtTimeOut;
float tem;
float hum;
float set_tem;
uint8_t power_on;

void setup() 
{
  
  RemoteXY_Init (); 
  
  pinMode (PIN_SWITCH_1, OUTPUT);
  
  // TODO you setup code
   dht.begin(); 
   hum = dht.readHumidity();
   tem = dht.readTemperature();   
   dhtTimeOut= millis();

   EEPROM.begin(4);
   set_tem = EEPROM.read(0) + EEPROM.read(1)/10;
   power_on = EEPROM.read(2);
   RemoteXY.T_min = set_tem;
   RemoteXY.switch_1 = power_on;
   power(false);
     
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
  if (isnan(tem)){
    tem = -50;
  }
  else
  {
    power(RemoteXY.T_min > tem);
  }
  RemoteXY.level_T = tem + 50;

  if (isnan(hum)){
    hum = 0;
  }
  RemoteXY.level_H = hum;
  
  if ((millis() - dhtTimeOut) > 10000){
	  hum = dht.readHumidity();
	  tem = dht.readTemperature();
	  if (isnan(hum) || isnan(tem)) 
	  {
		  REMOTEXY__DEBUGLOGS.println("Failed to read from DHT sensor!");
	  }
	  else
	  {
  		REMOTEXY__DEBUGLOGS.print("Temperature: ");
  		REMOTEXY__DEBUGLOGS.print(tem);
  		REMOTEXY__DEBUGLOGS.print(" Humidity: ");
  		REMOTEXY__DEBUGLOGS.print(hum);
  		REMOTEXY__DEBUGLOGS.println("Sending data ready");    
	  }
	  dhtTimeOut= millis();	  
  }  
	
}

void power(bool on){
  if (!on || RemoteXY.switch_1 == 0){
    digitalWrite(PIN_SWITCH_1, LOW);
    RemoteXY.led_1_r = 0;
  }
  if (on && RemoteXY.switch_1 != 0){
    digitalWrite(PIN_SWITCH_1, HIGH);
    RemoteXY.led_1_r = 128;
  }  
}

