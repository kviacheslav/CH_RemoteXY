/*
   -- New project --
   
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
#define REMOTEXY_MODE__ESP8266_HARDSERIAL_CLOUD
#include <RemoteXY.h>

// настройки соединения 
#define REMOTEXY_SERIAL Serial1
#define REMOTEXY_SERIAL_SPEED 115200
#define REMOTEXY_WIFI_SSID "viacheslavk"
#define REMOTEXY_WIFI_PASSWORD "756235D394"
#define REMOTEXY_SERVER_PORT 6377
#define REMOTEXY_CLOUD_SERVER "cloud.remotexy.com"
#define REMOTEXY_CLOUD_PORT 6376
#define REMOTEXY_CLOUD_TOKEN "73b4e6b3bb1459c32b8b9d93dcca3b3d"

#define REMOTEXY_MODULE_TIMEOUT 60000
#define REMOTEXY_INET_TIMEOUT 120000


// конфигурация интерфейса  
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] = 
  { 255,1,0,12,0,47,0,8,13,2,
  2,0,39,24,22,11,39,24,22,11,
  2,26,31,31,79,78,0,79,70,70,
  0,65,4,46,7,9,9,46,7,9,
  9,67,4,40,44,35,8,40,46,20,
  5,2,26,11 }; 
  
// структура определяет все переменные вашего интерфейса управления 
struct {

    // input variable
  uint8_t switch_1; // =1 если переключатель включен и =0 если отключен 

    // output variable
  uint8_t led_1_r; // =0..255 яркость красного цвета индикатора 
  char text_1[11];  // =строка UTF8 оканчивающаяся нулем 


    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0 

} RemoteXY;
#pragma pack(pop)

/////////////////////////////////////////////
//           END RemoteXY include          //
/////////////////////////////////////////////

#define PIN_SWITCH_1 13

#include "CH_RemoteXY.h"

uint32_t moduleTimeOut;
uint32_t inetTimeOut;

CH_CRemoteXY *ch;

void setup() 
{
  ch = new CH_CRemoteXY (RemoteXY_CONF_PROGMEM, &RemoteXY, REMOTEXY_ACCESS_PASSWORD, &REMOTEXY_SERIAL, REMOTEXY_SERIAL_SPEED, REMOTEXY_WIFI_SSID, REMOTEXY_WIFI_PASSWORD, REMOTEXY_CLOUD_SERVER, REMOTEXY_CLOUD_PORT, REMOTEXY_CLOUD_TOKEN);

  
  pinMode (PIN_SWITCH_1, OUTPUT);
  
  // TODO you setup code
  pinMode (2, OUTPUT);
  digitalWrite(2, LOW);
  RemoteXY.switch_1= 1;
  digitalWrite(2, (RemoteXY.switch_1==1)?LOW:HIGH);
  RemoteXY.led_1_r= (RemoteXY.switch_1==1)?255:0;
  inetTimeOut= moduleTimeOut= millis();

#if defined(REMOTEXY__DEBUGLOGS)
          REMOTEXY__DEBUGLOGS.println();
          REMOTEXY__DEBUGLOGS.print("CloudState = ");
          REMOTEXY__DEBUGLOGS.println(remotexy->getCloudState ());

#endif     
 
}

void loop() 
{
	ch->display_state(RemoteXY.switch_1);
	if (ch->getmoduleRunning()){
		ch->handler ();
	  
		digitalWrite(PIN_SWITCH_1, (RemoteXY.switch_1==0)?LOW:HIGH);
	  
	  // TODO you loop code
	  // используйте структуру RemoteXY для передачи данных
		digitalWrite(2, (RemoteXY.switch_1==1)?LOW:HIGH);
		RemoteXY.led_1_r= (RemoteXY.switch_1==1)?255:0;
		dtostrf(ch->t, 7, 3, RemoteXY.text_1);
		RemoteXY.text_1[7]=0;
    if (ch->getCloudState () == REMOTEXY_CLOUD_STATE_WORKING){
      inetTimeOut= millis();
    }else if ((millis() - inetTimeOut) > REMOTEXY_INET_TIMEOUT){
      digitalWrite(2, HIGH);
      delay(3000); 
      digitalWrite(2, LOW);
      ch->initModule ();
      inetTimeOut= moduleTimeOut= millis();
    }
	}
	else if ((millis() - moduleTimeOut) > REMOTEXY_MODULE_TIMEOUT){	
		
#if defined(REMOTEXY__DEBUGLOGS)
          REMOTEXY__DEBUGLOGS.println();
          REMOTEXY__DEBUGLOGS.println("Retry initModule...");

#endif
    ch->initModule ();
		moduleTimeOut= millis();
	}

}
