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
#define REMOTEXY_MODE__ETHERNET_LIB_CLOUD
#include <Ethernet.h>
/* Ethernet shield used pins: 10(SS) */
#include <SPI.h>
/* SPI interface used pins: 11(MOSI), 12(MISO), 13(SCK) */
#define REMOTEXY__DEBUGLOGS Serial
#include <RemoteXY.h>

// настройки соединения 
#define REMOTEXY_SERIAL Serial
#define REMOTEXY_SERIAL_SPEED 115200
#define REMOTEXY_WIFI_SSID "viacheslavk"
#define REMOTEXY_WIFI_PASSWORD "756235D394"
#define REMOTEXY_SERVER_PORT 6377
#define REMOTEXY_ETHERNET_MAC "DE:AD:BE:EF:EF:ED"
#define REMOTEXY_CLOUD_SERVER "cloud.remotexy.com"
#define REMOTEXY_CLOUD_PORT 6376
#define REMOTEXY_CLOUD_TOKEN "73b4e6b3bb1459c32b8b9d93dcca3b3d"


// конфигурация интерфейса  
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] =
  { 255,0,0,42,0,20,0,8,13,0,
  66,132,37,12,29,23,2,24,67,4,
  9,46,83,8,2,26,41 };
  
// структура определяет все переменные вашего интерфейса управления 
struct {

    // output variable
  int8_t level_1; // =0..100 положение уровня 
  char text_1[41];  // =строка UTF8 оканчивающаяся нулем 

    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0 

} RemoteXY;
#pragma pack(pop)

/////////////////////////////////////////////
//           END RemoteXY include          //
/////////////////////////////////////////////
#define CH_LCD	 		LiquidCrystal(28, 29, 24, 25, 26, 27)
#define CH_DHT DHT(22, DHT11)
#define CH_CD 4
//#define CH_GSM 3
#define PIN_PWRKEY 30 
#include "CH_Gsm.h"


#define PIN_SWITCH_1 23 


#define REMOTEXY_MODULE_TIMEOUT 60000
#define REMOTEXY_INET_TIMEOUT 120000

#include "CH_RemoteXY.h"

uint32_t moduleTimeOut;
uint32_t inetTimeOut;

CH_CRemoteXY*	ch;
CH_Gsm*	modem;
//#include <SoftwareSerial.h>
 
//SoftwareSerial SoftSerial(5, 6); // RX, TX

void setup() 
{
	
	
	pinMode (PIN_SWITCH_1, OUTPUT);
	digitalWrite(PIN_SWITCH_1, LOW);
	
	
	
	//SoftSerial.begin(115200);
	//Serial.println("Begin...");
	ch = new CH_CRemoteXY (RemoteXY_CONF_PROGMEM, &RemoteXY, REMOTEXY_ACCESS_PASSWORD, &REMOTEXY_SERIAL, REMOTEXY_SERIAL_SPEED, REMOTEXY_WIFI_SSID, REMOTEXY_WIFI_PASSWORD, REMOTEXY_ETHERNET_MAC, REMOTEXY_CLOUD_SERVER, REMOTEXY_CLOUD_PORT, REMOTEXY_CLOUD_TOKEN);
	modem = new CH_Gsm(Serial3);
	modem->pwrkeyOn();
	// TODO you setup code	
	inetTimeOut= moduleTimeOut= millis();
	ch->display_state(RemoteXY.connect_flag);
	Serial.println(memoryFree());
}

void loop() 
{ 
	

	modem->handler();
	ch->ch_handler();
	if (ch->getmoduleRunning()){
		ch->handler ();		
	  
	  // TODO you loop code
	  // используйте структуру RemoteXY для передачи данных
		
		if (ch->getCloudState () == REMOTEXY_CLOUD_STATE_WORKING){
		  inetTimeOut= millis();
		}else if ((millis() - inetTimeOut) > REMOTEXY_INET_TIMEOUT){
			
			digitalWrite(PIN_SWITCH_1, HIGH);
			delay(3000); 
			digitalWrite(PIN_SWITCH_1, LOW);
			ch->initModule ();
			inetTimeOut= moduleTimeOut= millis();
		}
	}
	else if ((millis() - moduleTimeOut) > REMOTEXY_MODULE_TIMEOUT){	
		
#if defined(REMOTEXY__DEBUGLOGS)
          REMOTEXY__DEBUGLOGS.println();
          REMOTEXY__DEBUGLOGS.println("Retry initModule...");

#endif
		ch->display_state(RemoteXY.connect_flag);
		
		digitalWrite(PIN_SWITCH_1, HIGH);
		delay(3000); 
		digitalWrite(PIN_SWITCH_1, LOW);
		ch->sd_init();
		//ch->initModule ();
		Serial.println(memoryFree());
		moduleTimeOut= millis();  
	}
	
}

// Переменные, создаваемые процессом сборки,
// когда компилируется скетч
extern int __bss_end;
extern void *__brkval;
 
// Функция, возвращающая количество свободного ОЗУ (RAM)
int memoryFree()
{
   int freeValue;
   if((int)__brkval == 0)
      freeValue = ((int)&freeValue) - ((int)&__bss_end);
   else
      freeValue = ((int)&freeValue) - ((int)__brkval);
   return freeValue;
}
// include the SD library:
//#include <SPI.h>


// set up variables using the SD utility library functions:
//Sd2Card card;

