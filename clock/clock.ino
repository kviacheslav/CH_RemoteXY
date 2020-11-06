
/*
   -- wemosD1-mini-clock --
   
   This source code of graphical user interface 
   has been generated automatically by RemoteXY editor.
   To compile this code using RemoteXY library 2.4.3 or later version 
   download by link http://remotexy.com/en/library/
   To connect using RemoteXY mobile app by link http://remotexy.com/en/download/                   
     - for ANDROID 4.5.1 or later version;
     - for iOS 1.4.1 or later version;
    
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
#define REMOTEXY_CLOUD_TOKEN "2042b321e1648a82eb2be11e1436674e" //"2042b321e1648a82eb2be11e1436674e"


// конфигурация интерфейса  
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] =
  { 255,0,0,14,0,34,0,10,5,1,
  66,132,25,17,13,10,2,24,130,2,
  7,33,50,13,17,67,5,10,35,44,
  8,2,28,11,69,0,28,55,10,10,
  1 };
  
// структура определяет все переменные и события вашего интерфейса управления 
struct {

    // output variables
  int8_t level_1; // =0..100 положение уровня 
  char text_1[11];  // =строка UTF8 оканчивающаяся нулем 
  int16_t sound_1; // =0 нет звука, иначе ID звука, для примера 1001, смотри список звуков в приложении 

    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0 

} RemoteXY;
#pragma pack(pop)

/////////////////////////////////////////////
//           END RemoteXY include          //
/////////////////////////////////////////////
/*
  Добавляем необходимые библиотеки
*/
#include <Adafruit_GFX.h>
#include "Adafruit_SSD1306.h"
#define OLED_RESET -1
Adafruit_SSD1306 display(OLED_RESET);

#include <EEPROM.h>

#include <Wire.h> // must be included here so that Arduino library object file references work
#include <RtcDS3231.h>
RtcDS3231<TwoWire> Rtc(Wire);

#include <DHT.h>
DHT dht(D5, DHT22); //

int show = -1; 
float tem;
float hum;

void setup() 
{
  int error;
  RemoteXY_Init ();   
  
  // TODO you setup code
  
  dht.begin(); 
  
 // RemoteXY.sound_1 = 0;
 // See http://playground.arduino.cc/Main/I2cScanner how to test for a I2C device.
 /* Wire.begin();
  Wire.beginTransmission(0x27);
  error = Wire.endTransmission();
  REMOTEXY__DEBUGLOGS.print("Error: ");
  REMOTEXY__DEBUGLOGS.print(error);*/
 error = 0;
 REMOTEXY__DEBUGLOGS.println("");
  if (error == 0) {
    REMOTEXY__DEBUGLOGS.println(": LCD found.");
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.display();
    delay(1000);  
    display.setTextSize(2);
    display.setTextColor(WHITE);
  } else {
    REMOTEXY__DEBUGLOGS.println(": LCD not found.");
  } // if

    REMOTEXY__DEBUGLOGS.print("compiled: ");
    REMOTEXY__DEBUGLOGS.print(__DATE__);
    REMOTEXY__DEBUGLOGS.println(__TIME__);

    //--------RTC SETUP ------------
    // if you are using ESP-01 then uncomment the line below to reset the pins to
    // the available pins for SDA, SCL
    // Wire.begin(0, 2); // due to limited pins, use pin 0 and 2 for SDA, SCL
    
    Rtc.Begin();

    RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);
    printDateTime(compiled);
    REMOTEXY__DEBUGLOGS.println();

    if (!Rtc.IsDateTimeValid()) 
    {
        if (Rtc.LastError() != 0)
        {
            // we have a communications error
            // see https://www.arduino.cc/en/Reference/WireEndTransmission for 
            // what the number means
            REMOTEXY__DEBUGLOGS.print("RTC communications error = ");
            REMOTEXY__DEBUGLOGS.println(Rtc.LastError());
        }
        else
        {
            // Common Causes:
            //    1) first time you ran and the device wasn't running yet
            //    2) the battery on the device is low or even missing

            REMOTEXY__DEBUGLOGS.println("RTC lost confidence in the DateTime!");

            // following line sets the RTC to the date & time this sketch was compiled
            // it will also reset the valid flag internally unless the Rtc device is
            // having an issue

            Rtc.SetDateTime(compiled);
        }
    }

    if (!Rtc.GetIsRunning())
    {
        REMOTEXY__DEBUGLOGS.println("RTC was not actively running, starting now");
        Rtc.SetIsRunning(true);
    }

    RtcDateTime now = Rtc.GetDateTime();
    if (now < compiled) 
    {
        REMOTEXY__DEBUGLOGS.println("RTC is older than compile time!  (Updating DateTime)");
        Rtc.SetDateTime(compiled);
    }
    else if (now > compiled) 
    {
        REMOTEXY__DEBUGLOGS.println("RTC is newer than compile time. (this is expected)");
    }
    else if (now == compiled) 
    {
        REMOTEXY__DEBUGLOGS.println("RTC is the same as compile time! (not expected but all is fine)");
    }

    // never assume the Rtc was last configured by you, so
    // just clear them to your needed state
    Rtc.Enable32kHzPin(false);
    Rtc.SetSquareWavePin(DS3231SquareWavePin_ModeNone); 
 
}

void loop() 
{ 
  //RemoteXY_Handler ();
  if (RemoteXY_isConnected() == 1)
    RemoteXY.sound_1 = 1002;
  
  // TODO you loop code
  // используйте структуру RemoteXY для передачи данных
  // не используйте функцию delay() 
  hum = dht.readHumidity();    
  tem = dht.readTemperature();
  if (isnan(hum) && isnan(tem)) {
      REMOTEXY__DEBUGLOGS.println("DHT failed");
  } else {
      REMOTEXY__DEBUGLOGS.print("Temperature: ");
      REMOTEXY__DEBUGLOGS.print(tem,1);
      REMOTEXY__DEBUGLOGS.print(" Humidity: ");
      REMOTEXY__DEBUGLOGS.println(hum,1);        
  }
     
  if (!Rtc.IsDateTimeValid()) 
    {
        if (Rtc.LastError() != 0)
        {
            // we have a communications error
            // see https://www.arduino.cc/en/Reference/WireEndTransmission for 
            // what the number means
            REMOTEXY__DEBUGLOGS.print("RTC communications error = ");
            REMOTEXY__DEBUGLOGS.println(Rtc.LastError());
        }
        else
        {
            // Common Causes:
            //    1) the battery on the device is low or even missing and the power line was disconnected
            REMOTEXY__DEBUGLOGS.println("RTC lost confidence in the DateTime!");
        }
    }

    RtcDateTime now = Rtc.GetDateTime();

    displayTimeTemp(now,tem,hum);
    
    printDateTime(now);
    REMOTEXY__DEBUGLOGS.println();

  RtcTemperature temp = Rtc.GetTemperature();
  temp.Print(Serial);
  // you may also get the temperature as a float and print it
    // Serial.print(temp.AsFloatDegC());
    REMOTEXY__DEBUGLOGS.println("C");

 
    delay(10000); // ten seconds
}

#define countof(a) (sizeof(a) / sizeof(a[0]))

void displayTimeTemp(const RtcDateTime& dt,float tem,float hum){
  char datestring[7];
  snprintf_P(datestring, 
            countof(datestring),
            PSTR("%02u:%02u"),
            dt.Hour(),
            dt.Minute()
            );
  display.clearDisplay();
  display.setCursor(0,0);
  display.println(datestring);  

  display.print(round(tem),0);
  display.println(" c");
  display.print(round(hum),0);
  display.print(" %");
  
  display.display();        
}

void printDateTime(const RtcDateTime& dt)
{
    char datestring[20];

    snprintf_P(datestring, 
            countof(datestring),
            PSTR("%02u.%02u.%04u %02u:%02u:%02u"),
            dt.Day(),
            dt.Month(),            
            dt.Year(),
            dt.Hour(),
            dt.Minute(),
            dt.Second() );
    REMOTEXY__DEBUGLOGS.print(datestring);
}
