
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
   { 255,5,0,45,0,67,0,10,13,1,
  66,132,19,16,29,23,2,24,2,0,
  21,56,24,13,2,26,31,31,79,78,
  0,79,70,70,0,7,44,21,44,23,
  7,2,26,2,2,65,36,6,22,13,
  13,66,0,49,18,9,21,2,26,67,
  5,3,92,57,4,2,26,41,65,34,
  31,28,5,5 };
  
// структура определяет все переменные и события вашего интерфейса управления 
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

#define PIN_RELAY D6 // relay
#define PIN_DIOD D0
#define PIN_BUTTON D8 // pull down on Wemos D1

float tem;
float hum;
float set_tem;
int tem_last;
int hum_last;
uint8_t power_on;
uint8_t display_show = 1; // display time and temperatur
boolean relay = false;    // relay off, true - on
boolean light = true;     // diod lighting, false - pause
boolean button = false;   // true - button press
uint32_t dhtTimeOut;
uint32_t rtcTimeOut;
uint32_t displayTimeOut;

void setup() 
{
  int error;
  RemoteXY_Init ();   
  
  // TODO you setup code
  rtcTimeOut = dhtTimeOut = displayTimeOut = millis();
  
  //--------PIN SETUP ------------
  pinMode(PIN_BUTTON, INPUT);
  pinMode (PIN_RELAY, OUTPUT);
  pinMode(PIN_DIOD, OUTPUT);
  
  //--------EEPROM SETUP ------------
  EEPROM.begin(4);
  set_tem = EEPROM.read(0)*10 + EEPROM.read(1);
  set_tem /= 10;
  power_on = EEPROM.read(2);
  RemoteXY.T_min = set_tem;
  RemoteXY.switch_1 = power_on;

  //--------DHT SETUP ------------  
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
     
  
 // RemoteXY.sound_1 = 0;
   REMOTEXY__DEBUGLOGS.println("");
   //--------Display SETUP ------------
   // See http://playground.arduino.cc/Main/I2cScanner how to test for a I2C device.
   display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
   display.display();
   display.setTextSize(2);
   display.setTextColor(WHITE);
 

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
  RemoteXY_Handler ();
  //if (RemoteXY_isConnected() == 1)
    //RemoteXY.sound_1 = 1002;
  
  // TODO you loop code
  // используйте структуру RemoteXY для передачи данных
  // не используйте функцию delay() 

  if ((millis() - dhtTimeOut) > 60000){ // 1 min. 
    REMOTEXY__DEBUGLOGS.println();
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
    RtcTemperature temp = Rtc.GetTemperature();
   
    temp.Print(Serial);
    // you may also get the temperature as a float and print it
    // Serial.print(temp.AsFloatDegC());
    REMOTEXY__DEBUGLOGS.println("C");
    dhtTimeOut = millis();
  }
  
  if ((millis() - rtcTimeOut) > 30000){ // 30 sec.
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
              //    1) the battery on the device is low or even missing and the power line was disconnected
              REMOTEXY__DEBUGLOGS.println("RTC lost confidence in the DateTime!");
          }
      }
  
      RtcDateTime now = Rtc.GetDateTime();
  
      displayTimeTemp(now,tem,hum);
      
      printDateTime(now);
      
      rtcTimeOut = millis();
    }

    if (digitalRead(PIN_BUTTON)) {
      if (!button) //подождем пока отпустишь
        button = true;
    } else {
      if (button) {// дождались, отпустил
        if (display_show){
          display_show = 0;
        } else {
          display_show = 1;                    
        }
        displayTimeTemp(Rtc.GetDateTime(),tem,hum);
        button = false;
      }         
    }
    
    
}

#define countof(a) (sizeof(a) / sizeof(a[0]))

void displayTimeTemp(const RtcDateTime& dt,float tem,float hum){
  char datestring[7];
  display.clearDisplay();
  if (display_show){
    snprintf_P(datestring, 
              countof(datestring),
              PSTR("%02u:%02u"),
              dt.Hour(),
              dt.Minute()
              );
   
    display.setCursor(0,0);
    display.println(datestring);  
  
    display.print(round(tem),0);
    display.println(" c");
    display.print(round(hum),0);
    display.print(" %");
  }  
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
    REMOTEXY__DEBUGLOGS.println(datestring);
}

boolean power(float diff){
  int result;
  if (diff < 0 || RemoteXY.switch_1 == 0){
    digitalWrite(PIN_RELAY, LOW);
    light = false;
    digitalWrite(PIN_DIOD, light);
    RemoteXY.led_1_r = 0;
    result = false;
  REMOTEXY__DEBUGLOGS.println("Power off.");
  }
  if (diff > 0 && RemoteXY.switch_1 != 0){
    digitalWrite(PIN_RELAY, HIGH);
    light = true;
    digitalWrite(PIN_DIOD, light);
    RemoteXY.led_1_r = 196;
    result = true;
  REMOTEXY__DEBUGLOGS.println("Power on!");
  }  
  return result;  
}
