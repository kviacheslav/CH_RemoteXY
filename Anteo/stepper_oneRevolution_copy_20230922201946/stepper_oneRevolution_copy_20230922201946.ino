
/*
 Stepper Motor Control - one revolution

 This program drives a unipolar or bipolar stepper motor.
 The motor is attached to digital pins 8 - 11 of the Arduino.

 The motor should revolve one revolution in one direction, then
 one revolution in the other direction.


 Created 11 Mar. 2007
 Modified 30 Nov. 2009
 by Tom Igoe

 */

//#include <Stepper.h>

#include <Arduino.h>
//#define MAVLINK_STX 253 //mavlink2
#include <mavlink1.h>
#include "mavlink_msg_rc_channels.h"
int leds_modo = 1;
mavlink_rc_channels_t rc_channels;
mavlink_rc_channels_raw_t rc_channels_raw;



//#include <IBusBM.h>
//IBusBM IBus; // IBus object

#include <Servo.h>
Servo myservo;  // create servo object to control a servo

#include <SoftwareSerial.h>
SoftwareSerial mp3_1_Serial(A0, 4); // RX (D14), TX
SoftwareSerial mp3_2_Serial(A1, 7); // RX (D15), TX

#include "DFRobotDFPlayerMini.h"
DFRobotDFPlayerMini DFPlayer1;
DFRobotDFPlayerMini DFPlayer2;
void printDetail(uint8_t type, int value);


#include <AccelStepper.h>
AccelStepper s28BYJ_48(8, 8, 10, 9, 11);


// for your motor
//const int stepsPerRevolution = 200;  // change this to fit the number of steps per revolution
// initialize the stepper library on pins 8 through 11:
//Stepper myStepper(stepsPerRevolution, 8, 10, 9, 11);

void setup() {
  // iBUS connected to serial
  //IBus.begin(Serial);    // iBUS connected to Serial0 - change to Serial1 or Serial2 port when required
  Serial.begin(57600);

  myservo.attach(3);  // attaches the servo on pin 9 to the servo object

  Serial.println("Start IBus2PWM");
  // set the speed at 60 rpm:
  //myStepper.setSpeed(60);
  s28BYJ_48.setMaxSpeed(900.0);
  s28BYJ_48.setAcceleration(100.0);
  s28BYJ_48.setSpeed(400);
  s28BYJ_48.moveTo(4076); //~2 оборота

  mp3_2_Serial.begin(9600);  
  Serial.println();  
  Serial.println(F("Initializing DFPlayer 2 ... (May take 3~5 seconds)")); // Инициализация модуля
  if (!DFPlayer2.begin(mp3_2_Serial, true, true)) {  //Use softwareSerial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!")); // Проверьте соединение
    Serial.println(F("2.Please insert the SD card!")); // Вставьте SD карту
    while(true);
  }
  Serial.println(F("DFPlayer 2 online.")); // Готов к работе
  DFPlayer2.volume(30);  // Установка громкости от 0 до 30
  //myDFPlayer.play(1);  // Проигрывать с первого MP3 файла
  mp3_1_Serial.begin(9600);  
  Serial.println();  
  Serial.println(F("Initializing DFPlayer 1 ... (May take 3~5 seconds)")); // Инициализация модуля
  if (!DFPlayer1.begin(mp3_1_Serial, true, true)) {  //Use softwareSerial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!")); // Проверьте соединение
    Serial.println(F("2.Please insert the SD card!")); // Вставьте SD карту
    while(true);
  }
  Serial.println(F("DFPlayer 1 online.")); // Готов к работе
  DFPlayer1.volume(30);  // Установка громкости от 0 до 30

  
}
int saveval=0;
int chan1_raw=0;
int saveval_key4=0;
int saveval_key9=0;
int saveral_motor = 0;
uint8_t player_type;
int player_value;

void loop() {
  int val;
  int motor;

  mavlink_msg_receive();

  //Запуск двигателя (Armed) 
  val = 0;//IBus.readChannel(9); // get latest value for servo channel 9 (swD)
  if (saveval_key9 != val) {
    Serial.print(9);
    Serial.print(":");
    Serial.println(val); // display new value in microseconds on PC
    saveval_key9 = val;
    //DFPlayer1.volume(1);
    if (saveval_key9 > 1500){
      DFPlayer1.play(1);
      DFPlayer1.loop(1);       
    }   
    else{
      DFPlayer1.pause();
    }   
  } 

  val = 0;//IBus.readChannel(0); // get latest value for servo channel 1 (rudder)
  val = mavlink_msg_rc_channels_get_chan1_raw((mavlink_message_t *)&rc_channels);
  if (saveval != val) {
    Serial.print(1);
    Serial.print(" mavlink2:");
    Serial.println(val); // display new value in microseconds on PC
    saveval = val;    
    myservo.writeMicroseconds(val);   // sets the servo position 
  }
  val = rc_channels_raw.chan1_raw;//mavlink_msg_rc_channels_raw_get_chan1_raw((mavlink_message_t *)&rc_channels_raw);
  if (chan1_raw != val) {
    Serial.print(1);
    Serial.print(" mavlink1:");
    Serial.println(val); // display new value in microseconds on PC
    chan1_raw = val;    
    myservo.writeMicroseconds(val);   // sets the servo position 
  }
  //Гудок
  val = rc_channels_raw.chan5_raw;//IBus.readChannel(4); // get latest value for servo channel 4 (key left)
  if (saveval_key4 != val) {
    Serial.print(4);
    Serial.print(":");
    Serial.println(val); // display new value in microseconds on PC
    saveval_key4 = val;
    if (saveval_key4 > 1500)
      DFPlayer2.loop(2);
    else
      DFPlayer2.pause();    
  }
  

  val = rc_channels_raw.chan3_raw;//IBus.readChannel(2); // get latest value for servo channel 3 (motor)  
    
  motor = constrain(val, 1000, 2000);
  motor = map(motor,1000,2000,-30,30);
  if (saveral_motor != motor) {
    Serial.print(2);
    Serial.print(":");
    Serial.println(motor); // display new value in microseconds on PC
    DFPlayer1.volume(constrain(abs(motor),1,30));
    saveral_motor = motor;   
  } 

  if (DFPlayer1.available()) {
    Serial.println("a1");
    mp3_1_Serial.listen();
    player_type = DFPlayer1.readType();
    player_value = DFPlayer1.read();
    
    if ((player_type == DFPlayerPlayFinished) && (saveval_key9 > 1500)){
      //DFPlayer1.enableLoopAll();//(plaer_value);
      DFPlayer1.play(player_value);
    }

    printDetail(player_type, player_value,1); // Выводить сообщения о статусе и ошибках модуля
  }

  if (DFPlayer2.available()) {
    Serial.println("a2");
    //mp3_2_Serial.listen();
    printDetail(DFPlayer2.readType(), DFPlayer2.read(),2); // Выводить сообщения о статусе и ошибках модуля
  }
  
  // step one revolution  in one direction:
  //Serial.println("clockwise");
  //myStepper.step(stepsPerRevolution);
  //delay(500);

  // step one revolution in the other direction:
  //Serial.println("-counterclockwise");
  //myStepper.step(-stepsPerRevolution);
  //delay(500);
  // Изменяем направление, если пройдено заданное число шагов
  if(s28BYJ_48.distanceToGo()==0)
    s28BYJ_48.moveTo(-s28BYJ_48.currentPosition());
  s28BYJ_48.run();

}

void printDetail(uint8_t type, int value, int player){
  switch (type) {
    case TimeOut:
      Serial.print(player);
      Serial.println(F("Time Out!")); // Время вышло
      break;
    case WrongStack:
      Serial.print(player);
      Serial.println(F("Stack Wrong!"));
      break;
    case DFPlayerCardInserted:
      Serial.print(player);
      Serial.println(F("Card Inserted!")); // Карта вставлена
      break;
    case DFPlayerCardRemoved:
      Serial.print(player);
      Serial.println(F("Card Removed!")); // Карта удалена
      break;
    case DFPlayerCardOnline:
      Serial.print(player);
      Serial.println(F("Card Online!")); // Карта готова
      break;
    case DFPlayerPlayFinished:
      Serial.print(player);
      Serial.print(F("Number:")); // Номер
      Serial.print(value);
      Serial.println(F(" Play Finished!")); // Воспроизведение окончено
      break;
    case DFPlayerError:
      Serial.print(player);
      Serial.print(F("DFPlayerError:")); // Ошибка
      switch (value) {
        case Busy:
          Serial.println(F("Card not found")); // Карта не найдена
          break;
        case Sleeping:
          Serial.println(F("Sleeping")); // Сон
          break;
        case SerialWrongStack:
          Serial.println(F("Get Wrong Stack"));
          break;
        case CheckSumNotMatch:
          Serial.println(F("Check Sum Not Match"));
          break;
        case FileIndexOut:
          Serial.println(F("File Index Out of Bound"));
          break;
        case FileMismatch:
          Serial.println(F("Cannot Find File")); // Не могу найти файл
          break;
        case Advertise:
          Serial.println(F("In Advertise")); // Реклама
          break;
        default:
          break;
      }      
      break;
    default:
      break;
  }
}

void mavlink_msg_receive() {
  mavlink_message_t msg;
  mavlink_status_t status;

  while(Serial.available()>0) {
    uint8_t c = Serial.read();
    //Serial.println(c, HEX);
    // Try to get a new message
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      if (msg.msgid != 0)
        Serial.println(msg.msgid, DEC);
      // Handle message
      switch(msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:  // #0: Heartbeat
          {
            // E.g. read GCS heartbeat and go into
            // comm lost mode if timer times out
            //Serial.println(F("HEARTBEAT"));
          }
          break;

        case MAVLINK_MSG_ID_SYS_STATUS:  // #1: SYS_STATUS
          {
            /* Message decoding: PRIMITIVE
             *    mavlink_msg_sys_status_decode(const mavlink_message_t* msg, mavlink_sys_status_t* sys_status)
             */
            //mavlink_message_t* msg;
            mavlink_sys_status_t sys_status;
            mavlink_msg_sys_status_decode(&msg, &sys_status);
          }
          break;

        case MAVLINK_MSG_ID_RC_CHANNELS_RAW:  // #35: RC_CHANNELS_RAW
          {
            /* Message decoding: PRIMITIVE
             *    void mavlink_msg_rc_channels_decode(const mavlink_message_t* msg, mavlink_rc_channels_t* rc_channels)
             */
            //mavlink_message_t* msg;
            //mavlink_rc_channels_t rc_channels;
            mavlink_msg_rc_channels_raw_decode(&msg, &rc_channels_raw);
            /*
            Serial.print("35: ");           
            Serial.print(rc_channels_raw.chan1_raw);
            Serial.print(" ");
            Serial.print(rc_channels_raw.chan2_raw);
            Serial.print(" ");
            Serial.print(rc_channels_raw.chan3_raw);
            Serial.print(" ");
            Serial.print(rc_channels_raw.chan4_raw);
            Serial.println(";");
            */ 
          }
          break;

        case MAVLINK_MSG_ID_RC_CHANNELS:  // #65: RC_CHANNELS
          {
            /* Message decoding: PRIMITIVE
             *    void mavlink_msg_rc_channels_decode(const mavlink_message_t* msg, mavlink_rc_channels_t* rc_channels)
             */
            //mavlink_message_t* msg;
            //mavlink_rc_channels_t rc_channels;
            mavlink_msg_rc_channels_decode(&msg, &rc_channels);
          }
          break;

        case MAVLINK_MSG_ID_RAW_IMU:  // #27: RAW_IMU
          {
            /* Message decoding: PRIMITIVE
             *    static inline void mavlink_msg_raw_imu_decode(const mavlink_message_t* msg, mavlink_raw_imu_t* raw_imu)
             */
            mavlink_raw_imu_t raw_imu;
            mavlink_msg_raw_imu_decode(&msg, &raw_imu);
          }
          break;

        case MAVLINK_MSG_ID_ATTITUDE:  // #30
          {
            /* Message decoding: PRIMITIVE
             *    mavlink_msg_attitude_decode(const mavlink_message_t* msg, mavlink_attitude_t* attitude)
             */
            mavlink_attitude_t attitude;
            mavlink_msg_attitude_decode(&msg, &attitude);

            if(attitude.roll>1) leds_modo = 0;
            else if(attitude.roll<-1) leds_modo = 2;
            else leds_modo=1;
          }
          break;

        
       default:
          break;
      }
    }
  }
}

