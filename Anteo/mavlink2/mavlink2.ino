//---------------
// RemoteXY определение режима соединения и подключение библиотеки 
#define REMOTEXY_MODE__ESP8266WIFI_LIB_POINT
#include <ESP8266WiFi.h>
#include <RemoteXY.h>

// настройки соединения 
#define REMOTEXY_WIFI_SSID "Anteo"
#define REMOTEXY_WIFI_PASSWORD "12345678"
#define REMOTEXY_SERVER_PORT 6377

// конфигурация интерфейса  
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] =   // 37 bytes
  { 255,1,0,4,0,30,0,16,0,1,1,0,26,40,12,12,2,31,208,147,
  209,131,208,180,208,190,208,186,0,68,17,23,65,20,20,8,36 };
// структура определяет все переменные и события вашего интерфейса управления 
struct {

    // input variables
  uint8_t button_1; // =1 если кнопка нажата, иначе =0 

    // output variables
  float onlineGraph_1;

    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0 

} RemoteXY;
#pragma pack(pop)

//---------------------
// My Servo
#include <Servo.h>
Servo myservo;  // create servo object to control a servo
#define MY_SERVO_PIN 0 // wemos: D3

//---------------------
// Speed sensor digital
#define SPEED_SENSOR_PIN 13 // Wemos D7 (см. выше на serial1)
volatile int pulseCount = 0;
int speedCurrent = 0;

// ServoDriver I2C 
#include <Adafruit_PWMServoDriver.h>
#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
Adafruit_PWMServoDriver servo_driver = Adafruit_PWMServoDriver(); //0x40

// Mavlink UART 57600
#include <mavlink_ardupilot.h>
unsigned long previousMillisMAVLink = 0;     // will store last time MAVLink was transmitted and listened
unsigned long next_interval_MAVLink = 1000;  // next interval to count
const int num_hbs = 60;                      // # of heartbeats to wait before activating STREAMS from Pixhawk. 60 = one minute.
int num_hbs_pasados = num_hbs;
mavlink_rc_channels_t rc_channels;
mavlink_rc_channels_raw_t rc_channels_raw;
#define MAVLINK_SERIAL Serial

// Player
#include <SoftwareSerial.h>
SoftwareSerial mp3_2_Serial(14, 12); // (RX,TX) Wemos D1: D5, D6 
//SoftwareSerial serial1(13, 15); // (RX,TX) Wemos D1: D7, D8
#include "DFRobotDFPlayerMini.h"
DFRobotDFPlayerMini DFPlayer2;
void printDetail(uint8_t type, int value);

void setup() {
  // RemoteXY
  RemoteXY_Init ();

  // Display
   pinMode(LED_BUILTIN, OUTPUT); // Wemos: D4

  // Speed sensor
  pinMode(D7, INPUT);
  attachInterrupt(digitalPinToInterrupt(D7), countPulses, RISING); //SPEED_SENSOR_PIN

  //mavlink 
  //Serial.begin(57600);
  MAVLINK_SERIAL.begin(57600);    
 
  //servo
  myservo.attach(MY_SERVO_PIN);

  //ServoDriver I2C
  servo_driver.begin();  
  /*
   * In theory the internal oscillator (clock) is 25MHz but it really isn't
   * that precise. You can 'calibrate' this by tweaking this number until
   * you get the PWM update frequency you're expecting!
   * The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
   * is used for calculating things like writeMicroseconds()
   * Analog servos run at ~50 Hz updates, It is importaint to use an
   * oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
   * 1) Attach the oscilloscope to one of the PWM signal pins and ground on
   *    the I2C PCA9685 chip you are setting the value for.
   * 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
   *    expected value (50Hz for most ESCs)
   * Setting the value here is specific to each individual I2C PCA9685 chip and
   * affects the calculations for the PWM update frequency. 
   * Failure to correctly set the int.osc value will cause unexpected PWM results
   
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  delay(10);
  */
  servo_driver.setOscillatorFrequency(27000000);  
  servo_driver.setPWMFreq(SERVO_FREQ);
  delay(500);

  //player
  mp3_2_Serial.begin(9600);  
  Serial.println("");  
  Serial.println(F("Initializing DFPlayer 2 ... (May take 3~5 seconds)")); // Инициализация модуля
  if (!DFPlayer2.begin(mp3_2_Serial, true, true)) {  //Use softwareSerial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!")); // Проверьте соединение
    Serial.println(F("2.Please insert the SD card!")); // Вставьте SD карту
    
  }
  
}

int chan1_raw2=0;
int chan1_raw1=0;
int saveval_key4=0;
uint8_t button_1=0;

void loop() {
  //Serial.println(".");
  int val;
    // MAVLink
  /* The default UART header for your MCU */ 
  int sysid = 255;                   ///< ID 20 for this airplane. 1 PX, 255 ground station
  int compid = 158;                ///< The component sending the message
  int type = MAV_TYPE_QUADROTOR;   ///< This system is an airplane / fixed wing
 
  // Define the system type, in this case an airplane -> on-board controller
  // uint8_t system_type = MAV_TYPE_FIXED_WING;
  uint8_t system_type = MAV_TYPE_GENERIC;
  uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;
 
  uint8_t system_mode = MAV_MODE_PREFLIGHT; ///< Booting up
  uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter
  uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight
  // Initialize the required buffers
  /*
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
 
  // Pack the message
  //mavlink_msg_heartbeat_pack(sysid,compid, &msg, type, autopilot_type, system_mode, custom_mode, system_state);
  mavlink_msg_heartbeat_pack(255,1, &msg, type, autopilot_type, system_mode, custom_mode, system_state);
 
  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
 
  // Send the message with the standard UART send function
  // uart0_send might be named differently depending on
  // the individual microcontroller / library in use.
  */

  RemoteXY_Handler ();

  unsigned long currentMillisMAVLink = millis();
  if (currentMillisMAVLink - previousMillisMAVLink >= next_interval_MAVLink) {
    // Guardamos la última vez que se cambió el modo
    previousMillisMAVLink = currentMillisMAVLink;
    
    // Speed
    if (pulseCount > 0){ 
      Serial.print("speed ");
      Serial.println(pulseCount);
    } else if (speedCurrent > 0){
      Serial.println("speed 0");
    }
    RemoteXY.onlineGraph_1 = 60.0 * pulseCount; // Оборотов в минуту
    speedCurrent = pulseCount; // Оборотов в секунду - next_interval_MAVLink
    pulseCount = 0;
    
    //Mav_Request_Data();
    
    
    if(num_hbs_pasados>0) {
      // Request streams from Pixhawk
      //Mav_Request_Data();
      //Serial.println(num_hbs_pasados);
      digitalWrite(LED_BUILTIN, HIGH);
      num_hbs_pasados=0;
    }
    else{
      //Serial.println(num_hbs_pasados);
      //DFPlayer2.play(2);
      digitalWrite(LED_BUILTIN, LOW);
      num_hbs_pasados=1;
    }

  }  

  if (button_1 != RemoteXY.button_1) {
    button_1 = RemoteXY.button_1;
    Serial.print("button_1:");
    Serial.println(button_1); 
    if (button_1 > 0){
      DFPlayer2.play(2);
      //DFPlayer2.loop(1);       
    }
    else{
      DFPlayer2.pause();  
    } 
  }

  mavlink_msg_receive();

  //Руль
  val = rc_channels.chan1_raw;//mavlink_msg_rc_channels_get_chan1_raw((mavlink_message_t *)&rc_channels);
  if (chan1_raw2 != val) {
    Serial.print(1);
    Serial.print(" mavlink2:");
    Serial.println(val); // display new value in microseconds on PC
    chan1_raw2 = val;    
    myservo.writeMicroseconds(val);   // sets the servo position 
    servo_driver.writeMicroseconds(1,val);
  }
  val = rc_channels_raw.chan1_raw;//mavlink_msg_rc_channels_get_chan1_raw((mavlink_message_t *)&rc_channels);
  if (chan1_raw1 != val) {
    Serial.print(1);
    Serial.print(" mavlink1:");
    Serial.println(val); // display new value in microseconds on PC
    chan1_raw1 = val;    
    //myservo.writeMicroseconds(val);   // sets the servo position 
  }

  //Гудок
  val = rc_channels.chan5_raw; // get latest value for servo channel 4 (key left)
  val = constrain(val, 1000, 2000);
  val = map(val,1000,2000,-1,1); //map(value, fromLow, fromHigh, toLow, toHigh)
  if (saveval_key4 != val) {
    Serial.print(4);
    Serial.print(":");
    Serial.println(val); // display new value in microseconds on PC
    saveval_key4 = val;
    if (saveval_key4 > 0){
      DFPlayer2.play(2);
      //DFPlayer2.loop(1);       
    }
    else{
      DFPlayer2.pause();  
    } 
  }

  if (DFPlayer2.available()) {
    //mp3_2_Serial.listen();
    printDetail(DFPlayer2.readType(), DFPlayer2.read(),2); // Выводить сообщения о статусе и ошибках модуля
  }
  
}

IRAM_ATTR void countPulses() {
  pulseCount++;
}

void mavlink_msg_receive() {
  mavlink_message_t msg;
  mavlink_status_t status;

  while(MAVLINK_SERIAL.available()>0) {
    uint8_t c = MAVLINK_SERIAL.read();
    //Serial.println(c, HEX);
    // Try to get a new message
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      if (msg.msgid != 0)
        ;//Serial.println(msg.msgid, DEC);
      // Handle message
      switch(msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:  // #0: Heartbeat
          {
            // E.g. read GCS heartbeat and go into
            // comm lost mode if timer times out
            /*
            Serial.print(F("HEARTBEAT "));
            Serial.print(msg.sysid);
            Serial.print(" ");
            Serial.println(msg.compid);
            */
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
            //Serial.println(F("CHANNELS35"));
            //mavlink_msg_rc_channels_raw_decode(&msg, &rc_channels_raw);
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
            //Serial.println(F("CHANNELS65"));
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

            //if(attitude.roll>1) leds_modo = 0;
            //else if(attitude.roll<-1) leds_modo = 2;
            //else leds_modo=1;
          }
          break;

        
       default:
          break;
      }
    }
  }
}

void Mav_Request_Data()
{
  //mavlink_message_t msg;
  //uint8_t buf[MAVLINK_MAX_PACKET_LEN];


  // STREAMS that can be requested
  /*
   * Definitions are in common.h: enum MAV_DATA_STREAM
   *   
   * MAV_DATA_STREAM_ALL=0, // Enable all data streams
   * MAV_DATA_STREAM_RAW_SENSORS=1, /* Enable IMU_RAW, GPS_RAW, GPS_STATUS packets.
   * MAV_DATA_STREAM_EXTENDED_STATUS=2, /* Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS
   * MAV_DATA_STREAM_RC_CHANNELS=3, /* Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW
   * MAV_DATA_STREAM_RAW_CONTROLLER=4, /* Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT.
   * MAV_DATA_STREAM_POSITION=6, /* Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages.
   * MAV_DATA_STREAM_EXTRA1=10, /* Dependent on the autopilot
   * MAV_DATA_STREAM_EXTRA2=11, /* Dependent on the autopilot
   * MAV_DATA_STREAM_EXTRA3=12, /* Dependent on the autopilot
   * MAV_DATA_STREAM_ENUM_END=13,
   * 
   * Data in PixHawk available in:
   *  - Battery, amperage and voltage (SYS_STATUS) in MAV_DATA_STREAM_EXTENDED_STATUS
   *  - Gyro info (IMU_SCALED) in MAV_DATA_STREAM_EXTRA1
   */

  // To be setup according to the needed information to be requested from the Pixhawk
  const int  maxStreams = 2;
  const uint8_t MAVStreams[maxStreams] = {MAV_DATA_STREAM_EXTENDED_STATUS, MAV_DATA_STREAM_EXTRA1};
  const uint16_t MAVRates[maxStreams] = {0x02,0x05};

    
  for (int i=0; i < maxStreams; i++) {
    /*
     * mavlink_msg_request_data_stream_pack(system_id, component_id, 
     *    &msg, 
     *    target_system, target_component, 
     *    MAV_DATA_STREAM_POSITION, 10000000, 1);
     *    
     * mavlink_msg_request_data_stream_pack(uint8_t system_id, uint8_t component_id, 
     *    mavlink_message_t* msg,
     *    uint8_t target_system, uint8_t target_component, uint8_t req_stream_id, 
     *    uint16_t req_message_rate, uint8_t start_stop)
     * 
     */
     /*
    mavlink_msg_request_data_stream_pack(255,1, &msg, 1, 1, MAVStreams[i], MAVRates[i], 1);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial.print(MAVStreams[i]);
    Serial.print("-");
    Serial.println(msg.magic);    
    Serial1.write(buf, len);
    */

  }
  
  // Request: PARAM_REQUEST_LIST. Only for full log recording
  /*
   * Primitive: mavlink_msg_param_request_list_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                   uint8_t target_system, uint8_t target_component)
   */
/*
  // Configure
  uint8_t system_id=2;
  uint8_t component_id=200;
  // mavlink_message_t* msg;
  uint8_t target_system=1;
  uint8_t target_component=0;

  // Pack
  mavlink_msg_param_request_list_pack(system_id, component_id, &msg,
    target_system, target_component);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  // Send
#ifdef SOFT_SERIAL_DEBUGGING
    pxSerial.write(buf,len);
#else
    Serial.write(buf, len);
#endif
*/
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
      if (player == 2){
        DFPlayer2.play(value);
      }
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


