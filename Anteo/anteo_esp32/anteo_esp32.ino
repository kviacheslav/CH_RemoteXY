//---------------
// RemoteXY
// определение режима соединения и подключение библиотеки RemoteXY 
#define REMOTEXY_MODE__ESP32CORE_BLE
#include <BLEDevice.h>
#include <RemoteXY.h>

// настройки соединения 
#define REMOTEXY_BLUETOOTH_NAME "ANTEO"

// конфигурация интерфейса  
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] =   // 115 bytes
  { 255,2,0,8,0,108,0,16,160,1,1,0,26,32,12,12,2,31,208,147,
  209,131,208,180,208,190,208,186,0,71,56,9,48,45,45,18,2,24,165,0,
  0,200,194,0,0,200,66,0,0,200,65,0,0,0,0,0,0,160,64,24,
  0,94,0,0,200,193,0,0,200,65,1,0,0,200,193,0,0,200,194,67,
  6,26,94,13,5,2,26,6,129,0,41,95,10,3,17,208,190,208,177,47,
  208,188,208,184,208,189,0,3,131,2,8,32,12,2,26 };
  
// структура определяет все переменные и события вашего интерфейса управления 
struct {

    // input variables
  uint8_t button_1; // =1 если кнопка нажата, иначе =0 
  uint8_t select_1; // =0 если переключатель в положении A, =1 если в положении B, =2 если в положении C, ... 

    // output variables
  int16_t telegraph;  // oт -100 до 100 
  char rotation[6];  // =строка UTF8 оканчивающаяся нулем 

    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0 

} RemoteXY;
#pragma pack(pop)

//---------------
// Mavlink2
#include <mavlink_ardupilot.h>
unsigned long previousMillisMAVLink = 0;     // will store last time MAVLink was transmitted and listened
unsigned long next_interval_MAVLink = 1000;  // next interval to count (1 cek)
const int num_hbs = 60;                      // # of heartbeats to wait before activating STREAMS from Pixhawk. 60 = one minute.
int num_hbs_pasados = num_hbs;

// конфигурация пакетов
mavlink_rc_channels_t rc_channels;
mavlink_servo_output_raw_t servo_output_raw;
mavlink_rc_channels_raw_t rc_channels_raw;
mavlink_sys_status_t sys_status;
mavlink_vfr_hud_t vfr_hud;
#define MAX_STATUSTEXT 10
mavlink_statustext_t statustext[MAX_STATUSTEXT];
uint8_t next_statustext = 0;
uint8_t prev_statustext = 0;
uint8_t mavlink_heartbeat = LOW;

// UART0 (Serial) по умолчанию подключается к пинам GPIO1 (TX0) и GPIO3 (RX0)
#define MAVLINK_SERIAL Serial

/*
//---------------------
// My Servo
#include <Servo.h>
Servo myservo;  // create servo object to control a servo
#define MY_SERVO_PIN 0 // wemos: D3
*/

//---------------------
// Speed sensor digital
#define SPEED_SENSOR_PIN 13 // Wemos D7 (см. выше на serial1)
volatile int pulseCount = 0;
#define ROTATION_ACCUM_N 5
int  rotationAccum[ROTATION_ACCUM_N];
int  rotationAccumCnt = 0;


// ServoDriver I2C 
#include <Adafruit_PWMServoDriver.h>
#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
Adafruit_PWMServoDriver servo_driver = Adafruit_PWMServoDriver(); //0x40
#define Servo servo_driver


// Player
uint8_t statusPlayer = 0;
//#include <SoftwareSerial.h>
//SoftwareSerial mp3_2_Serial(14, 12); // (RX,TX) Wemos D1: D5, D6 
//SoftwareSerial serial1(13, 15); // (RX,TX) Wemos D1: D7, D8
#define PlayerSerial Serial2
#include "DFRobotDFPlayerMini.h"
DFRobotDFPlayerMini DFPlayer2;
void printDetail(uint8_t type, int value);
#define Player DFPlayer2

// TFT
 #include <TFT_eSPI.h>
 TFT_eSPI tft = TFT_eSPI();

#define MAX_DISPLAYTEXT 5
mavlink_statustext_t displaytext[MAX_DISPLAYTEXT];
uint8_t next_displaytext = 0;

#define LED_BUILTIN 2
void setup() {
  // RemoteXY
  RemoteXY_Init ();

  //mavlink
  //Serial.begin(57600); 
  MAVLINK_SERIAL.begin(57600); 

  // Display
   pinMode(LED_BUILTIN, OUTPUT); // GPIO2

 // Speed sensor
  #ifdef SPEED_SENSOR_PIN
  pinMode(SPEED_SENSOR_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(SPEED_SENSOR_PIN), countPulses, FALLING); //SPEED_SENSOR_PIN
  #endif
 
  //servo
  #ifdef MY_SERVO_PIN
  myservo.attach(MY_SERVO_PIN);
  #endif

  //ServoDriver I2C
  #ifdef Servo
  Servo.begin();  
  Servo.setOscillatorFrequency(27000000);  
  Servo.setPWMFreq(SERVO_FREQ);
  delay(500);
  #endif

  //player

  #ifdef Player
  PlayerSerial.begin(9600);  
  Serial.println("");  
  Serial.println(F("Initializing DFPlayer 2 ... (May take 3~5 seconds)")); // Инициализация модуля
  
  if (!Player.begin(PlayerSerial, true, true)) {  //Use softwareSerial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!")); // Проверьте соединение
    Serial.println(F("2.Please insert the SD card!")); // Вставьте SD карту
    
  } else {
    statusPlayer = 1;
  }

  #endif

  //TFT
  tft.init();
  //tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.drawLine(10, 130, 150, 130, TFT_GREEN);

  sprintf(displaytext[next_displaytext].text, "Hello! Player %s",(statusPlayer == 0)?"unanble" : "ok");
  Display_Text();
  
} 

int chan1_raw2 = 0;
int chan1_raw1 = 0;
int saveval_key4 = -1;
uint8_t button_1 = 0;
uint8_t select_1 = 0;
uint8_t display = LOW;

void loop() {
  int val;
    // MAVLink
  /* The default UART header for your MCU */ 
  /*
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
  */
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
  int delta = currentMillisMAVLink - previousMillisMAVLink;
  if (delta >= next_interval_MAVLink) {
    // Guardamos la última vez que se cambió el modo
    previousMillisMAVLink = currentMillisMAVLink;
    // Мигает, значит жив
    if (display == LOW)
      display = HIGH;
    else
      display = LOW; 
    digitalWrite(LED_BUILTIN, display);
    // Шлёт, значит жив
    if (mavlink_heartbeat == HIGH) {
      tft.drawString((display == HIGH)? "*" : " ", 5, 0, 2);
      mavlink_heartbeat = LOW;
    } else {
      tft.drawString("?", 5, 0, 2);
    }

    // Speed
    // rotation - среднек значение за N замеров
    rotationAccum[rotationAccumCnt] = pulseCount;
    if (++rotationAccumCnt >= ROTATION_ACCUM_N)
      rotationAccumCnt = 0;
    int rotation = 0;
    for (int i = 0; i < ROTATION_ACCUM_N; i++) 
      rotation += rotationAccum[i];
    rotation = rotation / ROTATION_ACCUM_N;  
    if (rotation > 0) {
      Serial.print("rotation ");
      Serial.print(rotation);
      Serial.print(" ");    
      Serial.println(rotation * 30); //  2 импульса за оборот (снятие с цифрового выхода датчика FALLING)
    }
    itoa(rotation * 30, RemoteXY.rotation, 10);
    tft.drawString(RemoteXY.rotation, 130, 20);
    pulseCount = 0;
    
    //Mav_Request_Data();  
  }  

  mavlink_msg_receive();

  // status text
  if (prev_statustext != next_statustext){
    uint8_t t = prev_statustext;    
    for(uint8_t i = 0; i < MAX_STATUSTEXT; i++){      
      tft.drawString(statustext[t].text, 10, 30 + (i * 10));
      if (t > 0 )
        t--;
       else
        t = MAX_STATUSTEXT - 1; 
    }
    prev_statustext = next_statustext;
  }
  // нечто
  if (select_1 != RemoteXY.select_1) {
    select_1 = RemoteXY.select_1;
    Serial.print("select_1:");
    Serial.print(select_1);
    val= map(select_1,0,2,1000,2000);
    Serial.print(" ");
    Serial.println(val);    
    #ifdef Servo
    Servo.writeMicroseconds(15,val);
    #endif
  }

  //Руль
  val = rc_channels.chan1_raw;//mavlink_msg_rc_channels_get_chan1_raw((mavlink_message_t *)&rc_channels);
  if (chan1_raw2 != val) {
    Serial.print(1);
    Serial.print(" mavlink2:");
    Serial.println(val); // display new value in microseconds on PC
    #ifdef MY_SERVO_PIN    
    myservo.writeMicroseconds(val);   // sets the servo position 
    #endif
    #ifdef Servo
    Servo.writeMicroseconds(1,val);
    #endif

    chan1_raw2 = val;
  }
  val = rc_channels_raw.chan1_raw;//mavlink_msg_rc_channels_get_chan1_raw((mavlink_message_t *)&rc_channels);
  if (chan1_raw1 != val) {
    Serial.print(1);
    Serial.print(" mavlink1:");
    Serial.println(val); // display new value in microseconds on PC
    
    chan1_raw1 = val;    
  }

  //Машинный телеграф
  char throttle[6];
  val = rc_channels.chan3_raw; // get latest value for rc channel 3   
  itoa(val, throttle, 10);
  tft.drawString(throttle, 10, 20);

  val = servo_output_raw.servo3_raw; // get latest value for servo output 3 
  itoa(val, throttle, 10);
  tft.drawString(throttle, 40, 20);

  val = vfr_hud.throttle; // от -100 до +100
  if (servo_output_raw.servo3_raw < 1500)  // не работает -if (sys_status.onboard_control_sensors_present & MAV_SYS_STATUS_REVERSE_MOTOR)
    val = -val;
  sprintf(throttle,"%+4d",val );  
  tft.drawString(throttle, 70, 20);

  RemoteXY.telegraph = val;
  
  //Гудок
  val = rc_channels.chan5_raw; // get latest value for servo channel 4 (key left)
  val = constrain(val, 1000, 2000);
  val = map(val,1000,2000,-1,1); //map(value, fromLow, fromHigh, toLow, toHigh)
  
  if (saveval_key4 != val) {
    Serial.print("chan5:");
    Serial.println(val); 
    saveval_key4 = val;
    #ifdef Player
    if (saveval_key4 > 0) {
      Player.play(2);            
    }
    else {
      Player.pause();  
    }
    #endif
  } else if (button_1 != RemoteXY.button_1) {
    Serial.print("button_1:");
    Serial.println(button_1);
    button_1 = RemoteXY.button_1;
    #ifdef Player
    if (button_1 > 0) {
      Player.play(2);            
    }
    else {
      Player.pause();  
    }
    #endif
  }  

  #ifdef Player
  if (Player.available()) {
    //mp3_2_Serial.listen();
    printDetail(Player.readType(), Player.read(),2); // Выводить сообщения о статусе и ошибках модуля
  }
  #endif
  
}

IRAM_ATTR void countPulses() {  
  pulseCount++;
}

void mavlink_msg_receive() {
  mavlink_message_t msg;
  mavlink_status_t status;

  while(MAVLINK_SERIAL.available()>0) {
    uint8_t c = MAVLINK_SERIAL.read();
      // Try to get a new message
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      // Handle message
      switch(msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:  // #0: Heartbeat
          {
            // E.g. read GCS heartbeat and go into
            // comm lost mode if timer times out
            mavlink_heartbeat = HIGH;            
          }
          break;

        case MAVLINK_MSG_ID_SYS_STATUS:  // #1: SYS_STATUS
          {
            mavlink_msg_sys_status_decode(&msg, &sys_status);            
          }
          break;

        case MAVLINK_MSG_ID_VFR_HUD:  // #74: VFR_HUD
          {
            mavlink_msg_vfr_hud_decode(&msg, &vfr_hud);            
          }
          break;

        case MAVLINK_MSG_ID_RC_CHANNELS_RAW:  // #35: RC_CHANNELS_RAW
          {
            mavlink_msg_rc_channels_raw_decode(&msg, &rc_channels_raw);
          }
          break;

        case MAVLINK_MSG_ID_RC_CHANNELS:  // #65: RC_CHANNELS
          {
            mavlink_msg_rc_channels_decode(&msg, &rc_channels);
          }
          break;

        case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW: // #36
          {
            mavlink_msg_servo_output_raw_decode(&msg, &servo_output_raw);            
          }
          break;

        case MAVLINK_MSG_ID_STATUSTEXT: // #253
          {
            mavlink_msg_statustext_decode(&msg, &statustext[next_statustext]);
            Prepare_Display_Text(statustext[next_statustext].text);
            prev_statustext = next_statustext;
            if (++next_statustext >= MAX_STATUSTEXT)
                next_statustext = 0;       
          }
          break;
          
        case MAVLINK_MSG_ID_RAW_IMU:  // #27: RAW_IMU
          {
            
          }
          break;

        case MAVLINK_MSG_ID_ATTITUDE:  // #30
          {
            
          }
          break;
        
       default:
          break;
      }
    }
  }
}

#ifdef Player
void printDetail(uint8_t type, int value, uint8_t player){
  displaytext[next_displaytext].text[0] = 0;
  switch (type) {
    case TimeOut:
      sprintf(displaytext[next_displaytext].text, "%d %s", player, F("Time Out"));      
      break;
    case WrongStack:
      sprintf(displaytext[next_displaytext].text, "%d %s", player, F("Stack Wrong"));      
      break;
    case DFPlayerCardInserted:
      sprintf(displaytext[next_displaytext].text, "%d %s", player, F("Card Inserted")); 
      // Карта вставлена
      break;
    case DFPlayerCardRemoved:
      sprintf(displaytext[next_displaytext].text, "%d %s", player, F("Card Removed")); 
      // Карта удалена
      break;
    case DFPlayerCardOnline:
      sprintf(displaytext[next_displaytext].text, "%d %s", player, F("Card Online"));
      // Карта готова
      break;
    case DFPlayerPlayFinished:
      sprintf(displaytext[next_displaytext].text, "%d Play %d %s", player, value, (player != 2)?F("Finished"):F("Continue"));
      // Воспроизведение окончено
      if (player == 2){
        DFPlayer2.play(value);
      }
      break;
    case DFPlayerError:
      // Ошибка
      switch (value) {
        case Busy:
          sprintf(displaytext[next_displaytext].text, "%d %s", player, F("Card not found"));
          // Карта не найдена
          break;
        case Sleeping:
          sprintf(displaytext[next_displaytext].text, "%d %s", player, F("Sleeping"));
          // Сон
          break;
        case SerialWrongStack:
          sprintf(displaytext[next_displaytext].text, "%d %s", player, F("Get Wrong Stack"));
          
          break;
        case CheckSumNotMatch:
          sprintf(displaytext[next_displaytext].text, "%d %s", player, F("Check Sum Not Match"));
          
          break;
        case FileIndexOut:
          sprintf(displaytext[next_displaytext].text, "%d %s", player, F("File Index Out of Bound"));
          
          break;
        case FileMismatch:
          sprintf(displaytext[next_displaytext].text, "%d %s", player, F("Cannot Find File"));
          // Не могу найти файл
          break;
        case Advertise:
          sprintf(displaytext[next_displaytext].text, "%d %s", player, F("In Advertise"));
          // Реклама
          break;
        default:
          break;
      }      
      break;
    default:
      break;
  }
  if (displaytext[next_displaytext].text[0] != 0) {
    Serial.println(displaytext[next_displaytext].text);
    Display_Text();
  }
}
#endif

//
void Display_Text()
{
  Prepare_Display_Text(displaytext[next_displaytext].text);
  uint8_t t = next_displaytext;    
  for(uint8_t i = 0; i < MAX_DISPLAYTEXT; i++) {      
    tft.drawString(displaytext[t].text, 10, 140 + (i * 10));
    if (t > 0 )
      t--;
    else
      t = MAX_DISPLAYTEXT - 1; 
  }    
  if (++next_displaytext >= MAX_DISPLAYTEXT)
    next_displaytext = 0;
}

void Prepare_Display_Text(char *text) {
  uint8_t j= strlen(text);
  for (; j < 49; j++){ // дописываем пробелы, чтобы текст стал 49 символов
    text[j] = ' ';
  }
  text[j] = 0;
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



