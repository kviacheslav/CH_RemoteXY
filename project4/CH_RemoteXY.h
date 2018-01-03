//подключаем библиотеки
#if defined(CH_GSM)
	
#endif //defined(CH_GSM)
#if defined(CH_LCD)
	#include  <LiquidCrystal.h> 
	
	#define CH_PIN_LITING	10
	#define CH_BUTTON_NONE   0 //присваиваем постоянное значение для BUTTON_NONE 
	#define CH_BUTTON_RIGHT  1 //присваиваем постоянное значение для BUTTON_RIGHT
	#define CH_BUTTON_UP     2 //присваиваем постоянное значение для BUTTON_UP
	#define CH_BUTTON_DOWN   3 //присваиваем постоянное значение для BUTTON_DOWN 
	#define CH_BUTTON_LEFT   4 //присваиваем постоянное значение для BUTTON_LEFT
	#define CH_BUTTON_SELECT 5 //присваиваем постоянное значение для BUTTON_SELECT
#endif
#if defined(CH_DHT)
	#include  <DHT.h>
	#define CH_DHT_TIMEOUT	120000 
#endif //defined(CH_DHT)
#if defined(CH_CD)
	#include <SD.h>
	#define CD_PIN CH_CD
#endif //defined(CH_CD)

class CH_CRemoteXY: public CRemoteXY {
 public:
 
 CH_CRemoteXY(const void * _conf, void * _var, const char * _accessPassword, HardwareSerial * _serial, long _serialSpeed, const char * _wifiSsid, const char * _wifiPassword, const char * _macAddress, const char * _cloudServer, uint16_t _cloudPort, const char * _cloudToken)
#if defined(REMOTEXY_MODE__ETHERNET_LIB_CLOUD) 
	: CRemoteXY (_conf, _var, _accessPassword, _macAddress, _cloudServer, _cloudPort, _cloudToken)
#endif 
#if defined(REMOTEXY_MODE__ESP8266_HARDSERIAL_CLOUD) 
 :  CRemoteXY(_conf, _var, _accessPassword, _serial,  _serialSpeed,  _wifiSsid,  _wifiPassword, _cloudServer, _cloudPort, _cloudToken)
 #endif
 {
#if defined(CH_LCD)
	//Инициализируем дисплей
	lcd= new CH_LCD ;									//: объект управления LCD
	//pinMode (CH_PIN_LITING, OUTPUT);					//: пин подсветки LCD
	//analogWrite(CH_PIN_LITING, 500);					//: подсветка 0.5 VCC
	lcd->begin(16, 2); 									//: 2 строки по 16 символов            
	lcd->print("INIT...");
	button= last_button= CH_BUTTON_NONE;
#endif
#if defined(CH_DHT)
	//Инициализируем термометр
	dht= new CH_DHT;
	dhtTimeOut= millis();
	dht_h=dht_t=0;
#endif
	
#if defined(CH_CD)
	sd_init();
#endif //defined(CH_CD)
#if defined(CH_GSM)
	gsm_init();
#endif //defined(CH_GSM)
 }
 
#if defined(CH_DHT)  
	DHT *dht;
	uint32_t dhtTimeOut;
	float dht_h; 
	float dht_t;	
#endif
#if defined(CH_LCD)
	LiquidCrystal *lcd;	
	uint8_t button;  //вводим числовые значения для кнопок
	uint8_t last_button;
#endif
#if defined(CH_CD)
	File myFile; 
	Sd2Card card;
#endif //defined(CH_CD)
#if defined(CH_GSM)
	uint8_t gsm_state;
	//TinyGsm modem();
#endif //defined(CH_GSM)
  // 
  void ch_handler(){
	int c;
	/*if (Serial1.available()) {
		//Serial.print("available - ");
		//Serial.println(Serial.available());
		c = Serial1.read();
		// open the file. note that only one file can be open at a time,
		// so you have to close this one before opening another.
		
		// if the file opened okay, write to it:
		if (myFile) {
			
			while (c != -1){
				myFile.write(c);
				Serial.write(c);
				c = Serial1.read();
			}
			// close the file:
			myFile.flush();
		}	
	}
	if (Serial.available()) {
		c = Serial.read();
		while (c != -1){			
			Serial1.write(c);
			c = Serial.read();
		}
	}*/
#if defined(CH_DHT) 	
	// Reading temperature or humidity takes about 250 milliseconds!
	// Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
	if ((millis() - dhtTimeOut) > CH_DHT_TIMEOUT){
#if defined(REMOTEXY__DEBUGLOGS)
		REMOTEXY__DEBUGLOGS.println("Humidity reading.");		
#endif  
		float h = dht->readHumidity();
		// Read temperature as Celsius (the default)
		float t = dht->readTemperature();
		// Check if any reads failed and exit early (to try again).	
		if (!isnan(h)){
			dht_h=h;		  
		}
#if defined(REMOTEXY__DEBUGLOGS)
		else REMOTEXY__DEBUGLOGS.println("Humidity unavailable.");		
#endif     
	
		if (!isnan(t)){
			dht_t=t;			
		}
		dhtTimeOut= millis();
		display_state(RemoteXY.connect_flag);
	}	
#endif //defined(CH_DHT) 	
	
  }
  // if (getmoduleRunning() == 1) then module connected and working
  uint8_t getmoduleRunning()
  {
    return moduleRunning;
  }
  
	void display_state(uint8_t switch_state)
	{
#if defined(CH_LCD)
    lcd->setCursor(0, 1);
    lcd->print(switch_state,DEC);
    lcd->setCursor(1, 1);
    lcd->print(cloudState, DEC);
    lcd->setCursor(2, 1);
    lcd->print(moduleRunning,DEC);
	lcd->setCursor(4, 1);
	#if defined(CH_DHT) 
		lcd->print("H:");
		lcd->setCursor(6, 1);
		lcd->print(dht_h);
		lcd->setCursor(10, 1);
		lcd->print("T:");
		lcd->setCursor(12, 1);
		lcd->print(dht_t);	
	#endif //defined(CH_DHT) 
#else //defined(CH_LCD)
		Serial.print(switch_state,DEC);		
		Serial.print(cloudState, DEC);		
		Serial.print(moduleRunning,DEC);
	#if defined(CH_DHT) 
		Serial.print(" h");
		Serial.print(dht_h);
		Serial.print(" t");
		Serial.println(dht_t);
	#endif //defined(CH_DHT) 
			
#endif //defined(CH_LCD)
 	}

#if defined(CH_LCD)	
	uint8_t getPressedButton() //инициализация переменной
	{
	  int buttonValue = analogRead(0); // считываем значения с аналогового входа
	  if (buttonValue < 100) { //если при нажатии кнопки значение меньше 100
		return CH_BUTTON_RIGHT;   // выводим значение BUTTON_RIGHT
	  }
	  else if (buttonValue < 150) { //если при нажатии кнопки значение меньше 200
		return CH_BUTTON_UP; // выводим значение BUTTON_UP
	  }
	  else if (buttonValue < 350){ //если при нажатии кнопки значение меньше 400
		return CH_BUTTON_DOWN; // выводим значение BUTTON_DOWN
	  }
	  else if (buttonValue < 600){ //если при нажатии кнопки значение меньше 600
		return CH_BUTTON_LEFT; // выводим значение BUTTON_LEFT
	  }
	  else if (buttonValue < 800){ //если при нажатии кнопки значение меньше 800
		return CH_BUTTON_SELECT; // выводим значение BUTTON_SELECT
	  }
	  return CH_BUTTON_NONE; //иначе, выводим значение BUTTON_NONE
	}
#endif //defined(CH_LCD)
#if defined(CH_CD)
void sd_init(){
	if (myFile) {
		Serial.print("Already to test.txt...");	
		return;
	}
  Serial.print("Initializing SD card...");
 // Этот пин обязательно должен быть определен как OUTPUT
 // pinMode(10, OUTPUT);
 
  // Пытаемся проинициализировать модуль
  if (!SD.begin(CD_PIN)) {
    Serial.println("Card failed, or not present");
    // Если что-то пошло не так, завершаем работу:
    return;
  }
  Serial.println("card initialized.");
  Serial.print("\nInitializing SD card...");

  // we'll use the initialization code from the utility libraries
  // since we're just testing if the card is working!
  if (!card.init(SPI_HALF_SPEED, CD_PIN)) {
    Serial.println("initialization failed. Things to check:");
    Serial.println("* is a card inserted?");
    Serial.println("* is your wiring correct?");
    Serial.println("* did you change the CD_PIN pin to match your shield or module?");
    return;
  } else {
    Serial.println("Wiring is correct and a card is present.");
  }

  // print the type of card
  Serial.print("\nCard type: ");
  switch (card.type()) {
    case SD_CARD_TYPE_SD1:
      Serial.println("SD1");
      break;
    case SD_CARD_TYPE_SD2:
      Serial.println("SD2");
      break;
    case SD_CARD_TYPE_SDHC:
      Serial.println("SDHC");
      break;
    default:
      Serial.println("Unknown");
  }
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  myFile = SD.open("test.txt", FILE_WRITE);

  // if the file opened okay, write to it:
  if (myFile) {
    Serial.print("Ready to test.txt...");
	
 /*   myFile.println("testing 1, 2, 3.");
    // close the file:
    myFile.close();
    Serial.println("done.");*/
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
/*
  // Now we will try to open the 'volume'/'partition' - it should be FAT16 or FAT32
  if (!volume.init(card)) {
    Serial.println("Could not find FAT16/FAT32 partition.\nMake sure you've formatted the card");
    return;
  }
  // print the type and size of the first FAT-type volume
  uint32_t volumesize;
  Serial.print("\nVolume type is FAT");
  Serial.println(volume.fatType(), DEC);
  Serial.println();

  volumesize = volume.blocksPerCluster();    // clusters are collections of blocks
  volumesize *= volume.clusterCount();       // we'll have a lot of clusters
  volumesize *= 512;                            // SD card blocks are always 512 bytes
  Serial.print("Volume size (bytes): ");
  Serial.println(volumesize);
  Serial.print("Volume size (Kbytes): ");
  volumesize /= 1024;
  Serial.println(volumesize);
  Serial.print("Volume size (Mbytes): ");
  volumesize /= 1024;
  Serial.println(volumesize);


  Serial.println("\nFiles found on the card (name, date and size in bytes): ");
  root.openRoot(volume);

  // list all files in the card with date and size
  root.ls(LS_R | LS_DATE | LS_SIZE);*/

}
#endif //defined(CH_CD)
#if defined(CH_GSM)
	void gsm_init(){
		gsm_state= 0;
		// Set GSM module baud rate
		SerialAT.begin(115200);
		delay(3000);
		// Restart takes quite some time
		// To skip it, call init() instead of restart()
		DBG("Initializing modem...");
		if (!modem.restart()){
			//delay(10000);
			return;
		}
		gsm_state= 1;
		DBG("Waiting for network...");
		if (!modem.waitForNetwork()) {
			//delay(10000);
			return;
		}
		gsm_state= 2;
		String ussd_balance = modem.sendUSSD("*100#");
		DBG("Balance (USSD):", ussd_balance);

		String ussd_phone_num = modem.sendUSSD("*111*0887#");
		DBG("Phone number (USSD):", ussd_phone_num);
		
  	};
#endif //defined(CH_GSM)
};