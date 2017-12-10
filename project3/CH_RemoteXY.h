#include  <LiquidCrystal.h> //подключаем библиотеки
#include  <DHT.h>

#define CH_PIN_LITING  10
#define CH_BUTTON_NONE   0 //присваиваем постоянное значение для BUTTON_NONE 
#define CH_BUTTON_RIGHT  1 //присваиваем постоянное значение для BUTTON_RIGHT
#define CH_BUTTON_UP     2 //присваиваем постоянное значение для BUTTON_UP
#define CH_BUTTON_DOWN   3 //присваиваем постоянное значение для BUTTON_DOWN 
#define CH_BUTTON_LEFT   4 //присваиваем постоянное значение для BUTTON_LEFT
#define CH_BUTTON_SELECT 5 //присваиваем постоянное значение для BUTTON_SELECT

#define CH_DHTTYPE DHT11
#define CH_DHTPIN 23 
#define CH_DHT_TIMEOUT 120000 

class CH_CRemoteXY: public CRemoteXY {
 public:
 
 CH_CRemoteXY(const void * _conf, void * _var, const char * _accessPassword, HardwareSerial * _serial, long _serialSpeed, const char * _wifiSsid, const char * _wifiPassword, const char * _cloudServer, uint16_t _cloudPort, const char * _cloudToken)
 :  CRemoteXY( _conf,  _var, _accessPassword, _serial,  _serialSpeed,  _wifiSsid,  _wifiPassword, _cloudServer, _cloudPort, _cloudToken)
 {
	lcd= new LiquidCrystal(28, 29, 24, 25, 26, 27 );
  pinMode (CH_PIN_LITING, OUTPUT);
  analogWrite(CH_PIN_LITING, 500);
	lcd->begin(16, 2); //Инициализируем дисплей: 2 строки по 16 символов            
	lcd->print("INIT...");
	button= last_button= CH_BUTTON_NONE;
	dht= new DHT(CH_DHTPIN, CH_DHTTYPE);
	dhtTimeOut= millis();
	h=t=0;
 }
  // return value:
  // if (getmoduleRunning() == 1) then module connected and working
  uint8_t getmoduleRunning()
  {
    return moduleRunning;
  }
  float h; 
  float t;
	void display_state(uint8_t switch_state)
	{
    lcd->setCursor(0, 1);
    lcd->print(switch_state,DEC);
    lcd->setCursor(1, 1);
    lcd->print(cloudState, DEC);
    lcd->setCursor(2, 1);
    lcd->print(moduleRunning,DEC);
	
	// Reading temperature or humidity takes about 250 milliseconds!
	// Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
	if ((millis() - dhtTimeOut) > CH_DHT_TIMEOUT){
		h = dht->readHumidity();
		// Read temperature as Celsius (the default)
		t = dht->readTemperature();
		// Check if any reads failed and exit early (to try again).
#if defined(REMOTEXY__DEBUGLOGS)
		REMOTEXY__DEBUGLOGS.println();
		REMOTEXY__DEBUGLOGS.print("Humidity: ");
		REMOTEXY__DEBUGLOGS.print(h);
		REMOTEXY__DEBUGLOGS.print(" %\t");
		REMOTEXY__DEBUGLOGS.print("Temperature: ");
		REMOTEXY__DEBUGLOGS.print(t);
		REMOTEXY__DEBUGLOGS.print(" *C ");
#endif     
		if (!isnan(h)){
			lcd->setCursor(4, 1);
			lcd->print("H:");
			lcd->setCursor(6, 1);
			lcd->print(h);		  
		}	
		if (!isnan(t)){
			lcd->setCursor(10, 1);
			lcd->print("T:");
			lcd->setCursor(12, 1);
			lcd->print(t);			
		}
		dhtTimeOut= millis();
	}
 	}
	protected:
	LiquidCrystal *lcd;
	DHT *dht;
	uint32_t dhtTimeOut;
	uint8_t button;  //вводим числовые значения для кнопок
	uint8_t last_button;
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
};