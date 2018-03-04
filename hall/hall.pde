
int ledPin = 13;      // Пин светового индикатора изменения КУ
 
// следующую константу (typVbg) для определения опорного напряжения необходимо откалибровать под используемый контроллер
const float typVbg = 1.179; // 1.0 -- 1.2
float referenceVcc = 0; 			// Опорное напряжение в вольтах

byte q = 4;							// Количество датчиков Холла
byte pin[] = {A0,A1,A2,A3};			// Пины, к которым подключены  датчики Холла

// Показания, считанные с датчиков
float sensor_reading[] = {0,0,0,0};	
float sensor_pos[][4] = {{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0}};
float sensor_max[] = {0,0,0,0};
float sensor_min[] = {1000.0f,1000.0f,1000.0f,1000.0f};
unsigned long sum[] = {0,0,0,0};
unsigned int  cnt[] = {0,0,0,0};

// Курсовые углы, при которых показание соответствующего датчика будет максимальным
// Одинаковые углы между датчиками дадут максимально точный КУ 
unsigned int mechanical_relative_bearing[] = {45,135,225,315}; 
unsigned int mechanical__bearing[] = {0,90,180,270}; 
float sensors_angle = 90.0f;

float relative_bearing; 			// Вычесленный (текущий) КУ 

float r_b_exp2;
float r_exp2 = 0;
float r_b_exp1;
float r_exp1 = 0;
float r_b_line;
float r_line = 0;

float apparent_wind;
bool wind_vane_ready = false;
uint16_t r_b;
uint16_t last_r_b;
float last_rb = 1110;
uint32_t moduleTimeOut;
uint32_t costs_exp2;
uint32_t costs_exp1;
uint32_t costs_line;
uint32_t costs0;

void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  // declare the ledPin as an OUTPUT:
  pinMode(ledPin, OUTPUT);
  // the current time
  moduleTimeOut= millis();
  
  
}

void loop() {
  
  // постоянно считываем показания датчиков
  analogSample();
  
  if ((millis() - moduleTimeOut) > 1000){ // 5 раза в секунду
	// усредняем показания датчиков за прошедшее время 
	analogAverage(); 
	// вычисляем КУ
	wind_vane_ready = true;
    relativeBearing();	
    
	  //if (last_r_b != r_b){
		// Зажигаем световой индикатор
	    digitalWrite(ledPin, HIGH);
		referenceVcc = readVcc();
        sensors();
	    //last_r_b = r_b;	  
	 // }
	 // else{	
		// Гасим световой индикатор
	 //   digitalWrite(ledPin, LOW);    
      //}
        
	moduleTimeOut= millis();
  }
    
}
/****************************************************************************
 * Функции
 ****************************************************************************/
 
void analogSample(){	
  byte i;
  for (i = 0; i < q; i++){
	sum[i] += analogRead(pin[i]);
	cnt[i]++;
  }	
}

float analogAverage(){
  byte i;
  for (i = 0; i < q; i++){
	if (cnt[i] > 0){
      sensor_reading[i] = sum[i] / (float)cnt[i];	 
      cnt[i] = 0;
      sum[i] = 0;	
    } 
  }
}

void read_wind_vane(void)
{
	uint8_t pin;
	float tmp = 0;
	uint8_t max_i = 0;
	uint8_t right_i;
	uint8_t left_i;
	// усредненные за время прошедшее после предыдущего чтения показания датчиков
	//float sensor_reading[4];	
	uint8_t cnt = 0;
	
	if (!wind_vane_ready){
		apparent_wind = 90.0f;
		return;
	}
    // Найдем датчик с максимальным показанием	
	for (pin = 0; pin < 4; pin++){
		//sensor_reading[pin] = wind_vane_sensors[pin]->read_average();
		if (sensor_reading[pin] > sensor_max[pin])
			sensor_max[pin] = sensor_reading[pin];
		if (sensor_reading[pin] < sensor_min[pin])
			sensor_min[pin] = sensor_reading[pin];	
		if (sensor_reading[pin] > tmp){
			max_i = pin;
			tmp = sensor_reading[pin];			
		}
		if (sensor_reading[pin] == tmp)
			cnt++;
	}
	// Проверим корректность показаний, чтобы небыло деления на 0
	if (cnt == 0){
		apparent_wind = 360.0f;
		return;
	}	
    // Датчик, который левее
 	if (left_i == 255)
		left_i = 4 - 1;
	// Датчик, который правее
	right_i = max_i + 1;
	if (right_i >= 4)
		right_i = 0;

	// Разность показаний  с левым датчиком
	tmp = sensor_reading[max_i] - sensor_reading[left_i];

	// Чувствительность датчика SS49E и его показание при нулевой магнитной индукции зависят от величины опорного напряжения.
	// При питании от батарей опорное напряжение имеет тенженцию к уменьшению по мере разряда или при значительном увеличении потребления тока.
	// Для определения КУ бужем использовать относительные показания.
	// Угол отклонения от механического КУ вычисляем из пропорции разностей показаний с левым и правым датчиком
	// Показания датчиков нестабильны даже когда флюгер стабилен.
	
	apparent_wind =	mechanical_relative_bearing[left_i] +	
	((tmp  * 90.0f) / (tmp + sensor_reading[max_i] - sensor_reading[right_i]));
	if ( apparent_wind >= 360.0f )
		apparent_wind -= 360.0f;
	r_b = apparent_wind + 0.5;	
	/*
	relative_bearing =
	mechanical_relative_bearing[left_i] +
	((tmp  * sensors_angle) / (tmp + sensor_reading[max_i] - sensor_reading[right_i]));
 if ( relative_bearing >= 360.0f )
   relative_bearing = relative_bearing - 360.0f;
   r_b = relative_bearing + 0.5;
   apparent_wind = relative_bearing;*/
}

/*
Вычисляет курсовой угол
*/
void relativeBearing(){
 byte i; 
 byte j;
 unsigned long tmp = 0;
 byte max_i;
 byte right_i;
 byte left_i;
 // Найдем датчик с максимальным показанием
 for ( i = 0; i < q; i ++){
	if (sensor_reading[i] > sensor_max[i]){
		sensor_max[i] = sensor_reading[i];
		for( j = 0; j < q; j++)
			sensor_pos[i][j] = sensor_reading[j];			
	}
	if (sensor_reading[i] < sensor_min[i])
			sensor_min[i] = sensor_reading[i];
	if (sensor_reading[i] > tmp){
		max_i = i;
		tmp = sensor_reading[i];
	}
 }
 // Датчик, который левее
 left_i = max_i - 1; 
 if (left_i == 255)
   left_i = q - 1;
 // Датчик, который правее 
 right_i = max_i + 1;
 if (right_i >= q)
   right_i = 0;
 // Разность показаний  с левым датчиком
 costs0 = micros();
 tmp = sensor_reading[max_i] - sensor_reading[left_i]; 
 
 // Чувствительность датчика SS49E и его показание при нулевой магнитной индукции зависят от величины опорного напряжения.
 // При питании от батарей опорное напряжение имеет тенженцию к уменьшению по мере разряда или при значительном увеличении потребления тока.
 // Для определения КУ бужем использовать относительные показания.
 // Угол отклонения от механического КУ вычисляем из пропорции разностей показаний с левым и правым датчиком
 relative_bearing =
	mechanical_relative_bearing[left_i] +
	((tmp  * sensors_angle) / (tmp + sensor_reading[max_i] - sensor_reading[right_i]));
 if ( relative_bearing >= 360.0f )
   relative_bearing = relative_bearing - 360.0f;
 costs0 = micros() - costs0;
   r_b = relative_bearing + 0.5;
   apparent_wind = relative_bearing;
  
  
  // апроксимация exp(-x^2)  
  costs_exp2 = micros();
  tmp = sensor_reading[max_i]-sensor_min[max_i];
  if (tmp > 0){ // прошли колибровку ?
	r_b_exp2 = squaref(
		-logf(tmp/(sensor_pos[max_i][max_i]-sensor_min[max_i]))
	)*45;
	r_exp2 = r_b_exp2 + 0.5;
	if ((sensor_reading[right_i]- sensor_pos[max_i][right_i]) > (sensor_reading[left_i]-sensor_pos[max_i][left_i]))
		r_b_exp2 = mechanical__bearing[max_i] + r_b_exp2;
	else{
		r_b_exp2 = mechanical__bearing[max_i] - r_b_exp2;
		if ( r_b_exp2 < 0 )
			r_b_exp2 += 360.0f;
	}	
  }
  else{
	r_b_exp2 = -1.0f;
  }
  costs_exp2 = micros() - costs_exp2;
  
  // апроксимация exp(-x)  
  costs_exp1 = micros();
  tmp = (sensor_reading[max_i]-sensor_min[max_i]);
  if (tmp > 0){ // прошли колибровку ?
	r_b_exp1 = 	-logf(tmp/(sensor_pos[max_i][max_i]-sensor_min[max_i])) * 18;	
	r_exp1 = r_b_exp1 + 0.5;
	if ((sensor_reading[right_i]- sensor_pos[max_i][right_i]) > (sensor_reading[left_i]-sensor_pos[max_i][left_i]))
		r_b_exp1 = mechanical__bearing[max_i] + r_b_exp1;
	else{
		r_b_exp1 = mechanical__bearing[max_i] - r_b_exp1;
		if ( r_b_exp1 < 0 )
			r_b_exp1 += 360.0f;
	}	
  }
  else{
	r_b_exp1 = -1.0f;
  }
  costs_exp1 = micros() - costs_exp1;
  
  // апроксимация линейная  
  costs_line = micros();
  tmp = (sensor_reading[max_i]-sensor_min[max_i]);
  if (tmp > 0){ // прошли колибровку ?
	r_b_line = 	( 1 - tmp/(sensor_pos[max_i][max_i]-sensor_min[max_i])) * 90;	
	r_line = r_b_line + 0.5;
	if ((sensor_reading[right_i]- sensor_pos[max_i][right_i]) > (sensor_reading[left_i]-sensor_pos[max_i][left_i]))
		r_b_line = mechanical__bearing[max_i] + r_b_line;
	else{
		r_b_line = mechanical__bearing[max_i] - r_b_line;
		if ( r_b_line < 0 )
			r_b_line += 360.0f;
	}	
  }
  else{
	r_b_line = -1.0f;
  }
  costs_line = micros() - costs_line;
}

/*
Определение опорного напряжения
https://github.com/tim4dev/arduino/tree/master/sensor-test/true_voltmeter
*/
float readVcc() {
  byte i;
  float result = 0.0;
  float tmp = 0.0;
  int tmp1 = 0;
  int val = 0;
  

  for (i = 0; i < 5; i++) {
    // Read 1.1V reference against AVcc
    // set the reference to Vcc and the measurement to the internal 1.1V reference
    #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
        ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
    #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
        ADMUX = _BV(MUX5) | _BV(MUX0);
    #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
        ADMUX = _BV(MUX3) | _BV(MUX2);
    #else
        // works on an Arduino 168 or 328
        ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
    #endif

    delay(3); // Wait for Vref to settle
    ADCSRA |= _BV(ADSC); // Start conversion
    while (bit_is_set(ADCSRA,ADSC)); // measuring

    uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
    uint8_t high = ADCH; // unlocks both

	tmp1 = (high<<8) | low;
    tmp = (high<<8) | low;
    tmp = (typVbg * 1023.0) / tmp;
    result = result + tmp;
	val = val + tmp1;         
    delay(5);
  }

  result = result / 5;
  val = val / 5;
  return result;
}

/*
Вывод на консоль
*/
void sensors(){
	byte i;
	byte j;
	Serial.print("r_b0: ");
	Serial.print(apparent_wind);Serial.print(" / / ");Serial.print(costs0);
	Serial.println("");
	Serial.print("exp2: ");
	Serial.print(r_b_exp2);Serial.print(" / ");Serial.print(r_exp2);Serial.print(" / ");Serial.print(costs_exp2);
	Serial.println("");
	Serial.print("exp1: ");
	Serial.print(r_b_exp1);Serial.print(" / ");Serial.print(r_exp1);Serial.print(" / ");Serial.print(costs_exp1);
	Serial.println("");
	Serial.print("line: ");
	Serial.print(r_b_line);Serial.print(" / ");Serial.print(r_line);Serial.print(" / ");Serial.print(costs_line);
	//Serial.print(" Vcc: ");
	//Serial.print(referenceVcc);
	Serial.println("");
	Serial.print(" samp:");
	for(i = 0; i < q; i++){
	  Serial.print(" t"); Serial.print(i);Serial.print(": ");
	  Serial.print(sensor_reading[i]);
	}
	Serial.println("");
	Serial.print(" Min : ");
	for(i = 0; i < q; i++){
	  Serial.print(" t"); Serial.print(i);Serial.print(": ");
	  Serial.print(sensor_min[i]);
	}
	for(j = 0; j < q; j++){
		Serial.println("");
		Serial.print(" Pos"); Serial.print(j);Serial.print(": ");
		for(i = 0; i < q; i++){
		  Serial.print(" t"); Serial.print(i);Serial.print(": ");
		  Serial.print(sensor_pos[j][i]);
		}
	}
	Serial.println("");
	
}