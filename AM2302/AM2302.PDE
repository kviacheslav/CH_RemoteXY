#include "DHT.h"

#define DHTPIN A0     // вывод, к которому подключается датчик
#define DHTTYPE DHT22   // DHT 22  (AM2302)

DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(9600);
  dht.begin();
}

void loop() {
  delay(2000);

  // считывание температуры или влажности занимает примерно 250 мс!
  // считанные показания могут отличаться от актуальных примерно на 2 секунды (это очень медленный датчик)
  float h = dht.readHumidity();
  // Считывание температуры в цельсиях
  float t = dht.readTemperature();

  // проверяем, были ли ошибки при считывании и, если были, начинаем заново
  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  //выводим информацию в Монитор последовательного порта
  Serial.print("Humidity: ");
  Serial.print(h);
  Serial.print(" %\t");
  Serial.print("Temperature: ");
  Serial.print(t);
  Serial.println(" *C ");
}