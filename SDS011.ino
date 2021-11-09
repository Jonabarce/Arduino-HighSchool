#include <DHT.h>

#define DHTPIN 9
DHT dht22(DHTPIN, DHT22);

void setup() {
  Serial.begin(9600);
}

void loop() {
  // Declare variables for sensor readings
  float temperature = 0;
  float humidity = 0;
  // Take readings from sensor
  temperature = dht22.readTemperature();
  humidity = dht22.readHumidity();

  // Print out readings
  Serial.print(temperature);
  Serial.print("°C");
  Serial.print("\t");
  Serial.print(humidity);
  Serial.print("%");
  Serial.println("");

  // Wait 2.5 seconds until next reading.
  delay(2500);
}
