#define LED_RED   A1
#define LED_GREEN A0

#include <DHT.h>

#define DHTPIN 9
DHT dht22(DHTPIN, DHT22);

#include <SoftwareSerial.h>
#include <SDS011.h>

#define PM_TX 2
#define PM_RX 3

SDS011 sds;

#include <SoftwareSerial.h>
#include <TinyGPS++.h>

#define LED_RED A1
#define LED_GREEN A0

#define GPS_RX 7
#define GPS_TX 6

SoftwareSerial gpsCom(GPS_RX, GPS_TX);
TinyGPSPlus gps;

#include <SD.h>

#define SD_CS_PIN 10

File file;
int counter;
float temperature = 0;
float humidity = 0;

float pm25, pm10;


void setup() {



  pinMode(LED_RED, OUTPUT); // Enable red LED control
  pinMode(LED_GREEN, OUTPUT); // Enable green LED control



  Serial.begin(9600);

  sds.begin(PM_TX, PM_RX);
  Serial.begin(9600);

  // Activate control over LEDs
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);

  // Initialize serial communication
  Serial.begin(9600); // to Computer (USB)
  gpsCom.begin(9600); // to GPS antenna

  // Activate CS-Pin control
  pinMode(SD_CS_PIN, OUTPUT);

  // Startup SD-card reader
  SD.begin(SD_CS_PIN);

  // Define filename
  char filename[] = "testfile.txt";

  if (SD.exists(filename)) {
    // Open existing file for writing and append
    file = SD.open(filename, O_WRITE | O_APPEND);
    file.println("--------------------");
    file.println("Filen ble åpnet på nytt.");
  } else {
    file = SD.open(filename, O_CREAT | O_WRITE);
    file.println("Dette er den første linjen i filen.");
  }
  file.flush(); // Force saving data to SD-card

  // Start counter at 0
  counter = 0;


}





void loop() {

  lagring();

  ledblink();

  stovsensoren ();

  temp ();






}





void ledblink () {
  //loop
  digitalWrite(LED_RED, HIGH);
  delay(1000);
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, HIGH);
  delay(1000);
  digitalWrite(LED_GREEN, LOW);
}

void temp   () {

  //loop


  // Declare variables for sensor readings


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





void stovsensoren() {


  int error;
  do {
    error = sds.read(&pm25, &pm10);
  } while (error != 0);

  Serial.print(pm25);
  Serial.print("\t");
  Serial.print(pm10);
  Serial.println("");

  delay(1000);


}





void GPS () {


  gpsCom.listen();
  bool gpsEncodeComplete = false;
  do {
    if (!gpsCom.available()) {
      // No new data available.
      // Immediately jump to next iteration
      continue;
    }
    gpsEncodeComplete = gps.encode(gpsCom.read());
    if (!gpsEncodeComplete) {
      // Data is incomplete,
      // Jump to next iteration and try again
      continue;
    }
  } while (!gpsEncodeComplete); // Loop until gps data was successfully read and encoded from GPS module

  bool gpsValid = gps.location.isValid();
  bool gpsUpdated = gps.location.isUpdated();
  bool isUseful = gpsValid && gpsUpdated;
  if (!isUseful) {
    // No valid position.
    // I.e. no GPS fix.
    Serial.println("No valid GPS position");
    digitalWrite(LED_RED, HIGH);
    delay(500);
    digitalWrite(LED_RED, LOW);
    return;
  }

  digitalWrite(LED_GREEN, HIGH);
  delay(500);
  digitalWrite(LED_GREEN, LOW);

  Serial.print("Time: ");
  Serial.print(gps.date.day());
  Serial.print(".");
  Serial.print(gps.date.month());
  Serial.print(".");
  Serial.print(gps.date.year());
  Serial.print(" ");
  Serial.print(gps.time.hour());
  Serial.print(":");
  Serial.print(gps.time.minute());
  Serial.print(":");
  Serial.print(gps.time.second());
  Serial.print(".");
  Serial.print(gps.time.centisecond());

  Serial.print("\t");

  Serial.print("Latitude: ");
  Serial.print(gps.location.lat(), 6); // Latitude in degrees
  Serial.print("\t");
  Serial.print("Longitude: ");
  Serial.print(gps.location.lng(), 6); // Longitude in degrees

  Serial.println("");

}





void lagring () {


  counter += 1;

  file.print("Dette er temperatur.:");
  file.print(" ");
  file.println(temperature);

  g
  file.print("DHT-22 luftfuktighet");
  file.print(" ");
  file.println(humidity);

  
  file.print("Luftkvalitet til nova");
  file.print(" ");
  file.println(pm25);

  
  file.print("Luftkvalitet til nova x2");
  file.print(" ");
  file.println(pm10);

  file.flush();

  delay(1000); // Wait a second.


}
