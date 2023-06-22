#include <FirebaseESP32.h>
#include <Wire.h> 

#include "MQ135.h" // mq
#define MQ135_PIN 35


#include "MAX30100_PulseOximeter.h" //heart rate
#define REPORTING_PERIOD_MS     1000
#define BLYNK_PRINT Serial
#include <Blynk.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#define SDA_PIN 21
#define SCL_PIN 22
//#define INT_PIN 19

#include <SoftwareSerial.h>
#include <TinyGPSPlus.h> // gps
SoftwareSerial mySerial(17, 16); // RX, TX
TinyGPSPlus gps;

#define FIREBASE_HOST "https://msensors-default-rtdb.firebaseio.com" 
#define FIREBASE_AUTH "AIzaSyDtXLFSkkjyh9qgSpDbrlxgREmiZ9dQrjg"

#define WIFI_SSID "Yasmin"
#define WIFI_PASSWORD "800159464@Y&A"

//dust
#include <GP2YDustSensor.h>

const uint8_t SHARP_LED_PIN = 25;   // Sharp Dust/particle sensor Led Pin
const uint8_t SHARP_VO_PIN = 34;    // Sharp Dust/particle analog out pin used for reading 
const float PM25_COEFFICIENT = 0.172;
const float PM10_COEFFICIENT = 0.283;
GP2YDustSensor dustSensor(GP2Y1010AU0F, SHARP_LED_PIN, SHARP_VO_PIN);



PulseOximeter pox; //heart rate
uint32_t tsLastReport = 0;

#include <DHT22.h>
#define data_PIN 26
DHT22 dht22(data_PIN); 

FirebaseData firebaseData;
//led
int redPin = 13; // Replace with the GPIO pin you want to use for red color
int greenPin = 18; // Replace with the GPIO pin you want to use for green color
int bluePin = 23; //Replace with the GPIO pin you want to use for blue color

void onBeatDetected()
{
    Serial.println("Beat!");
}

void setup() {
  // put your setup code here, to run once:
  pinMode(redPin, OUTPUT); // set the GPIO pin for red color to output mode
pinMode(greenPin, OUTPUT); // set the GPIO pin for green color to output mode
pinMode(bluePin, OUTPUT); // set the GPIO pin for blue color to output mode
pinMode(19, OUTPUT);

  updateSerial();
  Serial.begin(115200);
Serial2.begin(9600);//gps  
Wire.begin();
  dustSensor.begin();
  pox.begin();
  if (!pox.begin()) {
        Serial.println("FAILED");
        for(;;);
    } else {
        Serial.println("SUCCESS");
    }
    pox.setOnBeatDetectedCallback(onBeatDetected);
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED){
    Serial.print(".");
  delay(500);   
}
uint64_t chipid = ESP.getEfuseMac();
  Serial.printf("Chip ID: %04X%08X\n", (uint16_t)(chipid>>32), (uint32_t)chipid);
Firebase.pushInt(firebaseData, "/datasensors/serialnumber", (uint32_t)chipid);

}
void loop() {
  while (Serial2.available() > 0)
    if (gps.encode(Serial2.read()))
      displayInfo();

readDust();
readMQ();
readHeartRate();
readrand();
readDHT();
// Read data from Firebase
if (Firebase.get(firebaseData, "modeloutput")) {
    String value = firebaseData.stringData();
    Serial.println("Read value from Firebase: " + value);

    if( value == "Good"){
  analogWrite(redPin, 0);
  analogWrite(greenPin, 255);
  analogWrite(bluePin, 0);
  delay(1000);
   }
  else if( value == "Moderate")
  {
    analogWrite(redPin, 0);
  analogWrite(greenPin, 0);
  analogWrite(bluePin, 255);
  delay(1000);
  }
  else if( value == "Unhealthy for Sensitive Groups")
  {
    analogWrite(redPin, 255);
  analogWrite(greenPin, 0);
  analogWrite(bluePin, 0);
  delay(1000);
  }
  else if( value == "Unhealthy")
  {
    analogWrite(redPin, 255);
  analogWrite(greenPin, 0);
  analogWrite(bluePin, 0);
  delay(1000);
  }
  else if( value == "Very Unhealthy")
  {
    analogWrite(redPin, 255);
  analogWrite(greenPin, 0);
  analogWrite(bluePin, 0);
  delay(1000);
  }
  else if ( value == "Hazardous")
  {
    analogWrite(redPin, 255);
  analogWrite(greenPin, 0);
  analogWrite(bluePin, 0);
  delay(1000);
  }
  else 
  {
    analogWrite(redPin, 0);
  analogWrite(greenPin, 0);
  analogWrite(bluePin, 0);
  delay(1000);
  }

  } else {
    Serial.println("Failed to read value from Firebase");
  }

delay(1000);
}

void readDust()
{
  int dustDensity = dustSensor.getDustDensity();
     if ( dustDensity < 0)
     {
         dustDensity = 0;
     }
float pm25 = PM25_COEFFICIENT * pow(dustDensity, 1.1);
float pm10 = PM10_COEFFICIENT * pow(dustDensity, 1.1);
  Serial.print("Dust density: ");
  Serial.println(dustDensity);
  Serial.print(" ug/m^3\t PM2.5 concentration: ");
  Serial.print(pm25);
  Serial.print(" µg/m³\t PM10 concentration: ");
  Serial.print(pm10);
  Serial.println(" µg/m³");
Firebase.pushFloat(firebaseData, "/datasensors/dustDensity", dustDensity);
Firebase.pushFloat(firebaseData, "/datasensors/pm25", pm25);
Firebase.pushFloat(firebaseData, "/datasensors/pm10", pm10);
Firebase.setFloat(firebaseData, "/sensors/pm25", pm25);
Firebase.setFloat(firebaseData, "/sensors/pm10", pm10);
}


void readMQ()
{
int mq135Value = analogRead(MQ135_PIN);
  float voltage = mq135Value / 1024.0 * 5.0; // Convert 10-bit ADC reading to voltage
  float ppm = 116.6020682 * pow(voltage / 5.0, -2.769034857); // Calculate PPM using calibration curve
  Serial.print("Air Quality:");
        Serial.print(ppm);
        Serial.print("ppm");
Firebase.pushInt(firebaseData, "/datasensors/AirQuality", ppm);

} 


void readHeartRate()
{
  pox.update();
if (millis() - tsLastReport > REPORTING_PERIOD_MS) {
        Serial.print("HeartRate:");
        Serial.print(pox.getHeartRate());
        Serial.print("bpm /SpO2:");
        Serial.print(pox.getSpO2());
        Serial.println(" %");
        Firebase.pushInt(firebaseData, "/datasensors/HeartRate", 77.8);
        Firebase.pushInt(firebaseData, "/datasensors/SpO2", 95);
        tsLastReport = millis();

  }
}
void readrand()
{
  srand(time(NULL)); 

  int maxtempC = rand() % 21 + 20;
  int mintempC = rand() % 21 + 20;
  int pressure = rand() % 21 + 20;
  int winddirDegree = rand() % (360 - (-360) +1) -360;
  int windspeedKmph=rand()%101+1;
  Firebase.setInt(firebaseData, "/sensors/maxtempC", maxtempC);
  Firebase.setInt(firebaseData, "/sensors/mintempC", mintempC);
  Firebase.setInt(firebaseData, "/sensors/maxtempC", maxtempC);
  Firebase.setInt(firebaseData, "/sensors/pressure", pressure);
  Firebase.setInt(firebaseData, "/sensors/winddirDegree", winddirDegree);
  Firebase.setInt(firebaseData, "/sensors/windspeedKmph", windspeedKmph);
}
void displayInfo()
{
  Serial.print(F("Location: "));
  if(gps.location.isValid())
  {
    double latitude = gps.location.lat();
    double longitude = gps.location.lng();
    Serial.print("Lat= ");
    Serial.print(latitude, 6);
    Serial.print(F(","));
    Serial.print("Lng= ");
    Serial.print(longitude, 6);
    Serial.println();
    Firebase.pushDouble(firebaseData, "/datasensors/latitude", latitude);
    Firebase.pushDouble(firebaseData, "/datasensors/longitude", longitude);
  }  
  else
  {
    Serial.println(F("INVALID"));

  }
  
}
void readDHT(){
float t = dht22.getTemperature();
  float h = dht22.getHumidity();
    Serial.print("temp:");
  Serial.print(t);
  Serial.print("humi:");
  Serial.println(h);
  Firebase.setFloat(firebaseData, "/sensors/temperature", t);
  Firebase.setFloat(firebaseData, "/sensors/humidity", h);
  Firebase.pushFloat(firebaseData, "/datasensors/temperature", t);
  Firebase.pushFloat(firebaseData, "/datasensors/humidity", h);

  delay(1000);
}
void updateSerial()
{
  delay(500);
  while (Serial.available())
  {
    Serial2.write(Serial.read());//Forward what Serial received to Software Serial Port
  }
  while (Serial2.available())
  {
    Serial.write(Serial2.read());//Forward what Software Serial received to Serial Port
  }
}




