#include <Wire.h>
const int MPU = 0x68;  // I2C address of the MPU6050
#define LED 9
int16_t accelerometer_x, accelerometer_y, accelerometer_z;
int16_t gyroscope_x, gyroscope_y, gyroscope_z;

void setup() {
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // Set to zero to wake up the MPU6050
  Wire.endTransmission(true);
  Serial.begin(9600);
  pinMode(LED,OUTPUT);
}


void loop() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true);  // request a total of 14 registers

  // Read accelerometer data
  accelerometer_x = Wire.read() << 8 | Wire.read();
  accelerometer_y = Wire.read() << 8 | Wire.read();
  accelerometer_z = Wire.read() << 8 | Wire.read();

  // Read gyroscope data
  gyroscope_x = Wire.read() << 8 | Wire.read();
  gyroscope_y = Wire.read() << 8 | Wire.read();
  gyroscope_z = Wire.read() << 8 | Wire.read();

  // Print the sensor values
  Serial.print("Accelerometer: ");
   accelerometer_x/=1000;
  Serial.print("X = "); Serial.println(accelerometer_x);
  Serial.print("Accelerometer: ");
   accelerometer_y/=1000;
  Serial.print("y = "); Serial.println(accelerometer_y);
  accelerometer_z/=1000;
  Serial.print("z = "); Serial.println(accelerometer_z);
  if(accelerometer_x==0&&accelerometer_y==0&&accelerometer_z==16)
  {
    digitalWrite(LED,HIGH);
  }
  else{
  
    digitalWrite(LED,LOW);
    }
  //Serial.print(", Y = "); Serial.print(accelerometer_y);
 // Serial.print(", Z = "); Serial.println(accelerometer_z);

 // Serial.print("Gyroscope: ");
  //Serial.print("X = "); Serial.print(-gyroscope_x);
 // Serial.print(", Y = "); Serial.print(gyroscope_y);
  //Serial.print(", Z = "); Serial.println(gyroscope_z);

  delay(1000);  // Delay for 1 second
}
