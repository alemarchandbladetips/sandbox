#include <SPI.h>
#include <SD.h>
#include <MPU6050_tockn.h>
#include <Wire.h>

MPU6050 mpu6050(Wire);

const int chipSelect = 4;
String filename;
int state, log_state, state2;
long click_time,timer;
File dataFile;


void setup() {
  // put your setup code here, to run once
  Serial.begin(115200);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial.println("start");

  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  
}

void loop() {
  // put your main code here, to run repeatedly:

   mpu6050.update();

  if( (micros() - timer > 5000) ){
    
    timer = micros();
    
    Serial.print(1000*mpu6050.getAccX());Serial.print("\t");
    Serial.print(1000*mpu6050.getAccY());Serial.print("\t");
    Serial.print(1000*mpu6050.getAccZ());Serial.print("\t");
  
    Serial.print(mpu6050.getGyroX());Serial.print("\t");
    Serial.print(mpu6050.getGyroY());Serial.print("\t");
    Serial.println(mpu6050.getGyroZ());

  }

  
}
