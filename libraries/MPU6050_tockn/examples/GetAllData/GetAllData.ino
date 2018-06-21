
#include <MPU6050_tockn.h>
#include <Wire.h>

MPU6050 mpu6050(Wire);

long timer = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
}

void loop() {
  mpu6050.update();

  if(micros() - timer > 10000){
    
    timer = micros();
    
//    Serial.print(mpu6050.getAccX());Serial.print("\t");
//    Serial.print(mpu6050.getAccY());Serial.print("\t");
//    Serial.println(mpu6050.getAccZ());Serial.print("\t");
  
    Serial.print(mpu6050.getGyroX());Serial.print("\t");
    Serial.print(mpu6050.getGyroY());Serial.print("\t");
    Serial.println(mpu6050.getGyroZ());
  }

}
