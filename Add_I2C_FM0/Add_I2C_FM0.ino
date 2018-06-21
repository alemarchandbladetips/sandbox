
#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_tockn.h>
#include "wiring_private.h" // pinPeripheral() function

// WARNING !!!! 2-10K pullup resistors are required on SDA and SCL, both go to 3.3V! You can use your oscilloscope to see the data traces

///////////////// declaration of I2C on pin 11(SDA) and 13(SCL) /////////////////////////
TwoWire myWire(&sercom1, 11, 13);

#define MCP4725_CMD_WRITEDAC            (0x40)
#define MCP4725_ADDR                    (0x62)

//////////////////////////////////////////////////////////////////

MPU6050 mpu6050(myWire);

long timer;

void setup() {

  Serial.begin(115200);
  myWire.begin();

    // Assign pins 13 & 11 to SERCOM functionality
  pinPeripheral(11, PIO_SERCOM);
  pinPeripheral(13, PIO_SERCOM);

  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
}

void loop() {

  mpu6050.update();

  if( (micros() - timer > 10000) )
  {
  
    timer = micros();

    Serial.print(timer);Serial.print("\t");
    Serial.print(1000*mpu6050.getAccX());Serial.print("\t");
    Serial.print(1000*mpu6050.getAccY());Serial.print("\t");
    Serial.print(1000*mpu6050.getAccZ());Serial.print("\t");
  
    Serial.print(mpu6050.getGyroX());Serial.print("\t");
    Serial.print(mpu6050.getGyroY());Serial.print("\t");
    Serial.println(mpu6050.getGyroZ());

  }

}
