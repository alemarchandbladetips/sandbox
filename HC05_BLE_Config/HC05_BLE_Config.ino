#include <SoftwareSerial.h>

#define BLE_serial Serial1

int x;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(23,OUTPUT);
  digitalWrite(23,HIGH);
  Serial.println("Enter AT commands:");
  BLE_serial.begin(38400);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(BLE_serial.available())
  {
    Serial.write(BLE_serial.read());
  }

  if(Serial.available())
  {
    x = Serial.read();
    Serial.write(x);
    BLE_serial.write(x);
  }
  
}
