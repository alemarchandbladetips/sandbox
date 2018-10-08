#include <Arduino.h>
#include "wiring_private.h" // pinPeripheral() function
#include <SPI.h>
#include <SD.h>

// variables for the log
const int chipSelect = 4;
String filename;
int log_state = 0;
int long_num = 0;
File dataFile;

const int led_pin = 19;
const int32_t Ts=1000;
uint8_t input;
uint32_t timer, timer_tmp;
uint32_t dt;

float Courant=0;
const float gain = 1.0;
const float offset = 0.0;

void setup() {

  Serial.begin(115200);
  Serial1.begin(115200);
  // put your setup code here, to run once:
  
}

void loop() {
  
  if(Serial1.available())
  {
    timer_tmp = micros();
    dt = timer_tmp - timer;
    timer = timer_tmp;
    input = Serial1.read();   
    //Serial.print(input); Serial.print(" ");
      Courant = (input-18)*7.0/11.0;
      Serial.println(Courant);
  }
}
