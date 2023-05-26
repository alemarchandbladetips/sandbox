#include <Arduino.h>

uint8_t speed = 0;
int8_t acc_pin_L = 11;
int8_t acc_pin_R = 3;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(acc_pin_L,OUTPUT);
  pinMode(acc_pin_R,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(9,OUTPUT);
  pinMode(10,OUTPUT);

// pins 9 & 10;
  TCCR1A = 0b00000001; // 8bit
  TCCR1B = 0b00001011; // 1kHz
  //TCCR1B = 0b00000010; // 4kHz

// Pins 3 et micros
  //TCCR0A = 0b00000001; // 8bit
  //TCCR0B = 0b00001011; // 1kHz 

// Pin 5
  TCCR3A = 0b00000001; // 8bit
  //TCCR3B = 0b00001011; // 1kHz
  TCCR3B = 0b00000010; // 4kHz

// Pin 6
  TCCR4A = 0b00000001; // 8bit
  //TCCR4B = 0b00000100; // 4kHz
  //TCCR4B = 0b00000101; // 2kHz
  TCCR4B = 0b00000110; // 1kHz

}

void loop() {
  // put your main code here, to run repeatedly:

  analogWrite(acc_pin_L,speed);
  analogWrite(acc_pin_R,speed);
  analogWrite(5,speed);
  analogWrite(6,speed);
  analogWrite(9,speed);
  analogWrite(10,speed);
  Serial.println(speed);
  delay(2000);
  speed+=5;

}