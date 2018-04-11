// Control of a brushless motor ensuring the yaw of IMU BNO055 sticks to 0, using interuption at constant rate

// It transmits data via Tx in the following form:
// 0xAA, Yaw (Float, 4bytes), Pitch (Float, 4bytes), Roll (Float, 4bytes), 
// Omegax (Float, 4bytes), Omegay (Float, 4bytes), Omegaz (Float, 4bytes), 0x55
// = 26 bytes

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "math.h"

// definition of some constants to ease computations
const float pi = 3.14159265359;
const float two_pi = 6.28318530718;
const float four_pi_on_three = 4.18879020479;
const float two_pi_on_three = 2.09439510239;
const float rad_to_deg = 57.2957795; // 180/pi

float cumulated_angle = 0;

uint32_t last_time, time_since_last;
uint32_t time_period_us = 10000;

const int ledy_pin = 13;

// for BNO
Adafruit_BNO055 bno = Adafruit_BNO055();
uint8_t sys, gyr, accel, mag = 0;
float omega[3];

void setup() {
  Serial.begin(115200);

  if(!bno.begin(Adafruit_BNO055::OPERATION_MODE_NDOF))
  {
    // There was a problem detecting the BNO055 ... check your connections 
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  digitalWrite(ledy_pin, HIGH);

  while(gyr!=3)
  {
    bno.getCalibration(&sys, &gyr, &accel, &mag);
  }

  digitalWrite(ledy_pin, LOW);
  last_time = micros();
}

void loop() {

  time_since_last = micros()-last_time;
  if(time_since_last >= time_period_us)
  {
    last_time+=time_since_last;
    // gyro data, only gyro data on z axis will be used
    imu::Vector<3> gyro=bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

    cumulated_angle += gyro.z()*time_period_us/1000000;
    Serial.print(time_since_last); Serial.print(" ");
    Serial.println(cumulated_angle*rad_to_deg);
  }
  
  
}

