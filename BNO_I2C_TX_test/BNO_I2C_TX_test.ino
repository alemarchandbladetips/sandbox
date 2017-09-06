#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "math.h"

// Choose between transmitting data to next level or printing for debug
const int8_t transmit_raw = 1;
const int8_t print_data = 0;

Adafruit_BNO055 bno = Adafruit_BNO055();
float euler_angle[3], omega[3], quaternion[4], rpy[3], proper_acc[3];
uint8_t sys, gyr, accel, mag = 0;
uint8_t imu_init = 0;

uint32_t last_time;
uint32_t time_period_us = 10000;

float buffer_float;
unsigned char *ptr_buffer = (unsigned char *)&buffer_float;
uint32_t buffer_uint32;
unsigned char *ptr_buffer_uint32 = (unsigned char *)&buffer_uint32;

long t0,t1;

/////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(115200);

  if(!bno.begin(Adafruit_BNO055::OPERATION_MODE_NDOF))
  {
    // There was a problem detecting the BNO055 ... check your connections 
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  bno.setMode(Adafruit_BNO055::OPERATION_MODE_AMG);
  //bno.setExtCrystalUse(true);

  while(gyr!=3)
  {
    bno.getCalibration(&sys, &gyr, &accel, &mag);
  }

  last_time = micros();
  t0 = micros();
}

/////////////////////////////////////////////////////////////////////

void loop() {

  int8_t i,j;
  long time_since_last;
  uint32_t accuracy_mask;

  time_since_last = micros()-last_time;
  if(time_since_last >= time_period_us)
  {
    
    //Serial.print(time_since_last);Serial.print(" ");
    last_time += time_period_us;
    
    t1 = micros();
        //// reading BNO data
    // Accel data
    
    imu::Vector<3> acc=bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    proper_acc[0] = acc.x();
    proper_acc[1] = acc.y();
    proper_acc[2] = acc.z();
/*
    // gyro data
    imu::Vector<3> gyro=bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    omega[0] = gyro.x();
    omega[1] = gyro.y();
    omega[2] = gyro.z();

    // quaternion
    imu::Quaternion quat = bno.getQuat();
    quaternion[0] = quat.w();
    quaternion[1] = quat.x();
    quaternion[2] = quat.y();
    quaternion[3] = quat.z();

    // Accuracy flags
    bno.getCalibration(&sys, &gyr, &accel, &mag); //900us
    accuracy_mask = (uint32_t)mag + (((uint32_t)accel)<<2) + (((uint32_t)gyr)<<4) + (((uint32_t)sys)<<6);

  */
    //if(transmit_raw){ Serial.write(0x55); }
    Serial.println(micros()-t1);//Serial.print(" ");
    //Serial.println(micros()-t0);
    //t0=micros();

  }

}
