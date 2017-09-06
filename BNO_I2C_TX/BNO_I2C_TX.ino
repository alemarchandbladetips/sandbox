#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "math.h"

#define POZYX_PACKET_SIZE 5 // number of bytes to be recieved from 
#define POZYX_PACKET_START 137 // starting char of package
#define POZYX_PACKET_STOP 173 // starting char of package

// Pozyx variables
uint8_t pozyx_data[POZYX_PACKET_SIZE-1] = {0,0,0,0};
uint8_t pozyx_data_buffer[POZYX_PACKET_SIZE];

// Choose between transmitting data to next level or printing for debug
const int8_t transmit_raw = 1;
const int8_t print_data = 0;

Adafruit_BNO055 bno = Adafruit_BNO055();
float euler_angle[3], omega[3], quaternion[4], rpy[3], proper_acc[3];
uint8_t sys, gyr, accel, mag = 0;
uint32_t accuracy_mask = 0;
uint8_t first_update = 0;

uint32_t last_time;
uint32_t time_period_us = 10000;

float buffer_float;
unsigned char *ptr_buffer = (unsigned char *)&buffer_float;
uint32_t buffer_uint32;
unsigned char *ptr_buffer_uint32 = (unsigned char *)&buffer_uint32;

long t0,t1;
uint8_t has_data = 0;

// DBG
long t_dbg;

/////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(115200);

  if(!bno.begin(Adafruit_BNO055::OPERATION_MODE_NDOF))
  {
    // There was a problem detecting the BNO055 ... check your connections 
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  while(gyr!=3)
  {
    bno.getCalibration(&sys, &gyr, &accel, &mag);
  }

  last_time = micros();
}

/////////////////////////////////////////////////////////////////////

void loop() {

  int8_t i,j;
  long time_since_last;
  uint8_t x;


  if (Serial.available() > POZYX_PACKET_SIZE) // Number of data corresponding to the IMU packet size is waiting in the biffer of serial
  { 
    x = Serial.read(); // read first data
    if(x == POZYX_PACKET_START) // check that first data correspond to start char
    {
      Serial.readBytes(pozyx_data_buffer,POZYX_PACKET_SIZE); // Reading the POZYX packet

      if(pozyx_data_buffer[POZYX_PACKET_SIZE-1] == POZYX_PACKET_STOP) // check taht the last data correspond to the packet end char
      {
        for(i=0;i<4;i++)
        {
          pozyx_data[i] = pozyx_data_buffer[i];
        }
      }
    }
  }

  time_since_last = micros()-last_time;
  if(time_since_last >= time_period_us)
  {
    if(print_data){ Serial.print(time_since_last); Serial.print(" ");}
    if(first_update)
    {
      last_time = micros();
      first_update = 0;
    } else
    {
      last_time += time_period_us;
    }
  
    if(has_data) //// Transmitting data
    {
          // header of the package
      if(transmit_raw){ Serial.write(0xAA);}
  
      // Transmition of acc data
      for(i=0;i<3;i++) 
      {
        buffer_float = proper_acc[i];
        if(print_data){ Serial.print(buffer_float); Serial.print(" ");}
        for(j=0;j<4;j++)
        {
          if(transmit_raw){ Serial.write(ptr_buffer[j]);}
        }
      }
      
      // Transmition of gyro data
      for(i=0;i<3;i++) 
      {
        buffer_float = omega[i];
        if(print_data){ Serial.print(buffer_float); Serial.print(" "); }
        for(j=0;j<4;j++)
        {
          if(transmit_raw){ Serial.write(ptr_buffer[j]);}
        }
      }
  
      // transmition of quaternion data
      for(i=0;i<4;i++) // Transmition of quat data
      {
        buffer_float = quaternion[i];
        if(print_data){ Serial.print(buffer_float); Serial.print(" ");}
        for(j=0;j<4;j++)
        {
          if(transmit_raw){ Serial.write(ptr_buffer[j]);}
        }
      }
  
      // Accuracy flags
      buffer_uint32 = accuracy_mask;
      if(print_data){ Serial.print(buffer_uint32); Serial.print(" "); }
      for(j=0;j<4;j++)
      {
        if(transmit_raw){ Serial.write(ptr_buffer_uint32[j]);}
      }

      for(i=0;i<4;i++) // Transmition of quat data
      {
        if(print_data){ Serial.print(pozyx_data[i]); Serial.print(" ");}
        if(transmit_raw){ Serial.write(pozyx_data[i]);}
      }
  
      // footer of the package
      if(transmit_raw){ Serial.write(0x55); }
    }

    
    
    //// reading BNO data
    // Accel data
    imu::Vector<3> acc=bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    proper_acc[0] = acc.x();
    proper_acc[1] = acc.y();
    proper_acc[2] = acc.z();

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
    bno.getCalibration(&sys, &gyr, &accel, &mag);
    accuracy_mask = (uint32_t)mag + (((uint32_t)accel)<<2) + (((uint32_t)gyr)<<4) + (((uint32_t)sys)<<6);

    has_data = 1;
    
    if(print_data){ Serial.println(" "); }
  }

}
