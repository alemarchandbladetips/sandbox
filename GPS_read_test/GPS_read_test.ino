#include <SoftwareSerial2.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "math.h"

int i,j;
uint8_t x;
uint8_t gps_msg[44];
uint8_t gps_header[2];
int32_t int32_buffer;
unsigned char *ptr_int32_buffer = (unsigned char *)&int32_buffer;
uint32_t uint32_buffer;
unsigned char *ptr_uint32_buffer = (unsigned char *)&uint32_buffer;

float NED_coordinates[3];
float NED_coordinates_accuracy[3];
float NED_speed[3];
float NED_acc[3];

// for BNO
Adafruit_BNO055 bno = Adafruit_BNO055();
uint8_t sys, gyr, accel, mag = 0;
float quaternion[4], proper_acc[3];

uint32_t last_time,time_since_last, t0;
uint32_t time_period_us = 10000;

uint32_t accuracy_mask;

int8_t expected_size = 40;

uint8_t first_update, header_read;

SoftwareSerial mySerial(10, 11); // RX, TX

//////////////////////////////////////////////////////////////////////

// Projection of a vector using a quaternion
// input: q[4]: quaternion in flt
// input: v_in[3]: vector to be projected in the new base in flt
// output: v_out[3]: v_in rotated thanks to quaternion input

void vector_quat_proj(float q[4], float v_in[3], float v_out[3])
{
  v_out[0] = (1 - 2*q[2]*q[2] - 2*q[3]*q[3]) * v_in[0] + (2*q[1]*q[2] - 2*q[3]*q[0]) * v_in[1]     + (2*q[1]*q[3] + 2*q[2]*q[0]) * v_in[2];
  v_out[1] = (2*q[1]*q[2] + 2*q[3]*q[0]) * v_in[0]     + (1 - 2*q[1]*q[1] - 2*q[3]*q[3]) * v_in[1] + (2*q[2]*q[3] - 2*q[1]*q[0]) * v_in[2];
  v_out[2] = (2*q[1]*q[3] - 2*q[2]*q[0]) * v_in[0]     + (2*q[2]*q[3] + 2*q[1]*q[0]) * v_in[1]     + (1 - 2*q[1]*q[1] - 2*q[2]*q[2]) * v_in[2];
}

/////////////////////////////////////////////////////////////////////

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);

  if(!bno.begin(Adafruit_BNO055::OPERATION_MODE_NDOF))
  {
    // There was a problem detecting the BNO055 ... check your connections 
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  }
  
  Serial.println("Start");
  i = 0;
  last_time = micros();
  first_update = 1;
  header_read = 0;
  mySerial.begin(19200);
}

/////////////////////////////////////////////////////////////////////

void loop() {
  // put your main code here, to run repeatedly:

//Serial.println(mySerial.available());
  
  if(header_read==0 && mySerial.available()>3)
  {
    x = mySerial.read();
    //Serial.print(x);Serial.print(" ");
    if (x==0xB5) //181
    {
      x = mySerial.read();
      //Serial.print(x); Serial.print(" ");
      if (x==0x62) //98
      {
        mySerial.readBytes(gps_header,2); 

        //Serial.print(gps_header[0]); Serial.print(" ");Serial.print(gps_header[1]);Serial.print(" ");Serial.println("Head");
        
        if (gps_header[0] == 0x01 && gps_header[1] == 0x3C )
        { // position
          expected_size = 44;
          header_read = 1;
        } else if (gps_header[0] == 0x01 && gps_header[1] == 0x12 )
        { // speed
          expected_size = 40;
          header_read = 1;
        }
      }
    }
  }
  time_since_last = micros()-last_time;
  if(header_read==1 && mySerial.available()>expected_size-1 )
  {
    t0 = micros();
    //Serial.print(mySerial.available());Serial.print(" ");
    header_read = 0;
    if (gps_header[0] == 0x01 && gps_header[1] == 0x3C )
    { // position
      mySerial.readBytes(gps_msg,44);

      Serial.println("pos ");
      
      for(i=0;i<3;i++)
      {
        for(j=0;j<4;j++)
        {
          ptr_int32_buffer[j] = gps_msg[4*i+j+10];
        }
        NED_coordinates[i] = ((float)(int32_buffer)+((float)gps_msg[i+22]*0.01));
        Serial.print(NED_coordinates[i]);Serial.print(" ");
      }
      Serial.println(" ");
      for(i=0;i<3;i++)
      {
        for(j=0;j<4;j++)
        {
          ptr_uint32_buffer[j] = gps_msg[4*i+j+26];
        }
        NED_coordinates_accuracy[i] = (float)(uint32_buffer)*0.01;
        //Serial.print(NED_coordinates_accuracy[i]);Serial.print(" ");
      }
      for(j=0;j<4;j++)
      {
        ptr_uint32_buffer[j] = gps_msg[j+38];
      }
      //Serial.print(uint32_buffer);Serial.print(" ");
      //Serial.println(" ");
    } else if (gps_header[0] == 0x01 && gps_header[1] == 0x12 )
    { // speed
      mySerial.readBytes(gps_msg,40);

      Serial.println("speed ");

      for(i=0;i<3;i++)
      {
        for(j=0;j<4;j++)
        {
          ptr_int32_buffer[j] = gps_msg[4*i+j+6];
        }
        NED_speed[i] = ((float)(int32_buffer));
        Serial.print(NED_speed[i]);Serial.print(" ");
      }
      Serial.println(" ");
      for(j=0;j<4;j++)
      {
        ptr_uint32_buffer[j] = gps_msg[j+30];
      }
      //Serial.print(uint32_buffer);Serial.print(" ");
      //Serial.println(" ");
    }
    //Serial.print(micros()-t0);Serial.print(" ");
  }
  /*
  time_since_last = micros()-last_time;
  if(time_since_last >= time_period_us)
  {
    Serial.print(time_since_last); Serial.print(" ");
    if(first_update)
    {
      last_time = micros();
      first_update = 0;
    } else
    {
      last_time = micros();//+= time_period_us;
    }
    imu::Vector<3> acc=bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    proper_acc[0] = acc.x();
    proper_acc[1] = acc.y();
    proper_acc[2] = acc.z();
    
    // quaternion
    imu::Quaternion quat = bno.getQuat();
    quaternion[0] = quat.w();
    quaternion[1] = quat.x();
    quaternion[2] = quat.y();
    quaternion[3] = quat.z();

    // Accuracy flags
    bno.getCalibration(&sys, &gyr, &accel, &mag);
    accuracy_mask = (uint32_t)mag + (((uint32_t)accel)<<2) + (((uint32_t)gyr)<<4) + (((uint32_t)sys)<<6);

    vector_quat_proj(quaternion,proper_acc,NED_acc);
    /*
    for(i=0;i<3;i++)
    {
      Serial.print(NED_coordinates[i]);Serial.print(" ");
      Serial.print(NED_speed[i]);Serial.print(" ");
      Serial.print(NED_acc[i]);Serial.print(" ");
    }
    Serial.println(" ");
  }*/
}
