#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "math.h"

#define POZYX_PACKET_SIZE 5 // number of bytes to be recieved from 
#define POZYX_PACKET_START 0xAA // starting char of package
#define POZYX_PACKET_STOP 0x55 // starting char of package

#define DELAY_SAMPLE 10

// Pozyx variables
uint8_t pozyx_data[POZYX_PACKET_SIZE-1] = {0,0,0,0};
uint8_t pozyx_data_buffer[POZYX_PACKET_SIZE];

// Choose between transmitting data to next level or printing for debug
const int8_t transmit_raw = 0;
const int8_t print_data = 1;

Adafruit_BNO055 bno = Adafruit_BNO055();
float euler_angle[3], omega[3], quaternion[4], rpy[3], proper_acc[3], proper_acc_world[3];
uint8_t sys, gyr, accel, mag = 0;
uint32_t accuracy_mask = 0;
uint8_t first_update = 0;

float acc_offset = 0;
float acc_filt = 0;
float acc_filt_buffer[DELAY_SAMPLE];

uint32_t last_time;
uint32_t time_period_us = 10000;
float dt;

float speed_mmps;
float speed_mmps_delay;
float speed_mmps_buffer[DELAY_SAMPLE];
float mes_speed_mmps;
float position_mm;
float position_mm_delay;
float position_mm_buffer[DELAY_SAMPLE];
float mes_position_mm;

float est_speed_mmps;
float est_speed_mmps_delay;
float est_speed_mmps_buffer[DELAY_SAMPLE];
float est_position_mm;
float est_position_mm_delay;
float est_position_mm_buffer[DELAY_SAMPLE];

int16_t buffer_int16;
unsigned char *ptr_buffer_int16 = (unsigned char *)&buffer_int16;

long t0,t1;
uint8_t has_data = 0;
uint8_t new_position_data = 0;
uint8_t first_sample = 1;

// DBG
long t_dbg;
/////////////////////////////////////////////////////////////////////

float add_to_buffer(float *ptr_buffer, float new_data, int32_t buffer_length)
{
  int32_t i;
  //float oldest_value;

  //oldest_value = ptr_buffer[0];
  for(i=0;i<buffer_length-1;i++)
  {
    ptr_buffer[i] = ptr_buffer[i+1];
  }
  ptr_buffer[buffer_length-1] = new_data;

  return ptr_buffer[0];
}

void clear_buffer(float *ptr_buffer, int32_t buffer_length)
{
  int32_t i;

  for(i=0;i<buffer_length;i++)
  {
    ptr_buffer[i] = 0;
  }
}

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
  Serial.begin(115200);

  if(!bno.begin(Adafruit_BNO055::OPERATION_MODE_NDOF))
  {
    // There was a problem detecting the BNO055 ... check your connections 
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  est_speed_mmps = 0;
  est_position_mm = 0;
  dt = time_period_us/1000000.0;
  new_position_data = 0;

  clear_buffer(acc_filt_buffer,DELAY_SAMPLE);
  clear_buffer(position_mm_buffer,DELAY_SAMPLE);
  clear_buffer(speed_mmps_buffer,DELAY_SAMPLE);
  clear_buffer(est_position_mm_buffer,DELAY_SAMPLE);
  clear_buffer(est_speed_mmps_buffer,DELAY_SAMPLE);

  last_time = micros();
}

/////////////////////////////////////////////////////////////////////

void loop() {

  int8_t i,j;
  long time_since_last;
  uint8_t x;

////////////////////////////////
// reading speed and position info
  if (Serial.available() > POZYX_PACKET_SIZE) // Number of data corresponding to the IMU packet size is waiting in the biffer of serial
  { 
    x = Serial.read(); // read first data
    if(x == POZYX_PACKET_START) // check that first data correspond to start char
    {
      Serial.readBytes(pozyx_data_buffer,POZYX_PACKET_SIZE); // Reading the POZYX packet

      if(pozyx_data_buffer[POZYX_PACKET_SIZE-1] == POZYX_PACKET_STOP) // check taht the last data correspond to the packet end char
      {
        for(j=0;j<2;j++)
        {
          ptr_buffer_int16[j] = pozyx_data_buffer[j];
        }
        speed_mmps = buffer_int16/32768.0*2000.0;
        speed_mmps_delay = add_to_buffer(speed_mmps_buffer,speed_mmps,DELAY_SAMPLE);
        for(j=0;j<2;j++)
        {
          ptr_buffer_int16[j] = pozyx_data_buffer[j+2];
        }
        position_mm = buffer_int16/32768.0*1200.0;
        position_mm_delay = add_to_buffer(position_mm_buffer,position_mm,DELAY_SAMPLE);
        new_position_data++;
      }
    }
  }

////////////////////////////////
// Start of fusion algo

  time_since_last = micros()-last_time;
  if(time_since_last >= time_period_us)
  {

    est_position_mm += est_speed_mmps*dt + (acc_filt)*dt*dt;
    est_position_mm_delay = add_to_buffer(est_position_mm_buffer,est_position_mm,DELAY_SAMPLE);
    
    est_speed_mmps += (acc_filt)*dt;
    est_speed_mmps_delay = add_to_buffer(est_speed_mmps_buffer,est_speed_mmps,DELAY_SAMPLE);

    if(new_position_data>=10)
    {
      mes_position_mm = position_mm_delay + random(-5,5);
      mes_speed_mmps = speed_mmps_delay + random(-5,5);

      est_position_mm_buffer[0] += 0.5*(mes_position_mm-est_position_mm_buffer[0]);
      est_speed_mmps_buffer[0] += 0.9*(mes_speed_mmps-est_speed_mmps_buffer[0]);
      
      for(i=0;i<DELAY_SAMPLE-1;i++)
      {
        est_position_mm_buffer[i+1] = est_position_mm_buffer[i] + est_speed_mmps_buffer[i]*dt + acc_filt_buffer[i]*dt*dt;
        est_speed_mmps_buffer[i+1] = est_speed_mmps_buffer[i] + acc_filt_buffer[i]*dt;
      }
      
      est_position_mm = est_position_mm_buffer[DELAY_SAMPLE-1];
      est_speed_mmps = est_speed_mmps_buffer[DELAY_SAMPLE-1];
      
      new_position_data = 0;
    }
  
    //if(print_data){ Serial.print(time_since_last); Serial.print(" ");}
    if(first_update)
    {
      last_time = micros();
      first_update = 0;
    } else
    {
      last_time += time_period_us;
    }
    
  //// reading BNO data
    // Accel data
    imu::Vector<3> acc=bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    proper_acc[0] = -acc.x()*1000;
    proper_acc[1] = acc.y();
    proper_acc[2] = acc.z();
    
    if(has_data) //// Transmitting data
    {
      if(print_data)
      {
        Serial.print(position_mm); Serial.print(" ");
        //Serial.print(position_mm_delay); Serial.print(" ");
        Serial.print(mes_position_mm); Serial.print(" ");
        Serial.print(est_position_mm); Serial.print(" ");
        
        Serial.print(speed_mmps); Serial.print(" ");
        //Serial.print(speed_mmps_delay); Serial.print(" ");
        Serial.print(mes_speed_mmps); Serial.print(" ");
        Serial.print(est_speed_mmps); Serial.print(" ");
        
        //Serial.print(proper_acc[0]); Serial.print(" ");
        //Serial.print(acc_filt); Serial.print(" ");
        //Serial.print(acc_offset); Serial.print(" ");
      }
    }
    
    has_data = 1;
    acc_offset += 0.0001*(proper_acc[0]-acc_offset);
    acc_filt += 1*(proper_acc[0]-acc_offset-acc_filt);
    add_to_buffer(acc_filt_buffer,acc_filt,DELAY_SAMPLE);
    
    if(print_data){ Serial.println(" "); }
  }

}
