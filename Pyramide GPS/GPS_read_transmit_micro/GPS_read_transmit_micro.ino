#include <SoftwareSerial.h>

#include "math.h"

#define PACKET_START 0xAA // starting char of package
#define PACKET_STOP 0x55 // starting char of package

#define LED_PIN 13

// constant used to enable/disable communication, debug, timing checks
const int8_t transmit_raw = 1;
const int8_t print_data = 1;
const int8_t print_timing = 0;

int i,j;
uint8_t x;
uint8_t gps_msg_speed[44]; uint8_t gps_msg_pos[44];
uint8_t gps_header[2];
int32_t int32_buffer;
unsigned char *ptr_int32_buffer = (unsigned char *)&int32_buffer;
uint32_t uint32_buffer;
unsigned char *ptr_uint32_buffer = (unsigned char *)&uint32_buffer;
int16_t buffer_int16;
unsigned char *ptr_buffer_int16 = (unsigned char *)&buffer_int16;
int32_t buffer_int32;
unsigned char *ptr_buffer_int32 = (unsigned char *)&buffer_int32;

float NED_coordinates[3], NED_coordinates_prev[3], NED_coordinates_offset[3];
float NED_coordinates_accuracy[3];
int32_t NED_speed[3];
uint32_t NED_speed_accuracy;
float NED_acc[3];
uint8_t gps_flag, diffSoln, carrSoln, carrSoln_prev, relPosValid, gpsSanity = 0;

uint32_t last_time,time_since_last, t0;
uint32_t time_period_us = 10000;

uint32_t accuracy_mask;

int8_t expected_size = 40;

int8_t led_counter, led_status = 0;

uint8_t first_update, header_read, position_recieved;

SoftwareSerial mySerial(9, 8); // RX, TX

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
  Serial1.begin(57600);
  Serial.begin(57600);
  //Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  led_status = 1;
  Serial.println("Start");
  delay(1000);
  i = 0;
  last_time = micros();
  first_update = 1;
  header_read = 0;
  position_recieved = 0;
  mySerial.begin(38400);
  led_counter = 0;
}

/////////////////////////////////////////////////////////////////////

void loop() {
  // put your main code here, to run repeatedly:

//Serial.println(mySerial.available());

  if(header_read==0 && Serial1.available()>3)
  {
    //digitalWrite(13, HIGH);
    x = Serial1.read();
    //Serial.print(x); Serial.print(" ");
    
    if (x==0xB5) //181
    {
      x = Serial1.read();
      //Serial.print(x); Serial.print(" ");
      if (x==0x62) //98
      {
        Serial1.readBytes(gps_header,2); 

        led_counter++;
        //Serial.print(led_counter);Serial.print(" ");
        if(led_counter >=5)
        {
          if(led_status==1)
          {
            digitalWrite(LED_PIN, LOW);
            led_status = 0;
          } else
          {
            digitalWrite(LED_PIN, HIGH);
            led_status = 1;
          }
          //Serial.println(led_status);
          led_counter = 0;
        }
        
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
    //Serial.println(" ");
  }
  if(header_read==1 && Serial1.available()>expected_size-1 )
  {
    t0 = micros();
    header_read = 0;
    if (gps_header[0] == 0x01 && gps_header[1] == 0x3C )
    { // position
      Serial1.readBytes(gps_msg_pos,44);
      position_recieved = 1;
    } else if (gps_header[0] == 0x01 && gps_header[1] == 0x12 )
    { // speed
      Serial1.readBytes(gps_msg_speed,40);
    }
  }

  if (position_recieved == 1)
  {
    position_recieved = 0;
//////
    gps_flag = gps_msg_pos[38];
    diffSoln = (gps_flag&0x02)>>1;
    relPosValid = (gps_flag&0x04)>>2;
    carrSoln = (gps_flag&0x18)>>3;
/*
    if(print_data){ Serial.print(gps_flag); Serial.print(" ");}
    if(print_data){ Serial.print(diffSoln); Serial.print(" ");}
    if(print_data){ Serial.print(relPosValid); Serial.print(" ");}
    if(print_data){ Serial.print(carrSoln); Serial.print(" ");}
    if(print_data){ Serial.print(gpsSanity); Serial.print(" ");}
*/
    gpsSanity = (diffSoln==1 && relPosValid==1 && carrSoln>0);
    
    for(i=0;i<3;i++)
      {
        for(j=0;j<4;j++)
        {
          ptr_int32_buffer[j] = gps_msg_pos[4*i+j+10];
        }
        NED_coordinates[i] = ((float)(int32_buffer)+((float)gps_msg_pos[i+22]*0.01));
        Serial.print(NED_coordinates[i]);Serial.print(" ");
      }
      //Serial.println(Serial.available());
      for(i=0;i<3;i++)
      {
        for(j=0;j<4;j++)
        {
          ptr_uint32_buffer[j] = gps_msg_pos[4*i+j+26];
        }
        NED_coordinates_accuracy[i] = (float)(uint32_buffer)*0.01;
        Serial.print(NED_coordinates_accuracy[i]);Serial.print(" ");
      }

      //if(print_data){ Serial.print(carrSoln); Serial.print(" ");}
      //if(print_data){ Serial.print(carrSoln_prev); Serial.print(" ");}
      //if(print_data){ Serial.print(NED_coordinates_prev[0]); Serial.print(" ");}
      //if(print_data){ Serial.print(NED_coordinates[0]); Serial.print(" ");}
      

      if(((carrSoln==1) && (carrSoln_prev==2)) || ((carrSoln==2) && (carrSoln_prev==1)))
      {
        if(print_data){ Serial.print("switch"); Serial.print(" ");}
        for(i=0;i<3;i++)
        {
          NED_coordinates_offset[i] = NED_coordinates_prev[i]-NED_coordinates[i];
        }
      }

      for(i=0;i<3;i++)
      {
        NED_coordinates[i] = NED_coordinates[i]+NED_coordinates_offset[i];
        NED_coordinates_prev[i] = NED_coordinates[i];
      }
      carrSoln_prev = carrSoln;
      if(print_data){ Serial.print(NED_coordinates[0]); Serial.print(" ");}
      if(print_data){ Serial.print(NED_coordinates_offset[0]); Serial.print(" ");}
//////
    for(i=0;i<3;i++)
    {
      for(j=0;j<4;j++)
      {
        ptr_int32_buffer[j] = gps_msg_speed[4*i+j+6];
      }
      NED_speed[i] = ((float)(int32_buffer));
      Serial.print(NED_speed[i]);Serial.print(" ");
      }
      
      //Serial.println(Serial.available());
      for(j=0;j<4;j++)
      {
        ptr_uint32_buffer[j] = gps_msg_speed[j+30];
      }
      NED_speed_accuracy = uint32_buffer;

    // Data transmition, to the control part of the drone

    if(transmit_raw){ mySerial.write(PACKET_START); } // starting byte
    
    // NED_coordinates
    for(i=0;i<3;i++)
    {
      buffer_int32 = (int32_t)constrain((NED_coordinates[i]*100.0),-2147483648,2147483647);
      //if(print_data){ Serial.print(buffer_int32); Serial.print(" ");}
      for(j=0;j<4;j++)
      {
        if(transmit_raw){ mySerial.write(ptr_buffer_int32[j]); }
      }
    }

    // NED_coordinates_accuracy
    for(i=0;i<3;i++)
    {
      buffer_int16 = (int16_t)constrain((NED_coordinates_accuracy[i]*100.0),-32768,32767)*(int16_t)gpsSanity;
      //if(print_data){ Serial.print(buffer_int16); Serial.print(" ");}
      for(j=0;j<2;j++)
      {
        if(transmit_raw){ mySerial.write(ptr_buffer_int16[j]); } // we transmit each bytes of the int16 buffer using this pointer
      }
    }

    // NED_speed
    for(i=0;i<3;i++)
    {
      buffer_int16 = (int16_t)constrain((NED_speed[i]*1.0),-32768,32767);
      //if(print_data){ Serial.print(buffer_int16); Serial.print(" ");}
      for(j=0;j<2;j++)
      {
        if(transmit_raw){ mySerial.write(ptr_buffer_int16[j]); } // we transmit each bytes of the int16 buffer using this pointer
      }
    }
    Serial.println(" ");

    //if(print_data){ Serial.println(" ");}

    if(transmit_raw){ mySerial.write(PACKET_STOP); } // ending byte
  }
}
