#include <SoftwareSerial.h>

#define PACKET_START 0xAA // starting char of package
#define PACKET_STOP 0x55 // starting char of package
#define PACKET_SIZE 26 // size to be recieved

// constant used to enable/disable communication, debug, timing checks
const int8_t transmit_raw = 1;
const int8_t print_data = 0;
const int8_t print_timing = 1;

uint8_t gps_buffer[26];
int16_t buffer_int16;
unsigned char *ptr_buffer_int16 = (unsigned char *)&buffer_int16;
int32_t buffer_int32;
unsigned char *ptr_buffer_int32 = (unsigned char *)&buffer_int32;

float NED_coordinates[3];
float NED_coordinates_accuracy[3];
float NED_speed[3];

int32_t t0;

SoftwareSerial mySerial(9, 2); // RX, TX

void setup() {
  // put your setup code here, to run once:
  mySerial.begin(57600);
  Serial.begin(115200);

}

void loop() {
  // put your main code here, to run repeatedly:

  uint8_t i,j,x;

  if(mySerial.available() > PACKET_SIZE-1)
  {
    if(print_timing){ t0 = micros();}
    x = mySerial.read();
    if(x == PACKET_START)
    {
      mySerial.readBytes(gps_buffer,PACKET_SIZE-1);
      if (gps_buffer[PACKET_SIZE-2] == PACKET_STOP)
      {
        for(i=0;i<3;i++)
        {
          for(j=0;j<4;j++)
          {
            ptr_buffer_int32[j] = gps_buffer[4*i+j];
          }
          NED_coordinates[i] = (float)buffer_int32/10000.0;
          if(print_data){ Serial.print(NED_coordinates[i]); Serial.print(" ");}
        }
    
        // NED_coordinates_accuracy
        for(i=0;i<3;i++)
        {
          for(j=0;j<2;j++)
          {
            ptr_buffer_int16[j] = gps_buffer[2*i+j+12];
          }
          NED_coordinates_accuracy[i] = (float)buffer_int16/10000.0;
          if(print_data){ Serial.print(NED_coordinates_accuracy[i]); Serial.print(" ");}
        }

        // NED_coordinates_accuracy
        for(i=0;i<3;i++)
        {
          for(j=0;j<2;j++)
          {
            ptr_buffer_int16[j] = gps_buffer[2*i+j+18];
          }
          NED_speed[i] = (float)buffer_int16/100.0;
          if(print_data){ Serial.print(NED_speed[i]); Serial.print(" ");}
        }
        if(print_data){ Serial.println(" ");}
        if(print_timing){ Serial.println(micros()-t0);}
      }
    }
  }

}
