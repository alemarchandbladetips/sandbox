#include <SoftwareSerial.h>


#define POZYX_PACKET_SIZE 5 // number of bytes to be recieved from 
#define POZYX_PACKET_START 137 // starting char of package
#define POZYX_PACKET_STOP 173 // starting char of package

// constant used to enable/disable communication, debug, timing checks
const int8_t transmit_raw = 0;
const int8_t print_data = 1;

const float pi = 3.14159265359;

// variables for the serial read an data recomposition
float all_data[8];
uint8_t raw_data[100];
float buffer_float;
unsigned char *ptr_buffer = (unsigned char *)&buffer_float;
int16_t buffer_int16;
unsigned char *ptr_buffer_int16 = (unsigned char *)&buffer_int16;

const float rad_to_deg = 57.2957795; // 180/pi

SoftwareSerial mySerial(8, 9); // RX, TX
 
void setup() {
  Serial.begin(115200);

  mySerial.begin(115200);

}

//////////////////////////////////////////////////////////////////////
 
void loop() {
int i,j,x;
//Serial.println(mySerial.available());
  if (mySerial.available() > POZYX_PACKET_SIZE) // Number of data corresponding to the IMU packet size is waiting in the biffer of serial
  { 
    x = mySerial.read(); // read first data
    //Serial.print(x);
    if(x == POZYX_PACKET_START) // check that first data correspond to start char
    {
      mySerial.readBytes(raw_data,POZYX_PACKET_SIZE); // Reading the IMU packet
      //Serial.print(raw_data[GIMBAL_PACKET_SIZE-1]);
      if(raw_data[POZYX_PACKET_SIZE-1] == POZYX_PACKET_STOP) // check taht the last data correspond to the packet end char
      {
        for(i=0;i<4;i++) // decode YPR data
        {
          if(print_data){ Serial.print(raw_data[i]); Serial.print(" "); }
        }
        Serial.println(" ");
      }
    }
  }

  if (Serial.available() > POZYX_PACKET_SIZE) // Number of data corresponding to the IMU packet size is waiting in the biffer of serial
  { 
    x = Serial.read(); // read first data
    Serial.print(x);
    if(x == PACKET_START) // check that first data correspond to start char
    {
      Serial.readBytes(raw_data,POZYX_PACKET_SIZE); // Reading the IMU packet
      //Serial.print(raw_data[GIMBAL_PACKET_SIZE-1]);
      if(raw_data[POZYX_PACKET_SIZE-1] == PACKET_STOP) // check taht the last data correspond to the packet end char
      {
        for(i=0;i<4;i++) // decode YPR data
        {
          if(print_data){ Serial.print(raw_data[i]); Serial.print(" "); }
        }
        Serial.println(" ");
      }
    }
  }
  
}


