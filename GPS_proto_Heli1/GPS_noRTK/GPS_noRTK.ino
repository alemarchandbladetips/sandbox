#include <SoftwareSerial.h>

#define INIT_TIME_MN 3 //init time in minutes

#define R_TERRE 6366000

#define PACKET_START 0xAA // starting char of package
#define PACKET_STOP 0x55 // starting char of package

const int transmit_raw = 1;
const int print_data = 1;

uint16_t idx,i,j;
uint8_t GPS_buffer[100];
String inString = "";    // string to hold input
float NED[3];

float lat, lat_int_tmp, lat_dec_tmp;
float lon, lon_int_tmp, lon_dec_tmp;
float alt, alt_tmp;

float lat_0_int;
float lat_0_dec;

float lon_0_int;
float lon_0_dec;

float alt_0;

int8_t init_done = 0;
uint32_t timer;

int idx_alt_end;

float buffer_float;
unsigned char *ptr_buffer = (unsigned char *)&buffer_float;

SoftwareSerial mySerial(8, 9); // RX, TX

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  Serial1.begin(57600);
  mySerial.begin(38400);
  timer = millis();
}

void loop() {
  uint8_t x;
  // put your main code here, to run repeatedly:
  if(Serial1.available())
  {
    x = Serial1.read();
    GPS_buffer[idx] = x;

////////////////////////////////////////////////////////
    // latitude
    if(idx>=16 && idx<= 17)
    {
      inString += (char)x;
    }
    if(idx==17)
    {
      lat_int_tmp = inString.toFloat();   
      inString = "";
    }
    if(idx>=18 && idx<= 26)
    {
      inString += (char)x;
    }
    if(idx==26)
    {
      lat_dec_tmp = inString.toFloat()/60;
      inString = "";
    }
    if(idx==28)
    {
      if(x==0x53) // "S"
      {
        lat_int_tmp = -lat_int_tmp;
        lat_dec_tmp = -lat_dec_tmp;
      }
      if(init_done==0)
      {
        lat_0_int = lat_int_tmp;
        lat_0_dec = lat_dec_tmp;
      }
      lat = lat_int_tmp-lat_0_int;
      lat += (lat_dec_tmp-lat_0_dec);
      NED[0] = lat*R_TERRE/180*PI;
      
      
    }

////////////////////////////////////////////////////////
    // longitude

    if(idx>=30 && idx<= 32)
    {
      inString += (char)x;
    }
    if(idx==32)
    {
      lon_int_tmp = inString.toFloat();
      inString = "";
    }
    if(idx>=33 && idx<= 40)
    {
      inString += (char)x;
    }
    if(idx==40)
    {
      lon_dec_tmp = inString.toFloat()/60;
      inString = "";
    }
    if(idx==42)
    {
      if(x==0x57) // "W"
      {
        lon_int_tmp = -lon_int_tmp;
        lon_0_dec = -lon_0_dec;
      }
      if(init_done==0)
      {
        lon_0_int = lon_int_tmp;
        lon_0_dec = lon_dec_tmp;
      }
      lon = lon_int_tmp-lon_0_int;
      lon += (lon_dec_tmp-lon_0_dec);
      NED[1] = lon*R_TERRE/180*PI;
      
      //if(print_data){ Serial.print(NED[1],20);Serial.print(" ");}
    }

////////////////////////////////////////////////////////
    // altitude

    if(idx>=54 && idx<= 59)
    {
      if(x!=44)
      {
        inString += (char)x;
      }
    }
    if(idx==59)
    {
      alt_tmp = inString.toFloat();
      if(init_done==0)
      {
        alt_0 = alt_tmp;
      }
      alt = alt_tmp-alt_0;
      inString = "";
      NED[2] = -alt;

      //if(print_data){ Serial.print(NED[2],20);Serial.print(" ");}
    }

////////////////////////////////////////////////////////
  // transmission

  if(idx==60)
  {
    
    if(transmit_raw){ mySerial.write(PACKET_START); }
    for (i = 0; i < 3; i++)
    {
      buffer_float = NED[i];
      for (j = 0; j < 4; j++)
      {
        if (transmit_raw) { mySerial.write(ptr_buffer[j]); }
      }
    }
    if(transmit_raw){ mySerial.write(PACKET_STOP); }
  }
    
    idx++;
    
    if(x==0x24)
    {
      //if(print_data){ Serial.println(" ");}
      if(print_data){ Serial.print(NED[0],20);Serial.print(" ");
    Serial.print(NED[1],20);Serial.print(" ");
    Serial.print(NED[2],20);Serial.println(" ");}
      if(millis()-timer>60000*INIT_TIME_MN)
      {
        init_done = 1; 
      }      
      
      idx = 0;
    }
//    Serial.write(x);
//    Serial.print(" ");
  }
}


