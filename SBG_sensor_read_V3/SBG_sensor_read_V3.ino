// Read and treat IMU package before retransmission.
// Taget : Arduino UNO on the top of the "Big pyramide" for the V2 version (led connected on pin 12 and 13)
// Treatments: reduction of header and footer, Conversion to fxp

// Output Package 40 bytes
// 0xAA | int16 SBG_gyr[3]*1000/32768 dps | int16 SBG_quaternion[4]/32768 | int8 SBG_accuracy | 0x55

#include "MemoryFree.h"

//#define RAD_TO_DEG 57.2957795 // 180/pi

const int8_t transmit_raw = 1;
const int8_t print_data = 1;


uint8_t raw_data[100], start_char, footer[3], data_availability;
uint8_t device_status;
uint16_t datalen, checkSum, checkSumCalc;

float buffer_float, quaternion[4], omega[3];
unsigned char *ptr_buffer = (unsigned char *)&buffer_float;
uint32_t buffer_uint32;
unsigned char *ptr_buffer_uint32 = (unsigned char *)&buffer_uint32;
int16_t buffer_int16;
unsigned char *ptr_buffer_int16 = (unsigned char *)&buffer_int16;

// CheckSum function: provided by SBG
uint16_t calcCRC(const void *pBuffer, uint16_t bufferSize)
{
  const uint8_t *pBytesArray =  (const uint8_t*)pBuffer;
  uint16_t poly = 0x8408;
  uint16_t crc = 0;
  uint8_t carry;
  uint8_t i_bits;
  uint16_t j;
  for (j =0; j < bufferSize; j++)
  {
    crc = crc ^ pBytesArray[j];
    for (i_bits = 0; i_bits < 8; i_bits++)
    {
      carry = crc & 1;
      crc = crc / 2;
      if (carry)
      {
        crc = crc^poly;
      } 
    }
  }
  return crc; 
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  
  Serial.println(" ");
  Serial.println("%%%%%");
  Serial.println("Start");
  Serial.println("%%%%%");
  Serial.println(" ");
  
  data_availability = 0;
}

void loop() {
  int i,j;

  //Serial.println(Serial1.available());
  if(Serial1.available()>55)
  {
    
    start_char = Serial1.read();
    Serial.println(start_char);
    if(start_char == 0xFF) // first character is OK, we can start reading the rest of package
    {    
      Serial1.readBytes(raw_data,4);

      datalen = raw_data[3] + (raw_data[2]<<8); // Number of data to read
      
      Serial.print(raw_data[0]); Serial.print(" ");Serial.print(raw_data[1]); Serial.print(" ");Serial.print(raw_data[2]); Serial.print(" ");Serial.print(raw_data[3]); Serial.print(" ");Serial.print(datalen); Serial.print(" ");
      if(raw_data[0] == 0x02 && datalen == 32) // second char is OK too
      {

        if(print_data){ Serial.print(datalen); Serial.print(" "); }
        
        Serial1.readBytes(raw_data+4,datalen); // read the data with the length specified in the header
        Serial1.readBytes(footer,3); // read the footer

        checkSum = footer[1] + ((uint16_t)footer[0]<<8);
        checkSumCalc = calcCRC(raw_data+1,datalen+3);

        if(footer[2]==0x03 && checkSum==checkSumCalc)// && raw_data[52] == 0xFF && (raw_data[53] & 0x01) == 0x01) 
        {// The end char is OK and the computed checksum correspond to the one in the footer.
          //analogWrite(redLedPin, 0);
          data_availability = 1;

//////////// decoding input serial data

          // Read the quaternion
          for(i=0;i<4;i++) // decode YPR data
          {
            for(j=0;j<4;j++)
            {
              ptr_buffer[j] = raw_data[4*i+j+4];
            }
            quaternion[i] = buffer_float; // 180/pi
            if(print_data){ Serial.print(quaternion[i]); Serial.print(" "); }
          }

          // read gyroscope data
          for(i=0;i<3;i++) // decode sensor data, acc, gyr, mag
          {
            for(j=0;j<4;j++)
            {
              ptr_buffer[j] = raw_data[4*i+j+20];
            }
            omega[i] = buffer_float*RAD_TO_DEG;
            if(print_data){ Serial.print(omega[i]); Serial.print(" "); }
          }

          device_status = raw_data[33];
          
          if(print_data){ Serial.print(device_status); Serial.print(" "); }
          if((device_status & 0x67) == 0x67)
          {
            if(print_data){ Serial.print(2); Serial.print(" "); }
          } else if ((device_status & 0x67) == 0x27)
          {
            if(print_data){ Serial.print(1); Serial.print(" "); }
          } else
          {
            if(print_data){ Serial.print(0); Serial.print(" "); }
          }
          
//////////// Transmitting data to next layer

          
          // header of the package
          if(transmit_raw){ Serial1.write(0xAA);}
          
          // Transmition of gyro data
          for(i=0;i<3;i++)
          {
            buffer_int16 = (int16_t)(omega[i]*32768/1000);
            for(j=0;j<2;j++)
            {
              if(transmit_raw){ Serial1.write(ptr_buffer_int16[j]); }
            }
          }
      
          // transmition of quaternion data
          for(i=0;i<4;i++)
          {
            buffer_int16 = (int16_t)(quaternion[i]*32768);
            for(j=0;j<2;j++)
            {
              if(transmit_raw){ Serial1.write(ptr_buffer_int16[j]); }
            }
          }

          // device status
          if(transmit_raw){ Serial1.write(device_status);}

          // footer of the package
          if(transmit_raw){ Serial1.write(0x55); }

          if(print_data){ Serial.println(" "); }
          
        }
      }
    }
  } 
}
