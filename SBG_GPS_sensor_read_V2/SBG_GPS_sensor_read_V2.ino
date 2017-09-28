#include <SoftwareSerial.h>

#define RAD_TO_DEG 57.2957795 // 180/pi

#define PACKET_START 0xAA // starting char of package
#define PACKET_STOP 0x55 // starting char of package
#define PACKET_SIZE 26 // size to be recieved

#define PACKET_START_IMU 0xFF // starting char of package
#define PACKET_START_IMU2 0x02 // starting char of package
#define PACKET_STOP_IMU 0x03 // starting char of package
#define PACKET_SIZE_IMU 44 // size to be recieved
#define DEVICE_STATUS_MASK_IMU 0x67 // mask to read interesting values of device status
#define DEVICE_STATUS_MASK_IMU_HEADING_NOK 0x27 // mask to read interesting values of device status

float gravity_field_norm = 9.81;

const int8_t transmit_raw =0;
const int8_t print_data = 1;


uint8_t raw_data[100], start_char, footer[3], data_availability;
uint8_t device_status;
uint16_t datalen, checkSum, checkSumCalc;

float buffer_float, quaternion[4], omega[3], acc[3], proper_acc[3];
unsigned char *ptr_buffer = (unsigned char *)&buffer_float;
uint32_t buffer_uint32;
unsigned char *ptr_buffer_uint32 = (unsigned char *)&buffer_uint32;
int32_t buffer_int32;
unsigned char *ptr_buffer_int32 = (unsigned char *)&buffer_int32;
int16_t buffer_int16;
unsigned char *ptr_buffer_int16 = (unsigned char *)&buffer_int16;

uint8_t gps_buffer[26];
float NED_coordinates[3];
float NED_coordinates_accuracy[3];
float NED_speed[3];


int redLedPin = 13;
int greenLedPin = 12;
//int gndLedPin = 7;

int red_led_blink = 0;
int red_led_blink_counter = 0;
int red_led_blink_status = 0;

int32_t time1, time2, time3, dt;

SoftwareSerial mySerial(9, 2); // RX, TX

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

/////////////////////////////////////////////////////////////////////////////

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

/////////////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(115200);
  mySerial.begin(57600);
  
  Serial.println(" ");
  Serial.println("%%%%%");
  Serial.println("Start");
  Serial.println("%%%%%");
  Serial.println(" ");

  //pinMode(gndLedPin,OUTPUT);
  //digitalWrite(gndLedPin, LOW);
  pinMode(redLedPin,OUTPUT);
  digitalWrite(redLedPin, LOW);
  pinMode(greenLedPin,OUTPUT);
  digitalWrite(greenLedPin, LOW);
  Serial.println("Corrupted Data");
  data_availability = 0;
}

/////////////////////////////////////////////////////////////////////////////

void loop() {
  uint8_t i,j,x;
  if(mySerial.available() > PACKET_SIZE-1)
  {
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
          //if(print_data){ Serial.print(NED_coordinates[i]); Serial.print(" ");}
        }
    
        // NED_coordinates_accuracy
        for(i=0;i<3;i++)
        {
          for(j=0;j<2;j++)
          {
            ptr_buffer_int16[j] = gps_buffer[2*i+j+12];
          }
          NED_coordinates_accuracy[i] = (float)buffer_int16/10000.0;
          //if(print_data){ Serial.print(NED_coordinates_accuracy[i]); Serial.print(" ");}
        }

        // NED_coordinates_accuracy
        for(i=0;i<3;i++)
        {
          for(j=0;j<2;j++)
          {
            ptr_buffer_int16[j] = gps_buffer[2*i+j+18];
          }
          NED_speed[i] = (float)buffer_int16/100.0;
          //if(print_data){ Serial.print(NED_speed[i]); Serial.print(" ");}
        }
        //if(print_data){ Serial.println(" ");}
      }
    }
  }

  
  if(Serial.available()>PACKET_SIZE_IMU+8-1)
  {
    
    start_char = Serial.read();
    //Serial.print(start_char); Serial.print(" ");
    if(start_char == PACKET_START_IMU) // first character is OK, we can start reading the rest of package
    {     
      time2 = micros();
      dt = time2 - time1;
      time1 = time2;
      //Serial.print(dt); Serial.print(" "); 

      digitalWrite(greenLedPin, HIGH);
      Serial.readBytes(raw_data,4);

      datalen = raw_data[3] + (raw_data[2]<<8); // Number of data to read
      //if(print_data){ Serial.print(datalen); Serial.println(" "); }
      
      //Serial.print(raw_data[0]); Serial.print(" ");Serial.print(raw_data[1]); Serial.print(" ");Serial.print(raw_data[2]); Serial.print(" ");Serial.print(raw_data[3]); Serial.print(" ");
      if(raw_data[0] == PACKET_START_IMU2 && datalen == PACKET_SIZE_IMU) // second char is OK too
      {

        Serial.readBytes(raw_data+4,datalen); // read the data with the length specified in the header
        Serial.readBytes(footer,3); // read the footer

        checkSum = footer[1] + ((uint16_t)footer[0]<<8);
        checkSumCalc = calcCRC(raw_data+1,datalen+3);

        if(footer[2]==PACKET_STOP_IMU && checkSum==checkSumCalc)// && raw_data[52] == 0xFF && (raw_data[53] & 0x01) == 0x01) 
        {// The end char is OK and the computed checksum correspond to the one in the footer.
          //analogWrite(redLedPin, 0);
          data_availability = 1;

//////////// decoding input serial data

          // Read the quaternion
          for(i=0;i<4;i++) 
          {
            for(j=0;j<4;j++)
            {
              ptr_buffer[j] = raw_data[4*i+j+4];
            }
            quaternion[i] = buffer_float; // 180/pi
            //if(print_data){ Serial.print(quaternion[i]); Serial.print(" "); }
          }

          // read gyroscope data
          for(i=0;i<3;i++) // decode sensor data, acc, gyr, mag
          {
            for(j=0;j<4;j++)
            {
              ptr_buffer[j] = raw_data[4*i+j+32];
            }
            acc[i] = buffer_float;
            //if(print_data){ Serial.print(acc[i]); Serial.print(" "); }
          }

          // read gyroscope data
          for(i=0;i<3;i++) // decode sensor data, acc, gyr, mag
          {
            for(j=0;j<4;j++)
            {
              ptr_buffer[j] = raw_data[4*i+j+20];
            }
            omega[i] = buffer_float*RAD_TO_DEG;
            //if(print_data){ Serial.print(omega[i]); Serial.print(" "); }
          }

          device_status = raw_data[45];
          
          //if(print_data){ Serial.print(device_status); Serial.print(" "); }
          if((device_status & DEVICE_STATUS_MASK_IMU) == DEVICE_STATUS_MASK_IMU)
          {
            //if(print_data){ Serial.print(2); Serial.print(" "); }
            digitalWrite(redLedPin, LOW);
            red_led_blink_status = 0;
          } else if ((device_status & DEVICE_STATUS_MASK_IMU) == DEVICE_STATUS_MASK_IMU_HEADING_NOK)
          {
            //if(print_data){ Serial.print(1); Serial.print(" "); }
            red_led_blink_counter++;
            if(red_led_blink_counter>=50)
            {
              red_led_blink_counter = 0;
              if(red_led_blink_status)
              {
                digitalWrite(redLedPin, LOW);
                red_led_blink_status = 0;
              } else
              {
                digitalWrite(redLedPin, HIGH);
                red_led_blink_status = 1;
              }
            }
          } else
          {
            //if(print_data){ Serial.print(0); Serial.print(" "); }
            digitalWrite(redLedPin, HIGH);
            red_led_blink_status = 1;
          }

          vector_quat_proj(quaternion,acc,proper_acc);
          proper_acc[2] += gravity_field_norm;
          for(i=0;i<3;i++) // decode sensor data, acc, gyr, mag
          {
            if(print_data){ Serial.print(proper_acc[i]); Serial.print(" "); }
          }
          for(i=0;i<3;i++) // decode sensor data, acc, gyr, mag
          {
            if(print_data){ Serial.print(NED_coordinates[i]); Serial.print(" "); }
          }
          for(i=0;i<3;i++) // decode sensor data, acc, gyr, mag
          {
            if(print_data){ Serial.print(NED_speed[i]); Serial.print(" "); }
          }
//////////// Transmitting data to next layer

          
          // header of the package
          if(transmit_raw){ Serial.write(0xAA);}
          
          // Transmition of gyro data
          for(i=0;i<3;i++)
          {
            buffer_int16 = (int16_t)(omega[i]*32768/1000);
            for(j=0;j<2;j++)
            {
              if(transmit_raw){ Serial.write(ptr_buffer_int16[j]); }
            }
          }
      
          // transmition of quaternion data
          for(i=0;i<4;i++)
          {
            buffer_int16 = (int16_t)(quaternion[i]*32768);
            for(j=0;j<2;j++)
            {
              if(transmit_raw){ Serial.write(ptr_buffer_int16[j]); }
            }
          }

          // device status
          if(transmit_raw){ Serial.write(device_status);}

          // footer of the package
          if(transmit_raw){ Serial.write(0x55); }

          if(print_data){ Serial.println(" "); }
          
        } else
        {
          if(print_data){ Serial.println("Corrupted Data"); }
          if(transmit_raw){ Serial.write(0xAA);Serial.write(0xAA); }
          data_availability = 0;
          digitalWrite(redLedPin, HIGH);
          digitalWrite(greenLedPin, LOW);
        }
      }
    }
  } else if( (micros() - time1) >15000)
  {
    if(print_data){ Serial.println("No Data"); }
    digitalWrite(greenLedPin, LOW);
    digitalWrite(redLedPin, LOW);
    red_led_blink = 0;
    if(transmit_raw){ Serial.write(0x55);Serial.write(0x55); }
  }
}
