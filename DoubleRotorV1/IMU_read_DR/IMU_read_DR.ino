// Gimbal control for the double rotor prototype V1
// Target board : arduino pro-micro.

// 1 - Reception and treatment of the packet from the SBG
// 2 - Transmission of data to the control part of the drone

// Input Package 44 bytes
// SBG: quaternion, gyro and acc + sanity flags

// Output Package 15 bytes -> teensy
// 0xAA |int16 Roll*32768/180 deg | int16 Pitch*32768/180 deg | int16 Yaw*32768/180 deg | 
// int16 OmegaX*32768/2000 dps) | int16 OmegaY*32768/2000 dps | int16 OmegaZ*32768/2000 dps | int8_t sanity_flag | 0x55

#define GIMBAL_PACKET_SIZE 33 // number of bytes to be recieved from 
#define PACKET_START 0xAA // starting char of package
#define PACKET_STOP 0x55 // starting char of package

// Definitions for SBG IMU
#define PACKET_START_IMU 0xFF                   // starting char of package
#define PACKET_START_IMU2 0x02                  // starting char of package
#define PACKET_STOP_IMU 0x03                    // starting char of package
#define PACKET_SIZE_IMU 44                      // size to be recieved
#define DEVICE_STATUS_MASK_IMU 0x67             // mask to read interesting values of device status
#define DEVICE_STATUS_MASK_IMU_HEADING_NOK 0x27 // mask to read interesting values of device status

// constant used to enable/disable communication, debug, timing checks
const int8_t transmit_raw = 1;
const int8_t print_data = 0;

// definition of some constants to ease computations
const float rad_to_deg = 57.2957795; // 180/pi

///////////// SBG IMU data ////////////////////////
float rpy[3], acc[3];
uint8_t sanity_flag;

uint8_t raw_data[100], start_char, footer[3], data_availability;
uint8_t device_status,accuracy_flags;
uint16_t datalen, checkSum, checkSumCalc;

float buffer_float, quaternion[4], omega[3], offset[3];
unsigned char *ptr_buffer = (unsigned char *)&buffer_float;
int16_t buffer_int16;
unsigned char *ptr_buffer_int16 = (unsigned char *)&buffer_int16;
int32_t buffer_int32;
unsigned char *ptr_buffer_int32 = (unsigned char *)&buffer_int32;

int8_t init_;

long t0,t1,t2,t3;

void setup() {
  // serial port opening
  Serial.begin(115200);
  Serial1.begin(115200);
}

//////////////////////////////////////////////////////////////////////
// Main program


void loop() {
int i,j,x;

////////////////////////////////////////////////////////////////////////
//////////// Availability and sanity of serial data ////////////////////

  if (Serial1.available() > PACKET_SIZE_IMU + 8 - 1) // 8 is the header(5)+footer(3) size
  { // data available
//    time5 = millis();
    start_char = Serial1.read(); //reading first char
    //Serial.print("IMU "); Serial.print(" ");
    //Serial.print(start_char); Serial.println(" ");
    
    if (start_char == PACKET_START_IMU) // first character is OK, we can start reading the rest of package
    {
//      time2 = micros();
//      dt_tmp = (time2 - time1);
//      time1 = time2;
      //if(print_data){ Serial.println(" "); Serial.print(start_char); Serial.println(" "); }

      Serial1.readBytes(raw_data, 4); //reading rest of the header

      datalen = raw_data[3] + (raw_data[2] << 8); // Number of data to read
      //if(print_data){ Serial.print(datalen); Serial.print(" "); }

      //Serial.print(raw_data[0]); Serial.print(" ");Serial.print(raw_data[1]); Serial.println(" ");
      
      if (raw_data[0] == PACKET_START_IMU2 && datalen == PACKET_SIZE_IMU) // second char and data length are OK too
      {

        Serial1.readBytes(raw_data + 4, datalen); // read the data with the length specified in the header
        
        Serial1.readBytes(footer, 3); // read the footer

        // reading checksum in packet footer
        checkSum = footer[1] + ((uint16_t)footer[0] << 8);
        // computing checksum from recieved data
        checkSumCalc = calcCRC(raw_data + 1, datalen + 3);

        if (footer[2] == PACKET_STOP_IMU && checkSum == checkSumCalc) // && raw_data[52] == 0xFF && (raw_data[53] & 0x01) == 0x01)
        { // The end char is OK and the computed checksum correspond to the one in the footer.

          //digitalWrite(synchPin, HIGH);
          
          data_availability = 1;

////////////////////////////////////////////////////////////
//////////// decoding SBG input serial data ////////////////////      
          
          // Read the quaternion
          for (i = 0; i < 4; i++)
          {
            for (j = 0; j < 4; j++)
            {
              ptr_buffer[j] = raw_data[4 * i + j + 4];
            }
            quaternion[i] = buffer_float; // 180/pi
            //if(print_data){ Serial.print(quaternion[i]); Serial.print(" "); }
          }
          // transformation of quaternion to rpy
          quat2rpy_ellipse_east(quaternion,rpy);
          rpy[1] = -rpy[1];
          rpy[2] = -rpy[2];

          // read gyroscope data
          for (i = 0; i < 3; i++) 
          {
            for (j = 0; j < 4; j++)
            {
              ptr_buffer[j] = raw_data[4 * i + j + 20];
            }
            omega[i] = buffer_float * RAD_TO_DEG;
            //if(print_data){ Serial.print(omega[i]); Serial.print(" "); }
          }
          omega[1] = -omega[1];
          omega[2] = -omega[2];

          // accelerometer data
          for (i = 0; i < 3; i++) 
          {
            for (j = 0; j < 4; j++)
            {
              ptr_buffer[j] = raw_data[4 * i + j + 32];
            }
            acc[i] = buffer_float;
            //if(print_data){ Serial.print(acc[i]); Serial.print(" "); }
          }

          // Status flags
          accuracy_flags = raw_data[45] & DEVICE_STATUS_MASK_IMU;
          
          sanity_flag = (((accuracy_flags & 0x26) == 0x26) << 1 )
                    + (((accuracy_flags & 0x66) == 0x66) << 2 );


////////////////////////////////////////////////////////////
//////////// transmision to main board ////////////////////

          // starting byte
          if(transmit_raw){ Serial1.write(PACKET_START); } 
          if(print_data){ Serial.print(PACKET_START); Serial.print(" "); }
          
          // Roll and pitch yaw
          for(i=0;i<3;i++)
          {
            buffer_int16 = (int16_t)(mod180(rpy[i]*rad_to_deg)*32768.0/180.0);
            if(print_data){ Serial.print(mod180(rpy[i]*rad_to_deg)); Serial.print(" "); }
            for(j=0;j<2;j++)
            {
              if(transmit_raw){ Serial1.write(ptr_buffer_int16[j]); }
            }
          }

          // rotation speeds (roll and pitch derivatives)
          for(i=0;i<3;i++)
          {
            buffer_int16 = (int16_t)(omega[i]*32768.0/2000.0);
            if(print_data){ Serial.print(omega[i]); Serial.print(" "); }
            for(j=0;j<2;j++)
            {
              if(transmit_raw){ Serial1.write(ptr_buffer_int16[j]); }
            }
          }
  
          // sanity flag
          if(transmit_raw){ Serial1.write(sanity_flag); }
          if(print_data){ Serial.print(sanity_flag); Serial.print(" ");}
          
          // packet stop
          if(transmit_raw){ Serial1.write(PACKET_STOP); }
          if(print_data){ Serial.print(PACKET_STOP);}
          if(print_data){ Serial.println(" ");}
          
        } // end footer checksum
      } // end datalength
    } //end start char
  } // end available 
} // end loop


//////////////////////////////////////////////////////////////////////

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

//////////////////////////////////////////////////////////////////////

// transformation from quaternion to Roll Pitch Yaw angles
// input: q[4]: quaternion in flt
// output: rpy[3]: Euler angles in flt (rd)
// Measured execution time for a random quaternion : 500us to 750us

void quat2rpy_ellipse_east(float q[4], float rpy[3]) // 
{
  rpy[0] = atan2(2*q[1]*q[3] + 2*q[2]*q[0], 1 - 2*q[1]*q[1] - 2*q[2]*q[2]);
  rpy[1] = asin(2*q[2]*q[3] - 2*q[1]*q[0]);
  rpy[2] = atan2(2*q[1]*q[2] + 2*q[3]*q[0], 1 - 2*q[1]*q[1] - 2*q[3]*q[3]);
}

void quat2rpy_ellipse_north(float q[4], float rpy[3]) // 
{
  rpy[0] = -atan2(2*q[2]*q[3] - 2*q[1]*q[0], 1 - 2*q[1]*q[1] - 2*q[2]*q[2]);
  rpy[1] = asin(2*q[1]*q[3] + 2*q[2]*q[0]);
  rpy[2] = -atan2(2*q[1]*q[2] - 2*q[3]*q[0], 1 - 2*q[2]*q[2] - 2*q[3]*q[3]);
}
//////////////////////////////////////////////////////////////////////

// Set an angle between -180 and 180 deg
// input: angle in deg

float mod180(float angle)
{
  return fmod(angle+3780,360)-180;
}
// fmod: 12us<t<40us

//////////////////////////////////////////////////////////////////////
