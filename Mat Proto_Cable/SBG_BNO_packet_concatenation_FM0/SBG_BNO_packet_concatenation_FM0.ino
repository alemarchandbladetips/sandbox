
#include <Arduino.h>   // required before wiring_private.h
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "math.h"

#define RAD_TO_DEG 57.2957795 // 180/pi

#define PACKET_START 0xAA // starting char of package
#define PACKET_STOP 0x55  // starting char of package
#define PACKET_SIZE 26    // size to be recieved

// Definitions for SBG IMU
#define PACKET_START_IMU 0xFF                   // starting char of package
#define PACKET_START_IMU2 0x02                  // starting char of package
#define PACKET_STOP_IMU 0x03                    // starting char of package
#define PACKET_SIZE_IMU 44                      // size to be recieved
#define DEVICE_STATUS_MASK_IMU 0x67             // mask to read interesting values of device status
#define DEVICE_STATUS_MASK_IMU_HEADING_NOK 0x27 // mask to read interesting values of device status

const int8_t transmit_raw = 1;
const int8_t print_data = 0;
const int8_t print_timing = 0;

// definition of some constants to ease computations
const float pi = 3.14159265359;
const float two_pi = 6.28318530718;
const float four_pi_on_three = 4.18879020479;
const float two_pi_on_three = 2.09439510239;

/////////// serial communication with IMU //////////
char raw_data[100], start_char, footer[3], data_availability;
uint8_t device_status;
uint16_t datalen, checkSum, checkSumCalc;

//////////// for BNO ////////////////////
Adafruit_BNO055 bno = Adafruit_BNO055();
uint8_t sys, gyr, accel, mag = 0;
uint32_t accuracy_flags;
float alpha_omega = 0.33;
float nominal_speed_dps = 0;
//const float BNO_corrective_gain = 0.95491; // MESURÉ SUR LA PYRAMIDE
//const float BNO_corrective_gain = 0.95087; // MESURÉ SUR le petit proto V1
const float BNO_corrective_gain = 0.95238; // MESURÉ SUR le petit proto V2

///////////// SBG IMU data ////////////////////////
float omega[3], quaternion[4], rpy[3], acc[3];
uint8_t sanity_flag;
int32_t convergence_timmer;
uint8_t imu_init = 0;
float angle_error_deg, yaw_ref = 90;

/////////// buffers and ptr for data decoding ///////
float buffer_float;
unsigned char *ptr_buffer = (unsigned char *)&buffer_float;
uint32_t buffer_uint32;
unsigned char *ptr_buffer_uint32 = (unsigned char *)&buffer_uint32;
int32_t buffer_int32;
unsigned char *ptr_buffer_int32 = (unsigned char *)&buffer_int32;
int16_t buffer_int16;
unsigned char *ptr_buffer_int16 = (unsigned char *)&buffer_int16;

/////////// LED handeling /////////////
int8_t greenLedPin = 14;
int8_t green_led_counter = 0;
int8_t green_led_status = 1;
int8_t green_led_period = 10;

int8_t yellowLedPin = 15;
int8_t yellow_led_counter = 0;
int8_t yellow_led_status = 1;
int8_t yellow_led_period = 100;

int8_t redLedPin = 16;
int8_t red_led_counter = 0;
int8_t red_led_status = 1;
int8_t red_led_period = 50;

//int8_t synchPin = 12;

/////////// timing/debug /////////////
uint32_t time1, time2, time3, time4, dt_tmp, time5, time6;

int32_t dbg = 0;
float integrated_yaw = 0;

//////////////////////////////////////////////////////////////////

void setup() {

  // Starting coms
  Serial1.begin(115200);
  Serial.begin(115200);

  Serial.println("Starting...  ");
  // Turning LED oon
  pinMode(greenLedPin, OUTPUT);
  digitalWrite(greenLedPin, green_led_status);
  pinMode(yellowLedPin,OUTPUT);
  digitalWrite(yellowLedPin, yellow_led_status);
  pinMode(redLedPin, OUTPUT);
  digitalWrite(redLedPin, red_led_status);

  //pinMode(synchPin, OUTPUT);
  //digitalWrite(synchPin, LOW);

  delay(2000);

  Serial.println("waiting BNO Startup");
  
  if(!bno.begin(Adafruit_BNO055::OPERATION_MODE_NDOF))
  {
    // There was a problem detecting the BNO055 ... check your connections 
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    //while(1);
  }

  gyr = 0;
  Serial.println("waiting BNO calibration");
  while(gyr!=3) // Waiting for gyro to be calibrated
  {
    bno.getCalibration(&sys, &gyr, &accel, &mag);
  }
  Serial.println("BNO ready");

  // turning off leds
  green_led_status = 0;
  digitalWrite(greenLedPin, green_led_status);
  yellow_led_status = 0;
  digitalWrite(yellowLedPin, yellow_led_status);
  red_led_status = 0;
  digitalWrite(redLedPin, red_led_status);

  Serial.print("good to go");
  
  integrated_yaw = 0;
  
}

/////////////////////////////////////////////////////////////////////////////

void loop() {
  uint8_t i, j, x;


////////////////////////////////////////////////////////////
////////// reception of IMU data on serial ////////////////

  if (Serial1.available() > PACKET_SIZE_IMU + 8 - 1)
  { // data available
    time5 = millis();
    start_char = Serial1.read(); //reading first char
    //Serial.print("IMU "); Serial.print(" ");
    //Serial.print(start_char); Serial.println(" ");
    
    if (start_char == PACKET_START_IMU) // first character is OK, we can start reading the rest of package
    {
      time2 = micros();
      dt_tmp = (time2 - time1);
      time1 = time2;
      //if(print_data){ Serial.println(" "); Serial.print(dt*1000000-10000); Serial.println(" "); }
      
      //digitalWrite(greenLedPin, HIGH);
      Serial1.readBytes(raw_data, 4); //reading rest of the header

      datalen = raw_data[3] + (raw_data[2] << 8); // Number of data to read
      //if(print_data){ Serial.print(datalen); Serial.print(" "); }

      //Serial.print(raw_data[0]); Serial.print(" ");Serial.print(raw_data[1]); Serial.println(" ");
      if (raw_data[0] == PACKET_START_IMU2 && datalen == PACKET_SIZE_IMU) // second char and data length is OK too
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
          dbg++;

////////////////////////////////////////////////////////////
//////////// decoding input serial data ////////////////////

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

          // Status flags
          device_status = raw_data[45] & DEVICE_STATUS_MASK_IMU;

          if(print_data){ Serial.print(device_status); Serial.write(9); }
          if ((device_status & DEVICE_STATUS_MASK_IMU) == DEVICE_STATUS_MASK_IMU)
          {
            angle_error_deg = mod180(rpy[2]* RAD_TO_DEG-yaw_ref);
            if(abs(angle_error_deg<3) && imu_init==0)
            {
              if( (millis()-convergence_timmer) > 5000)
              {
                imu_init = 1;
                yellow_led_period = 10; // led blinks 5Hz
              }
            } else
            {
              convergence_timmer = millis();
              yellow_led_period = 20;
            }
            red_led_period = -1;
          } else if ((device_status & DEVICE_STATUS_MASK_IMU) == DEVICE_STATUS_MASK_IMU_HEADING_NOK)
          {
            angle_error_deg = mod180(rpy[2]* RAD_TO_DEG-yaw_ref);
            red_led_period = 50;
            convergence_timmer = millis();
          } else
          {
             red_led_period = 0;
             convergence_timmer = millis();
          }

///////////////////////////////////////////////////////         
/////////// Transmitting data to next layer ///////////

        //constructing the bitmasked sanity flag
        sanity_flag = (imu_init == 1)
                    + (((device_status & DEVICE_STATUS_MASK_IMU_HEADING_NOK) == DEVICE_STATUS_MASK_IMU_HEADING_NOK) << 1 )
                    + (((device_status & DEVICE_STATUS_MASK_IMU) == DEVICE_STATUS_MASK_IMU) << 2 );
                    //+ (pozyx_correction_flag << 7); 
        //pozyx_correction_flag = 0;

        if(print_timing) { time3 = micros(); }

        // Data transmition, to the gimbal regulation arduino
 
        if(transmit_raw){ Serial1.write(PACKET_START); } // starting byte
        
        // Roll and pitch and yaw
        for(i=0;i<3;i++)
        {
          buffer_int16 = (int16_t)(mod180(rpy[i]* RAD_TO_DEG)*32768/180);
          if(print_data){ Serial.print(mod180(rpy[i]* RAD_TO_DEG)); Serial.write(9); }
          for(j=0;j<2;j++)
          {
            if(transmit_raw){ Serial1.write(ptr_buffer_int16[j]); }
          }
        }

        // rotation speeds (roll and pitch derivatives)
        for(i=0;i<2;i++)
        {
          buffer_int16 = (int16_t)(omega[i]*32768/2000);
          //if(print_data){ Serial.print(buffer_int16); Serial.write(9); }
          for(j=0;j<2;j++)
          {
            if(transmit_raw){ Serial1.write(ptr_buffer_int16[j]); }
          }
        }

        // blade rotation speed
        buffer_int16 = (int16_t)(nominal_speed_dps*32768/2000);
        //if(print_data){ Serial.print(buffer_int16); Serial.write(9); }
        for(j=0;j<2;j++)
        {
          if(transmit_raw){ Serial1.write(ptr_buffer_int16[j]); }
        }

        // transmition of Pozyx position data (empty)
        for (i = 0; i < 8; i++)
        {
          if (transmit_raw) { Serial1.write(i); }
        }

        if(transmit_raw){ Serial1.write(sanity_flag); }
        //if(print_data){ Serial.print(sanity_flag); Serial.write(9); }
        //if(print_data){ Serial.print(device_status); Serial.write(9); }
        
        if(transmit_raw){ Serial1.write(PACKET_STOP); } // ending byte

        if(print_data){ Serial.println(" "); }

        //digitalWrite(synchPin, LOW);

 /////////////////// Updating LEDs ///////////////////

          // yellow LED
          if(yellow_led_period == 0)
          {
            yellow_led_status = 1;
          } else if(yellow_led_period < 0)
          {
            yellow_led_status = 0;
          } else if(yellow_led_counter>yellow_led_period)
          {
            yellow_led_status = !yellow_led_status;
            yellow_led_counter = 0;
          }
          digitalWrite(yellowLedPin, yellow_led_status);
          yellow_led_counter ++;

          // red LED
          if(red_led_period == 0)
          {
            red_led_status = 1;
          } else if(red_led_period < 0)
          {
            red_led_status = 0;
          } else if(red_led_counter>red_led_period)
          {
            red_led_status = !red_led_status;
            red_led_counter = 0;
          }
          digitalWrite(redLedPin, red_led_status);
          red_led_counter ++;

///////////////////////////////////// Lecture du BNO du bas /////////////

        if(print_timing) { time4 = micros(); }

        // gyro data, only gyro data on z axis will be used
        imu::Vector<3> gyro=bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
        nominal_speed_dps = BNO_corrective_gain*gyro.y()*RAD_TO_DEG;
//        Serial.print(gyro.x()); Serial.write(9);
//        Serial.print(gyro.y()); Serial.write(9);
//        Serial.println(gyro.z()); Serial.write(9);
   
/////////////////////////////////////////////////////////////////////
///////////// Error cases ///////////////////////////////////////////

        } else
        { // header and/or footer does not corraspond to expected syntax
          if (print_data) { Serial.println("Corrupted Data"); }
          data_availability = 0;
          digitalWrite(yellowLedPin, LOW);
          digitalWrite(redLedPin, HIGH);
        }
      }
    }
  } else if ( (micros() - time1) > 15000)
  { // last data is too old, must be a
    if (print_data) {
      //Serial.println("No Data");
    }
    //time1 = micros();
    digitalWrite(yellowLedPin, LOW);
    digitalWrite(redLedPin, HIGH);
  }
  
}

//////////////////////////////////////////////////////////////////////
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

// Projection of a vector using a quaternion
// input: q[4]: quaternion in flt
// input: v_in[3]: vector to be projected in the new base in flt
// output: v_out[3]: v_in rotated thanks to quaternion input

void vector_quat_proj(float q[4], float v_in[3], float v_out[3])
{
  v_out[0] = (1 - 2 * q[2] * q[2] - 2 * q[3] * q[3]) * v_in[0] + (2 * q[1] * q[2] - 2 * q[3] * q[0]) * v_in[1]     + (2 * q[1] * q[3] + 2 * q[2] * q[0]) * v_in[2];
  v_out[1] = (2 * q[1] * q[2] + 2 * q[3] * q[0]) * v_in[0]     + (1 - 2 * q[1] * q[1] - 2 * q[3] * q[3]) * v_in[1] + (2 * q[2] * q[3] - 2 * q[1] * q[0]) * v_in[2];
  v_out[2] = (2 * q[1] * q[3] - 2 * q[2] * q[0]) * v_in[0]     + (2 * q[2] * q[3] + 2 * q[1] * q[0]) * v_in[1]     + (1 - 2 * q[1] * q[1] - 2 * q[2] * q[2]) * v_in[2];
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
  for (j = 0; j < bufferSize; j++)
  {
    crc = crc ^ pBytesArray[j];
    for (i_bits = 0; i_bits < 8; i_bits++)
    {
      carry = crc & 1;
      crc = crc / 2;
      if (carry)
      {
        crc = crc ^ poly;
      }
    }
  }
  return crc;
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
