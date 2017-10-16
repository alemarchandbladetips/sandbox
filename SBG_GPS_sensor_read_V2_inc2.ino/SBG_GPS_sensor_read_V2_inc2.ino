#include <NeoSWSerial.h>

#define RAD_TO_DEG 57.2957795 // 180/pi

#define PACKET_START 0xAA // starting char of package
#define PACKET_STOP 0x55  // starting char of package
#define PACKET_SIZE 26    // size to be recieved

#define PACKET_START_IMU 0xFF                   // starting char of package
#define PACKET_START_IMU2 0x02                  // starting char of package
#define PACKET_STOP_IMU 0x03                    // starting char of package
#define PACKET_SIZE_IMU 44                      // size to be recieved
#define DEVICE_STATUS_MASK_IMU 0x67             // mask to read interesting values of device status
#define DEVICE_STATUS_MASK_IMU_HEADING_NOK 0x27 // mask to read interesting values of device status

#define DELAY_SAMPLE 22         // Delay between IMU and GPS in samples (100Hz = 0.35s)
#define GRAVITY_FIELD_NORM 9.81 // Gravity field norm m/s2

const int8_t transmit_raw = 1;
const int8_t print_data = 0;

/////////// serial communication with IMU //////////
uint8_t raw_data[100], start_char, footer[3], data_availability;
uint8_t device_status;
uint16_t datalen, checkSum, checkSumCalc;

/////////// serial communication with GPS //////////
NeoSWSerial mySerial(9, 2); // RX, TX
uint8_t gps_buffer[26];
uint8_t gps_correction_flag = 0;
int32_t NED_coordinates[3];
int16_t NED_coordinates_accuracy[3];
int16_t NED_speed[3];

/////////// buffers and ptr for data decoding ///////
float buffer_float, quaternion[4], omega[3], acc[3], proper_acc[3];
unsigned char *ptr_buffer = (unsigned char *)&buffer_float;
uint32_t buffer_uint32;
unsigned char *ptr_buffer_uint32 = (unsigned char *)&buffer_uint32;
int32_t buffer_int32;
unsigned char *ptr_buffer_int32 = (unsigned char *)&buffer_int32;
int16_t buffer_int16;
unsigned char *ptr_buffer_int16 = (unsigned char *)&buffer_int16;

/////////// LED handeling /////////////

// V2 connection, real gnd is used
int redLedPin = 13;
int greenLedPin = 12;
/*
// V1 connections, gnd is done using pin 7
int redLedPin = 5;
int greenLedPin = 6;
int gndLedPin = 7;
*/
int red_led_blink = 0;
int red_led_blink_counter = 0;
int red_led_blink_status = 0;

/////////// timing /////////////
uint32_t time1, time2, time3, time4, dt_tmp;

/////////// Kalman filter //////////
float est_speed_cmps[3] = {0,0,0};            // estimation state speed
float est_speed_cmps_buffer[DELAY_SAMPLE*3];  // speed buffer
int16_t est_speed_cmps_buffer_index = 0;      // speed buffer index
float est_position_cm[3] = {0,0,0};           // estimation state position
float est_position_cm_buffer[DELAY_SAMPLE*3]; // position buffer
int16_t est_position_cm_buffer_index = 0;     // position buffer index
float proper_acc_buffer[DELAY_SAMPLE*3];      // proper acc buffer
int16_t proper_acc_buffer_index = 0;          // proper acc buffer index
float K_pos = 0.1;                            // correction gain position
float K_speed = 0.1;                          // correction gain speed
float dt;                                     // actual sampling period

int8_t dbg = 0;

void setup() {
  // Starting coms
  Serial.begin(115200);
  mySerial.begin(38400);

  Serial.println(" ");
  Serial.println("Start");

  // Turning LED off
  /*
  // needed for v1 only
  {
    pinMode(gndLedPin,OUTPUT);
    digitalWrite(gndLedPin, LOW);
  }*/
  
  pinMode(redLedPin, OUTPUT);
  digitalWrite(redLedPin, LOW);
  pinMode(greenLedPin, OUTPUT);
  digitalWrite(greenLedPin, LOW);

  // Initialisation of variables and buffers
  data_availability = 0;
  clear_buffer3(est_speed_cmps_buffer, &est_speed_cmps_buffer_index, DELAY_SAMPLE);
  clear_buffer3(est_position_cm_buffer, &est_position_cm_buffer_index, DELAY_SAMPLE);
  clear_buffer3(proper_acc_buffer, &proper_acc_buffer_index, DELAY_SAMPLE);
}

/////////////////////////////////////////////////////////////////////////////

void loop() {
  uint8_t i, j, x;

//////////////////////////////////////////////////////////////////////
////////// reception of GPS correction on soft serial ////////////////

  if (mySerial.available() > PACKET_SIZE - 1)
  {
    x = mySerial.read();
    if (x == PACKET_START)
    {
      mySerial.readBytes(gps_buffer, PACKET_SIZE - 1);
      if (gps_buffer[PACKET_SIZE - 2] == PACKET_STOP)
      {
        for (i = 0; i < 3; i++)
        {
          for (j = 0; j < 4; j++)
          {
            ptr_buffer_int32[j] = gps_buffer[4 * i + j];
          }
          NED_coordinates[i] = buffer_int32;
          //if(print_data){ Serial.print(NED_coordinates[i]); Serial.print(" ");}
        }

        // NED_coordinates_accuracy
        for (i = 0; i < 3; i++)
        {
          for (j = 0; j < 2; j++)
          {
            ptr_buffer_int16[j] = gps_buffer[2 * i + j + 12];
          }
          NED_coordinates_accuracy[i] = buffer_int16;
          //if(print_data){ Serial.print(NED_coordinates_accuracy[i]); Serial.print(" ");}
        }

        // NED_coordinates_accuracy
        for (i = 0; i < 3; i++)
        {
          for (j = 0; j < 2; j++)
          {
            ptr_buffer_int16[j] = gps_buffer[2 * i + j + 18];
          }
          NED_speed[i] = buffer_int16;
          //if(print_data){ Serial.print(NED_speed[i]); Serial.print(" ");}
        }
        gps_correction_flag = 1;
        time3 = micros();
        //Serial.print(dbg);Serial.println(" ");
        time4 = time3;
        dbg = 0;
        //if(print_data){ Serial.println(" ");}
      }
    }
  }
 
////////////////////////////////////////////////////////////
////////// reception of IMU data on serial ////////////////

  if (Serial.available() > PACKET_SIZE_IMU + 8 - 1)
  { // data available

    start_char = Serial.read();
    //Serial.print(start_char); Serial.print(" ");
    if (start_char == PACKET_START_IMU) // first character is OK, we can start reading the rest of package
    {
      time2 = micros();
      dt_tmp = (time2 - time1);
      dt = (float)dt_tmp/1000000.0;
      time1 = time2;
      
      if(print_data){ Serial.println(" "); Serial.print(dt*1000000-10000); Serial.println(" "); }
      
      digitalWrite(greenLedPin, HIGH);
      Serial.readBytes(raw_data, 4);

      datalen = raw_data[3] + (raw_data[2] << 8); // Number of data to read
      //if(print_data){ Serial.print(datalen); Serial.print(" "); }

      //Serial.print(raw_data[0]); Serial.print(" ");Serial.print(raw_data[1]); Serial.println(" ");
      if (raw_data[0] == PACKET_START_IMU2 && datalen == PACKET_SIZE_IMU) // second char and data length is OK too
      {

        Serial.readBytes(raw_data + 4, datalen); // read the data with the length specified in the header
        Serial.readBytes(footer, 3); // read the footer

        checkSum = footer[1] + ((uint16_t)footer[0] << 8);
        checkSumCalc = calcCRC(raw_data + 1, datalen + 3);

        if (footer[2] == PACKET_STOP_IMU && checkSum == checkSumCalc) // && raw_data[52] == 0xFF && (raw_data[53] & 0x01) == 0x01)
        { // The end char is OK and the computed checksum correspond to the one in the footer.
          //analogWrite(redLedPin, 0);
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

          // read acc data
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
          device_status = raw_data[45] & DEVICE_STATUS_MASK_IMU;

          //if(print_data){ Serial.print(device_status); Serial.print(" "); }
          if ((device_status & DEVICE_STATUS_MASK_IMU) == DEVICE_STATUS_MASK_IMU)
          {
            //if(print_data){ Serial.print(2); Serial.print(" "); }
            digitalWrite(redLedPin, LOW);
            red_led_blink_status = 0;
          } else if ((device_status & DEVICE_STATUS_MASK_IMU) == DEVICE_STATUS_MASK_IMU_HEADING_NOK)
          {
            //if(print_data){ Serial.print(1); Serial.print(" "); }
            red_led_blink_counter++;
            if (red_led_blink_counter >= 50)
            {
              red_led_blink_counter = 0;
              if (red_led_blink_status)
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

          /*
          // projection of acc in world frame and substration of gravity
          vector_quat_proj(quaternion, acc, proper_acc);
          proper_acc[2] += GRAVITY_FIELD_NORM;
          for(i=0;i<3;i++)
          {
            proper_acc[i] = proper_acc[i]; // conversion to cm/s2
          }

          // buffering data
          add_to_buffer3(proper_acc, proper_acc_buffer, &proper_acc_buffer_index, DELAY_SAMPLE);

///////////////////////////////////////////
////////// KF : prediction ////////////////

          for(i=0;i<3;i++)
          { // integration of proper acc and speed
            est_position_cm[i] += est_speed_cmps[i]*dt + (proper_acc[i])*dt*dt;
            est_speed_cmps[i] += (proper_acc[i])*dt;
          }
          // adding new data to buffer
          add_to_buffer3(est_speed_cmps, est_speed_cmps_buffer, &est_speed_cmps_buffer_index, DELAY_SAMPLE);
          add_to_buffer3(est_position_cm, est_position_cm_buffer, &est_position_cm_buffer_index, DELAY_SAMPLE);

////////// KF : correction ////////////////

          if (gps_correction_flag)
          { // gps correction is available

            gps_correction_flag = 0;
            
            // getting oldest data in the buffer
            x = get_in_buffer3(est_speed_cmps_buffer, &est_speed_cmps_buffer_index, DELAY_SAMPLE, 0, est_speed_cmps);
            if(x==-1) delay(10000);
            x = get_in_buffer3(est_position_cm_buffer, &est_position_cm_buffer_index, DELAY_SAMPLE, 0, est_position_cm);
            if(x==-1) delay(10000);
  
            for(i=0;i<3;i++)
            { // correction of data with the GPS measurements
              est_position_cm[i] += K_pos*(NED_coordinates[i]-est_position_cm[i]);
              est_speed_cmps[i] += K_speed*(NED_speed[i]-est_speed_cmps[i]);
            }
  
            // storing in the buffer
            add_to_buffer3(est_speed_cmps, est_speed_cmps_buffer, &est_speed_cmps_buffer_index, DELAY_SAMPLE);
            add_to_buffer3(est_position_cm, est_position_cm_buffer, &est_position_cm_buffer_index, DELAY_SAMPLE);
  
            for (j=1;j<DELAY_SAMPLE;j++)
            { // replaying the prediction of the buffer to propagate correction to present time
              // starting at 1, because element 0 was computed with the correction
              
              // get proper acc data in the buffer
              x = get_in_buffer3(proper_acc_buffer, &proper_acc_buffer_index, DELAY_SAMPLE, j, proper_acc_buffer);
              if(x==-1) delay(10000);
  
              for(i=0;i<3;i++)
              { // replay prediction
                est_position_cm[i] += est_speed_cmps[i]*dt + (proper_acc[i])*dt*dt;
                est_speed_cmps[i] += (proper_acc[i])*dt;
              }
  
              // store in buffer
              add_to_buffer3(est_speed_cmps, est_speed_cmps_buffer, &est_speed_cmps_buffer_index, DELAY_SAMPLE);
              add_to_buffer3(est_position_cm, est_position_cm_buffer, &est_position_cm_buffer_index, DELAY_SAMPLE);
            }
          }
          dbg++;
          // printing data on 1 axis
          if (print_data) {
            //Serial.print(dt*1000);
            //Serial.print(" ");
            //Serial.print(proper_acc[1]*100);
            //Serial.print(" ");
            
            //Serial.print(NED_speed[1]*1000);
            //Serial.print(" ");
            //Serial.print(est_speed_cmps[1]*1000);
            //Serial.print(" ");

            
            //Serial.print(NED_coordinates[1]*1000+2160);
            //Serial.print(" ");
            //Serial.print(est_position_cm[1]*1000);
          }
*/
///////////////////////////////////////////////////////         
/////////// Transmitting data to next layer ///////////
          
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

          //if (print_data) { Serial.println(" "); }
/////////////////////////////////////////////////////////

        } else
        { // header and/or footer does not corraspond to expected syntax
          if (print_data) { Serial.println("Corrupted Data"); }
          if (transmit_raw) {
            Serial.write(0xAA);
            Serial.write(0xAA);
          }
          data_availability = 0;
          digitalWrite(redLedPin, HIGH);
          digitalWrite(greenLedPin, LOW);
        }
      }
    }
  } else if ( (micros() - time1) > 15000)
  { // last data is too old, must be a
    if (print_data) {
      Serial.println("No Data");
    }
    //time1 = micros();
    digitalWrite(greenLedPin, LOW);
    digitalWrite(redLedPin, LOW);
    red_led_blink = 0;
    if (transmit_raw) {
      Serial.write(0x55);
      Serial.write(0x55);
    }
  }
}

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////

// Add a 3 element data vector in the ring buffer
// float input[3]: data vector to be added
// float *buffer_: adress of the data buffer of size 3*buffer_size;
// int16_t *buffer_index: adress of the index of the data buffer, points to the next element to be replaced (oldest element)
// int16_t buffer_size: size of the buffer

void add_to_buffer3(float input[3],float *buffer_, int16_t *buffer_index, int16_t buffer_size)
{
  int16_t i;
  for(i=0;i<3;i++)
  {
    buffer_[3*(*buffer_index)+i] = input[i];
  }
  (*buffer_index)++;
  if(*buffer_index > buffer_size-1)
  {
    *buffer_index = 0;
  }
}

//////////////////////////////////////////////////////////////////////

// get a 3 element data vector in the ring buffer

// float *buffer_: adress of the data buffer of size 3*buffer_size;
// int16_t *buffer_index: adress of the index of the data buffer, points to the next element to be replaced (oldest element)
// int16_t buffer_size: size of the buffer
// int16_t data_index: index of the data to get (0: oldest value, buffer_size-1: last value)
// float output[3]: output vector
// return -1 if data index is out of bounds, 0 otherwise.

int8_t get_in_buffer3(float *buffer_, int16_t *buffer_index, int16_t buffer_size, int16_t data_index, float output[3])
{
  int16_t index_in_buffer, i;

  if(data_index>buffer_size-1 || data_index<0)
  {
    return -1;
  }

  index_in_buffer = data_index+*buffer_index;
  if(index_in_buffer > buffer_size-1)
  {
    index_in_buffer -= buffer_size;
  }
  
  for(i=0;i<3;i++)
  {
    output[i] = buffer_[3*index_in_buffer+i];
  }
  return 0;
}

//////////////////////////////////////////////////////////////////////

// Fill the buffer with 0 values
// float *buffer_: adress of the data buffer of size 3*buffer_size;
// int16_t *buffer_index: adress of the index of the data buffer, points to the next element to be replaced (oldest element), will be 0 at the end of the execution of this function.
// int16_t buffer_size: size of the buffer

void clear_buffer3(float *buffer_, int16_t *buffer_index, int16_t buffer_size)
{
  int16_t i,j;
  for(i=0;i<buffer_size;i++)
  {
    for(j=0;j<3;j++)
    {
      buffer_[3*i+j] = 0;
    }
  }
  *buffer_index = 0;
}

//////////////////////////////////////////////////////////////////////
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

/////////////////////////////////////////////////////////////////////////////
