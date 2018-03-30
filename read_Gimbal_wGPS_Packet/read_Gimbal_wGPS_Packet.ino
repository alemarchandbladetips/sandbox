
#define GIMBAL_PACKET_SIZE 33 // number of bytes to be recieved from 
#define PACKET_START 0xAA // starting char of package
#define PACKET_STOP 0x55 // starting char of package

// constant used to choose printed data
const int8_t print_rpy = 1;
const int8_t print_rpy_p = 1;
const int8_t print_NED = 0;
const int8_t print_NED_speed = 0;
const int8_t print_flag = 0;

const float pi = 3.14159265359;

// variables for the serial read an data recomposition
float all_data[8];
float rpy[3];
float rpy_p[3];
int32_t NED_coordinates[3];
float NED_coordinates_flt[3];
int16_t NED_speed[3];
uint8_t sanity_flag;

uint8_t raw_data[100];
float buffer_float;
unsigned char *ptr_buffer = (unsigned char *)&buffer_float;
int16_t buffer_int16;
unsigned char *ptr_buffer_int16 = (unsigned char *)&buffer_int16;
int32_t buffer_int32;
unsigned char *ptr_buffer_int32 = (unsigned char *)&buffer_int32;

const float rad_to_deg = 57.2957795; // 180/pi
long t0;
 
void setup() {
  Serial.begin(115200);
  Serial.println("Start");
  t0 = micros();
}

//////////////////////////////////////////////////////////////////////
 
void loop() {
int i,j,x;

  if (Serial.available() > GIMBAL_PACKET_SIZE-1) // Number of data corresponding to the IMU packet size is waiting in the biffer of serial
  { 
    x = Serial.read(); // read first data
    //Serial.print(x); Serial.print(" ");
    if(x == PACKET_START) // check that first data correspond to start char
    {
      //Serial.print((int32_t)(micros()-t0));Serial.write(9);
      t0 = micros();
      Serial.readBytes(raw_data,GIMBAL_PACKET_SIZE-1); // Reading the IMU packet
      //Serial.print(raw_data[GIMBAL_PACKET_SIZE-1]);
      if(raw_data[GIMBAL_PACKET_SIZE-2] == PACKET_STOP) // check taht the last data correspond to the packet end char
      {
        for(i=0;i<6;i++) // decode YPR data
        {
          for(j=0;j<2;j++) // filling the 4 bytes of the float with the data of serial port
          {
            ptr_buffer_int16[j] = raw_data[2*i+j];
          }
          if(i<3)
          { // roll, pitch, tips0 angle
            rpy[i] = (float)buffer_int16*180.0/32768.0;
            if(print_rpy){ Serial.print(rpy[i]); Serial.write(9); }
          } else
          { // derivatives of roll, pitch, tips0 angle
            rpy_p[i-3] = (float)buffer_int16*2000.0/32768.0;
            if(i<5)
            {
            if(print_rpy_p){ Serial.print(rpy_p[i-3]); Serial.write(9);}
            }
          }
        }
         /// GPS DATA
        for (i = 0; i < 3; i++)
        {
          for (j = 0; j < 4; j++)
          {
            ptr_buffer_int32[j] = raw_data[4*i+j+12];
          }
          NED_coordinates[i] = buffer_int32;
          NED_coordinates_flt[i] = (float)NED_coordinates[i]/100.0;
          if(print_NED){ Serial.print(NED_coordinates_flt[i]); Serial.write(9);}
        }
        for (i = 0; i < 3; i++)
        {
          for (j = 0; j < 2; j++)
          {
            ptr_buffer_int16[j] = raw_data[2*i+j+24];
          }
          NED_speed[i] = buffer_int16;
          if(print_NED_speed){ Serial.print(NED_speed[i]); Serial.write(9);}
        }

        sanity_flag = raw_data[30];
        if(print_flag){ Serial.print(sanity_flag); Serial.write(9);}
        Serial.println(" ");
      }
    }
  }
}


