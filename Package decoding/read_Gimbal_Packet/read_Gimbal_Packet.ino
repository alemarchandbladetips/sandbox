
#define GIMBAL_PACKET_SIZE 23 // number of bytes to be recieved from 
#define PACKET_START 0xAA // starting char of package
#define PACKET_STOP 0x55 // starting char of package

// constant used to enable/disable communication, debug, timing checks
const int8_t transmit_raw = 0;
const int8_t print_data = 1;

const float pi = 3.14159265359;

// variables for the serial read an data recomposition
float all_data[8];
float rpy[3];
float rpy_p[3];
uint8_t pozyx[2];
uint8_t pozyx_p[2];
float pozyx_pp[3];
uint8_t raw_data[100];
float buffer_float;
unsigned char *ptr_buffer = (unsigned char *)&buffer_float;
int16_t buffer_int16;
unsigned char *ptr_buffer_int16 = (unsigned char *)&buffer_int16;

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

  if (Serial.available() > GIMBAL_PACKET_SIZE) // Number of data corresponding to the IMU packet size is waiting in the biffer of serial
  { 
    x = Serial.read(); // read first data
    //Serial.print(x);
    if(x == PACKET_START) // check that first data correspond to start char
    {
      Serial.println(micros()-t0);
      t0 = micros();
      Serial.readBytes(raw_data,GIMBAL_PACKET_SIZE); // Reading the IMU packet
      //Serial.print(raw_data[GIMBAL_PACKET_SIZE-1]);
      if(raw_data[GIMBAL_PACKET_SIZE-1] == PACKET_STOP) // check taht the last data correspond to the packet end char
      {
        for(i=0;i<9;i++) // decode YPR data
        {
          for(j=0;j<2;j++) // filling the 4 bytes of the float with the data of serial port
          {
            ptr_buffer_int16[j] = raw_data[2*i+j];
          }
          if(i<3)
          { // roll, pitch, tips0 angle
            rpy[i] = (float)buffer_int16*180.0/32768.0;
            //if(print_data){ Serial.print(rpy[i]); Serial.print(" "); }
          } else if(i<6)
          { // derivatives of roll, pitch, tips0 angle
            rpy_p[i-3] = (float)buffer_int16*2000.0/32768.0;
            //if(print_data){ Serial.print(rpy_p[i-3]); Serial.print(" "); }
          } else
          { // accelerations in ENU frame
            pozyx_pp[i-6] = (float)buffer_int16*40.0/32768.0;
            //if(print_data){ Serial.print(pozyx_pp[i-6]*10); Serial.print(" "); }
          }
        }
        // posyx positions and speeds
        pozyx[0] = raw_data[18];
        //if(print_data){ Serial.print(pozyx[0]); Serial.print(" "); }
        pozyx[1] = raw_data[19];
        //if(print_data){ Serial.print(pozyx[1]); Serial.print(" "); }
        pozyx_p[0] = raw_data[20];
        //if(print_data){ Serial.print(pozyx_p[0]-100); Serial.print(" "); }
        pozyx_p[1] = raw_data[21];
        //if(print_data){ Serial.print(pozyx_p[1]); Serial.print(" "); }
        
        if(print_data){ Serial.println(" ");}
      }
    }
  }
  
}


