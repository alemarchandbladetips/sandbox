// Gimbal control for the gaz prototype V2
// Target board : arduino pro-micro.

// 1 - Reception and treatment of the packet from the SBG
// 2 - Computation of the PI command to the Gimbal motor (speed)
// 3 - Set ref of Yaw (set to 0 once a certain accuracy level is reached)
// 4 - Transmission of data to the control part of the drone

// The increment of the targeted angle from the speed target is done in the 1kHz interruption.
// The setMotorAngle function is call all along the execution of the program, if the target angle has changed the new target will be applied if not nothing is done.

// Input Package 44 bytes
// SBG: quaternion, gyro and acc

// Output Package 15 bytes
// 0xAA |int16 Roll*32768/180 deg | int16 Pitch*32768/180 deg | int16 Yaw*32768/180 deg | 
// int16 OmegaX*32768/2000 dps) | int16 OmegaY*32768/2000 dps | int16 OmegaZ*32768/2000 dps | int8_t sanity_flag | 0x55

// Max speed 4,3 tours/s



#define U_MAX 3 // max speed command of the motor
#define U_PROP_MAX 1 // max speed command of the motor
#define SIN_APMLITUDE_MAX 125 // max amplitude of sinus (around a 127 offset)
#define SIN_APMLITUDE_MIN 60 // max amplitude of sinus (around a 127 offset)
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
// constant used to choose printed data
const int8_t print_rpy = 1;
const int8_t print_rpy_p = 1;

// definition of some constants to ease computations
const float pi = 3.14159265359;
const float two_pi = 6.28318530718;
const float four_pi_on_three = 4.18879020479;
const float two_pi_on_three = 2.09439510239;
const float rad_to_deg = 57.2957795; // 180/pi

// motor related constants and variables
const uint8_t nb_pole = 14;
float slice_angle_rd, angle_scale_factor;
int16_t normalized_angle;
float sin_amplitude = 1;

// Commande variable
volatile float angle_step_rd, current_angle_rd_prev, current_angle_rd = 0, angle_error_deg;
float motor_speed_rps = 0; // desired speed of the motor, rps
float u=0;            // Command, tr/s
float u_integral = 0; // integral part of the command, tr/s
float u_proportionel = 0; // proportional part of the command, tr/s
uint8_t sinAngleA, sinAngleB, sinAngleC; // the 3 sinusoide values for the PWMs
const float yaw_ref = 90;//180;
const float Ki = 0.03;//0.1; // integral gain
const float Kp = 0.005; // proportional gain
const float IMU_freq = 100; // IMU frequency
const float reg_freq = 1000; // regulation frequency
float nominal_speed_rps = 0;
float motor_angle_offset = 0;


// Pin definition for connection to ESC
const int EN1 = 4;   // pin enable bls
const int IN1 = 6;    //pins des pwm
const int IN2 = 9;   //pins des pwm
const int IN3 = 10;   //pins des pwm

// interuptions gestion
uint32_t time_counter, time_counter3, time_counter2=0;; // counting the number of interuptions
uint8_t interupt_happened; // interuption flag

// variables for the serial read an data recomposition
// variables for the serial read an data recomposition
float yaw;
float yaw_p;

///////////// SBG IMU data ////////////////////////
float rpy[3], acc[3];
uint8_t sanity_flag;
int32_t convergence_timmer;
uint8_t imu_init = 0;

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

// Others
float speed_step = 0.25;

long t0,t1,t2,t3;

//const int pwmSin[] = {127, 138, 149, 160, 170, 181, 191, 200, 209, 217, 224, 231, 237, 242, 246, 250, 252, 254, 254, 254, 252, 250, 246, 242, 237, 231, 224, 217, 209, 200, 191, 181, 170, 160, 149, 138, 127, 116, 105, 94, 84, 73, 64, 54, 45, 37, 30, 23, 17, 12, 8, 4, 2, 0, 0, 0, 2, 4, 8, 12, 17, 23, 30, 37, 45, 54, 64, 73, 84, 94, 105, 116 };
const uint8_t pwmSin[] = {128, 132, 136, 140, 143, 147, 151, 155, 159, 162, 166, 170, 174, 178, 181, 185, 189, 192, 196, 200, 203, 207, 211, 214, 218, 221, 225, 228, 232, 235, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 248, 249, 250, 250, 251, 252, 252, 253, 253, 253, 254, 254, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 254, 254, 253, 253, 253, 252, 252, 251, 250, 250, 249, 248, 248, 247, 246, 245, 244, 243, 242, 241, 240, 239, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 248, 249, 250, 250, 251, 252, 252, 253, 253, 253, 254, 254, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 254, 254, 253, 253, 253, 252, 252, 251, 250, 250, 249, 248, 248, 247, 246, 245, 244, 243, 242, 241, 240, 239, 238, 235, 232, 228, 225, 221, 218, 214, 211, 207, 203, 200, 196, 192, 189, 185, 181, 178, 174, 170, 166, 162, 159, 155, 151, 147, 143, 140, 136, 132, 128, 124, 120, 116, 113, 109, 105, 101, 97, 94, 90, 86, 82, 78, 75, 71, 67, 64, 60, 56, 53, 49, 45, 42, 38, 35, 31, 28, 24, 21, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 8, 7, 6, 6, 5, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 5, 6, 6, 7, 8, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 8, 7, 6, 6, 5, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 5, 6, 6, 7, 8, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 21, 24, 28, 31, 35, 38, 42, 45, 49, 53, 56, 60, 64, 67, 71, 75, 78, 82, 86, 90, 94, 97, 101, 105, 109, 113, 116, 120, 124};
int sineArraySize;
 
void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  
// augmentation fréquence PWM
  setPwmFrequency(IN1); 
  setPwmFrequency(IN2);
  setPwmFrequency(IN3);

// PWM output pin configuration
  pinMode(IN1, OUTPUT); 
  pinMode(IN2, OUTPUT); 
  pinMode(IN3, OUTPUT); 

// pin Enable
  pinMode(EN1, OUTPUT); 
  digitalWrite(EN1, HIGH);

// Pre-computation of variables for 
  sineArraySize = sizeof(pwmSin)/sizeof(uint8_t); 
  slice_angle_rd = 2*pi/(nb_pole/2.0);
  angle_scale_factor = sineArraySize/slice_angle_rd;
  Serial.println(slice_angle_rd*360/two_pi);
  current_angle_rd_prev = 5000;
  speed_step = 0.1;
  u = 0;

  u = 0.0;
  sin_amplitude = 125;
  init_ = 0;
  
  cli(); // Désactive l'interruption globale
  //bitClear (TCCR3A, WGM30); // WGM20 = 0
  //bitClear (TCCR3A, WGM31); // WGM21 = 0 
  TCCR3B = 0b00000100; // Clock / 128 soit 8 micro-s et WGM22 = 0
  TCCR3A = 0b00000100; // Clock / 128 soit 8 micro-s et WGM22 = 0
  TIMSK3 = 0b00000001; // Interruption locale autorisée par TOIE3
  sei(); // Active l'interruption globale

  t0 = micros();
  motor_angle_offset = 0;
}
//////////////////////////////////////////////////////////////////////////////


void setMotorAngle(float angle_rd)
{
  if(current_angle_rd!=current_angle_rd_prev)
  {
    // Computes normalized angle 0->2pi for one slice
    normalized_angle = (int16_t)(fmod(angle_rd+10*slice_angle_rd,slice_angle_rd)*angle_scale_factor);
  
    // Computes sin from the normalized angles
    sinAngleA = (uint8_t)(pwmSin[normalized_angle]*sin_amplitude);
    sinAngleB = (uint8_t)(pwmSin[(int16_t)fmod(normalized_angle+sineArraySize/3,sineArraySize)]*sin_amplitude);
    sinAngleC = (uint8_t)(pwmSin[(int16_t)fmod(normalized_angle+2*sineArraySize/3,sineArraySize)]*sin_amplitude);
  
    // Applies sin on PWM outputs
    analogWrite(IN1, sinAngleA);
    analogWrite(IN2, sinAngleB);
    analogWrite(IN3, sinAngleC);
    current_angle_rd_prev = current_angle_rd;
  }
}

/////////////////////////////////////////////////////////////////////
// Routine d'interruption
ISR(TIMER3_OVF_vect) 
{
  TCNT3 = 65536-62; // 125*8us = 1ms
  interupt_happened = 1; // interuption flag to trigger computation in main loop
  current_angle_rd = fmod((current_angle_rd + angle_step_rd),two_pi); // increment the desired angle
  time_counter++; // counter in ms (replace the micros())
  time_counter2++;
}


//////////////////////////////////////////////////////////////////////
 
void loop() {
int i,j,x;

// Applying the command if new
  
    //Serial.println(current_angle_rd);
    setMotorAngle(current_angle_rd);
    //Serial.println(Serial1.available() );


////////////////////////////////////////////////////////////
//////////// Availability and sanity of serial data ////////////////////

  if (Serial1.available() > PACKET_SIZE_IMU + 8 - 1)
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
      //if(print_data){ Serial.println(" "); Serial.print(dt*1000000-10000); Serial.println(" "); }
      
      //digitalWrite(greenLedPin, HIGH);
      Serial1.readBytes(raw_data, 4); //reading rest of the header

      datalen = raw_data[3] + (raw_data[2] << 8); // Number of data to read
      //if(print_data){ Serial.print(datalen); Serial.print(" "); }

      setMotorAngle(current_angle_rd);

      //Serial.print(raw_data[0]); Serial.print(" ");Serial.print(raw_data[1]); Serial.println(" ");
      if (raw_data[0] == PACKET_START_IMU2 && datalen == PACKET_SIZE_IMU) // second char and data length is OK too
      {

        Serial1.readBytes(raw_data + 4, datalen); // read the data with the length specified in the header

        setMotorAngle(current_angle_rd);
        
        Serial1.readBytes(footer, 3); // read the footer

        setMotorAngle(current_angle_rd);

        // reading checksum in packet footer
        checkSum = footer[1] + ((uint16_t)footer[0] << 8);
        // computing checksum from recieved data
        checkSumCalc = calcCRC(raw_data + 1, datalen + 3);

        if (footer[2] == PACKET_STOP_IMU && checkSum == checkSumCalc) // && raw_data[52] == 0xFF && (raw_data[53] & 0x01) == 0x01)
        { // The end char is OK and the computed checksum correspond to the one in the footer.

          //digitalWrite(synchPin, HIGH);
          
          data_availability = 1;
//          dbg++;

////////////////////////////////////////////////////////////
//////////// decoding SBG input serial data ////////////////////      
          setMotorAngle(current_angle_rd);
          
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
          
          sanity_flag = (imu_init == 1) 
                    + (((accuracy_flags & 0x26) == 0x26) << 1 )
                    + (((accuracy_flags & 0x66) == 0x66) << 2 );

////////////////////////////////////////////////////////////
//////////// Computing command ////////////////////
          setMotorAngle(current_angle_rd);
          
          u = 0;
          angle_error_deg = mod180(rpy[2]* rad_to_deg-yaw_ref);

          if((sanity_flag & 0x2) == 0x2 || imu_init==1)
          { 
            // integral part of the command
            u_integral += Ki*mod180(angle_error_deg)/IMU_freq;
            u_integral = constrain(u_integral,-U_MAX,U_MAX);
  
            // proportional part of the command
            u_proportionel = Kp * angle_error_deg;
            u_proportionel = constrain(u_proportionel,-U_PROP_MAX,U_PROP_MAX);
            
            u = constrain(u_proportionel + u_integral,-U_MAX,U_MAX); // computing the PI correction

            
    
            if(abs(angle_error_deg<3) && imu_init==0 && ((sanity_flag & 0x6) == 0x6))
            { //heading has converge, if it is the first time, we will set the 0 of blade0
              if(time_counter2 >5000)
              { // waiting for 5s immobility to take reference
                imu_init = 1; // so we will not set a ref again
                motor_angle_offset = fmod(current_angle_rd+pi,two_pi)-pi;
              }
            } else
            {
              time_counter2 = 0;
            }
          } else
          {
            u_integral = 0;
          }

  //u = 0.5;

          sin_amplitude = constrain(0.55+0.15*abs(u),0,1);
          motor_speed_rps = two_pi*u; // conversion from tr/s to rps
          angle_step_rd = motor_speed_rps/1000;

////////////////////////////////////////////////////////////
//////////// dtransmision to teensy ////////////////////
          setMotorAngle(current_angle_rd);


          if(transmit_raw){ Serial1.write(PACKET_START); } // starting byte
          //if(print_data){ Serial.print(PACKET_START); Serial.print(" "); }
          
          // Roll and pitch
           // Roll and pitch
          for(i=0;i<2;i++)
          {
            buffer_int16 = (int16_t)(mod180(rpy[i]* rad_to_deg)*32768/180);
            //if(print_data){ Serial.print(mod180(rpy[i]* rad_to_deg)); Serial.print(" "); }
            for(j=0;j<2;j++)
            {
              if(transmit_raw){ Serial1.write(ptr_buffer_int16[j]); }
            }
          }
          if(print_data){ Serial.print(angle_error_deg); Serial.print(" "); }

          // blade0 angle
          buffer_int16 = (int16_t)(-mod180((current_angle_rd-motor_angle_offset+rpy[2])*rad_to_deg-yaw_ref)*32768/180); // we put the data in the int16 buffer
          if(print_data){ Serial.print(-mod180((current_angle_rd-motor_angle_offset+rpy[2])*rad_to_deg-yaw_ref)); Serial.print(" "); }
          for(j=0;j<2;j++)
          {
            if(transmit_raw){ Serial1.write(ptr_buffer_int16[j]); } // we transmit each bytes of the int16 buffer using this pointer
          }

                  // rotation speeds (roll and pitch derivatives)
          for(i=0;i<2;i++)
          {
            buffer_int16 = (int16_t)(omega[i]*32768/2000);
            //if(print_data){ Serial.print(omega[i]); Serial.print(" "); }
            for(j=0;j<2;j++)
            {
              if(transmit_raw){ Serial1.write(ptr_buffer_int16[j]); }
            }
          }
  
          // blade rotation speed
          buffer_int16 = (int16_t)((-motor_speed_rps*rad_to_deg)*32768/2000);
          if(print_data){ Serial.print(-motor_speed_rps*rad_to_deg); Serial.print(" "); }
          for(j=0;j<2;j++)
          {
            if(transmit_raw){ Serial1.write(ptr_buffer_int16[j]); }
          }
  

          if(transmit_raw){ Serial1.write(sanity_flag); }
          if(print_data){ Serial.print(sanity_flag); Serial.print(" ");}
          
        
          if(transmit_raw){ Serial1.write(PACKET_STOP); }
          //if(print_data){ Serial.print(PACKET_STOP);}
if(print_data){ Serial.println(" ");}
          
        }
      }
    }
  }


  
 
  
}

// sert à changer la fréquence du pwm 
// sert à changer la fréquence du pwm 
void setPwmFrequency(int pin) {
  if(pin == 3) {
      TCCR0B = TCCR0B & 0b11111000 | 0x01;
    }
  if(pin == 5) {
    TCCR3B = TCCR3B & 0b11111000 | 0x01;
    }
  if(pin == 6) {
    TCCR4B = TCCR4B & 0b11111000 | 0x01;
   }
    if(pin == 9) {
    TCCR1B = TCCR1B & 0b11111000 | 0x01;
   }
   if(pin == 10) {
    TCCR1A = TCCR1A & 0b11111000 | 0x01;
   }
}

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
