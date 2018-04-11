#include "math.h"

#define GIMBAL_PACKET_SIZE 23 // number of bytes to be recieved from 
#define PACKET_START 0xAA // starting char of package
#define PACKET_STOP 0x55 // starting char of package

#define U_MAX 0.5 // max speed command of the motor

////////////////// constant used to enable/disable communication, debug, timing checks ////////////////
const int8_t transmit_raw = 1;
const int8_t print_data = 0;
const int8_t print_timing = 0;

////////////////// definition of some constants to ease computations ////////////////
const float pi = 3.14159265359;
const float two_pi = 6.28318530718;
const float four_pi_on_three = 4.18879020479;
const float two_pi_on_three = 2.09439510239;
const float rad_to_deg = 57.2957795; // 180/pi

////////////////// motor related constants and variable ////////////////
const uint8_t nb_pole = 14;
float slice_angle_rd, angle_scale_factor;
int16_t normalized_angle;
float sin_amplitude = 0.3;
const float reg_freq = 1000; // regulation frequency

////////////////// Commande variable ////////////////
volatile float angle_step_rd, current_angle_rd_prev, current_angle_rd, motor_angle_offset = 0;
float motor_speed_rps = 0; // desired speed of the motor, rps
uint8_t sinAngleA, sinAngleB, sinAngleC; // the 3 sinusoide values for the PWMs
float u=0;            // Command, tr/s
float u_integral = 0; // integral part of the command, tr/s
float u_proportionel = 0; // proportional part of the command, tr/s
float speed_step;
float angle_error_deg;
float yaw;
const float yaw_ref = 90;//180;
const float Ki = 0.015;//0.1; // integral gain
const float Kp = 0.01; // proportional gain
const float IMU_freq = 100; // IMU frequency
uint8_t accuracy_flags,imu_init = 0;

////////////////// Pin definition for connection to ESC ////////////////
const int EN1 = 4;   // pin enable bls
const int IN1 = 10;    //pins des pwm
const int IN2 = 6;   //pins des pwm
const int IN3 = 9;   //pins des pwm

const int pin_dbg = 16;
volatile int pin_dbg_status = 0;

////////////////// interuptions gestion ////////////////
uint32_t time_counter, time_counter3, time_counter2=0; // counting the number of interuptions
uint8_t interupt_happened; // interuption flag

/////////// buffers and ptr for data decoding ///////
float buffer_float;
unsigned char *ptr_buffer = (unsigned char *)&buffer_float;
uint32_t buffer_uint32;
unsigned char *ptr_buffer_uint32 = (unsigned char *)&buffer_uint32;
int32_t buffer_int32;
unsigned char *ptr_buffer_int32 = (unsigned char *)&buffer_int32;
int16_t buffer_int16;
unsigned char *ptr_buffer_int16 = (unsigned char *)&buffer_int16;
uint8_t raw_data[100];

float motor_speed_rps_prev = 0;

long t0,t1,t2,t3,t4,t5,t6,t_;
uint8_t time_;

//const uint8_t pwmSin[] = {128, 132, 136, 140, 143, 147, 151, 155, 159, 162, 166, 170, 174, 178, 181, 185, 189, 192, 196, 200, 203, 207, 211, 214, 218, 221, 225, 228, 232, 235, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 248, 249, 250, 250, 251, 252, 252, 253, 253, 253, 254, 254, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 254, 254, 253, 253, 253, 252, 252, 251, 250, 250, 249, 248, 248, 247, 246, 245, 244, 243, 242, 241, 240, 239, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 248, 249, 250, 250, 251, 252, 252, 253, 253, 253, 254, 254, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 254, 254, 253, 253, 253, 252, 252, 251, 250, 250, 249, 248, 248, 247, 246, 245, 244, 243, 242, 241, 240, 239, 238, 235, 232, 228, 225, 221, 218, 214, 211, 207, 203, 200, 196, 192, 189, 185, 181, 178, 174, 170, 166, 162, 159, 155, 151, 147, 143, 140, 136, 132, 128, 124, 120, 116, 113, 109, 105, 101, 97, 94, 90, 86, 82, 78, 75, 71, 67, 64, 60, 56, 53, 49, 45, 42, 38, 35, 31, 28, 24, 21, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 8, 7, 6, 6, 5, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 5, 6, 6, 7, 8, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 8, 7, 6, 6, 5, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 5, 6, 6, 7, 8, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 21, 24, 28, 31, 35, 38, 42, 45, 49, 53, 56, 60, 64, 67, 71, 75, 78, 82, 86, 90, 94, 97, 101, 105, 109, 113, 116, 120, 124};
const uint8_t pwmSin[] = {127, 127, 127, 128, 128, 128, 130, 130, 130, 132, 132, 132, 133, 133, 133, 135, 135, 135, 137, 137, 137, 139, 139, 139, 140, 140, 140, 142, 142, 142, 144, 144, 144, 146, 146, 146, 147, 147, 147, 149, 149, 149, 151, 151, 151, 152, 152, 152, 154, 154, 154, 156, 156, 156, 157, 157, 157, 159, 159, 159, 161, 161, 161, 162, 162, 162, 164, 164, 164, 166, 166, 166, 167, 167, 167, 169, 169, 169, 170, 170, 170, 172, 172, 172, 173, 173, 173, 175, 175, 175, 177, 177, 177, 178, 178, 178, 179, 179, 179, 181, 181, 181, 182, 182, 182, 184, 184, 184, 185, 185, 185, 187, 187, 187, 188, 188, 188, 189, 189, 189, 191, 191, 191, 192, 192, 192, 193, 193, 193, 195, 195, 195, 196, 196, 196, 197, 197, 197, 198, 198, 198, 200, 200, 200, 201, 201, 201, 202, 202, 202, 203, 203, 203, 204, 204, 204, 205, 205, 205, 206, 206, 206, 207, 207, 207, 208, 208, 208, 209, 209, 209, 210, 210, 210, 211, 211, 211, 212, 212, 212, 213, 213, 213, 214, 214, 214, 215, 215, 215, 216, 216, 216, 216, 216, 216, 217, 217, 217, 218, 218, 218, 219, 219, 219, 219, 219, 219, 220, 220, 220, 220, 220, 220, 221, 221, 221, 222, 222, 222, 222, 222, 222, 223, 223, 223, 223, 223, 223, 224, 224, 224, 224, 224, 224, 224, 224, 224, 225, 225, 225, 225, 225, 225, 225, 225, 225, 226, 226, 226, 226, 226, 226, 226, 226, 226, 226, 226, 226, 226, 226, 226, 226, 226, 226, 226, 226, 226, 226, 226, 226, 227, 227, 227, 226, 226, 226, 226, 226, 226, 226, 226, 226, 226, 226, 226, 226, 226, 226, 226, 226, 226, 226, 226, 226, 226, 226, 226, 225, 225, 225, 225, 225, 225, 225, 225, 225, 224, 224, 224, 224, 224, 224, 224, 224, 224, 223, 223, 223, 223, 223, 223, 222, 222, 222, 222, 222, 222, 221, 221, 221, 220, 220, 220, 220, 220, 220, 219, 219, 219, 219, 219, 219, 218, 218, 218, 217, 217, 217, 216, 216, 216, 216, 216, 216, 215, 215, 215, 214, 214, 214, 213, 213, 213, 212, 212, 212, 211, 211, 211, 210, 210, 210, 209, 209, 209, 208, 208, 208, 207, 207, 207, 206, 206, 206, 205, 205, 205, 204, 204, 204, 203, 203, 203, 202, 202, 202, 201, 201, 201, 200, 200, 200, 198, 198, 198, 197, 197, 197, 196, 196, 196, 195, 195, 195, 193, 193, 193, 192, 192, 192, 191, 191, 191, 189, 189, 189, 188, 188, 188, 187, 187, 187, 185, 185, 185, 184, 184, 184, 182, 182, 182, 181, 181, 181, 179, 179, 179, 178, 178, 178, 176, 176, 176, 175, 175, 175, 173, 173, 173, 172, 172, 172, 170, 170, 170, 169, 169, 169, 167, 167, 167, 166, 166, 166, 164, 164, 164, 162, 162, 162, 161, 161, 161, 159, 159, 159, 157, 157, 157, 156, 156, 156, 154, 154, 154, 152, 152, 152, 151, 151, 151, 149, 149, 149, 147, 147, 147, 146, 146, 146, 144, 144, 144, 142, 142, 142, 140, 140, 140, 139, 139, 139, 137, 137, 137, 135, 135, 135, 133, 133, 133, 132, 132, 132, 130, 130, 130, 128, 128, 128, 126, 126, 126, 125, 125, 125, 123, 123, 123, 121, 121, 121, 120, 120, 120, 118, 118, 118, 116, 116, 116, 114, 114, 114, 113, 113, 113, 111, 111, 111, 109, 109, 109, 107, 107, 107, 106, 106, 106, 104, 104, 104, 102, 102, 102, 101, 101, 101, 99, 99, 99, 97, 97, 97, 96, 96, 96, 94, 94, 94, 92, 92, 92, 91, 91, 91, 89, 89, 89, 87, 87, 87, 86, 86, 86, 84, 84, 84, 83, 83, 83, 81, 81, 81, 80, 80, 80, 78, 78, 78, 76, 76, 76, 75, 75, 75, 74, 74, 74, 72, 72, 72, 71, 71, 71, 69, 69, 69, 68, 68, 68, 66, 66, 66, 65, 65, 65, 64, 64, 64, 62, 62, 62, 61, 61, 61, 60, 60, 60, 58, 58, 58, 57, 57, 57, 56, 56, 56, 55, 55, 55, 53, 53, 53, 52, 52, 52, 51, 51, 51, 50, 50, 50, 49, 49, 49, 48, 48, 48, 47, 47, 47, 46, 46, 46, 45, 45, 45, 44, 44, 44, 43, 43, 43, 42, 42, 42, 41, 41, 41, 40, 40, 40, 39, 39, 39, 38, 38, 38, 37, 37, 37, 37, 37, 37, 36, 36, 36, 35, 35, 35, 34, 34, 34, 34, 34, 34, 33, 33, 33, 33, 33, 33, 32, 32, 32, 31, 31, 31, 31, 31, 31, 30, 30, 30, 30, 30, 30, 29, 29, 29, 29, 29, 29, 29, 29, 29, 28, 28, 28, 28, 28, 28, 28, 28, 28, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 28, 28, 28, 28, 28, 28, 28, 28, 28, 29, 29, 29, 29, 29, 29, 29, 29, 29, 30, 30, 30, 30, 30, 30, 31, 31, 31, 31, 31, 31, 32, 32, 32, 33, 33, 33, 33, 33, 33, 34, 34, 34, 34, 34, 34, 35, 35, 35, 36, 36, 36, 37, 37, 37, 37, 37, 37, 38, 38, 38, 39, 39, 39, 40, 40, 40, 41, 41, 41, 42, 42, 42, 43, 43, 43, 44, 44, 44, 45, 45, 45, 46, 46, 46, 47, 47, 47, 48, 48, 48, 49, 49, 49, 50, 50, 50, 51, 51, 51, 52, 52, 52, 53, 53, 53, 55, 55, 55, 56, 56, 56, 57, 57, 57, 58, 58, 58, 60, 60, 60, 61, 61, 61, 62, 62, 62, 64, 64, 64, 65, 65, 65, 66, 66, 66, 68, 68, 68, 69, 69, 69, 71, 71, 71, 72, 72, 72, 74, 74, 74, 75, 75, 75, 77, 77, 77, 78, 78, 78, 80, 80, 80, 81, 81, 81, 83, 83, 83, 84, 84, 84, 86, 86, 86, 87, 87, 87, 89, 89, 89, 91, 91, 91, 92, 92, 92, 94, 94, 94, 96, 96, 96, 97, 97, 97, 99, 99, 99, 101, 101, 101, 102, 102, 102, 104, 104, 104, 106, 106, 106, 107, 107, 107, 109, 109, 109, 111, 111, 111, 113, 113, 113, 114, 114, 114, 116, 116, 116, 118, 118, 118, 120, 120, 120, 121, 121, 121, 123, 123, 123, 125, 125, 125};
int16_t sineArraySize;
 
void setup() {
  Serial1.begin(115200);
  Serial.begin(115200);

// augmentation fréquence PWM

  setPwmFrequency(IN1); 
  setPwmFrequency(IN2);
  setPwmFrequency(IN3);

// PWM output pin configuration
  pinMode(IN1, OUTPUT); 
  pinMode(IN2, OUTPUT); 
  pinMode(IN3, OUTPUT);
  pinMode(pin_dbg, OUTPUT);

// pin Enable
  pinMode(EN1, OUTPUT); 
  digitalWrite(EN1, HIGH);

  sineArraySize = sizeof(pwmSin)/sizeof(uint8_t); 
  Serial.println(" ");
  Serial.println(sineArraySize);

// Pre-computation of variables for 
  slice_angle_rd = 2*pi/(nb_pole/2.0);
  angle_scale_factor = sineArraySize/slice_angle_rd;
  Serial.println(slice_angle_rd*360/two_pi);
  current_angle_rd_prev = 5000;

  cli(); // Désactive l'interruption globale
  bitClear (TCCR3A, WGM30); // WGM20 = 0
  bitClear (TCCR3A, WGM31); // WGM21 = 0 
  //TCCR3B = 0b00000001; // Clock / 128 soit 8 micro-s et WGM22 = 0
  TIMSK3 = 0b00000001; // Interruption locale autorisée par TOIE3
  //sei(); // Active l'interruption globale

  motor_speed_rps = pi/4;//two_pi;
  sin_amplitude = constrain(0.65+0.1*abs(motor_speed_rps/two_pi),0,0.9); 
  angle_step_rd = motor_speed_rps/reg_freq;

  interupt_happened = 0;
  t0 = micros();
}
//////////////////////////////////////////////////////////////////////////////

// Computes and send the PWM values for a disered angle

void setMotorAngle(float angle_rd)
{
  if(current_angle_rd!=current_angle_rd_prev)
  {
    //digitalWrite(pin_dbg, LOW);
    // Computes normalized angle 0->2pi for one slice
    normalized_angle = (int16_t)(fmod(angle_rd,slice_angle_rd)*angle_scale_factor);
  
    // Computes sin from the normalized angles
    sinAngleA = (uint8_t)(pwmSin[normalized_angle]*sin_amplitude+20);
    sinAngleB = (uint8_t)(pwmSin[(int16_t)fmod(normalized_angle+sineArraySize/3,sineArraySize)]*sin_amplitude+20);
    sinAngleC = (uint8_t)(pwmSin[(int16_t)fmod(normalized_angle+2*sineArraySize/3,sineArraySize)]*sin_amplitude+20);
  
    // Applies sin on PWM outputs
    analogWrite(IN1, sinAngleA);
    analogWrite(IN2, sinAngleB);
    analogWrite(IN3, sinAngleC);
    current_angle_rd_prev = current_angle_rd;
  }
}

/////////////////////////////////////////////////////////////////////
// Routine d'interruption
// Just increment the desired angle

ISR(TIMER3_OVF_vect) 
{
  TCNT3 = 65536 - 250; // 250*4us = 1ms
  // increment the desired angle from the increment computed from the command
  current_angle_rd = fmod((current_angle_rd + angle_step_rd)+two_pi,two_pi); 
  pin_dbg_status = !pin_dbg_status;
  digitalWrite(pin_dbg, pin_dbg_status);
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


void loop() {
int i,j,x,n;

  delay(1);
  current_angle_rd = fmod((current_angle_rd + angle_step_rd)+two_pi,two_pi); 
  setMotorAngle(current_angle_rd); 

// Applying the command if new
  //setMotorAngle(current_angle_rd);               //  une fois par ms // Question : combien de temps elle prend ? 500us
  /*
  if (Serial1.available() > GIMBAL_PACKET_SIZE-1) // Number of data corresponding to the IMU packet size is waiting in the biffer of serial
  { 
    x = Serial1.read(); // read first data
    //Serial.print(x); Serial.print(" ");
    if(x == PACKET_START) // check that first data correspond to start char
    {
      Serial1.readBytes(raw_data,GIMBAL_PACKET_SIZE-1); // Reading the IMU packet
      
      setMotorAngle(current_angle_rd); 
      
      if(raw_data[GIMBAL_PACKET_SIZE-2] == PACKET_STOP) // check taht the last data correspond to the packet end char
      {
        for(j=0;j<2;j++) // filling the 4 bytes of the float with the data of serial port
        {
          ptr_buffer_int16[j] = raw_data[10+j];
        }
        motor_speed_rps = (float)buffer_int16*2000.0/32768.0/rad_to_deg;
        //Serial.print(motor_speed_rps);Serial.write(9);

        for(j=0;j<2;j++) // filling the 4 bytes of the float with the data of serial port
        {
          ptr_buffer_int16[j] = raw_data[4+j];
        }
        yaw = (float)buffer_int16*180/32768.0;
        //Serial.print(yaw);Serial.write(9);

        accuracy_flags = raw_data[20];
        //Serial.print(accuracy_flags);Serial.write(9);
        //Serial.println(" ");
        
///////////////////////////////////////////////////////   
////////////////// Calcul de la commande //////////////   
  
  // Computing command
        u = 0;
        angle_error_deg = mod180(yaw-yaw_ref);

        setMotorAngle(current_angle_rd); 
        
        if((accuracy_flags & 0x2) == 0x2)
        { // if the gyro has saturated, we go in open loop mode, just compensating speed measured by base gyro. (correspond to the "else")

          // integral part of the command
          u_integral += Ki*mod180(angle_error_deg)/IMU_freq;
          u_integral = constrain(u_integral,-U_MAX,U_MAX);

          // proportional part of the command
          u_proportionel = Kp * angle_error_deg;
          u_proportionel = constrain(u_proportionel,-U_MAX,U_MAX);
          
          u = u_proportionel + u_integral; // computing the PI correction
  
          if((accuracy_flags & 0x1) == 0x1 && imu_init==0 )
          { //heading has converge, if it is the first time, we will set the 0 of blade0
            imu_init = 1; // so we will not set a ref again
            motor_angle_offset = fmod(current_angle_rd+pi,two_pi)-pi;
          }
        } else
        {
          u_integral = 0;
        }

        setMotorAngle(current_angle_rd); 
        
        //Serial.print(u_integral*two_pi);Serial.write(9);
        //Serial.print(u_proportionel*two_pi);Serial.write(9);

        //Serial.print(normalized_angle);Serial.write(9);

        Serial.print(-motor_speed_rps+motor_speed_rps_prev);Serial.write(9);
        motor_speed_rps_prev = motor_speed_rps;
        motor_speed_rps  = 2*pi;//+= two_pi*constrain(u,-U_MAX,U_MAX);//tps*two_pi;

        
      
        sin_amplitude = 1;//constrain(0.5+0.1*abs(motor_speed_rps/two_pi),0,0.95); 
        angle_step_rd = motor_speed_rps/reg_freq;

        //Serial.print(sin_amplitude);Serial.write(9);

        //Serial.print(current_angle_rd*two_pi);Serial.write(9);
        //Serial.print(current_angle_rd_prev*two_pi);Serial.write(9);
        //Serial.print(normalized_angle);Serial.write(9);

        Serial.println(" ");
        

        buffer_int16 = (int16_t)(-motor_speed_rps*rad_to_deg*32768/2000);
        for(j=0;j<2;j++)
        {
          raw_data[10+j] = ptr_buffer_int16[j];
        }
        buffer_int16 = (int16_t)(-mod180((current_angle_rd-motor_angle_offset)*rad_to_deg+yaw-yaw_ref)*32768/180);
        for(j=0;j<2;j++) 
        {
          raw_data[4+j] = ptr_buffer_int16[j];
        }

        setMotorAngle(current_angle_rd); 

///////////////////////////////////////////////////////   
////// Transmission des data à la partie commande /////
        if(transmit_raw)
        {
          Serial1.write(PACKET_START);
          for(i=0;i<GIMBAL_PACKET_SIZE-1;i++)
          {
            Serial1.write(raw_data[i]);
            setMotorAngle(current_angle_rd); 
          }
        }
        
      }
    }
  }*/
}

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


