// Control of a brushless motor ensuring the yaw of IMU BNO055 sticks to 0, using interuption at constant rate

// It transmits data via Tx in the following form:
// 0xAA, Yaw (Float, 4bytes), Pitch (Float, 4bytes), Roll (Float, 4bytes), 
// Omegax (Float, 4bytes), Omegay (Float, 4bytes), Omegaz (Float, 4bytes), 0x55
// = 26 bytes

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "math.h"

#define IUM_PACKET_SIZE 45 // number of bytes to be recieved from IMU not counting starting char acc 3*4 + gyr 3*4 + quat 4*4 + accuracy 1*4 + stop char 1
#define PACKET_START 0xAA // starting char of package
#define PACKET_STOP 0x55 // starting char of package
#define U_MAX 0.5 // max speed command of the motor
#define SIN_APMLITUDE_MAX 125 // max amplitude of sinus (around a 127 offset)
#define SIN_APMLITUDE_MIN 60 // max amplitude of sinus (around a 127 offset)

// Choose between transmitting data to next level or printing for debug
const int8_t transmit_raw = 0;
const int8_t print_data = 1;
const int8_t print_timing = 0;

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
float sin_amplitude = 0.3;

// Commande variable
volatile float angle_step_rd, current_angle_rd_prev, current_angle_rd = 0;
float motor_speed_rps = 0; // desired speed of the motor, rps
float u=0;            // Command, tr/s
float u_integral = 0; // integral part of the command, tr/s
float u_proportionel = 0; // proportional part of the command, tr/s
float speed_step;
uint8_t sinAngleA, sinAngleB, sinAngleC; // the 3 sinusoide values for the PWMs
const float yaw_ref = 90;
const float Ki = 0.05;//0.1; // integral gain
const float Kp = 0.02; // proportional gain
const float IMU_freq = 100; // IMU frequency
const float reg_freq = 1000; // regulation frequency

// Pin definition for connection to ESC
const int EN1 = 12;   // pin enable bls
const int IN1 = 9;    //pins des pwm
const int IN2 = 10;   //pins des pwm
const int IN3 = 6;   //pins des pwm

const int led_pin = 13;
int led_status = 0;
int led_counter = 0;

// interuptions gestion
uint32_t time_counter, time_counter2=0; // counting the number of interuptions
uint8_t interupt_happened; // interuption flag

// variables for the serial read an data recomposition
float ypr_data[3];
uint8_t raw_data[48]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
float buffer_float;
unsigned char *ptr_buffer = (unsigned char *)&buffer_float;
uint32_t buffer_uint32;
unsigned char *ptr_buffer_uint32 = (unsigned char *)&buffer_uint32;

// for BNO
Adafruit_BNO055 bno = Adafruit_BNO055();
uint8_t sys, gyr, accel, mag = 0;
float omega[3], quaternion[4], rpy[3], proper_acc[3];
float motor_angle_offset = 0;
uint32_t accuracy_flags;
uint8_t imu_init = 0;

// For acc integration
float linear_accel_vector[3], pos_ENU[3], posp_ENU[3], pospp_ENU[3];

long t0,t1,t2,t3,t4;

const int pwmSin[] = {127, 138, 149, 160, 170, 181, 191, 200, 209, 217, 224, 231, 237, 242, 246, 250, 252, 254, 254, 254, 252, 250, 246, 242, 237, 231, 224, 217, 209, 200, 191, 181, 170, 160, 149, 138, 127, 116, 105, 94, 84, 73, 64, 54, 45, 37, 30, 23, 17, 12, 8, 4, 2, 0, 0, 0, 2, 4, 8, 12, 17, 23, 30, 37, 45, 54, 64, 73, 84, 94, 105, 116 };
int sineArraySize;
 
void setup() {
  Serial.begin(115200);

  if(!bno.begin(Adafruit_BNO055::OPERATION_MODE_NDOF))
  {
    // There was a problem detecting the BNO055 ... check your connections 
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

// augmentation fréquence PWM

  setPwmFrequency(IN1); 
  setPwmFrequency(IN2);
  setPwmFrequency(IN3);

// PWM output pin configuration
  pinMode(IN1, OUTPUT); 
  pinMode(IN2, OUTPUT); 
  pinMode(IN3, OUTPUT);
  
  pinMode(led_pin, OUTPUT); 
  digitalWrite(led_pin, HIGH);

// pin Enable
  pinMode(EN1, OUTPUT); 
  digitalWrite(EN1, HIGH);
  led_status = 1;

  sineArraySize = sizeof(pwmSin)/sizeof(int); 

// Pre-computation of variables for 
  slice_angle_rd = 2*pi/(nb_pole/2.0);
  angle_scale_factor = sineArraySize/slice_angle_rd;
  Serial.println(slice_angle_rd*360/two_pi);
  current_angle_rd_prev = 5000;
  speed_step = 0.1;
  u = 0;

  
  cli(); // Désactive l'interruption globale
  bitClear (TCCR2A, WGM20); // WGM20 = 0
  bitClear (TCCR2A, WGM21); // WGM21 = 0 
  TCCR2B = 0b00000101; // Clock / 128 soit 8 micro-s et WGM22 = 0
  TIMSK2 = 0b00000001; // Interruption locale autorisée par TOIE2
  sei(); // Active l'interruption globale

  while(gyr!=3)
  {
    bno.getCalibration(&sys, &gyr, &accel, &mag);
  }
  t0 = micros();

  Serial.println("gyr calibrated");
}
//////////////////////////////////////////////////////////////////////////////

void setMotorAngle(float angle_rd)
{
  if(current_angle_rd!=current_angle_rd_prev)
  {
    // Computes normalized angle 0->2pi for one slice
    normalized_angle = (int16_t)(fmod(angle_rd,slice_angle_rd)*angle_scale_factor);
  
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

ISR(TIMER2_OVF_vect) 
{
  TCNT2 = 256 - 125; // 125*8us = 1ms
  interupt_happened = 1; // interuption flag to trigger computation in main loop
  current_angle_rd = fmod((current_angle_rd + angle_step_rd)+two_pi,two_pi); // increment the desired angle
  //setMotorAngle(current_angle_rd);
  time_counter++; // counter in ms (replace the micros())
  time_counter2++;
}
//////////////////////////////////////////////////////////////////////

void quat2rpy(float q[4], float rpy[3]) // 500us<t<750us
{
  rpy[0] = atan2(2*q[1]*q[3] + 2*q[2]*q[0], 1 - 2*q[1]*q[1] - 2*q[2]*q[2]);
  rpy[1] = asin(2*q[2]*q[3] - 2*q[1]*q[0]);
  rpy[2] = atan2(2*q[1]*q[2] + 2*q[3]*q[0], 1 - 2*q[1]*q[1] - 2*q[3]*q[3]);
}

//////////////////////////////////////////////////////////////////////

void vector_quat_proj(float q[4], float v_in[3], float v_out[3])
{
  v_out[0] = (1 - 2*q[2]*q[2] - 2*q[3]*q[3]) * v_in[0] + (2*q[1]*q[2] - 2*q[3]*q[0]) * v_in[1]     + (2*q[1]*q[3] + 2*q[2]*q[0]) * v_in[2];
  v_out[1] = (2*q[1]*q[2] + 2*q[3]*q[0]) * v_in[0]     + (1 - 2*q[1]*q[1] - 2*q[3]*q[3]) * v_in[1] + (2*q[2]*q[3] - 2*q[1]*q[0]) * v_in[2];
  v_out[2] = (2*q[1]*q[3] - 2*q[2]*q[0]) * v_in[0]     + (2*q[2]*q[3] + 2*q[1]*q[0]) * v_in[1]     + (1 - 2*q[1]*q[1] - 2*q[2]*q[2]) * v_in[2];
}

//////////////////////////////////////////////////////////////////////

float mod180(float angle)
{
  return fmod(angle+3780,360)-180;
}
// fmod: 12us<t<40us

//////////////////////////////////////////////////////////////////////





 
void loop() {
int i,j,x;

// Applying the command if new
  
  setMotorAngle(current_angle_rd);               //  une fois par ms // Question : combien de temps elle prend ? 500us
    

  if (Serial.available() > IUM_PACKET_SIZE)
  { 
/////////////////////// Lecture des données du port série venant de l'arduino "du BNO du haut"
    if(print_timing) { t0 = micros(); }
    //Serial.println(Serial.available()); Serial.print(" ");
    x = Serial.read();
    //Serial.print(x); Serial.print(" ");
    if(x == PACKET_START)
    {
      Serial.readBytes(raw_data,IUM_PACKET_SIZE);
      setMotorAngle(current_angle_rd); 
/////////////////////// Décodage des données et transformation du quaternion en RPY
      
      if(print_timing) { t1 = micros();}
      //Serial.print(raw_data[IUM_PACKET_SIZE-1]); Serial.print(" ");
      if(raw_data[IUM_PACKET_SIZE-1] == PACKET_STOP)
      {
        led_counter ++;
        if(led_status == 1 && led_counter>=5)
        {
          digitalWrite(led_pin, LOW);
          led_status = 0;
          led_counter = 0;
        } else if(led_counter>=5)
        {
          digitalWrite(led_pin, HIGH);
          led_status = 1;
          led_counter = 0;
        }
        for(i=0;i<3;i++) // decode sensor data, proper acc
        {
          for(j=0;j<4;j++)
          {
            //if(transmit_raw){ Serial.write(raw_data[4*i+j+12]); }
            ptr_buffer[j] = raw_data[4*i+j];
          }
          proper_acc[i] = buffer_float;
          //if(print_data){ Serial.print(buffer_float); Serial.print(" "); }
        }
        
        for(i=0;i<3;i++) // decode sensor data, gyr
        {
          for(j=0;j<4;j++)
          {
            //if(transmit_raw){ Serial.write(raw_data[4*i+j+12]); }
            ptr_buffer[j] = raw_data[4*i+j+12];
          }
          omega[i] = buffer_float;
          //if(print_data){ Serial.print(buffer_float); Serial.print(" "); }
        }
        
        for(i=0;i<4;i++) // decode QUAT data
        {
          for(j=0;j<4;j++)
          {
            //if(transmit_raw){ Serial.write(raw_data[4*i+j]); }
            ptr_buffer[j] = raw_data[4*i+j+24];
          }
          quaternion[i] = buffer_float; 
          //if(print_data){ Serial.print(buffer_float); Serial.print(" "); }
        }
        
        for(j=0;j<4;j++)
        {
          //if(transmit_raw){ Serial.write(raw_data[4*i+j+12]); }
          ptr_buffer_uint32[j] = raw_data[j+40];
        }
        accuracy_flags = buffer_uint32;
        //if(print_data){ Serial.print((accuracy_flags & 0x03)); Serial.print(" "); }
        
        quat2rpy(quaternion,rpy);

        setMotorAngle(current_angle_rd); 
///////////////////////////////////// Lecture du BNO du bas
        if(print_timing) { t2 = micros(); }

        // gyro data 
        imu::Vector<3> gyro=bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
        omega[2] = gyro.z();

        if(print_timing) { t3 = micros();}
        setMotorAngle(current_angle_rd); 
///////////////////////////////////// Calcul de la commande
  
  // Computing command
        u_integral += Ki*mod180(rpy[2]* rad_to_deg-yaw_ref)/IMU_freq;
        u_integral = constrain(u_integral,-U_MAX,U_MAX); // constrain 8<t<20us
        u_proportionel = Kp * mod180(rpy[2]* rad_to_deg-yaw_ref);
        u_proportionel = constrain(u_proportionel,-U_MAX,U_MAX);
        
        u = u_proportionel + u_integral;//u_proportionel;// + u_integral + omega[2]/two_pi;
        u = constrain(u,-U_MAX,U_MAX) + omega[2]/two_pi;
        //u=0.05;
        sin_amplitude = constrain(0.3+0.012*abs(u),0,1);
        motor_speed_rps = two_pi*u;//tps*two_pi;
        angle_step_rd = motor_speed_rps/reg_freq;

        if(print_timing) { t4 = micros(); }
        if(print_timing) { Serial.print(t1-t0);Serial.print(" ");Serial.print(t2-t1);Serial.print(" ");Serial.print(t3-t2);Serial.print(" ");Serial.print(t4-t3);Serial.print(" ");Serial.print(t4-t0);Serial.println(" "); }
      }
    }
  }
}

// sert à changer la fréquence du pwm 

void setPwmFrequency(int pin) {
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | 0x01;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | 0x01;
    }
  }
  else if(pin == 3 || pin == 11) {
    TCCR2B = TCCR2B & 0b11111000 | 0x01;
  }
}

