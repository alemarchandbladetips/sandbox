// Control of a brushless motor ensuring the yaw of IMU BNO055 sticks to 0, using interuption at constant rate

// It transmits data via Tx in the following form:
// 0xAA, Yaw (Float, 4bytes), Pitch (Float, 4bytes), Roll (Float, 4bytes), 
// Omegax (Float, 4bytes), Omegay (Float, 4bytes), Omegaz (Float, 4bytes), 0x55
// = 26 bytes

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "math.h"

#define U_MAX 3.5 // max speed command of the motor
#define SIN_APMLITUDE_MAX 125 // max amplitude of sinus (around a 127 offset)
#define SIN_APMLITUDE_MIN 60 // max amplitude of sinus (around a 127 offset)

// Choose between transmitting data to next level or printing for debug
const int8_t transmit_raw = 0;
const int8_t print_data = 1;

// definition of some constants to ease computations
const float pi = 3.14159265359;
const float two_pi = 6.28318530718;
const float four_pi_on_three = 4.18879020479;
const float two_pi_on_three = 2.09439510239;
const float rad_to_deg = 57.2957795; // 180/pi

// motor related constants and variables
const uint8_t nb_pole = 14;
float slice_angle_rd, angle_scale_factor;
float normalized_angle;
float sin_amplitude = 50;

// Commande variable
volatile float angle_step_rd, current_angle_rd_prev, current_angle_rd = 0;
float motor_speed_rps = 0; // desired speed of the motor, rps
float u=0;            // Command, tr/s
float u_integral = 0; // integral part of the command, tr/s
float u_proportionel = 0; // proportional part of the command, tr/s
float speed_step;
uint8_t sinAngleA, sinAngleB, sinAngleC; // the 3 sinusoide values for the PWMs
const float Ki = 0.1; // integral gain
const float Kp = 0.05; // proportional gain
const float IMU_freq = 100; // IMU frequency
const float reg_freq = 1000; // regulation frequency

// Pin definition for connection to ESC
const int EN1 = 12;   // pin enable bls
const int IN1 = 9;    //pins des pwm
const int IN2 = 10;   //pins des pwm
const int IN3 = 6;   //pins des pwm

// interuptions gestion
uint32_t time_counter, time_counter2=0; // counting the number of interuptions
uint8_t interupt_happened; // interuption flag

// variables for the serial read an data recomposition
float ypr_data[3];
uint8_t raw_data[48];
float buffer_float;
unsigned char *ptr_buffer = (unsigned char *)&buffer_float;

// for BNO
const float yaw_ref = 90;
float euler_angle[3], omega[3], quaternion[4], rpy[3];
float motor_angle_offset = 0;
Adafruit_BNO055 bno = Adafruit_BNO055();
uint8_t sys, gyr, accel, mag = 0;
uint8_t imu_init = 0;

// For acc integration
float linear_accel_vector[3], pos_ENU[3], posp_ENU[3], pospp_ENU[3];
 
void setup() {
  Serial.begin(115200);

  if(!bno.begin(Adafruit_BNO055::OPERATION_MODE_NDOF))
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  bno.setMode(Adafruit_BNO055::OPERATION_MODE_AMG);
  
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
  slice_angle_rd = 2*pi/(nb_pole/2.0);
  angle_scale_factor = 2*pi/slice_angle_rd;
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
}
//////////////////////////////////////////////////////////////////////////////

void setMotorAngle(float angle_rd)
{
  // Computes normalized angle 0->2pi for one slice
  normalized_angle = fmod(angle_rd,slice_angle_rd)*angle_scale_factor;

  // Computes sin from the normalized angles
  sinAngleA = round(sin(normalized_angle)*sin_amplitude+127.0);
  sinAngleB = round(sin(normalized_angle+two_pi_on_three)*sin_amplitude+127.0);
  sinAngleC = round(sin(normalized_angle+four_pi_on_three)*sin_amplitude+127.0);

  // Applies sin on PWM outputs
  analogWrite(IN1, sinAngleA);
  analogWrite(IN2, sinAngleB);
  analogWrite(IN3, sinAngleC);

  while(gyr!=3)
  {
    bno.getCalibration(&sys, &gyr, &accel, &mag);
  }
}

/////////////////////////////////////////////////////////////////////
// Routine d'interruption
ISR(TIMER2_OVF_vect) 
{
  TCNT2 = 256 - 125; // 125*8us = 1ms
  interupt_happened = 1; // interuption flag to trigger computation in main loop
  current_angle_rd = fmod((current_angle_rd + angle_step_rd),two_pi); // increment the desired angle
  time_counter++; // counter in ms (replace the micros())
  time_counter2++;
}
//////////////////////////////////////////////////////////////////////

void quat2rpy(float q[4], float rpy[3])
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


//////////////////////////////////////////////////////////////////////
 
void loop() {
int i,j,x; 

// Applying the command if new
  if(current_angle_rd!=current_angle_rd_prev)
  {
    setMotorAngle(current_angle_rd);
    current_angle_rd_prev = current_angle_rd;
  }

  if(time_counter>9)
  {
    time_counter = 0;

    // reading BNO data
    // linear accel
    imu::Vector<3> acc=bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    linear_accel_vector[0] = acc.x();
    linear_accel_vector[1] = acc.y();
    linear_accel_vector[2] = acc.z();

    // gyro data
    imu::Vector<3> gyro=bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    omega[0] = gyro.x();
    omega[1] = gyro.y();
    omega[2] = gyro.z();

    // quaternion
    imu::Quaternion quat = bno.getQuat();
    quaternion[0] = quat.w();
    quaternion[1] = quat.x();
    quaternion[2] = quat.y();
    quaternion[3] = quat.z();

    // data projections/transformation
    vector_quat_proj(quaternion,linear_accel_vector,pospp_ENU);
    for(i=0;i<3;i++)
    {
      if(print_data){ Serial.print(pospp_ENU[i]); Serial.print(" "); }
    }
    quat2rpy(quaternion,rpy);
    

// Computing command
bno.getCalibration(&sys, &gyr, &accel, &mag);
    
    if(imu_init == 1 || mag ==3)
    {
      u_integral += Ki*mod180(rpy[2]* rad_to_deg-yaw_ref)/IMU_freq;
      u_integral = constrain(u_integral,-U_MAX,U_MAX);
      u_proportionel = Kp * mod180(rpy[2]* rad_to_deg-yaw_ref);
      u_proportionel = constrain(u_proportionel,-U_MAX-u_integral,U_MAX-u_integral);
      u = u_integral + u_proportionel;
      if(abs(mod180(rpy[2]* rad_to_deg-yaw_ref))<0.1 && imu_init==0)
      {
        if(time_counter2 >5000)
        {
          imu_init = 1;
          motor_angle_offset = fmod(current_angle_rd+pi,two_pi)-pi;
        }
      } else
      {
        time_counter2 = 0;
      }
    } 
    u = constrain(u,-U_MAX,U_MAX);
    sin_amplitude = constrain(50+2*abs(u),0,SIN_APMLITUDE_MAX);
  
    motor_speed_rps = two_pi*u;//tps*two_pi;
    angle_step_rd = motor_speed_rps/reg_freq;

// Data transmition
    rpy[2] = current_angle_rd-motor_angle_offset+rpy[2]-yaw_ref/rad_to_deg; // 180/pi
    omega[2] = motor_speed_rps;

    if(transmit_raw){ Serial.write(0xAA); }
    for(i=0;i<3;i++)
    {
      buffer_float = mod180(rpy[i]* rad_to_deg);
      //if(print_data){ Serial.print(buffer_float); Serial.print(" "); }
      for(j=0;j<4;j++)
      {
        if(transmit_raw){ Serial.write(ptr_buffer[j]); }
      }
    }
    
    /*
    for(i=0;i<3;i++)
    {
      buffer_float = omega[i];
      if(print_data){ Serial.print(buffer_float); Serial.print(" "); }
      for(j=0;j<4;j++)
      {
        if(transmit_raw){ Serial.write(ptr_buffer[j]); }
      }
    }*/
    if(transmit_raw){ Serial.write(0x55);Serial.write(0x55); }
    if(print_data){ Serial.println(" "); }
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

