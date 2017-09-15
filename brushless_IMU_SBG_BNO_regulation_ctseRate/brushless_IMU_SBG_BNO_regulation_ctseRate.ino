// Control of a brushless motor ensuring the yaw of IMU BNO055 sticks to 0, using interuption at constant rate

// It transmits data via Tx in the following form:
// 0xAA, Yaw (Float, 4bytes), Pitch (Float, 4bytes), Roll (Float, 4bytes), 
// Omegax (Float, 4bytes), Omegay (Float, 4bytes), Omegaz (Float, 4bytes), 0x55
// = 26 bytes

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "math.h"
#include <SoftwareSerial.h>

#define IUM_PACKET_SIZE 16 // number of bytes to be recieved from IMU not counting starting char + gyr 3*2 + quat 4*2 + accuracy 1 + stop char 1
#define PACKET_START 0xAA // starting char of package
#define PACKET_STOP 0x55 // starting char of package

#define POZYX_PACKET_SIZE 5 // number of bytes to be recieved from 
#define POZYX_PACKET_START 137 // starting char of package
#define POZYX_PACKET_STOP 173 // starting char of package

#define U_MAX 0.5 // max speed command of the motor

// constant used to enable/disable communication, debug, timing checks
const int8_t transmit_raw = 1;
const int8_t print_data = 0;
const int8_t print_timing = 0;

// definition of some constants to ease computations
const float pi = 3.14159265359;
const float two_pi = 6.28318530718;
const float four_pi_on_three = 4.18879020479;
const float two_pi_on_three = 2.09439510239;
const float rad_to_deg = 57.2957795; // 180/pi

// motor related constants and variables
const uint8_t nb_pole = 22;
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
float angle_error_deg;
uint8_t sinAngleA, sinAngleB, sinAngleC; // the 3 sinusoide values for the PWMs
const float yaw_ref = 180;
const float Ki = 0.01;//0.1; // integral gain
const float Kp = 0.0075; // proportional gain
const float IMU_freq = 100; // IMU frequency
const float reg_freq = 1000; // regulation frequency
float nominal_speed_rps = 0;

// Pin definition for connection to ESC
const int EN1 = 12;   // pin enable bls
const int IN1 = 9;    //pins des pwm
const int IN2 = 10;   //pins des pwm
const int IN3 = 6;   //pins des pwm

const int led_pin = 13;
int led_status = 0;
int led_counter = 0;
int led_half_period = 50;

// interuptions gestion
uint32_t time_counter, time_counter2=0; // counting the number of interuptions
uint8_t interupt_happened; // interuption flag

// variables for the serial read an data recomposition
float ypr_data[3];
uint8_t pozyx_data[4] = {0,0,0,0};
uint8_t pozyx_data_buffer[6] = {0,0,0,0,0,0};
uint8_t raw_data[48];
float buffer_float;
unsigned char *ptr_buffer = (unsigned char *)&buffer_float;
uint32_t buffer_uint32;
unsigned char *ptr_buffer_uint32 = (unsigned char *)&buffer_uint32;
int16_t buffer_int16;
unsigned char *ptr_buffer_int16 = (unsigned char *)&buffer_int16;

// for BNO
Adafruit_BNO055 bno = Adafruit_BNO055();
uint8_t sys, gyr, accel, mag = 0;
float omega[3], quaternion[4], rpy[3], proper_acc[3];
float motor_angle_offset = 0;
uint32_t accuracy_flags;
uint8_t imu_init = 0;
float alpha_omega = 0.33;

// For acc integration
float linear_accel_vector[3], pos_ENU[3], posp_ENU[3], pospp_ENU[3];

long t0,t1,t2,t3,t4,t5,t6,t_;

const int pwmSin[] = {127, 138, 149, 160, 170, 181, 191, 200, 209, 217, 224, 231, 237, 242, 246, 250, 252, 254, 254, 254, 252, 250, 246, 242, 237, 231, 224, 217, 209, 200, 191, 181, 170, 160, 149, 138, 127, 116, 105, 94, 84, 73, 64, 54, 45, 37, 30, 23, 17, 12, 8, 4, 2, 0, 0, 0, 2, 4, 8, 12, 17, 23, 30, 37, 45, 54, 64, 73, 84, 94, 105, 116 };
int sineArraySize;

//debug
int dbg_n = 0;
float u_dbg[3];
 
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

  digitalWrite(led_pin, LOW);
  led_status = 0;
}
//////////////////////////////////////////////////////////////////////////////

// Computes and send the PWM values for a disered angle

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
// Just increment the desired angle

ISR(TIMER2_OVF_vect) 
{
  TCNT2 = 256 - 125; // 125*8us = 1ms
  // increment the desired angle from the increment computed from the command
  current_angle_rd = fmod((current_angle_rd + angle_step_rd)+two_pi,two_pi); 
  time_counter2++;
  time_counter++;
}
//////////////////////////////////////////////////////////////////////

// transformation from quaternion to Roll Pitch Yaw angles
// input: q[4]: quaternion in flt
// output: rpy[3]: Euler angles in flt (rd)
// Measured execution time for a random quaternion : 500us to 750us

void quat2rpy(float q[4], float rpy[3]) // 
{
  rpy[0] = atan2(2*q[1]*q[3] + 2*q[2]*q[0], 1 - 2*q[1]*q[1] - 2*q[2]*q[2]);
  rpy[1] = asin(2*q[2]*q[3] - 2*q[1]*q[0]);
  rpy[2] = atan2(2*q[1]*q[2] + 2*q[3]*q[0], 1 - 2*q[1]*q[1] - 2*q[3]*q[3]);
}

void quat2rpy_ellipse(float q[4], float rpy[3]) // 
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
  v_out[0] = (1 - 2*q[2]*q[2] - 2*q[3]*q[3]) * v_in[0] + (2*q[1]*q[2] - 2*q[3]*q[0]) * v_in[1]     + (2*q[1]*q[3] + 2*q[2]*q[0]) * v_in[2];
  v_out[1] = (2*q[1]*q[2] + 2*q[3]*q[0]) * v_in[0]     + (1 - 2*q[1]*q[1] - 2*q[3]*q[3]) * v_in[1] + (2*q[2]*q[3] - 2*q[1]*q[0]) * v_in[2];
  v_out[2] = (2*q[1]*q[3] - 2*q[2]*q[0]) * v_in[0]     + (2*q[2]*q[3] + 2*q[1]*q[0]) * v_in[1]     + (1 - 2*q[1]*q[1] - 2*q[2]*q[2]) * v_in[2];
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

// Applying the command if new
  
  setMotorAngle(current_angle_rd);               //  une fois par ms // Question : combien de temps elle prend ? 500us

  if(time_counter>=20) // We don't recieve info from the IMU for more than 20ms, we will control the motor in open loop.
  { // This is a security, should not happen in nominal operation mode.

    time_counter = 0;
    digitalWrite(led_pin, LOW);
   
    
    // gyro data, only gyro data on z axis will be used
    imu::Vector<3> gyro=bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    nominal_speed_rps = alpha_omega*gyro.z()+(1-alpha_omega)*nominal_speed_rps;

    u = nominal_speed_rps/two_pi; // adding current rotation speed to PI correction
        
    sin_amplitude = constrain(0.4+0.1*abs(u),0,1); // Modulation of amplitude vs rotation speed to work at quasi constant current

    // Transformation from rotation speed to angle increment (incrementation is done in the interuption
    motor_speed_rps = two_pi*u;//tps*two_pi;
    angle_step_rd = motor_speed_rps/reg_freq;

    setMotorAngle(current_angle_rd); 
  }

  if (Serial.available() > IUM_PACKET_SIZE) // Number of data corresponding to the IMU packet size is waiting in the biffer of serial
  { 
    
/////////////////////// Lecture des données du port série venant de l'arduino "du BNO du haut"
    if(print_timing) { t0 = micros(); }

    t_ = t0;
    x = Serial.read(); // read first data
    //Serial.println(x);
    if(x == PACKET_START) // check that first data correspond to start char
    {
      Serial.readBytes(raw_data,IUM_PACKET_SIZE); // Reading the IMU packet
      setMotorAngle(current_angle_rd); 
      
/////////////////////// Décodage des données et transformation du quaternion en RPY
      
      if(print_timing) { t1 = micros();}

      if(raw_data[IUM_PACKET_SIZE-1] == PACKET_STOP) // check taht the last data correspond to the packet end char
      {
        time_counter = 0;
        // led blink to verify data reception
        led_counter ++;
        if(led_status == 1 && led_counter>=led_half_period)
        {
          digitalWrite(led_pin, LOW);
          led_status = 0;
          led_counter = 0;
        } else if(led_counter>=led_half_period)
        {
          digitalWrite(led_pin, HIGH);
          led_status = 1;
          led_counter = 0;
        }
        
        for(i=0;i<3;i++) // decode sensor data, gyr
        {
          for(j=0;j<2;j++) // filling the 4 bytes of the buffer using a pointer
          {
            ptr_buffer_int16[j] = raw_data[2*i+j];
          }
          omega[i] = buffer_int16*1000.0/32768.0;
        }
        
        for(i=0;i<4;i++) // decode QUAT data
        {
          for(j=0;j<2;j++)  // filling the 4 bytes of the buffer using a pointer
          {
            ptr_buffer_int16[j] = raw_data[2*i+j+6];
          }
          quaternion[i] = buffer_int16/32768.0; 
        }
        
        accuracy_flags = raw_data[14];

        // transformation of quaternion in rpy angles
        quat2rpy_ellipse(quaternion,rpy);

        // mounting of Ellipse Z pointing up
        rpy[0] += pi;

        setMotorAngle(current_angle_rd); 
        
        
///////////////////////////////////// Calcul de la commande

      if(print_timing) { t2 = micros(); }
  
  // Computing command
        u = 0;
        angle_error_deg = mod180(rpy[2]* rad_to_deg-yaw_ref);

        
        if((accuracy_flags & 0x26) == 0x26)
        { // if the gyro has saturated, we go in open loop mode, just compensating speed measured by base gyro. (correspond to the "else")

          // integral part of the command
          u_integral += Ki*mod180(angle_error_deg)/IMU_freq;
          u_integral = constrain(u_integral,-U_MAX,U_MAX);

          // proportional part of the command
          u_proportionel = Kp * angle_error_deg;
          u_proportionel = constrain(u_proportionel,-U_MAX,U_MAX);
          
          u = u_proportionel + u_integral; // computing the PI correction
  
          if(abs(angle_error_deg<3) && imu_init==0 && ((accuracy_flags & 0x66) == 0x66))
          { //heading has converge, if it is the first time, we will set the 0 of blade0
            led_half_period = 25; // 2hz led blink
            if(time_counter2 >5000)
            { // waiting for 5s immobility to take reference
              imu_init = 1; // so we will not set a ref again
              motor_angle_offset = fmod(current_angle_rd+pi,two_pi)-pi;
              led_half_period = 5; // led blinks 10Hz
              
            }
          } else
          {
            time_counter2 = 0;
          }
        } else
        {
          u_integral = 0;
        }

        u = constrain(u,-U_MAX,U_MAX) + nominal_speed_rps/two_pi; // adding current rotation speed to PI correction
        
        sin_amplitude = constrain(0.7+0.1*abs(u),0,1); // Modulation of amplitude vs rotation speed to work at quasi constant current

        // Transformation from rotation speed to angle increment (incrementation is done in the interuption
        motor_speed_rps = two_pi*u;//tps*two_pi;
        angle_step_rd = motor_speed_rps/reg_freq;

        setMotorAngle(current_angle_rd);
        
///////////////////////////////////// 

        // Data transmition, to the control part of the drone

        if(print_timing) { t3 = micros(); }
 
        if(transmit_raw){ Serial.write(PACKET_START); } // starting byte
        
        // Roll and pitch
        for(i=0;i<2;i++)
        {
          buffer_int16 = (int16_t)(mod180(rpy[i]* rad_to_deg)*32768/180);
          if(print_data){ Serial.print(buffer_int16); Serial.print(" "); }
          for(j=0;j<2;j++)
          {
            if(transmit_raw){ Serial.write(ptr_buffer_int16[j]); }
          }
        }

        // blade0 angle
        buffer_int16 = (int16_t)(mod180((current_angle_rd-motor_angle_offset+rpy[2])*rad_to_deg-yaw_ref)*32768/180); // we put the data in the int16 buffer
        if(print_data){ Serial.print(buffer_float); Serial.print(" "); }
        for(j=0;j<2;j++)
        {
          if(transmit_raw){ Serial.write(ptr_buffer_int16[j]); } // we transmit each bytes of the int16 buffer using this pointer
        }

        // rotation speeds (roll and pitch derivatives)
        for(i=0;i<2;i++)
        {
          buffer_int16 = (int16_t)(omega[i]*32768/2000);
          if(print_data){ Serial.print(buffer_float); Serial.print(" "); }
          for(j=0;j<2;j++)
          {
            if(transmit_raw){ Serial.write(ptr_buffer_int16[j]); }
          }
        }

        // blade rotation speed
        buffer_int16 = (int16_t)(motor_speed_rps*32768/2000);
        if(print_data){ Serial.print(buffer_float); Serial.print(" "); }
        for(j=0;j<2;j++)
        {
          if(transmit_raw){ Serial.write(ptr_buffer_int16[j]); }
        }

        // proper accelerations in world frame
        for(i=0;i<3;i++)
        {
          buffer_int16 = 0;//(int16_t)(proper_acc[i]*32768/40);
          //if(print_data){ Serial.print(buffer_float); Serial.print(" "); }
          for(j=0;j<2;j++)
          {
            if(transmit_raw){ Serial.write(ptr_buffer_int16[j]); }
          }
        }

        // pozyx data
        for(i=0;i<4;i++)
        {
          if(print_data){ Serial.print(0); Serial.print(" "); }
          if(transmit_raw){ Serial.write(0); }
        }
        
        if(transmit_raw){ Serial.write(PACKET_STOP); } // ending byte

        setMotorAngle(current_angle_rd);

  ///////////////////////////////////// Lecture du BNO du bas

        if(print_timing) { t4 = micros(); }

        // gyro data, only gyro data on z axis will be used
        imu::Vector<3> gyro=bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
        nominal_speed_rps = alpha_omega*gyro.z()+(1-alpha_omega)*nominal_speed_rps;

        setMotorAngle(current_angle_rd); 

 ///////////////////////////////////// Printing timings (optional)

        if(print_timing) { t6 = micros(); }
        if(print_timing) { 
          /*Serial.print(t1-t0);Serial.print(" ");
          Serial.print(t2-t1);Serial.print(" ");
          Serial.print(t3-t2);Serial.print(" ");
          Serial.print(t4-t3);Serial.print(" ");
          Serial.print(t6-t4);Serial.print(" ");
          //Serial.print(t6-t5);Serial.print(" ");
          Serial.print(t6-t0);Serial.print(" ");*/
          
          //Serial.println(" "); 
          }
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

