// run a brushless motor, using interuption at constant rate

#define U_MAX 10 // max speed command of the motor
#define SIN_APMLITUDE_MAX 125 // max amplitude of sinus (around a 127 offset)
#define SIN_APMLITUDE_MIN 60 // max amplitude of sinus (around a 127 offset)
#define GIMBAL_PACKET_SIZE 33 // number of bytes to be recieved from 
#define PACKET_START 0xAA // starting char of package
#define PACKET_STOP 0x55 // starting char of package

// constant used to enable/disable communication, debug, timing checks
const int8_t transmit_raw = 0;
const int8_t print_data = 1;
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
const uint8_t nb_pole = 22;
float slice_angle_rd, angle_scale_factor;
int16_t normalized_angle;
float sin_amplitude = 1;

// Commande variable
volatile float angle_step_rd, current_angle_rd_prev, current_angle_rd = 0;
float motor_speed_rps = 0; // desired speed of the motor, rps
float u=0;            // Command, tr/s
float u_integral = 0; // integral part of the command, tr/s
float u_proportionel = 0; // proportional part of the command, tr/s
uint8_t sinAngleA, sinAngleB, sinAngleC; // the 3 sinusoide values for the PWMs

// Pin definition for connection to ESC
const int EN1 = 4;   // pin enable bls
const int IN1 = 6;    //pins des pwm
const int IN2 = 9;   //pins des pwm
const int IN3 = 10;   //pins des pwm

// interuptions gestion
uint32_t time_counter; // counting the number of interuptions
uint8_t interupt_happened; // interuption flag

// variables for the serial read an data recomposition
// variables for the serial read an data recomposition
float yaw;
float yaw_p;

uint8_t raw_data[100];
float buffer_float;
unsigned char *ptr_buffer = (unsigned char *)&buffer_float;
int16_t buffer_int16;
unsigned char *ptr_buffer_int16 = (unsigned char *)&buffer_int16;
int32_t buffer_int32;
unsigned char *ptr_buffer_int32 = (unsigned char *)&buffer_int32;

int8_t init_;

// Others
float speed_step = 0.25;

long t0,t1;

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
ISR(TIMER3_OVF_vect) 
{
  TCNT3 = 65536-62; // 125*8us = 1ms
  interupt_happened = 1; // interuption flag to trigger computation in main loop
  current_angle_rd = fmod((current_angle_rd + angle_step_rd),two_pi); // increment the desired angle
  time_counter++; // counter in ms (replace the micros())
}


//////////////////////////////////////////////////////////////////////
 
void loop() {
int i,j,x;

// Applying the command if new
  
    //Serial.println(current_angle_rd);
    setMotorAngle(current_angle_rd);
    //Serial.println(Serial1.available() );
  if (Serial1.available() > GIMBAL_PACKET_SIZE-1) // Number of data corresponding to the IMU packet size is waiting in the biffer of serial
  { 
    x = Serial1.read(); // read first data
    //Serial.print(x); Serial.print(" ");
    if(x == PACKET_START) // check that first data correspond to start char
    {
      Serial1.readBytes(raw_data,GIMBAL_PACKET_SIZE-1); // Reading the IMU packet
      //Serial.print(raw_data[GIMBAL_PACKET_SIZE-1]);
      if(raw_data[GIMBAL_PACKET_SIZE-2] == PACKET_STOP) // check taht the last data correspond to the packet end char
      {
        for(i=0;i<6;i++) // decode YPR data
        {
          for(j=0;j<2;j++) // filling the 4 bytes of the float with the data of serial port
          {
            ptr_buffer_int16[j] = raw_data[2*i+j];
          }
          if(i==2)
          { // roll, pitch, tips0 angle
            yaw = (float)buffer_int16*180.0/32768.0;
            if(print_rpy){ Serial.print(yaw); Serial.write(9); }
          } else if(i==5)
          { // derivatives of roll, pitch, tips0 angle
            yaw_p = (float)buffer_int16*2000.0/32768.0;
            if(print_rpy_p){ Serial.print(yaw_p); Serial.write(9);}
          }
        }
        Serial.println(" ");
        current_angle_rd = yaw/rad_to_deg;
      }
    }
  }

  u = yaw_p/rad_to_deg/two_pi;
  
  sin_amplitude = constrain(0.7+0.15*abs(u),0,1);
  motor_speed_rps = two_pi*u; // conversion from tr/s to rps
//  Serial.print(current_angle_rd); Serial.print(" ");
//  Serial.println(u);
  angle_step_rd = motor_speed_rps/1000;
  
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

