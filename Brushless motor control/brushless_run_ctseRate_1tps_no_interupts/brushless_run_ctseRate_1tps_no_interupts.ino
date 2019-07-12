// run a brushless motor, using interuption at constant rate

#define U_MAX 5 // max speed command of the motor
#define SIN_APMLITUDE_MAX 125 // max amplitude of sinus (around a 127 offset)
#define SIN_APMLITUDE_MIN 60 // max amplitude of sinus (around a 127 offset)
#define GIMBAL_PACKET_SIZE 33 // number of bytes to be recieved from 
#define PACKET_START 0xAA // starting char of package
#define PACKET_STOP 0x55 // starting char of package

// constant used to enable/disable communication, debug, timing checks
const int8_t transmit_raw = 0;
const int8_t print_data = 1;

// definition of some constants to ease computations
const float pi = 3.14159265359;
const float two_pi = 6.28318530718;
const float four_pi_on_three = 4.18879020479;
const float two_pi_on_three = 2.09439510239;

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
uint8_t sinAngleA, sinAngleB, sinAngleC; // the 3 sinusoide values for the PWMs

// Pin definition for connection to ESC
const int EN1 = 2;   // pin enable bls
const int IN1 = 9;    //pins des pwm
const int IN2 = 4;   //pins des pwm
const int IN3 = 8;   //pins des pwm

// interuptions gestion
uint32_t time_counter; // counting the number of interuptions
uint8_t interupt_happened; // interuption flag

// variables for the serial read an data recomposition
float all_data[8];
uint8_t raw_data[100];
float buffer_float;
unsigned char *ptr_buffer = (unsigned char *)&buffer_float;

int8_t init_;

// Others
float speed_step = 1;

 
void setup() {
  Serial.begin(115200);
  
// augmentation fréquence PWM
 // setPwmFrequency(IN1); 
 // setPwmFrequency(IN2);
 // setPwmFrequency(IN3);

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

  current_angle_rd = 0;
  current_angle_rd_prev = 1;

  u = 0.5;
  sin_amplitude = 125;
  init_ = 0;
}
//////////////////////////////////////////////////////////////////////////////

void setMotorAngle(float angle_rd)
{
  if(current_angle_rd!=current_angle_rd_prev)
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
    
    current_angle_rd_prev = current_angle_rd;
  }
}

//////////////////////////////////////////////////////////////////////
 
void loop() {
int i,j,x;

// Applying the command if new
  
    //Serial.println(current_angle_rd);
    setMotorAngle(current_angle_rd);

// This generates a speed profile that increments fro speed step every second.
//  if(time_counter > 5000 )
//  {
//    interupt_happened = interupt_happened != 0 ? 0 : 100;
//    time_counter = 0;
//    
//    if(u >= 1.5)
//    {
//      init_ = 0;
//      //speed_step = -0.0;
//    }
//    if (init_ == 1)
//    {
//      speed_step = -speed_step;
//    }
//    if(u <= 0)
//    {
//      speed_step = 0.25;
//    }
//    u+=speed_step;
// 
//    u = constrain(u,-U_MAX,U_MAX);
//    sin_amplitude = constrain(10,SIN_APMLITUDE_MIN,SIN_APMLITUDE_MAX);
//  }

  u = constrain(u,-U_MAX,U_MAX);
  
  
  u = 0.5;
  sin_amplitude = constrain(40+10 *abs(u),SIN_APMLITUDE_MIN,SIN_APMLITUDE_MAX);
  motor_speed_rps = two_pi*u; // conversion from tr/s to rps
  Serial.print(sin_amplitude); Serial.print(" ");
  Serial.println(u);
  angle_step_rd = motor_speed_rps/1000;
  
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
