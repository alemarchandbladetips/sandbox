// run a brushless motor, using interuption at constant rate

#define U_MAX 1 // max speed command of the motor
#define SIN_APMLITUDE_MAX 125 // max amplitude of sinus (around a 127 offset)
#define SIN_APMLITUDE_MIN 60 // max amplitude of sinus (around a 127 offset)

#define PACKET_START 0xAA // starting char of package
#define PACKET_STOP 0x55 // starting char of package

#define POSITION_MAX 1095
#define POSITION_SLICE 91.25

// constant used to enable/disable communication, debug, timing checks
const int8_t transmit_raw = 1;
const int8_t print_data = 0;

// definition of some constants to ease computations
const float pi = 3.14159265359;
const float two_pi = 6.28318530718;
const float four_pi_on_three = 4.18879020479;
const float two_pi_on_three = 2.09439510239;

// motor related constants and variables
const uint8_t nb_pole = 22;
float slice_angle_rd, angle_scale_factor;
float normalized_angle;
float sin_amplitude = 50;

// Commande variable
volatile float angle_step_rd, current_angle_rd_prev, current_angle_rd = 0;
float motor_speed_rps = 0; // desired speed of the motor, rps
float u=0;            // Command, tr/s
float norm_u = 0;            // Command, tr/s
float sign_u = 1;
float u_integral = 0; // integral part of the command, tr/s
float u_proportionel = 0; // proportional part of the command, tr/s
uint8_t sinAngleA, sinAngleB, sinAngleC; // the 3 sinusoide values for the PWMs

float speed_mmps;
float position_mm;

// Pin definition for connection to ESC
const int EN1 = 12;   // pin enable bls
const int IN1 = 9;    //pins des pwm
const int IN2 = 10;   //pins des pwm
const int IN3 = 6;   //pins des pwm

// interuptions gestion
int32_t time_counter,time_counter2; // counting the number of interuptions
uint8_t interupt_happened; // interuption flag

// variables for the serial read an data recomposition
int16_t buffer_int16;
unsigned char *ptr_buffer_int16 = (unsigned char *)&buffer_int16;



// Others
float speed_step = 0.05;

 
void setup() {
  Serial.begin(115200);
  
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
  //Serial.println(slice_angle_rd*360/two_pi);

  current_angle_rd = 0;
  current_angle_rd_prev = -1;
  time_counter = 0;
  time_counter2 = 0;

  setMotorAngle(current_angle_rd);

  norm_u = 2;
  sign_u = 1;
  sin_amplitude = 125;
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

/////////////////////////////////////////////////////////////////////
// Routine d'interruption
ISR(TIMER2_OVF_vect) 
{
  TCNT2 = 256 - 125; // 125*8us = 1ms
  interupt_happened = 1; // interuption flag to trigger computation in main loop
  current_angle_rd = constrain(current_angle_rd + angle_step_rd,-0.1,6*two_pi+0.1);//fmod((current_angle_rd + angle_step_rd),two_pi); // increment the desired angle
  time_counter++; // counter in ms (replace the micros())
  time_counter2++; // counter in ms (replace the micros())
}


//////////////////////////////////////////////////////////////////////
 
void loop() {
int i,j,x;

// Applying the command if new
  
    //Serial.println(current_angle_rd);
  setMotorAngle(current_angle_rd);

  if( current_angle_rd + angle_step_rd > 6*two_pi)
  {
    sign_u = -1;
  } else if(current_angle_rd + angle_step_rd < 0)
  {
    sign_u = 1;
  }

  //if(time_counter2 >= 5000)
  {
    /*
    if( position_mm < POSITION_SLICE)
    { // ctse = 0.3
      norm_u = 0.3;
    } else if( position_mm < 2*POSITION_SLICE)
    { // linear 0.3 to 1
      norm_u = 0.3 + 0.7 * (position_mm-POSITION_SLICE)/(POSITION_SLICE);
    } else if( position_mm < 3*POSITION_SLICE)
    { // cste = 1.5
      norm_u = 1.5;
    } else if( position_mm < 4*POSITION_SLICE)
    { // cste = 1
      norm_u = 1;
    } else if( position_mm < 5*POSITION_SLICE)
    { // cste = 2
      norm_u = 2;
    } else if( position_mm < 6*POSITION_SLICE)
    { // cste = 1
      norm_u = 2 - 1.5 * ((position_mm - 5*POSITION_SLICE)/(POSITION_SLICE))*((position_mm - 5*POSITION_SLICE)/(POSITION_SLICE));
    } else if( position_mm < 7*POSITION_SLICE)
    { // cste = 1
      norm_u = 0.5 + 1 * ((position_mm - 6*POSITION_SLICE)/(POSITION_SLICE))*((position_mm - 6*POSITION_SLICE)/(POSITION_SLICE));
    } else if( position_mm < 8*POSITION_SLICE)
    { // cste = 1
      norm_u = 2;
    } else if( position_mm < 10*POSITION_SLICE)
    { // cste = 1
      norm_u = 1.5 + sin(two_pi*(position_mm - 8*POSITION_SLICE)/POSITION_SLICE);
    } 
    else if( position_mm < 11*POSITION_SLICE)
    { // cste = 1
      norm_u = 1.5;
    } else
    {
      norm_u = 0.5;
    }
    */
    norm_u = 0;
    if(time_counter2>0 && time_counter2<3000)
    {
      norm_u = 3*sin(two_pi*(time_counter2)/6000);
    } else if(time_counter2>5000 && time_counter2<8000)
    {
      norm_u = 3*sin(two_pi*(time_counter2-2000)/6000);
    } else if (time_counter2>8000)
    {
      time_counter2 = -2000;
    }
    //norm_u = 1;
  } /*else
  {
    norm_u = 0;
  }*/

  u =   norm_u * sign_u;

  speed_mmps = u*two_pi*(29.045777);
  position_mm = current_angle_rd*(29.045777);

  if(time_counter >= 10)
  {
    time_counter = 0;
    if (print_data)
    {
      Serial.print(speed_mmps); Serial.print(" ");
      Serial.print(position_mm); Serial.println(" ");
    }
    if (transmit_raw)
    {
      Serial.write(PACKET_START);
      buffer_int16 = (int16_t)(speed_mmps*32768/2000);
      for(j=0;j<2;j++)
      {
        if(transmit_raw){ Serial.write(ptr_buffer_int16[j]); }
      }

      buffer_int16 = (int16_t)(position_mm*32768/1200);
      for(j=0;j<2;j++)
      {
        if(transmit_raw){ Serial.write(ptr_buffer_int16[j]); }
      }
      Serial.write(PACKET_STOP);

    }
  }

  //u = constrain(u,-U_MAX,U_MAX);
  
  
  sin_amplitude = constrain(100+20*abs(u),SIN_APMLITUDE_MIN,SIN_APMLITUDE_MAX);
  motor_speed_rps = two_pi*u; // conversion from tr/s to rps
  
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

