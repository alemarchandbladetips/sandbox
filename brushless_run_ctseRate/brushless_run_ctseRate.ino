// run a brushless motor, using interuption at constant rate

// definition of some constants to ease computations
const float pi = 3.14159265359;
const float two_pi = 6.28318530718;
const float four_pi_on_three = 4.18879020479;
const float two_pi_on_three = 2.09439510239;

// motor related constants and variables
const uint8_t nb_pole = 22;
float slice_angle_rd, angle_scale_factor;
float normalized_angle;

// Commande variable
volatile float angle_step_rd, current_angle_rd_prev, current_angle_rd = 0;
float motor_speed_rps = 0; // desired speed of the motor, rps
float u=0;            // Command, tr/s
float u_integral = 0; // integral part of the command, tr/s
float u_proportionel = 0; // proportional part of the command, tr/s
uint8_t sinAngleA, sinAngleB, sinAngleC; // the 3 sinusoide values for the PWMs

// Pin definition for connection to ESC
const int EN1 = 12;   // pin enable bls
const int IN1 = 9;    //pins des pwm
const int IN2 = 10;   //pins des pwm
const int IN3 = 6;   //pins des pwm

// interuptions gestion
uint32_t time_counter; // counting the number of interuptions
uint8_t interupt_happened; // interuption flag

// variables for the serial read an data recomposition
float ypr_data[3];
uint8_t raw_data[48];
float buffer_float;
unsigned char *ptr_buffer = (unsigned char *)&buffer_float;

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
  Serial.println(slice_angle_rd*360/two_pi);

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
  sinAngleA = round(sin(normalized_angle)*125.0+127.0);
  sinAngleB = round(sin(normalized_angle+two_pi_on_three)*125.0+127.0);
  sinAngleC = round(sin(normalized_angle+four_pi_on_three)*125.0+127.0);

  // Applies sin on PWM outputs
  analogWrite(IN1, sinAngleA);
  analogWrite(IN2, sinAngleB);
  analogWrite(IN3, sinAngleC);
}

/////////////////////////////////////////////////////////////////////
// Routine d'interruption
ISR(TIMER2_OVF_vect) 
{
  TCNT2 = 256 - 125; // 125*8us = 1ms
  interupt_happened = 1; // interuption flag to trigger computation in main loop
  current_angle_rd = fmod((current_angle_rd + angle_step_rd),two_pi); // increment the desired angle
  time_counter++; // counter in ms (replace the micros())
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

// This generates a speed profile that increments fro speed step every second.
  if(time_counter > 999 )
  {
    interupt_happened = interupt_happened != 0 ? 0 : 100;
    time_counter = 0;
    if(u > 2)
    {
      speed_step = -0.05;
    }
    if(u < -2)
    {
      speed_step = 0.05;
    }
    u+=speed_step;
  }

  // monitoring the serial port
  if (Serial.available() > 12){
    x = Serial.read();
    if(x == 255)
    {
      //Serial.write(255);
      Serial.readBytes(raw_data,12);
      for(i=0;i<3;i++) // decode YPR data
      {
        for(j=0;j<4;j++) // filling the 4 bytes of the float with the data of serial port
        {
          ptr_buffer[j] = raw_data[4*i+j];
        }
        ypr_data[i] = buffer_float*57.2957795; // 180/pi
      }
      Serial.print(ypr_data[2]);
      Serial.print(" ");
      Serial.println(u);
    }
  }

  //u = 0;
  
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

