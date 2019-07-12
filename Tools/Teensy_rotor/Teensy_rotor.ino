
// constant used to enable/disable communication, debug, timing checks
const int8_t transmit_raw = 0;
const int8_t print_data = 1;

// definition of some constants to ease computations
const float pi = 3.14159265359;
const float two_pi = 6.28318530718;
const float four_pi_on_three = 4.18879020479;
const float two_pi_on_three = 2.09439510239;
const float rad_to_deg = 57.2957795; // 180/pi

// motor related constants and variables
const uint32_t PWM_freq = 50000;
const uint8_t nb_pole = 14;
float slice_angle_rd, angle_scale_factor;
int16_t normalized_angle;
float sin_amplitude = 1;
volatile float current_angle_rd_prev, current_angle_rd = 0;
volatile float current_angle_rd_prev_1, current_angle_rd_1 = 0;
volatile float current_angle_rd_prev_2, current_angle_rd_2 = 0;
uint8_t sinAngleA, sinAngleB, sinAngleC; // the 3 sinusoide values for the PWMs

// Pin definition for connection to ESC
const int EN1 = 2;   // pin enable bls

const int IN1 = 20;    //pins des pwm
const int IN2 = 10;   //pins des pwm
const int IN3 = 9;   //pins des pwm

const int IN1_1 = 4;    //pins des pwm
const int IN2_1 = 3;   //pins des pwm
const int IN3_1 = 5;   //pins des pwm

const int IN1_2 = 21;    //pins des pwm
const int IN2_2 = 22;   //pins des pwm
const int IN3_2 = 23;   //pins des pwm

uint32_t t0,t1,t0_period, t_period, t_period2, n_period;

const uint32_t period_ms = 3000;
uint32_t period2_ms = 3000;
float transition_time = 75.0;
const float AOA_amplitude = 25/rad_to_deg;
const float AOA_offset_1 = 30/rad_to_deg;
const float AOA_offset_2 = 20/rad_to_deg;

float square_sig;

//const int pwmSin[] = {127, 138, 149, 160, 170, 181, 191, 200, 209, 217, 224, 231, 237, 242, 246, 250, 252, 254, 254, 254, 252, 250, 246, 242, 237, 231, 224, 217, 209, 200, 191, 181, 170, 160, 149, 138, 127, 116, 105, 94, 84, 73, 64, 54, 45, 37, 30, 23, 17, 12, 8, 4, 2, 0, 0, 0, 2, 4, 8, 12, 17, 23, 30, 37, 45, 54, 64, 73, 84, 94, 105, 116 };
const uint8_t pwmSin[] = {128, 132, 136, 140, 143, 147, 151, 155, 159, 162, 166, 170, 174, 178, 181, 185, 189, 192, 196, 200, 203, 207, 211, 214, 218, 221, 225, 228, 232, 235, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 248, 249, 250, 250, 251, 252, 252, 253, 253, 253, 254, 254, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 254, 254, 253, 253, 253, 252, 252, 251, 250, 250, 249, 248, 248, 247, 246, 245, 244, 243, 242, 241, 240, 239, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 248, 249, 250, 250, 251, 252, 252, 253, 253, 253, 254, 254, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 254, 254, 253, 253, 253, 252, 252, 251, 250, 250, 249, 248, 248, 247, 246, 245, 244, 243, 242, 241, 240, 239, 238, 235, 232, 228, 225, 221, 218, 214, 211, 207, 203, 200, 196, 192, 189, 185, 181, 178, 174, 170, 166, 162, 159, 155, 151, 147, 143, 140, 136, 132, 128, 124, 120, 116, 113, 109, 105, 101, 97, 94, 90, 86, 82, 78, 75, 71, 67, 64, 60, 56, 53, 49, 45, 42, 38, 35, 31, 28, 24, 21, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 8, 7, 6, 6, 5, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 5, 6, 6, 7, 8, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 8, 7, 6, 6, 5, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 5, 6, 6, 7, 8, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 21, 24, 28, 31, 35, 38, 42, 45, 49, 53, 56, 60, 64, 67, 71, 75, 78, 82, 86, 90, 94, 97, 101, 105, 109, 113, 116, 120, 124};
int sineArraySize;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial1.begin(115200);
  

  // augmentation fréquence PWM
  analogWriteFrequency(IN1, PWM_freq);
  analogWriteFrequency(IN2, PWM_freq);
  analogWriteFrequency(IN3, PWM_freq);

  analogWriteFrequency(IN1_1, PWM_freq);
  analogWriteFrequency(IN2_1, PWM_freq);
  analogWriteFrequency(IN3_1, PWM_freq);

  analogWriteFrequency(IN1_2, PWM_freq);
  analogWriteFrequency(IN2_2, PWM_freq);
  analogWriteFrequency(IN3_2, PWM_freq);
  
// PWM output pin configuration
  pinMode(IN1, OUTPUT); 
  pinMode(IN2, OUTPUT); 
  pinMode(IN3, OUTPUT); 

  pinMode(IN1_1, OUTPUT); 
  pinMode(IN2_1, OUTPUT); 
  pinMode(IN3_1, OUTPUT); 

  pinMode(IN1_2, OUTPUT); 
  pinMode(IN2_2, OUTPUT); 
  pinMode(IN3_2, OUTPUT); 

// pin Enable
  pinMode(EN1, OUTPUT); 
  digitalWrite(EN1, HIGH);

// Pre-computation of variables for 
  sineArraySize = sizeof(pwmSin)/sizeof(uint8_t); 
  slice_angle_rd = 2*pi/(nb_pole/2.0);
  angle_scale_factor = sineArraySize/slice_angle_rd;
  Serial.println(slice_angle_rd*360/two_pi);

  current_angle_rd = 0;
  current_angle_rd_prev = 5000;
  current_angle_rd_1 = 0;
  current_angle_rd_prev_1 = 5000;
  current_angle_rd_2 = 0;
  current_angle_rd_prev_2 = 5000;

  t0 = micros();
  t1 = micros();
  t0_period = millis();
  n_period = 0;
}

void loop() {

    t1 = micros();
    if( (t1-t0) >10000 )
    {
      if( (millis()-t0_period) > period_ms)
      {
        t0_period += period_ms;
        n_period++;
        
      }    
      t_period = (millis()-t0_period);
      t_period2 = t_period;
      t0 = t1;
      
      if(n_period<7)
      {
        period2_ms = period_ms;
      } else
      {
        period2_ms = period_ms/2;
        if(t_period2 > period2_ms)
        {
          t_period2-= period2_ms;
        }
      }

      if( period2_ms < transition_time )
      {
        square_sig = t_period2/transition_time;
      } else if( t_period2 < period2_ms/2.0-transition_time)
      {
        square_sig = 1;
      } else if( t_period2 < period2_ms/2.0+transition_time)
      {
        square_sig = 1 - (t_period2-period2_ms/2.0+transition_time)/transition_time;
      } else if( t_period2 < period2_ms-transition_time)
      {
        square_sig = -1;
      } else
      {
        square_sig = -1 + (t_period2-(period2_ms-transition_time))/transition_time;
      } 
      

      // Angle rotor
      current_angle_rd += 2*pi/(period_ms/10);
      

      if(n_period < 4)
      {
        current_angle_rd_1 = AOA_amplitude*sinf(two_pi*t_period/period_ms) + AOA_offset_1;
        current_angle_rd_2 = -AOA_amplitude*sinf(two_pi*t_period/period_ms) + AOA_offset_2;
      } else if (n_period<7)
      {
        current_angle_rd_1 = AOA_amplitude/2.0*square_sig + AOA_offset_1;
        current_angle_rd_2 = -AOA_amplitude/2.0*square_sig + AOA_offset_2;
      } else
      {
        current_angle_rd_1 = AOA_amplitude/2.0*max(square_sig,0) + AOA_offset_1;
        current_angle_rd_2 = -AOA_amplitude/2.0*max(square_sig,0) + AOA_offset_2;
      }

      if(n_period > 9)
      {
        n_period = 0;
        
      }

      current_angle_rd = fmod(current_angle_rd+two_pi,two_pi);
      current_angle_rd = fmod(current_angle_rd+two_pi,two_pi);
      current_angle_rd = fmod(current_angle_rd+two_pi,two_pi);
      
      setMotorAngle(current_angle_rd);
      Serial.print(current_angle_rd_1);Serial.print(" ");
      Serial.print(current_angle_rd_2);Serial.print(" ");
      Serial.println(square_sig);
    }

  sin_amplitude = 0.7;
}

//////////////////////////////////////////////////////////////////////////////


void setMotorAngle(float angle_rd)
{
  if(current_angle_rd!=current_angle_rd_prev)
  {
    // Computes normalized angle 0->2pi for one slice
    normalized_angle = (int16_t)(fmod(current_angle_rd+4*slice_angle_rd,slice_angle_rd)*angle_scale_factor);
  
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

  if(current_angle_rd_1!=current_angle_rd_prev_1)
  {
    // Computes normalized angle 0->2pi for one slice
    normalized_angle = (int16_t)(fmod(current_angle_rd_1+4*slice_angle_rd,slice_angle_rd)*angle_scale_factor);
  
    // Computes sin from the normalized angles
    sinAngleA = (uint8_t)(pwmSin[normalized_angle]*sin_amplitude);
    sinAngleB = (uint8_t)(pwmSin[(int16_t)fmod(normalized_angle+sineArraySize/3,sineArraySize)]*sin_amplitude);
    sinAngleC = (uint8_t)(pwmSin[(int16_t)fmod(normalized_angle+2*sineArraySize/3,sineArraySize)]*sin_amplitude);
  
    // Applies sin on PWM outputs
    analogWrite(IN1_1, sinAngleA);
    analogWrite(IN2_1, sinAngleB);
    analogWrite(IN3_1, sinAngleC);
    current_angle_rd_prev_1 = current_angle_rd_1;
  }

  if(current_angle_rd_2!=current_angle_rd_prev_2)
  {
    // Computes normalized angle 0->2pi for one slice
    normalized_angle = (int16_t)(fmod(current_angle_rd_2+4*slice_angle_rd,slice_angle_rd)*angle_scale_factor);
  
    // Computes sin from the normalized angles
    sinAngleA = (uint8_t)(pwmSin[normalized_angle]*sin_amplitude);
    sinAngleB = (uint8_t)(pwmSin[(int16_t)fmod(normalized_angle+sineArraySize/3,sineArraySize)]*sin_amplitude);
    sinAngleC = (uint8_t)(pwmSin[(int16_t)fmod(normalized_angle+2*sineArraySize/3,sineArraySize)]*sin_amplitude);
  
    // Applies sin on PWM outputs
    analogWrite(IN1_2, sinAngleA);
    analogWrite(IN2_2, sinAngleB);
    analogWrite(IN3_2, sinAngleC);
    current_angle_rd_prev_2 = current_angle_rd_2;
  }
}

/////////////////////////////////////////////////////////////////////
