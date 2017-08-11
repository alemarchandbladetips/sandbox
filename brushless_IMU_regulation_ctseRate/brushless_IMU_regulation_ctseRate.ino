// 

const float pi = 3.14159265359;
const float two_pi = 6.28318530718;
const float four_pi_on_three = 4.18879020479;
const float two_pi_on_three = 2.09439510239;

String readString;  // permet de faire les tests et de lire les données entrées au clavier
int16_t j=0;
// variables
long i;             // peut servir à compter le nbre d'impulsions( nbre de boucles du pgm)
long t,t1;             // sert à compter le temps
float u=0;            // ordre en entrée
float u_integral = 0;
float u_proportionel = 0;
int interupt_happened=0;
uint32_t interupt_counter;
float ypr_data[3];
float slice_angle_rd, angle_scale_factor;
volatile float angle_step_rd, current_angle_rd = 0;
float motor_speed_rps = 0;
float normalized_angle;

// connections vers l'ESC
const int EN1 = 12;   // pin enable bls
const int IN1 = 9;    //pins des pwm
const int IN2 = 10;   //pins des pwm
const int IN3 = 6;   //pins des pwm


int16_t nb_pole = 22;

uint8_t raw_data[48];
float buffer_float;
unsigned char *ptr_buffer = (unsigned char *)&buffer_float;

uint8_t sinAngleA, sinAngleB, sinAngleC;


int32_t time_counter=0;
float tps = 1;
 
void setup() {
  Serial.begin(115200);
  
// augmentation fréquence PWM
  setPwmFrequency(IN1); 
  setPwmFrequency(IN2);
  setPwmFrequency(IN3);
 
  pinMode(IN1, OUTPUT); 
  pinMode(IN2, OUTPUT); 
  pinMode(IN3, OUTPUT); 
  
  pinMode(EN1, OUTPUT); 
  digitalWrite(EN1, HIGH);

// Find lookup table size
  slice_angle_rd = 2*pi/(nb_pole/2.0);
  angle_scale_factor = 2*pi/slice_angle_rd;
  Serial.println(slice_angle_rd*360/two_pi);

  u = 0;
  cli(); // Désactive l'interruption globale
  bitClear (TCCR2A, WGM20); // WGM20 = 0
  bitClear (TCCR2A, WGM21); // WGM21 = 0 
  TCCR2B = 0b00000101; // Clock / 128 soit 8us et WGM22 = 0
  TIMSK2 = 0b00000001; // Interruption locale autorisée par TOIE2
  sei(); // Active l'interruption globale
  interupt_counter = 0;
}
//////////////////////////////////////////////////////////////////////////////

void setMotorAngle(float angle_rd)
{
  
  normalized_angle = fmod(angle_rd,slice_angle_rd)*angle_scale_factor;

  sinAngleA = round(sin(normalized_angle)*125.0+127.0);
  sinAngleB = round(sin(normalized_angle+two_pi_on_three)*125.0+127.0);
  sinAngleC = round(sin(normalized_angle+four_pi_on_three)*125.0+127.0);

  analogWrite(IN1, sinAngleA);
  analogWrite(IN2, sinAngleB);
  analogWrite(IN3, sinAngleC);
}

/////////////////////////////////////////////////////////////////////
// Routine d'interruption
ISR(TIMER2_OVF_vect) {
  TCNT2 = 256 - 125;// 125*8us = 1ms
  {
    current_angle_rd = fmod((current_angle_rd + angle_step_rd),two_pi);
    time_counter++;
  }
}


//////////////////////////////////////////////////////////////////////
 
void loop() {
int k;
int x;
  // Command is applied to the motot
  setMotorAngle(current_angle_rd);
  
  if(time_counter > 999 ) // done every 1s
  {
    interupt_happened = interupt_happened != 0 ? 0 : 100;
    time_counter = 0;
    u+=0.1;
    /*
    Serial.print(current_angle_rd*360/two_pi);
    Serial.print(" ");
    Serial.print(sinAngleA);
    Serial.print(" ");
    Serial.print(sinAngleB);
    Serial.print(" ");
    Serial.println(sinAngleC);*/
    //Serial.println(tps);
  }
  if (Serial.available() > 48){
    x = Serial.read();
    if(x == 255)
    {
      Serial.write(255);
      Serial.readBytes(raw_data,48);
      for(i=0;i<3;i++) // decode YPR data
      {
        for(j=0;j<4;j++)
        {
          Serial.write(raw_data[4*i+j]);
          ptr_buffer[j] = raw_data[4*i+j];
        }
        ypr_data[i] = buffer_float*57.2957795; // 180/pi
        //Serial.print(ypr_data[i]); Serial.print(" ");
      }
     
      u_integral += 0.2*ypr_data[2]/100;
      u_integral = constrain(u_integral,-2.3,2.3);
      u_proportionel = 0.05 * ypr_data[2];
      u_proportionel = constrain(u_proportionel,-1,1);
      u = u_integral + u_proportionel;
      u = constrain(u,-2.3,2.3);
      //Serial.print(u_proportionel);
      //Serial.print(" ");
      //Serial.print(u_integral);
      //Serial.print(" ");
      //Serial.println(ypr_data[2]);
      //Serial.print(" ");
      //Serial.print(" ");
      //Serial.println(u);
    }
  }
 
  //u = 1.5;
  
  motor_speed_rps = two_pi*u;//tps*two_pi;
  
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

