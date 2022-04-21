#include <HX711_ADC.h>
#include <Adafruit_INA260.h>
#include "Servo.h"
#include "GlobalVariables.h"
#include "RadioSignals.h"

#define TEMPS_WAIT 60*1000  // correspond à min vitesse de 500 rpm 
#define TEMPS_LECT 1000

Servo S1, S2, M1;

//          pin, pwmMin, pwmMax, pwmInit, pwm, control, rot
motor Servo1{8, 800, 2200, 1602, 1602, 0, 1};
//motor Servo2{9, 800, 2200, 1431, 1431, 0, 1};
motor Servo2{9, 800, 2200, 1391, 1391, 0, 1};

motor Mot1  {7,1000, 2000, 1000, 1000, 0, 1};

Adafruit_INA260 ina260 = Adafruit_INA260();

////// Sonde poids //////

HX711_ADC sonde_poussee(18, 19);
//HX711_ADC sonde_poussee(25, 24);
//HX711_ADC sonde_poussee(17, 16);
//HX711_ADC sonde_couple(38, 37);
float calibrationValue_poussee = 1108.2;//447.14;
float calibrationValue_couple = 1121.51;

float poussee, couple = 0;

////////////////////////

float vitesse_rotation;

float poussee_f, couple_f, vitesse_rotation_f, If_f;



uint32_t dt, tt;

float V, I, P;
float Vf, If, Pf;

uint32_t dt_test;

uint32_t dt_exec, tt_exec;

char val1; 
char val2;

char val3;

uint32_t tt_test;
int16_t rx_val;

bool INA260_OK = false;

uint32_t DT1 = 5 *1000000; 
uint32_t DT2 = 5* 1000000;
 
uint32_t DT3 = 5*1000000;
uint32_t DT4 = 5*1000000;

uint32_t tt_step, dt_step;
uint16_t pwm_Servo=0;
uint16_t last_pwm_Servo=0;

uint32_t tt_init;
float f_init = 1.8/300;
uint16_t pwm_Mot, last_pwm_Mot;

////////////////////////////////////
//Paramètres mesure de RPM
uint32_t dt_wait, temps_wait, dt_wait_ok=0;
uint32_t dt_lect, temps_lect, dt_stop=0;
uint32_t temps2wait = TEMPS_WAIT;

uint16_t value_portLED=0;
boolean EtatA=HIGH, last_EtatA=HIGH;
float rpm=0;
float dt_filt=1000000;
int nb_pales = 2;
uint32_t const mindtwait=500, maxdtwait=450000;
int portLED = A0;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial.setTimeout(0);
  //while (!Serial) { delay(10); }

  Serial.print("Init PWMs ->> ");

  INA260_OK = ina260.begin(INA260_I2CADDR_DEFAULT,&Wire2);
  
  S1.attach(Servo1.pin);
  S1.writeMicroseconds(Servo1.pwmInit);

  S2.attach(Servo2.pin);
  S2.writeMicroseconds(Servo2.pwmInit);

  M1.attach(Mot1.pin);
  M1.writeMicroseconds(Mot1.pwmMin);

  Serial.println("Done");

  Serial.print("Init sonde poussee ->> ");
  sonde_poussee.begin();
  sonde_poussee.start(2000, 1);
  
  if (sonde_poussee.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    while (1);
  }
  else {
    sonde_poussee.setCalFactor(calibrationValue_poussee); // set calibration value (float)
    sonde_poussee.setSamplesInUse(1);
    Serial.println("Done");
  }

//  Serial.print("Init sonde couple ->> ");
//  sonde_couple.begin();
//  sonde_couple.start(2000, 1);
//  
//  if (sonde_couple.getTareTimeoutFlag()) {
//    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
//    while (1);
//  }
//  else {
//    sonde_couple.setCalFactor(calibrationValue_couple); // set calibration value (float)
//    sonde_couple.setSamplesInUse(1);
//    Serial.println("Done");
//  }
  
    
//  while (rx_val < 1)
//  {
//    if (Serial.available() > 0)
//     {
//        rx_val = Serial.parseInt();        
//     }
//  }
//  
//  delay(1000);
//  M1.writeMicroseconds(Mot1.pwmMin+200);
//  delay(1000);
//  M1.writeMicroseconds(Mot1.pwmMin+350);
//  delay(1000);
//  M1.writeMicroseconds(Mot1.pwmMin+479);

  tt= micros();
  tt_exec = micros();
  tt_step = micros();

  temps_lect = micros();
  temps_wait = micros();
    
  Serial.println("Init done");
  delay(1000);
}

void loop() {

  ///////////////////////////////////////////////////////
  /// Lecture de RADIO télécommande ////////////////////
  lectureRadio();

  //////////////////////////////////////////////////////
  //// Mesure de RPM du moteur
  dt_lect = micros() - temps_lect;
  if (dt_lect > TEMPS_LECT )
  {   
      temps_lect += dt_lect;
      dt_wait = micros() - temps_wait; 
       
      if ( dt_wait > temps2wait)
      {
          value_portLED = analogRead(portLED);
          if (value_portLED == 1023) 
          {
            EtatA = HIGH; // en face de surface réflective
          }
          else
          {
            EtatA = LOW;
          }

          if ( last_EtatA == LOW && EtatA == HIGH ) //Front ascendant
          {
              temps_wait += dt_wait;

              dt_filt = 0.97 * dt_filt + 0.03*dt_wait;
              rpm = 60.0*1000.0*1000.0/(nb_pales*dt_filt);
           
              temps2wait = constrain(0.250*dt_wait,mindtwait,maxdtwait);
//                Serial.print(rpm);Serial.print("\t");
//                Serial.print(rpm_);Serial.print("\n");

          }
          else if (dt_wait > maxdtwait )
          {   
              temps_wait += dt_wait;
              rpm = 0.5*rpm; // si pas de détection donc ca converge vitesse à zero 
              temps2wait = maxdtwait;
          }
          
          last_EtatA = EtatA;
      
      }
  }   
  ///////////////////////////////////////////////////////
  //// Lecture du capteur INA260 ///////////////////////
  dt = micros() - tt;
  if (dt > 5*1000)
  {
    tt += dt;
    
    if (INA260_OK )
    {   
        I = ina260.readCurrent() / 1000.0;
        V = ina260.readBusVoltage() / 1000.0; 
    }

    
    P = V*I;
    
    float f = 1.8/300;
    If = (1-f)*If + f*I;
    Vf = (1-f)*Vf + f*V;
    
    Pf = (1-f)*Pf + f*P;
     
  }

////////////////////////////////////////////////////////////
////// PWM du  MOTEUR,  PWMs des SERVOs

  dt_exec = micros() - tt_exec;
  if (dt_exec > 10000 )
  { 
     tt_exec += dt_exec;

     if (Serial.available() > 0)
     {
        rx_val = Serial.parseInt();        
        rx_val = constrain(rx_val,-400,400);
       // rx_val = constrain(rx_val,0,1000);

     }

//    Servo1.pwm = Servo1.pwmInit - 125;
//    Servo2.pwm = Servo2.pwmInit + 125;

    Servo1.pwm = Servo1.pwmInit + rx_val;
    Servo2.pwm = Servo2.pwmInit - rx_val;
    
//    Serial.println(Servo2.pwm);
    
   //Servo1.pwm = Servo1.pwmInit - 200*(knob-0.5)*2;
//    Servo2.pwm = Servo2.pwmInit + 200*(knob-0.5)*2;
   
    S1.writeMicroseconds(Servo1.pwm);
    S2.writeMicroseconds(Servo2.pwm);

    Mot1.pwm = Mot1.pwmInit + knob*1000;
    Mot1.pwm = constrain(Mot1.pwm,Mot1.pwmMin, 2000);
    M1.writeMicroseconds(Mot1.pwm);

    if(sonde_poussee.update())
    {
      poussee = sonde_poussee.getData();
    }
//    if(sonde_couple.update())
//    {
//      couple = sonde_couple.getData();
//    }

    float a = 0.9;
    poussee_f = a*poussee_f + (1-a)*poussee;
    couple_f = a*couple_f + (1-a)*couple;
    vitesse_rotation_f = vitesse_rotation;
    If_f = a*If_f + (1-a)*If;

    vitesse_rotation = rpm;
    
//    Serial.print(Mot1.pwm,0);Serial.print("\t");
//    Serial.print(vitesse_rotation);Serial.print("\t");
//    Serial.print(Vf);Serial.print("\t");
//    Serial.print(If,3);Serial.print("\t");
//    Serial.print(poussee);Serial.print("\t");
//    Serial.print(couple);Serial.print("\n");

    
    //Serial.print("RX_servo: ");Serial.print("\t"); Serial.print(rx_val);Serial.print("\t");
    //Serial.print("PWM_Mot: ");Serial.print("\t"); Serial.print(Mot1.pwm,0);Serial.print("\t");
    //Serial.print("RPM: ");Serial.print("\t"); Serial.print(vitesse_rotation_f);Serial.print("\t");
    //Serial.print("Volt: ");Serial.print("\t");Serial.print(Vf);Serial.print("\t");
    //Serial.print("Courant: ");Serial.print("\t");Serial.print(If_f,3);Serial.print("\t");
    //Serial.print("Poussee: ");Serial.print("\t");
    Serial.print(poussee);Serial.print("\t");
    Serial.print(poussee_f);Serial.print("\t");
    //Serial.print("Couple: ");Serial.print("\t");Serial.print(couple_f);Serial.print("\n");
    

//    Serial.print("S1: ");Serial.print("\t");Serial.print(Servo1.pwm);Serial.print("\t");
//    Serial.print("S2: ");Serial.print("\t");Serial.print(Servo2.pwm);Serial.print("\n");


    Serial.println(" ");
  }


  
}
