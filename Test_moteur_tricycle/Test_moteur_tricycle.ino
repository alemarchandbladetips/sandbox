//#include "RadioSignals.h"
//#include "GlobalVariables.h"
#include "bte_tricycle.h"
#include <WiiChuck.h>

#define RISING_TIME_FRONT_S 5.0
#define RISING_TIME_LAT_S 1.0
#define PARKING_BRAKE_TIME_MS 2000

Accessory nunchuck;
float joyX_f, joyX, joyY_f, joyY = 0;
float joyY_f_lock = 0;

float delta_joyY_max = 1.0/(RISING_TIME_FRONT_S*100);
float delta_joyX_max = 1.0/(RISING_TIME_LAT_S*100);

int8_t acc_pin_R = 14;
int8_t brake_pin_R = 5;
int8_t reverse_pin_R = 4;
bte_tricycle_controller controlleur_R = bte_tricycle_controller(acc_pin_R, brake_pin_R, reverse_pin_R );

int8_t acc_pin_L = 15;
int8_t brake_pin_L = 3;
int8_t reverse_pin_L = 2;
bte_tricycle_controller controlleur_L = bte_tricycle_controller(acc_pin_L, brake_pin_L, reverse_pin_L );

float front_speed_ref = 0.0f;
float lateral_speed_ref = 0.0f;
float speed_ref_R = 0.0f;
float speed_ref_L = 0.0f;

int8_t last_ButtonC = 0;

int8_t constant_cruise_flag = 0;
int8_t brake_flag =0;


uint32_t timer;
uint32_t timer_parking;
uint32_t dt_us = 10000;

void setup() {
  // put your setup code here, to run once:
  //Serial.begin(115200);
  nunchuck.begin();
	if (nunchuck.type == Unknown) {
		nunchuck.type = NUNCHUCK;
	}

  timer = micros();
}

void loop() {

  if( micros()-timer > dt_us)
  {
    timer += dt_us;

    nunchuck.readData();

    joyX = (((float)nunchuck.getJoyX())-128)/128;
    joyY = (((float)nunchuck.getJoyY())-128)/128;

    
    if( (joyX > 0) && (joyX > (joyX_f + delta_joyX_max)) )
    {
      joyX_f += delta_joyX_max;
    } else if ( (joyX < 0) && (joyX < (joyX_f - delta_joyX_max)) )
    {
      joyX_f -= delta_joyX_max;
    } else
    
    {
      joyX_f = joyX;
    }

    if( (joyY > 0) && (joyY > (joyY_f + delta_joyY_max)) )
    {
      joyY_f += delta_joyY_max;
    } else if ( (joyY < 0) && (joyY < (joyY_f - delta_joyY_max)) )
    {
      joyY_f -= delta_joyY_max;
    } else
    {
      joyY_f = joyY;
    }

    

    // Switch du frein
    if(nunchuck.getButtonZ() == 0)
    {
      controlleur_R.brake(0);
      controlleur_L.brake(0);
      brake_flag = 0;
    } else
    {
      controlleur_R.brake(120);
      controlleur_L.brake(120);
      brake_flag = 1;
      constant_cruise_flag = 0;
    }

    // Désenclanchement du cruise control si marche arrière
    if(joyY_f < -0.01 )
    {
      constant_cruise_flag = 0;
    }

    // Enclanchement du cruise control.
    if( (last_ButtonC == 0 && nunchuck.getButtonC() == 1) )
    {
      if ( (constant_cruise_flag == 0) && (joyY_f > 0.1) )
      {
        constant_cruise_flag = 1;
        joyY_f_lock = joyY_f;
      } else
      {
        constant_cruise_flag = 0;
      }
    }
    last_ButtonC = nunchuck.getButtonC();

    // vitesse d'avancée constante si cruise control
    if(constant_cruise_flag == 1)
    {
      front_speed_ref = joyY_f_lock;
    } else
    {
      if(joyY_f>0)
      {
        front_speed_ref = joyY_f;
      } else
      {
        front_speed_ref = 0.75*joyY_f;
      }
    }

    
    // Calcul des consigne de vitesse pour moteur droit et gauche
    lateral_speed_ref = 0.3*joyX_f;

    if(brake_flag == 1)
    {
      speed_ref_R = 0;
      speed_ref_L = 0;
      joyY_f = 0;
    } else
    {
      speed_ref_R = front_speed_ref - lateral_speed_ref;
      speed_ref_L = front_speed_ref + lateral_speed_ref;
    }

    if( abs(speed_ref_R)>0.02 && abs(speed_ref_R)>0.02)
    {
      timer_parking = millis();
    }

    if( (millis()-timer_parking) > PARKING_BRAKE_TIME_MS)
    {
      controlleur_R.brake(255);
      controlleur_L.brake(255);
      brake_flag = 1;
    }

    // Applicaztion des consignes
    controlleur_R.update_reference(speed_ref_R);
    //Serial.print(speed_ref_R);Serial.print("\t");
    controlleur_L.update_reference(speed_ref_L);

    Serial.print(joyY);Serial.print("\t");
    Serial.print(joyX);Serial.print("\t");
    Serial.print(nunchuck.getButtonC());Serial.print("\t");
    Serial.print(nunchuck.getButtonZ());Serial.print("\t");
    Serial.print(constant_cruise_flag);Serial.print("\t");
    Serial.print(front_speed_ref);Serial.print("\t");
    Serial.print(lateral_speed_ref);Serial.print("\t");
    Serial.print(speed_ref_L);Serial.print("\t");
    Serial.print(speed_ref_R);
    Serial.println("\t");
    
  }
}
