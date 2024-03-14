#include "bte_tricycle.h"

/*
// Usefull when using the nunchuck
#include <WiiChuck.h>

Accessory nunchuck;
*/

#define ENABLE_PRINT 0
#define ENABLE_PRINT_DEBUG 1

//// Definition des paramètres des la loi de contrôle

// flitrage entrées de commande
#define ALPHA_JOYSTICK_FRONT 0.003
#define ALPHA_JOYSTICK_LAT 0.02
#define M_JOYSTICK 0.8
#define ALPHA_FALL_JOYSTICK_FRONT 0.05
#define ALPHA_FALL_JOYSTICK_LAT 0.05

// définition de la vitesse max
#define FRONT_SPEED_GAIN 115.0f
//120.0f 
// Vitesse max arrière par rapport à la vitesse avant (la vitesse peut être saturé par les controleurs, sas que l'on puisse le configurer)
#define REAR_VS_FRONT_GAIN 0.05f 
// Vitesse différentielle max par rapport à la vitesse avant
#define LAT_VS_FRONT_GAIN 0.3f 
// vitesse max en mode régulation de vitesse
#define CRUISE_SPEED_MAX 1.0f

// Définition des créneaux pour le freinage
#define BRAKE_LEVEL 8
#define BRAKE_LEVEL_MAX0 18
#define BRAKE_LEVEL_MAX1 20
#define BRAKE_LEVEL_MAX2 22

#define BRAKE_TIME_PROP 2000
#define BRAKE_TIME_CSTE 1500
#define BRAKE_LEVEL2_TIME 2500
#define BRAKE_LEVEL1_TIME 1500

uint8_t brake_level_max = BRAKE_LEVEL_MAX0;
uint8_t brake_level = BRAKE_LEVEL;
uint8_t last_button_brake = 0;
uint32_t brake_time;

#define BRAKE_LEVEL_PWM 255
#define PARKING_BRAKE_LEVEL_PWM 255

// Temps de perte de connexion
#define CONNEXION_LOST_TIME_MS 2000 
// Temps maximal d'enclanchement du régulateur de vitesse
#define CRUISE_CONTROL_TIME 20000 

// Largeur de la zone morte normalisée (utilisé pour désenclancher le régulateur de vitesse)
#define DEAD_ZONE 0.05f 
#define DEAD_ZONE_X 0.4f 

//// Définition des paramètres de la connexion série avec le BT

#define WIFI_SERIAL Serial1
#define BLE_START1 0x42
#define BLE_START2 0x24

//// Définition des variables ////

// Contrôleur moteur droit
int8_t acc_pin_R = 6;
int8_t brake_pin_R = 9;
int8_t reverse_pin_R = 7;
bte_tricycle_controller controlleur_R = bte_tricycle_controller(acc_pin_R, brake_pin_R, reverse_pin_R );


// controleur moteur gauche
int8_t acc_pin_L = 10;
int8_t brake_pin_L = 5;
int8_t reverse_pin_L = 4;
bte_tricycle_controller controlleur_L = bte_tricycle_controller(acc_pin_L, brake_pin_L, reverse_pin_L );

// Télécommande
uint8_t serial_data;

float joyX_f, joyX, joyY_f, joyY, joyY_tmp, joyX_f1, joyY_f1, joyX_f2, joyY_f2 = 0;
float constant_cruise_speed = 0.1f;
uint8_t button_cruise, button_brake;
int8_t last_button_cruise = 0;

// Définition des paramètres des filtres

float ky = 1/(1+2*M_JOYSTICK/ALPHA_JOYSTICK_FRONT+(1/ALPHA_JOYSTICK_FRONT)*(1/ALPHA_JOYSTICK_FRONT));
float a1y = 2*M_JOYSTICK/ALPHA_JOYSTICK_FRONT+2*(1/ALPHA_JOYSTICK_FRONT)*(1/ALPHA_JOYSTICK_FRONT);
float a2y = -(1/ALPHA_JOYSTICK_FRONT)*(1/ALPHA_JOYSTICK_FRONT);

float kx = 1/(1+2*M_JOYSTICK/ALPHA_JOYSTICK_LAT+(1/ALPHA_JOYSTICK_LAT)*(1/ALPHA_JOYSTICK_LAT));
float a1x = 2*M_JOYSTICK/ALPHA_JOYSTICK_LAT+2*(1/ALPHA_JOYSTICK_LAT)*(1/ALPHA_JOYSTICK_LAT);
float a2x = -(1/ALPHA_JOYSTICK_LAT)*(1/ALPHA_JOYSTICK_LAT);

// Variables de contrôle
float front_speed_ref = 0.0f;
float lateral_speed_ref = 0.0f;
float speed_ref_R = 0.0f;
float speed_ref_L = 0.0f;
float d_lateral_speed_ref = 0.0f;

// flags
int8_t constant_cruise_flag = 0;
int8_t brake_flagR = 0;
int8_t brake_flagL = 0;
int8_t last_brake_flagR = 0;
int8_t last_brake_flagL = 0;
int8_t connexion_flag = 0;
int8_t parking_brake_flag = 1;

// timers
uint32_t timer, timer_parking, timer_remote, timer_cruise_control, timer_speed,timer_brake;
uint32_t time_since_since_last_data;
uint32_t time_tmp;
uint32_t dt_us = 10000;
uint32_t dt_speed_us = 1000;

uint8_t brake_counter = 0;

int32_t i,j;

void setup() {

  //// Configuration des ports série
  Serial.begin(115200);
  WIFI_SERIAL.begin(115200);

  //// initialisation des timers
  timer = micros();
  timer_speed = micros();
  timer_brake = millis();
  timer_remote = millis();
  timer_parking = millis();
  timer_cruise_control = millis();
}

void loop() {

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  //// Lecture des commandes de la télécommande via le sérial ////
  if(WIFI_SERIAL.available()>5)
  {
    //Serial.println("6 bytes available on serial");
    serial_data = WIFI_SERIAL.read();
    //Serial.println(serial_data);
    if(serial_data == BLE_START1)
    {
      serial_data = WIFI_SERIAL.read();
      //Serial.println(serial_data);
      if(serial_data == BLE_START2)
      {
        serial_data = WIFI_SERIAL.read();
        //Serial.print(serial_data);Serial.print("\t");
        joyX = (((float)serial_data)-128)/128;
        if(abs(joyX)<DEAD_ZONE_X)
        {
          joyX=0;
        } else
        {
          if(joyX>0)
          {
            joyX = (joyX-DEAD_ZONE_X)/(1-DEAD_ZONE_X);
          } else if(joyX<0)
          {
            joyX = (joyX+DEAD_ZONE_X)/(1-DEAD_ZONE_X);
          }
        }
        serial_data = WIFI_SERIAL.read();
        //Serial.print(serial_data);Serial.println("\t");
        joyY = (((float)serial_data)-128)/128;
        if(abs(joyY)<DEAD_ZONE)
        {
          joyY=0;
        } else
        {
          joyY = joyY;
          if (joyY<0)
          {
            joyX=0;
          }
        }
        button_cruise = WIFI_SERIAL.read();
        button_brake = WIFI_SERIAL.read();
        timer_remote = millis();
      }
    }
  }
  
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if( micros()-timer > dt_us)
  {
    timer += dt_us;
    if (ENABLE_PRINT_DEBUG)
    {
      Serial.print(joyY);Serial.print("\t");
      Serial.print(joyX);Serial.print("\t");
      Serial.print(button_brake);Serial.print("\t");

      Serial.println("\t");
    }
  }
}
