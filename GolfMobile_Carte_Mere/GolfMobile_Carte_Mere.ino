#include "bte_tricycle.h"

/*
// Usefull when using the nunchuck
#include <WiiChuck.h>

Accessory nunchuck;
*/

#define ENABLE_PRINT 1
#define ENABLE_PRINT_DEBUG 0

//// Definition des paramètres des la loi de contrôle

// flitrage entrées de commande
#define ALPHA_JOYSTICK_FRONT 0.003
#define M_JOYSTICK 0.8
#define ALPHA_FALL_JOYSTICK_FRONT 0.05
#define ALPHA_JOYSTICK_LAT 0.05

// définition de la vitesse max
#define FRONT_SPEED_GAIN 115.0f
//120.0f 
// Vitesse max arrière par rapport à la vitesse avant (la vitesse peut être saturé par les controleurs, sans que l'on puisse le configurer)
#define REAR_VS_FRONT_GAIN 0.05f 
// Vitesse différentielle max par rapport à la vitesse avant
#define LAT_VS_FRONT_GAIN 0.2f 
// vitesse max en mode régulation de vitesse
#define CRUISE_SPEED_MAX 1.0f

// Définition des créneaux pour le freinage
#define BRAKE_LEVEL 8
#define BRAKE_LEVEL_MAX0 16
#define BRAKE_LEVEL_MAX1 19
#define BRAKE_LEVEL_MAX2 21

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
#define DEAD_ZONE_X 0.1f 

//// Définition des paramètres de la connexion série avec le BT

#define WIFI_SERIAL Serial1
#define BLE_START1 0x42
#define BLE_START2 0x24

//// Définition des variables ////

// Contrôleur moteur droit
int8_t acc_pin_R = 9;
int8_t brake_pin_R = 10;
int8_t reverse_pin_R = 16;
bte_tricycle_controller controlleur_R = bte_tricycle_controller(acc_pin_R, brake_pin_R, reverse_pin_R );


// controleur moteur gauche
int8_t acc_pin_L = 5;
int8_t brake_pin_L = 6;
int8_t reverse_pin_L = 4;
bte_tricycle_controller controlleur_L = bte_tricycle_controller(acc_pin_L, brake_pin_L, reverse_pin_L );

// Accélérateur poignée
uint8_t acc_pin = 21;
uint16_t acc_value;
float acc_value_flt;
float acc_min = 190;
float acc_max = 850;

// Télécommande
uint8_t serial_data;
uint8_t button_cruise, button_brake;

// Définition des paramètres des filtres

float joyX_f, joyX, joyY_f, joyY, joyY_tmp, joyX_f1, joyY_f1, joyX_f2, joyY_f2 = 0;

float ky = 1/(1+2*M_JOYSTICK/ALPHA_JOYSTICK_FRONT+(1/ALPHA_JOYSTICK_FRONT)*(1/ALPHA_JOYSTICK_FRONT));
float a1y = 2*M_JOYSTICK/ALPHA_JOYSTICK_FRONT+2*(1/ALPHA_JOYSTICK_FRONT)*(1/ALPHA_JOYSTICK_FRONT);
float a2y = -(1/ALPHA_JOYSTICK_FRONT)*(1/ALPHA_JOYSTICK_FRONT);

// Variables de contrôle
float front_speed_ref = 0.0f;
float lateral_speed_ref = 0.0f;
float speed_ref_R = 0.0f;
float speed_ref_L = 0.0f;
float d_lateral_speed_ref = 0.0f;

// flags
int8_t brake_flagR = 0;
int8_t brake_flagL = 0;
int8_t last_brake_flagR = 0;
int8_t last_brake_flagL = 0;
int8_t connexion_flag = 0;
int8_t parking_brake_flag = 1;

// timers
uint32_t timer, timer_parking, timer_remote, timer_speed,timer_brake;
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

/*
  // Usefull when using the nunchuck
  nunchuck.begin();
	if (nunchuck.type == Unknown) {
		nunchuck.type = NUNCHUCK;
	}
*/

  //// Configuration des timers pour accélérer les PWM à 32kHz

  // pins 9 & 10;
  TCCR1A = 0b00000001; // 8bit
  TCCR1B = 0b00000001; // 32kHz

  // Pin 5
  TCCR3A = 0b00000001; // 8bit
  TCCR3B = 0b00000001; // 32kHz

  // Pin 6
  TCCR4A = 0b00000001; // 8bit
  TCCR4B = 0b00000001; // 32kHz


  //// configuration des contrôleur de moteur
  controlleur_R.set_motor_gain(FRONT_SPEED_GAIN);
  controlleur_L.set_motor_gain(FRONT_SPEED_GAIN);

  pinMode(acc_pin,INPUT);

  //// initialisation des timers
  timer = micros();
  timer_speed = micros();
  timer_brake = millis();
  timer_remote = millis();
  timer_parking = millis();
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
        //Serial.print(joyX);Serial.println("\t");
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
        //Serial.print(joyY);Serial.println("\t");
        if(abs(joyY)<DEAD_ZONE)
        {
          joyY=0;
        } else
        {
          joyY = min(joyY,0.75);
          if (joyY<-0.50)
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
  

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  //// Début de la routine de contrôle cadencée à 100Hz ////
  if( micros()-timer > dt_us)
  {
    timer += dt_us;
    
    /*
    // Usefull when using the nunchuck
    nunchuck.readData();
    joyX = ((float)nunchuck.getJoyX()-128.0)/128.0;
    joyY = ((float)nunchuck.getJoyY()-128.0)/128.0;
    button_cruise = nunchuck.getButtonC();
    button_brake = nunchuck.getButtonZ();
    timer_remote = millis();
    */

    //// Vérification du temps de la dernière info reçue de la télécommande, si trop vieux, on arrête tout ////
    time_since_since_last_data = (millis() - timer_remote);
    if( time_since_since_last_data > CONNEXION_LOST_TIME_MS && acc_value_flt < 0) 
    {
      joyX = 0.0f;
      joyY = 0.0f;
      connexion_flag = 0;
    } else
    {
      connexion_flag = 1;
    }

    acc_value = analogRead(acc_pin);
    acc_value_flt = min(((float)acc_value-acc_min)/(acc_max-acc_min),1.0);
    if(acc_value_flt>0)
    {
      joyX = 0.0f;
      joyY = acc_value_flt;
    }
    button_brake = 0;

      ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

      //////// Filtrage des commandes pour assouplir les accélérations ////////
      
      joyX_f1 = joyX_f;
      //// Axe gauche/droite ////
      if (abs(joyX_f)<DEAD_ZONE)
      {
        joyX_f = 0;
      } else
      {
        joyX_f = ALPHA_JOYSTICK_LAT*joyX + (1-ALPHA_JOYSTICK_LAT)*joyX_f1; 
      }
      joyX_f = min(joyX_f,1);
      joyX_f = max(joyX_f,-1);

      //// AYe avant/arrière ////
      joyY_tmp = joyY;
      joyY_f2 = joyY_f1;
      joyY_f1 = joyY_f;

      if ( (joyY_tmp > DEAD_ZONE) && (joyY_tmp > joyY_f ) )
      {
        joyY_f = ky*(joyY_tmp + a1y*joyY_f1 + a2y*joyY_f2);
        joyY_f = min(joyY_f,1);
        joyY_f = max(joyY_f,-REAR_VS_FRONT_GAIN);
      } else if ( (joyY_tmp < -DEAD_ZONE) )
      {
        joyY_f = 0;
        if ( (joyY_tmp < -0.5f) )
        {
          button_brake = 1;
        } else
        {
          button_brake = 0;
        }
      } else
      {
        if(joyY_f>0)
        {
          joyY_tmp = max(joyY_tmp,0);
        } else if(joyY_f<0)
        {
          joyY_tmp = min(joyY_tmp,0);
        } 
        joyY_f = ALPHA_FALL_JOYSTICK_FRONT*joyY_tmp + (1-ALPHA_FALL_JOYSTICK_FRONT)*joyY_f;
        if (abs(joyY_f)<DEAD_ZONE)
        {
          joyY_f = 0;
        }

        joyY_f = min(joyY_f,1);
        joyY_f = max(joyY_f,-REAR_VS_FRONT_GAIN);
        joyY_f2 = joyY_f;
        joyY_f1 = joyY_f;
      }


      ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

      //////// Gestion du frein et du frein de parking ////////

      //// Gestion du compteur pour le freinage 'ABS' ///
      brake_counter ++;
      if(brake_counter>brake_level_max-1 || ( (last_brake_flagR == 0 && last_brake_flagL == 0) && (brake_flagR == 1 || brake_flagL == 1)))
      {
        brake_counter = 0;
      }
      last_brake_flagR = brake_flagR;
      last_brake_flagL = brake_flagL;

      //// Lecture du bouton de frein ///
      if(button_brake == 0 )//&& connexion_flag == 1)
      {
        brake_flagR = 0;
        brake_flagL = 0;
        timer_brake = millis();
        
        
        if(joyX_f>0.1)
        {
          brake_flagR = 1;
          if(front_speed_ref+abs(lateral_speed_ref)<0.5f)
          {
            brake_level_max = BRAKE_LEVEL_MAX1;
          } else
          {
            brake_level_max = BRAKE_LEVEL_MAX2;
          } 
        } else
        {
          brake_flagR = 0;
        }
        if(joyX_f<-0.1)
        {
          brake_flagL = 1;
          if(front_speed_ref+abs(lateral_speed_ref)<0.5f)
          {
            brake_level_max = BRAKE_LEVEL_MAX1;
          } else
          {
            brake_level_max = BRAKE_LEVEL_MAX2;
          }
        } else
        {
          brake_flagL = 0;
        }
        
      } else if (parking_brake_flag==0)
      {
        if(last_button_brake==0)
        {
          brake_time = abs(front_speed_ref)*BRAKE_TIME_PROP + BRAKE_TIME_CSTE;
        }
        brake_flagR = 1;
        brake_flagL = 1;
        brake_level = BRAKE_LEVEL;
        time_tmp = millis();
        if( ((time_tmp - timer_brake) < (brake_time-BRAKE_LEVEL2_TIME)) && (brake_time>BRAKE_LEVEL2_TIME) )
        {
          brake_level_max = BRAKE_LEVEL_MAX2;
        } else if( ((time_tmp - timer_brake) < (brake_time-BRAKE_LEVEL1_TIME)) && (brake_time>BRAKE_LEVEL1_TIME) )
        {
          brake_level_max = BRAKE_LEVEL_MAX1;
        } else if((time_tmp - timer_brake) < brake_time )
        {
          brake_level_max = BRAKE_LEVEL_MAX0;
        } else
        {
          brake_level_max = 1;
          parking_brake_flag = 1;
        }
      }
      last_button_brake = button_brake;


      //// Freinage roue droite ///
      if(brake_flagR == 0 )
      {
        controlleur_R.brake(0);
      } else
      {
        if(brake_counter<brake_level)
        {
          controlleur_R.brake(BRAKE_LEVEL_PWM);
        } else
        {
          controlleur_R.brake(0);
        }
      }

      //// Freinage roue gauche ///
      if(brake_flagL == 0 )
      {
        controlleur_L.brake(0);
      } else
      {
        if(brake_counter<brake_level)
        {
          controlleur_L.brake(BRAKE_LEVEL_PWM);
        } else
        {
          controlleur_L.brake(0);
        }
      }

      //// On réinitialise le compteur d'inactivité si le joystick n'est pas en position centrale
      if( abs(joyY_f)>0.005 || abs(joyX_f)>0.005 )
      {
        timer_parking = millis();
        parking_brake_flag = 0;
      }

      if(parking_brake_flag)
      {
        controlleur_R.brake(PARKING_BRAKE_LEVEL_PWM);
        controlleur_L.brake(PARKING_BRAKE_LEVEL_PWM);
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


      //////// Calcul des vitesses frontale et différentielles et mixages pour envoie des commande aux moteurs droit et gauche ////////
      
      //// Vitesse frontale ////

      if(joyY_f>0) // marche avant
      {
        if(joyY<=0) // si le joystick est complétement relaché, on met la consigne frontale à 0.
        {
            front_speed_ref = 0;
        } else 
        {
          front_speed_ref = joyY_f;
        }
      } else // marche arrière on sature la vitesse à REAR_VS_FRONT_GAIN.
      {
        front_speed_ref = max(-REAR_VS_FRONT_GAIN,joyY_f);
      }

      //// Vitesse diffèrentielle ////
      // désactivé : //on utilise une vitesse différentielle plus grande quand on a une consigne frontale plus grande.
      lateral_speed_ref = LAT_VS_FRONT_GAIN*joyX_f; //(1+0.5*abs(front_speed_ref))*LAT_VS_FRONT_GAIN*joyX_f;

      //// Mixage des vitesse différentielle et frontale pour calculer les consignes du moteur droit et gauche ////
      if(button_brake == 1) // Si le frein est actif on met les consignes de vitesse à 0
      {
        speed_ref_R = 0;
        speed_ref_L = 0;

        // Remise à 0 des filtres pour avoir une reprise progressive de l'accélération après freinage (si l'utilisateur reste accéléré en freinant)
        joyY_f = 0;
        joyY_f2 = 0;
        joyY_f1 = 0;
        joyX_f = 0;
        joyX_f2 = 0;
        joyX_f1 = 0;
      } else
      {
        if(front_speed_ref>=0)
        {
          if(lateral_speed_ref>=0)
          {
            lateral_speed_ref = min(lateral_speed_ref,front_speed_ref+0.1);
            speed_ref_R = max(0,front_speed_ref - lateral_speed_ref);
            d_lateral_speed_ref = speed_ref_R-(front_speed_ref - lateral_speed_ref);
            speed_ref_L = front_speed_ref + d_lateral_speed_ref;
            speed_ref_L = max(0,speed_ref_L);
          } else
          {
            lateral_speed_ref = max(lateral_speed_ref,-front_speed_ref-0.1);
            speed_ref_L = max(0,front_speed_ref + lateral_speed_ref);
            d_lateral_speed_ref = speed_ref_L-(front_speed_ref + lateral_speed_ref);
            speed_ref_R = front_speed_ref + d_lateral_speed_ref;
            speed_ref_R = max(0,speed_ref_R);
          }
        }
      }

      //// Envoie des commandes aux controleurs de moteur ////
      controlleur_R.update_reference(speed_ref_R);
      controlleur_L.update_reference(speed_ref_L);

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    if (ENABLE_PRINT)
    {
      Serial.print(time_since_since_last_data);Serial.print("\t");
      Serial.print("Remote:");Serial.print("\t");
      Serial.print(joyY);Serial.print("\t");
      Serial.print(joyX);Serial.print("\t");
      //Serial.print(button_cruise);Serial.print("\t");
      //Serial.print(button_brake);Serial.print("\t");
      Serial.print(joyY_f);Serial.print("\t");
      Serial.print(joyX_f);Serial.print("\t");
      Serial.print(brake_level_max);Serial.print("\t");

      Serial.print("Flags:");Serial.print("\t");
      Serial.print(button_brake);Serial.print("\t");
      Serial.print(brake_flagL);Serial.print("\t");
      Serial.print(brake_flagR);Serial.print("\t");
      //Serial.print(parking_brake_flag);Serial.print("\t");
      //Serial.print(connexion_flag);Serial.print("\t");

      Serial.print("Vitesses:");Serial.print("\t");
      Serial.print(front_speed_ref);Serial.print("\t");
      Serial.print(lateral_speed_ref);Serial.print("\t");

      Serial.print("Mixage:");Serial.print("\t");
      Serial.print(speed_ref_L);Serial.print("\t");
      Serial.print(speed_ref_R);

      Serial.println("\t");
    }

    if (ENABLE_PRINT_DEBUG)
    {
      Serial.print(acc_value);Serial.print("\t");
      Serial.print(acc_value_flt);Serial.print("\t");
      Serial.print(button_brake);Serial.print("\t");
      Serial.print(brake_flagL);Serial.print("\t");
      Serial.print(brake_flagR);Serial.print("\t");
      Serial.print(joyY);Serial.print("\t");
      Serial.print(joyY_f);Serial.print("\t");
      Serial.print(joyX);Serial.print("\t");
      Serial.print(joyX_f);Serial.print("\t");
      Serial.print(front_speed_ref);Serial.print("\t");
      Serial.print(lateral_speed_ref);Serial.print("\t");
      Serial.print(speed_ref_L);Serial.print("\t");
      Serial.print(speed_ref_R);
      Serial.println("\t");
    }
  }
}
