#include "bte_tricycle.h"
#include <WiiChuck.h>

Accessory nunchuck;

#define ENABLE_PRINT 1
#define ENABLE_PRINT_DEBUG 0

//// Definition des paramètres des la loi de contrôle

// flitrage entrées de commande
#define ALPHA_JOYSTICK 0.006
#define M_JOYSTICK 0.8
#define ALPHA_FALL_JOYSTICK_FRONT 0.02
#define ALPHA_FALL_JOYSTICK_LAT 0.1

// définition de la vitesse max
#define FRONT_SPEED_GAIN 115.0f
//120.0f 
// Vitesse max arrière par rapport à la vitesse avant (la vitesse peut être saturé par les controleurs, sas que l'on puisse le configurer)
#define REAR_VS_FRONT_GAIN 0.15f 
// Vitesse différentielle max par rapport à la vitesse avant
#define LAT_VS_FRONT_GAIN 0.3f 
// vitesse max en mode régulation de vitesse
#define CRUISE_SPEED_MAX 1.0f

// Définition des créneaux pour le freinage
#define BRAKE_LEVEL 8
#define BRAKE_LEVEL_MAX0 15
#define BRAKE_LEVEL_MAX1 19
#define BRAKE_LEVEL_MAX2 23

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

//// Définition des paramètres de la connexion série avec le BT

#define WIFI_SERIAL Serial1
#define BLE_START1 0x42
#define BLE_START2 0x24

//// Définition des variables ////

// Contrôleur moteur droit
int8_t acc_pin_R = 10;
int8_t brake_pin_R = 5;
int8_t reverse_pin_R = 4;
bte_tricycle_controller controlleur_R = bte_tricycle_controller(acc_pin_R, brake_pin_R, reverse_pin_R );

// controleur moteur gauche
int8_t acc_pin_L = 6;
int8_t brake_pin_L = 9;
int8_t reverse_pin_L = 7;
bte_tricycle_controller controlleur_L = bte_tricycle_controller(acc_pin_L, brake_pin_L, reverse_pin_L );

// Télécommande
uint8_t serial_data;

float joyX_f, joyX, joyY_f, joyY, joyY_tmp, joyX_f1, joyY_f1, joyX_f2, joyY_f2 = 0;
float constant_cruise_speed = 0.1f;
uint8_t button_cruise, button_brake;
int8_t last_button_cruise = 0;

// Définition des paramètres des filtres
#define ALPHA_JOYSTICK 0.006
#define M_JOYSTICK 0.8

float ky = 1/(1+2*M_JOYSTICK/ALPHA_JOYSTICK+(1/ALPHA_JOYSTICK)*(1/ALPHA_JOYSTICK));
float a1y = 2*M_JOYSTICK/ALPHA_JOYSTICK+2*(1/ALPHA_JOYSTICK)*(1/ALPHA_JOYSTICK);
float a2y = -(1/ALPHA_JOYSTICK)*(1/ALPHA_JOYSTICK);

float kx = 1/(1+2*M_JOYSTICK/(ALPHA_JOYSTICK/LAT_VS_FRONT_GAIN)+(LAT_VS_FRONT_GAIN/ALPHA_JOYSTICK)*(LAT_VS_FRONT_GAIN/ALPHA_JOYSTICK));
float a1x = 2*M_JOYSTICK/(ALPHA_JOYSTICK/LAT_VS_FRONT_GAIN)+2*(LAT_VS_FRONT_GAIN/ALPHA_JOYSTICK)*(LAT_VS_FRONT_GAIN/ALPHA_JOYSTICK);
float a2x = -(LAT_VS_FRONT_GAIN/ALPHA_JOYSTICK)*(LAT_VS_FRONT_GAIN/ALPHA_JOYSTICK);

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

  nunchuck.begin();
	if (nunchuck.type == Unknown) {
		nunchuck.type = NUNCHUCK;
	}

  //// Configuration des timers pour accélérer les PWM à 32kHz
/*
  // pins 9 & 10;
  TCCR1A = 0b00000001; // 8bit
  TCCR1B = 0b00000001; // 32kHz

  // Pin 5
  TCCR3A = 0b00000001; // 8bit
  TCCR3B = 0b00000001; // 32kHz

  // Pin 6
  TCCR4A = 0b00000001; // 8bit
  TCCR4B = 0b00000001; // 32kHz

*/
  //// configuration des contrôleur de moteur
  controlleur_R.set_motor_gain(FRONT_SPEED_GAIN);
  controlleur_L.set_motor_gain(FRONT_SPEED_GAIN);


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
  /*
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
        //Serial.print(x);Serial.print("\t");
        joyX = (((float)serial_data)-128)/128;
        if(abs(joyX)<DEAD_ZONE)
        {
          joyX=0;
        } else
        {
          joyX = joyX;
          /*
          if(joyX > 0)
          {
            joyX = joyX*joyX;
          } else
          {
            joyX = -joyX*joyX;
          }
        }
        serial_data = WIFI_SERIAL.read();
        //Serial.print(x);Serial.println("\t");
        joyY = (((float)serial_data)-128)/128;
        if(abs(joyY)<DEAD_ZONE)
        {
          joyY=0;
        } else
        {
          joyY = joyY;
          /*if(joyY > 0)
          {
            joyY = joyY*joyY;
          } else
          {
            joyY = -joyY*joyY;
          }
        }
        button_cruise = WIFI_SERIAL.read();
        button_brake = WIFI_SERIAL.read();
        timer_remote = millis();
      }
    }
  }
  */
  

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  //// Début de la routine de contrôle cadencée à 100Hz ////
  if( micros()-timer > dt_us)
  {
    timer += dt_us;

    nunchuck.readData();
    joyX = ((float)nunchuck.getJoyX()-128.0)/128.0;
    joyY = ((float)nunchuck.getJoyY()-128.0)/128.0;
    button_cruise = nunchuck.getButtonC();
    button_brake = nunchuck.getButtonZ();
    timer_remote = millis();

    //// Vérification du temps de la dernière info reçue de la télécommande, si trop vieux, on arrête tout ////
    time_since_since_last_data = (millis() - timer_remote);
    if( time_since_since_last_data > CONNEXION_LOST_TIME_MS) 
    {
      speed_ref_R = 0;
      front_speed_ref = 0;
      lateral_speed_ref = 0;
      speed_ref_L = 0;
      constant_cruise_flag = 0;
      joyX_f = 0;
      joyX_f2 = joyX_f;
      joyX_f1 = joyX_f;
      joyY_f = 0;
      joyY_f2 = joyY_f;
      joyY_f1 = joyY_f;
      controlleur_R.update_reference(speed_ref_R);
      controlleur_L.update_reference(speed_ref_L);
      //controlleur_R.brake(PARKING_BRAKE_LEVEL_PWM);
      //controlleur_L.brake(PARKING_BRAKE_LEVEL_PWM);
      connexion_flag = 0;
    } else
    {
      connexion_flag = 1;
    }

      ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

      //////// Filtrage des commandes pour assouplir les accélérations ////////
      
      joyX_f2 = joyX_f1;
      joyX_f1 = joyX_f;
      //// Axe gauche/droite ////
      if ( (joyX > DEAD_ZONE) && (joyX > joyX_f ) )
      {
        joyX_f = kx*(joyX + a1x*joyX_f1 + a2x*joyX_f2);       
      } else if ( (joyX < -DEAD_ZONE) && (joyX < joyX_f ) )
      {
        joyX_f = kx*(joyX + a1x*joyX_f1 + a2x*joyX_f2);
      } else
      {
        if(constant_cruise_flag == 0 )
        {
          joyX_f = ALPHA_FALL_JOYSTICK_LAT*joyX + (1-ALPHA_FALL_JOYSTICK_LAT)*joyX_f1;
        } else
        {
          joyX_f = joyX;
        }
        if (abs(joyX_f)<DEAD_ZONE)
        {
          joyX_f = 0;
        }
        joyX_f2 = joyX_f;
        joyX_f1 = joyX_f;
      }
      joyX_f = min(joyX_f,1);
      joyX_f = max(joyX_f,-1);

      if(constant_cruise_flag==1)
      {
        joyY_tmp = 1.0f;// constant_cruise_speed;
      } else
      {
        joyY_tmp = joyY;
      }

      joyY_f2 = joyY_f1;
      joyY_f1 = joyY_f;

      //// AYe avant/arrière ////
      if ( (joyY_tmp > DEAD_ZONE) && (joyY_tmp > joyY_f ) )
      {
        joyY_f = ky*(joyY_tmp + a1y*joyY_f1 + a2y*joyY_f2);
        
        if(constant_cruise_flag==1)
        {
          joyY_f = min(joyY_f,constant_cruise_speed);
        } else
        {
          joyY_f = min(joyY_f,1);
          joyY_f = max(joyY_f,-REAR_VS_FRONT_GAIN);
        }
      } else if ( (joyY_tmp < -DEAD_ZONE) && (joyY_tmp < joyY_f ) )
      {
        joyY_f = ky*(joyY_tmp + a1y*joyY_f1 + a2y*joyY_f2);
        joyY_f = max(joyY_f,-REAR_VS_FRONT_GAIN);
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
        if(constant_cruise_flag==1)
        {
          joyY_f = min(joyY_f,constant_cruise_speed);
        } else
        {
          joyY_f = min(joyY_f,1);
          joyY_f = max(joyY_f,-REAR_VS_FRONT_GAIN);
        }
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
      if(button_brake == 0 && connexion_flag == 1)
      {
        brake_flagR = 0;
        brake_flagL = 0;
        timer_brake = millis();
        
        if(constant_cruise_flag==1)
        {
          if(joyX>0.25)
          {
            brake_flagR = 1;
            if(front_speed_ref<0.33f)
            {
              brake_level_max = BRAKE_LEVEL_MAX0;
            } else if (front_speed_ref<0.66f)
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
          if(joyX<-0.25)
          {
            brake_flagL = 1;
            if(front_speed_ref<0.33f)
            {
              brake_level_max = BRAKE_LEVEL_MAX0;
            } else if (front_speed_ref<0.66f)
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

          if(joyY>0.5)
          {
            constant_cruise_speed += 0.0015;
            constant_cruise_speed = min(constant_cruise_speed,CRUISE_SPEED_MAX);
          }
          if(joyY<-0.5)
          {
            constant_cruise_speed -= 0.002;
            constant_cruise_speed = max(0,constant_cruise_speed);
          }
        }
      } else
      {
        if(last_button_brake==0)
        {
          brake_time = front_speed_ref*front_speed_ref*1000 + 2000;
        } else if (connexion_flag == 0)
        {
          brake_time = 3000;
        }
        brake_flagR = 1;
        brake_flagL = 1;
        brake_level = BRAKE_LEVEL;
        time_tmp = millis();
        if( ((time_tmp - timer_brake) < (brake_time-2250)) && (brake_time>2250) )
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
      if( abs(joyY_f)>0.005 || constant_cruise_flag==1 )
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

      //////// Gestion du régulateur de vitesse ////////

      //// Désenclanchement du cruise control si on freine ////
      if( button_brake == 1 )
      {
        constant_cruise_flag = 0;
      }

      //// Enclanchement du régulateur de vitesse, ou désenclanchement si il était activé ////
      if( (last_button_cruise == 0 && button_cruise == 1) )
      {
        if ( constant_cruise_flag == 0 )
        {
          constant_cruise_flag = 1;
          constant_cruise_speed = min(constant_cruise_speed,0.75);
        } else
        {
          constant_cruise_flag = 0;
        }
      }
      last_button_cruise = button_cruise;


      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


      //////// Calcul des vitesses frontale et différentielles et mixages pour envoie des commande aux moteurs droit et gauche ////////
      
      //// Vitesse frontale ////

      if(joyY_f>0) // marche avant
      {
        front_speed_ref = joyY_f;
      } else // marche arrière on applique le REAR_VS_FRONT_GAIN sur la vitesse
      {
        front_speed_ref = max(-REAR_VS_FRONT_GAIN,joyY_f);
      }

      //// Vitesse diffèrentielle ////
      lateral_speed_ref = (1+0.5*abs(front_speed_ref))*LAT_VS_FRONT_GAIN*joyX_f;

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
          if(constant_cruise_flag==0)
          {
            if(lateral_speed_ref>=0)
            {
              speed_ref_R = max(0,front_speed_ref - lateral_speed_ref);
              d_lateral_speed_ref = speed_ref_R-(front_speed_ref - lateral_speed_ref);
              speed_ref_L = front_speed_ref + d_lateral_speed_ref;
              speed_ref_L = max(0,speed_ref_L);
            } else
            {
              speed_ref_L = max(0,front_speed_ref + lateral_speed_ref);
              d_lateral_speed_ref = speed_ref_L-(front_speed_ref + lateral_speed_ref);
              speed_ref_R = front_speed_ref + d_lateral_speed_ref;
              speed_ref_R = max(0,speed_ref_R);
            }
          } else
          {
            if(lateral_speed_ref>=0)
            {
              speed_ref_R = max(0,front_speed_ref - lateral_speed_ref);
              d_lateral_speed_ref = speed_ref_R-(front_speed_ref - lateral_speed_ref);
              speed_ref_L = front_speed_ref + d_lateral_speed_ref;
              speed_ref_L = max(0,speed_ref_L);
            } else
            {
              speed_ref_L = max(0,front_speed_ref + lateral_speed_ref);
              d_lateral_speed_ref = speed_ref_L-(front_speed_ref + lateral_speed_ref);
              speed_ref_R = front_speed_ref + d_lateral_speed_ref;
              speed_ref_R = max(0,speed_ref_R);
            }
          }
        } else if (front_speed_ref<0)
        {
          if(lateral_speed_ref<0 )
          {
            speed_ref_R = min(0,front_speed_ref - lateral_speed_ref);
            speed_ref_L = front_speed_ref;
            speed_ref_L = min(0,speed_ref_L);
          } else
          {
            speed_ref_R = front_speed_ref - 0.0*lateral_speed_ref;
            speed_ref_L = min(0,front_speed_ref + lateral_speed_ref);
            speed_ref_R = min(0,speed_ref_R);
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
      Serial.print(button_cruise);Serial.print("\t");
      Serial.print(button_brake);Serial.print("\t");
      Serial.print(joyY_f);Serial.print("\t");
      Serial.print(joyX_f);Serial.print("\t");

      Serial.print("Flags:");Serial.print("\t");
      Serial.print(constant_cruise_flag);Serial.print("\t");
      Serial.print(brake_flagL);Serial.print("\t");
      Serial.print(brake_flagR);Serial.print("\t");
      Serial.print(parking_brake_flag);Serial.print("\t");
      Serial.print(connexion_flag);Serial.print("\t");

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
      Serial.print(joyY);Serial.print("\t");
      Serial.print(joyY_f);Serial.print("\t");
      Serial.print(front_speed_ref);Serial.print("\t");
      Serial.print(constant_cruise_speed);Serial.print("\t");
      Serial.print(speed_ref_L);Serial.print("\t");
      Serial.print(speed_ref_R);
      Serial.println("\t");
    }
  }
}
