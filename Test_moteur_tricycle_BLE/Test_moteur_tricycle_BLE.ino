#include "bte_tricycle.h"
#include <WiiChuck.h>

#define ENABLE_PRINT 1

//// Definition des paramètres des la loi de contrôle

// Temps de montée de la commande de vitesse frontale (0-100%)
#define RISING_TIME_FRONT_S 5.0 
// Temps de montée de la commande de vitesse différentielle (0-100%)
#define RISING_TIME_LAT_S 1.0 

// définition de la vitesse max
#define FRONT_SPEED_GAIN 110.0f 
// Vitesse max arrière par rapport à la vitesse avant (la vitesse peut être saturé par les controleurs, sas que l'on puisse le configurer)
#define REAR_VS_FRONT_GAIN 0.25f 
// Vitesse différentielle max par rapport à la vitesse avant
#define LAT_VS_FRONT_GAIN 0.2f 

// niveau de freinage (0-10)
#define BRAKE_LEVEL 8
uint8_t brake_level = BRAKE_LEVEL;
#define BRAKE_LEVEL_MAX 15 
uint8_t brake_level_max = BRAKE_LEVEL_MAX;

// Temps d'inactivité avant enclanchement du frein de parking
#define PARKING_BRAKE_TIME_MS 1000 
// Temps de perte de connexion
#define CONNEXION_LOST_TIME_MS 500 
// Temps maximal d'enclanchement du régulateur de vitesse
#define CRUISE_CONTROL_TIME 20000 

// Largeur de la zone morte normalisée (utilisé pour désenclancher le régulateur de vitesse)
#define DEAD_ZONE 0.05f 
#define SPEED_SIGNAL_FILTER_SIZE 3
#define SPEED_FILTER_SIZE 10

//// Définition des paramètres de la connexion série avec le BT

#define BLE_SERIAL Serial
#define BLE_START1 0x42
#define BLE_START2 0x24

//// Définition des variables ////

// Contrôleur moteur droit
int8_t acc_pin_R = 14;
int8_t brake_pin_R = 5;
int8_t reverse_pin_R = 4;
bte_tricycle_controller controlleur_R = bte_tricycle_controller(acc_pin_R, brake_pin_R, reverse_pin_R );

// controleur moteur gauche
int8_t acc_pin_L = 15;
int8_t brake_pin_L = 3;
int8_t reverse_pin_L = 2;
bte_tricycle_controller controlleur_L = bte_tricycle_controller(acc_pin_L, brake_pin_L, reverse_pin_L );

// Télécommande
int8_t serial_data;

float joyX_f, joyX, joyY_f, joyY = 0;
float joyY_f_lock = 0;
uint8_t button_cruise, button_brake;
int8_t last_button_cruise = 0;

float delta_joyY_max = 1.0/(RISING_TIME_FRONT_S*100);
float delta_joyX_max = 1.0/(RISING_TIME_LAT_S*100);

// Variables de contrôle
float front_speed_ref = 0.0f;
float lateral_speed_ref = 0.0f;
float speed_ref_R = 0.0f;
float speed_ref_L = 0.0f;
float d_speed_ref_R = 0.0f;
float d_speed_ref_L = 0.0f;

// flags
int8_t constant_cruise_flag = 0;
int8_t brake_flagR = 0;
int8_t brake_flagL = 0;
int8_t last_brake_flagR = 0;
int8_t last_brake_flagL = 0;
int8_t connexion_flag = 0;
int8_t parking_brake_flag = 1;

// lecture vitesse
uint8_t speed_signal_buffer[SPEED_SIGNAL_FILTER_SIZE];
float speed_buffer[SPEED_FILTER_SIZE];
float speed_signal;
uint8_t speed_signal_counter = 0;
uint8_t speed_counter = 0;
int8_t speed_signal_flag = 0;
uint32_t timer_speed_measure;
float speed;


// timers
uint32_t timer, timer_parking, timer_remote, timer_cruise_control, timer_speed,timer_brake;
uint32_t dt_us = 10000;
uint32_t dt_speed_us = 1000;

uint8_t brake_counter = 0;

int32_t i,j;

void setup() {

  // Configuration des ports série
  Serial.begin(115200);
  BLE_SERIAL.begin(115200);

  pinMode(19,INPUT);

  // configuration des contrôleur de moteur
  controlleur_R.set_motor_gain(FRONT_SPEED_GAIN);
  controlleur_L.set_motor_gain(FRONT_SPEED_GAIN);

  // initialisation des timers
  timer = micros();
  timer_speed = micros();
  timer_speed_measure = micros();
  timer_brake = millis();
  timer_remote = millis();
  timer_parking = millis();
  timer_cruise_control = millis();

  Serial.println("Setup done");
}

void loop() {
  uint8_t x;
  //// Lecture des commandes de la télécommande via le BT ////
  if(BLE_SERIAL.available()>5)
  {
    //Serial.println("hfhf");
    serial_data = BLE_SERIAL.read();
    if(serial_data == BLE_START1)
    {
      serial_data = BLE_SERIAL.read();
      if(serial_data == BLE_START2)
      {
        x = BLE_SERIAL.read();
        //Serial.print(x);Serial.print("\t");
        joyX = (((float)x)-128)/128;
        if(abs(joyX)<DEAD_ZONE)
        {
          joyX=0;
        } else
        {
          joyX = joyX*joyX*joyX;
        }
        x = BLE_SERIAL.read();
        //Serial.print(x);Serial.println("\t");
        /*joyY = (((float)x)-128)/128;
        if(abs(joyY)<DEAD_ZONE)
        {
          joyY=0;
        } else
        {
          joyY = joyY*joyY*joyY;
        }*/
        button_cruise = BLE_SERIAL.read();
        button_brake = BLE_SERIAL.read();
        timer_remote = millis();
      }
    }
  }
/*
  //// Début de la routine de contrôle cadencée à 1000Hz ////
  if( micros()-timer_speed > dt_speed_us)
  {
    
    timer_speed += dt_speed_us;

    speed_signal_buffer[speed_signal_counter] = (uint8_t)analogRead(19);
    speed_signal_counter++;
    if(speed_signal_counter>=SPEED_SIGNAL_FILTER_SIZE)
    {
      speed_signal_counter = 0;
    }

    speed_signal = 0;
    for(i=0;i<SPEED_SIGNAL_FILTER_SIZE;i++)
    {
      speed_signal += (float)speed_signal_buffer[i]/SPEED_SIGNAL_FILTER_SIZE;
    }

    if( speed_signal_flag == 0 && speed_signal>64)
    {
      speed_signal_flag = 1;

      speed_buffer[speed_counter] = 1000000.0/(float)(micros()-timer_speed_measure);
      timer_speed_measure = micros();
      speed_counter++;
      if(speed_counter>=SPEED_FILTER_SIZE)
      {
        speed_counter = 0;
      }

      speed = 0;
      for(i=0;i<SPEED_FILTER_SIZE;i++)
      {
        speed += speed_buffer[i]/SPEED_FILTER_SIZE;
      }

      Serial.println(speed);
    }

    if( speed_signal_flag == 1 && speed_signal<64)
    {
      speed_signal_flag = 0;

      speed_buffer[speed_counter] = 1000000.0/(float)(micros()-timer_speed_measure);
      timer_speed_measure = micros();
      speed_counter++;
      if(speed_counter>=SPEED_FILTER_SIZE)
      {
        speed_counter = 0;
      }

      speed = 0;
      for(i=0;i<SPEED_FILTER_SIZE;i++)
      {
        speed += speed_buffer[i]/SPEED_FILTER_SIZE;
      }
 
      Serial.println(speed);
    } 

    

    
  }
*/
  //// Début de la routine de contrôle cadencée à 100Hz ////
  if( micros()-timer > dt_us)
  {
    timer += dt_us;

    //// Vérification du temps de la dernière info reçue de la télécommande, si trop vieux, on arrête tout ////
    if( (millis() - timer_remote) > CONNEXION_LOST_TIME_MS) 
    {
      speed_ref_R = 0;
      front_speed_ref = 0;
      lateral_speed_ref = 0;
      speed_ref_L = 0;
      parking_brake_flag = 1;
      constant_cruise_flag = 0;
      joyX_f = 0;
      joyY_f = 0;
      controlleur_R.update_reference(speed_ref_R);
      controlleur_L.update_reference(speed_ref_L);
      controlleur_R.brake(255);
      controlleur_L.brake(255);
      connexion_flag = 0;
    } else
    {

      connexion_flag = 1;

      //////// Filtrage des commandes pour assouplir les accélérations ////////
      float alpha1 = 0.02;
      //// Axe gauche/droite ////
      if( (joyX_f > -DEAD_ZONE) && (joyX > joyX_f ) )
      {
        joyX_f = alpha1*joyX + (1-alpha1)*joyX_f;
      } else if ( (joyX_f < DEAD_ZONE) && (joyX < joyX_f ) )
      {
        joyX_f = alpha1*joyX + (1-alpha1)*joyX_f;
      } else
      {
        joyX_f = 2*alpha1*joyX + (1-2*alpha1)*joyX_f;
        if (abs(joyX_f)<0.05)
        {
          joyX_f = 0;
        }
      }
/*
      if( (joyX_f > -DEAD_ZONE) && (joyX > (joyX_f + delta_joyX_max)) )
      {
        joyX_f += delta_joyX_max;
      } else if ( (joyX_f < DEAD_ZONE) && (joyX < (joyX_f - delta_joyX_max)) )
      {
        joyX_f -= delta_joyX_max;
      } else
      {
        joyX_f = joyX;
      }
*/
      //// Enclanchement du régulateur de vitesse ////
      if( (last_button_cruise == 0 && button_cruise == 1) )
      {
        if ( (constant_cruise_flag == 0) )
        {
          constant_cruise_flag = 1;
          joyY = 0.3;
          timer_cruise_control = millis();
        } else
        {
          timer_cruise_control = millis();
        }
      }
      last_button_cruise = button_cruise;

      float alpha2 = 0.01;
      //// AYe avant/arrière ////
      if( (joyY_f > -DEAD_ZONE) && (joyY > joyY_f ) )
      {
        joyY_f = alpha2*joyY + (1-alpha2)*joyY_f;
      } else if ( (joyY_f < DEAD_ZONE) && (joyY < joyY_f ) )
      {
        joyY_f = 2*alpha2*joyY + (1-2*alpha2)*joyY_f;
      } else if(abs(joyX_f)<0.2)
      {
        joyY_f = 4*alpha2*joyY + (1-4*alpha2)*joyY_f;
        if (abs(joyY_f)<0.05)
        {
          joyY_f = 0;
        }
      }
/*
      //// Axe avant/arrière ////
      if( (joyY_f > -DEAD_ZONE) && (joyY > (joyY_f + delta_joyY_max)) )
      {
        joyY_f += delta_joyY_max;
      } else if ( (joyY_f < DEAD_ZONE) && (joyY < (joyY_f - delta_joyY_max)) )
      {
        joyY_f -= delta_joyY_max;
      } else
      {
        joyY_f = joyY;
      }
*/
      ///////////////////////////////////////////////////////////////////////


      //////// Gestion du frein et du frein de parking ////////

      //// Lecture du bouton de frein ///
      brake_counter ++;
      if(brake_counter>brake_level_max-1 || ( (last_brake_flagR == 0 && last_brake_flagL == 0) && (brake_flagR == 1 || brake_flagL == 1)))
      {
        brake_counter = 0;
      }
      last_brake_flagR = brake_flagR;
      last_brake_flagL = brake_flagL;

      if(button_brake == 0 )
      {
        brake_flagR = 0;
        brake_flagL = 0;
        timer_brake = millis();
      } else
      {
        brake_flagR = 1;
        brake_flagL = 1;
        brake_level = BRAKE_LEVEL;
        if((millis() - timer_brake) <1000 )
        {
          brake_level_max = BRAKE_LEVEL_MAX;
        } else
        {
          brake_level_max = 1;
        }
      }

      if( (joyX>0.01 && speed_ref_R>=0) )
      {
        brake_flagR = 1;
        brake_level = 1*BRAKE_LEVEL;
        brake_level_max = (1*BRAKE_LEVEL_MAX + (1-abs(joyX))*1*BRAKE_LEVEL_MAX);
      } 
      if( (joyX<-0.01 && speed_ref_L>=0) )
      {
        brake_flagL = 1;
        brake_level = 1*BRAKE_LEVEL;
        brake_level_max = (1*BRAKE_LEVEL_MAX + (1-abs(joyX))*1*BRAKE_LEVEL_MAX);
      }

      if(brake_flagR == 0 )
      {
        controlleur_R.brake(0);
      } else
      {
        if(brake_counter<brake_level)
        {
          controlleur_R.brake(255);
        } else
        {
          controlleur_R.brake(0);
        }
      }

      if(brake_flagL == 0 )
      {
        controlleur_L.brake(0);
      } else
      {
        if(brake_counter<brake_level)
        {
          controlleur_L.brake(255);
        } else
        {
          controlleur_L.brake(0);
        }
      }

      //// On réinitialise le compteur d'inactivité sin le joystick n'est pas en position centrale
      if( abs(joyY_f)>DEAD_ZONE || abs(joyX_f)>DEAD_ZONE || constant_cruise_flag==1 )
      {
        timer_parking = millis();
        parking_brake_flag = 0;
      }

      //// Si l'inactivité dure on met le frein de parking ////
      if( (millis()-timer_parking) > PARKING_BRAKE_TIME_MS)
      {
        controlleur_R.brake(255);
        controlleur_L.brake(255);
        parking_brake_flag = 1;
      }

      ///////////////////////////////////////////////////////////////

      //////// Gestion du régulateur de vitesse ////////

      //// Désenclanchement du cruise control si marche arrière, ou frein, ou timer écoulé ////
      if( (joyY_f < -DEAD_ZONE) || (button_brake == 1) || ((millis() - timer_cruise_control) > CRUISE_CONTROL_TIME) )
      {
        constant_cruise_flag = 0;
        joyY = 0;
      }

      //// Enclanchement du régulateur de vitesse ////
      /*if( (last_button_cruise == 0 && button_cruise == 1) )
      {
        if ( (constant_cruise_flag == 0) && (joyY_f > 0.1) )
        {
          constant_cruise_flag = 1;
          joyY_f_lock = joyY_f;
          timer_cruise_control = millis();
        } else
        {
          timer_cruise_control = millis();
        }
      }
      last_button_cruise = button_cruise;
*/
      ///////////////////////////////////////////////////////////////


      //////// Calcul des vitesses frontale et différentielles et mixages pour envoie des commande aux moteurs droit et gauche ////////
      
      //// Vitesse frontale ////
      if(0) //constant_cruise_flag == 1) // vitesse d'avancée constante si cruise control
      {
        front_speed_ref = joyY_f_lock;
      } else
      {
        if(joyY_f>0) // marche avant
        {
          front_speed_ref = joyY_f;
        } else // marche arrière on applique le REAR_VS_FRONT_GAIN sur la vitesse
        {
          front_speed_ref = REAR_VS_FRONT_GAIN*joyY_f;
        }
      }

      //// Vitesse diffèrentielle ////
      lateral_speed_ref = (1+0.5*abs(front_speed_ref))*LAT_VS_FRONT_GAIN*joyX_f;

      d_speed_ref_R = 100*speed_ref_R;
      d_speed_ref_L = 100*speed_ref_L;

      //// Mixage des vitesse différentielle et frontale pour calculer les consignes du moteur droit et gauche ////
      if(button_brake == 1) // Si le frein est actif on met les consignes de vitesse à 0
      {
        speed_ref_R = 0;
        speed_ref_L = 0;

        // Remise à 0 des filtres pour avoir une reprise progressive de l'accélération après freinage (si l'utilisateur reste accéléré en freinant)
        joyY_f = 0;
        joyX_f = 0;
      } else
      {
        if(front_speed_ref>0)
        {
          if(lateral_speed_ref>0)
          {
            speed_ref_R = front_speed_ref - lateral_speed_ref;
            speed_ref_L = front_speed_ref + 0.3*lateral_speed_ref;
          } else
          {
            speed_ref_R = front_speed_ref - 0.3*lateral_speed_ref;
            speed_ref_L = front_speed_ref + lateral_speed_ref;
          }
        } else if (front_speed_ref<0)
        {
          if(lateral_speed_ref<0 )
          {
            speed_ref_R = front_speed_ref - lateral_speed_ref;
            speed_ref_L = front_speed_ref + 0.4*lateral_speed_ref;
          } else
          {
            speed_ref_R = front_speed_ref - 0.4*lateral_speed_ref;
            speed_ref_L = front_speed_ref + lateral_speed_ref;
          }
          if(speed_ref_R>0)
          {
            speed_ref_R = 0;
            controlleur_R.brake(255);
          }
          if(speed_ref_L>0)
          {
            speed_ref_L = 0;
            controlleur_L.brake(255);
          }
        } else
        {
          speed_ref_R = front_speed_ref - lateral_speed_ref;
          speed_ref_L = front_speed_ref + lateral_speed_ref;
        }
      }

      d_speed_ref_R -= 100*speed_ref_R;
      d_speed_ref_L -= 100*speed_ref_L;

      //// Envoie des commandes aux controleurs de moteur ////
      controlleur_R.update_reference(speed_ref_R);
      controlleur_L.update_reference(speed_ref_L);

      ///////////////////////////////////////////////////////////////

    }

    if (ENABLE_PRINT)
    {
      Serial.print("Remote:");Serial.print("\t");
      Serial.print(joyY);Serial.print("\t");
      Serial.print(joyX);Serial.print("\t");
      Serial.print(brake_level_max);Serial.print("\t");
      Serial.print(button_brake);Serial.print("\t");

      Serial.print("Flags:");Serial.print("\t");
      Serial.print(constant_cruise_flag);Serial.print("\t");
      Serial.print(brake_flagR);Serial.print("\t");
      Serial.print(brake_flagL);Serial.print("\t");
      Serial.print(parking_brake_flag);Serial.print("\t");
      Serial.print(connexion_flag);Serial.print("\t");

      Serial.print("Vitesses:");Serial.print("\t");
      Serial.print(front_speed_ref);Serial.print("\t");
      Serial.print(lateral_speed_ref);Serial.print("\t");

      Serial.print("Mixage:");Serial.print("\t");
      
      Serial.print(d_speed_ref_R);Serial.print("\t");
      Serial.print(d_speed_ref_L);Serial.print("\t");
      Serial.print(speed_ref_L);Serial.print("\t");
      Serial.print(speed_ref_R);

      Serial.println("\t");
    }
    
  }
}
