// Aile Disco avec Leddar, GPS et pitot.
#include <SD.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Wire.h>

/******* librairies internes **************/

#include "bte_functions.h"
#include "bte_servo.h"
#include "bte_motor.h"
#include "bte_controller.h"
#include "bte_GPS.h"
#include "bte_leddar.h"

/******* constantes fonction distance/altitude **************/

#define VITESSE_DES_DAUPHIN 12.0
#define PENTE_AERO_DAUPHIN1 35.0*DEG2RAD
#define PENTE_AERO_DAUPHIN2 20.0*DEG2RAD
#define HAUTEUR_BONZAI 11.0
#define LONGUEUR_BONZAI 42.0
#define TEMPS_BONZAI 3.0
#define HAUTEUR_DAUPHIN2_2 16.0
#define HAUTEUR_DAUPHIN2 20.0
#define HAUTEUR_LEDDAR 35.0
#define HAUTEUR_PALIER 7.0
#define HAUTEUR_CABRAGE 2.0

float distance_des, alti_, K_traj, K_traj_lat;
float longitudinal_distance, lateral_distance;
float alti_offset = 0;
float alti_declenchement,distance_declenchement;

/******* Servos et moteur **************/

#define PIN_SERVO_R 38
#define PIN_SERVO_L 37
#define PIN_MOTOR_PROP 36

bte_servo Servo_R = bte_servo(PIN_SERVO_R);
bte_servo Servo_L = bte_servo(PIN_SERVO_L);

bte_motor Motor_Prop = bte_motor(PIN_MOTOR_PROP);

// PWM normalisés pour les servos et moteurs
float pwm_norm_R, pwm_norm_L, pwm_norm_prop;

/******* Télécommande **************/

bte_controller remote = bte_controller(&Serial3);

float knob2, thrust;
uint16_t Switch_C_previous;

// remplace le trim de la télécommande, exprimé en valeur de servo normalisé.
// positif vers le haut et vers la droite
float elevation_trim = 0.0;
float aileron_trim = 0.0;

/******* GPS & pitot **************/

bte_GPS GPS = bte_GPS(&Serial2);

float v_pitot_mean;
float v_pitot_buffer[100];
float vh_pitot_mean;
float vh_pitot_buffer[100];
float v_wind, v_wind_mean, v_wind_mean_memory;
float v_wind_buffer[100];
float scalar_prod_buffer[100];
float scalar_prod_mean;
const int16_t Ndata = 100;
uint8_t first_GPS_data = 0;
float heading;
float v_horizontal_gps;

float slope_ground_mean;
float slope_ground_buffer[100];

float altitude_stabilisation = 60.0;

/******* leddar **************/

bte_leddar leddar = bte_leddar(&Serial4);

float hauteur_leddar_corrigee, hauteur_leddar;
const float leddar_mounting_vector[3] = {0.0,0.0,1.0};

/******* Carte SD **************/

File dataFile;
bool Open = false, Closed = true;
String dataString;
char *filename;
uint32_t nb_file = 0, counter = 0;
bool  OK_SDCARD = false;
char Num[8];
uint32_t temps_log, dt_log;

/******* BN0055, orientation **************/

Adafruit_BNO055 bno = Adafruit_BNO055();

// vitesse, angle euler, acc propre
float BNO_wx, BNO_wy, BNO_wz;
float BNO_roll, BNO_pitch, BNO_lacet;
float BNO_linear_acc_norm;

//Offsets
float RollOffs = 0, PitchOffs = 0, YawOffs = 90; //en deg

/******* Xbee **************/

#define XBEE_SERIAL Serial1

const uint8_t NbDataXB = 9;
int16_t DataXB[NbDataXB];

int16_t DataXB1[10];
int16_t DataXB2[10];
int16_t DataXB3[3];
uint8_t count_XB = 1;

float DataXB_float[5];
uint8_t NbDataXB_float = 5;

uint8_t GPS_init_validated = 0;

/******* Variables et paramètres des régulation **************/

const float dt_flt = 0.01;
uint8_t regulation_state;

//// régulation d'attitude et vitesse
// params
float K_Pitch_remote = 0.5, K_Pitch = 0, KD_Pitch = 0, KI_Pitch = 0, K_alti = 0;
float K_Roll_remote = -0.5, K_Roll = 0, KD_Roll = 0.2;
float K_Yaw = 0, KD_Yaw = 0, KI_Yaw = 0;
float KP_Moteur = 0, KI_Moteur = 0, Offset_gaz_reg = 0;
float alpha_stab = 0.025; // filtrage au passage en mode stabilisé
float offset_dauphin = 0.15;

//consignes
float pitch_des = 0, pitch_des_f = 0, yaw_des = 0, vitesse_des = 10, roll_des = 0.0;

// states
float Commande_Roll;
float Commande_Pitch, Commande_P_flaps, Commande_I_flaps, Commande_D_flaps, Commande_I_yaw;
float Commande_I_Moteur, Commande_KP_Moteur;

///// mode dauphin
// params
float flaps_amplitude, hyst_width, hauteur_switch, hauteur_cabrage, hauteur_palier;
float flaps_amplitude_plus, flaps_amplitude_moins;
float alpha_dauphin = 0.015; // filtrage au passage en mode dauphin
float thrust_anti_decrochage, vitesse_anti_decrochage;
// states
float Commande_dauphin = 0;
int8_t arrondi_almost_ready,arrondi_ready,flap_state_mem,flap_state = 1, leddar_track = 0, arrondi_final, palier, flag_dauphin2;
int8_t first_dive = 1;

// asservissement de la pente de descente
// params
float slope_aero, slope_ground, slope_aero_f = 0;
float alpha_slope = 0.008;
//consigne
float slope_des, slope_des_f_delay, slope_des_f = 0;
float slope_des_f_buffer[100];
// states
float Commande_I_slope, KI_slope;

// Asservissement de position
float gps_target[2] = {0,0};
float gps_target0[2] = {150,150};
float gps_target1[2] = {-150,-150};
float heading_to_target, heading_to_target_lock, yaw_to_target_lock_offset, distance_to_target;
uint8_t declenchement = 0;
uint8_t current_target = 0;
float alti_ref;

/******* debug **************/

float alpha_roll = 0.04;
float BNO_roll_f = 0;

float alpha_vitesse = 0.004;

float err_yaw_f;

uint32_t stability_counter;
uint8_t stability_achieved;

//autres
uint32_t temps, temps2, dt, time_idx, i, time_switch,timer_mode;
uint32_t dt1, dt2, dt3, dt4;

void setup()
{
  /******* ouverture des ports série **************/
  
  Serial.begin(115200);
  XBEE_SERIAL.begin(115200);

  /******* Initialisation Servo et Moteurs ************/
  
  Servo_R.set_range(750,2050,1400);
  Servo_R.set_normalized_pwm(0);
  Servo_R.power_on();
  
  Servo_L.set_range(750,2050,1400);
  Servo_L.set_normalized_pwm(0);
  Servo_L.power_on();

  Motor_Prop.set_range(1000,2000);
  Motor_Prop.power_off(); // should already be off (normally)...
  Motor_Prop.set_normalized_pwm(0);

  remote.set_connection_lost_time_millis(1000);

    /******* Initialisation BNO **************/
    
  if ( !bno.begin() )
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  // battement de l'aile droite pour valider le démarage de la BNO
  Servo_R.set_normalized_pwm(0.3);
  delay(500);
  Servo_R.set_normalized_pwm(-0.3);
  delay(500);
  Servo_R.set_normalized_pwm(0);
    
  temps = micros();
  while ( (micros() - temps) < 1000000 ) {};
  bno.setExtCrystalUse(true);

  
  /******* Initialisation de la carte SD **************/
  
  OK_SDCARD = SD.begin(BUILTIN_SDCARD);
  if ( !OK_SDCARD )
  {
    Serial.println("Card failed, or not present");
    //return;
  }
  else
  {
    Serial.println("card initialized.");
    filename = (char*) malloc( strlen("log") + strlen(Num) + strlen(".txt") + 1) ;
    checkExist(); //vérifie existence du fichier et l'écriture commence à partir du numéro de fichier que n'existe pas
    // Battement de l'aile gauche pour valider le démarage de la carte SD
    Servo_L.set_normalized_pwm(0.3);
    delay(500);
    Servo_L.set_normalized_pwm(-0.3);
    delay(500);
    Servo_L.set_normalized_pwm(0);

  }

  /*****************************************/

  temps = micros();
  time_idx = 0;
  // Offset de montage de la centrale inertielle
  PitchOffs = 2.0;
  
}

// 7 0 7
void loop()
  {

  // mise à jour des valeurs de la télécommande
  remote.update_controller();
  knob2 = constrain((0.3- remote._rudder) / 0.6, 0, 1);
  thrust = remote._thrust;

  // mise à jour des valeurs leddar
  leddar.read_leddar();

  // mise à jour des valeurs GPS et pitot
  GPS.read_GPS();

  if(GPS._x_gps != 0.0 && first_GPS_data != 1)
  {
    // Battement des 2 ailes pour valider la prise de référence du GPS
    first_GPS_data = 1;
    Servo_R.set_normalized_pwm(0.3);
    Servo_L.set_normalized_pwm(0.3);
    delay(500);
    Servo_R.set_normalized_pwm(-0.3);
    Servo_L.set_normalized_pwm(-0.3);
    delay(500);
    Servo_R.set_normalized_pwm(0);
    Servo_L.set_normalized_pwm(0);
  }
  
  dt = micros() - temps;
  
  if ( dt > 10000) // boucle principale cadencée à 100 Hz
  {
    temps += dt;
    
/********Info de BNO55***************/

    // Lectures des infos gyro
    imu::Vector<3> gyro_speed = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

    // transformation en DPS
    BNO_wz = gyro_speed.z() * RAD2DEG;
    BNO_wy = -gyro_speed.y() * RAD2DEG;
    BNO_wx = -gyro_speed.x() * RAD2DEG;

    // lecture de l'acc linéaire (pour log)
    imu::Vector<3> linear_acc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    BNO_linear_acc_norm = sqrtf(linear_acc.x()*linear_acc.x() + linear_acc.y()*linear_acc.y() + linear_acc.z()*linear_acc.z())/9.81;

    // lecture du vecteur gravité pour la projection de la mesure leddar (distance leddar -> altitude)
    imu::Vector<3> gravity_vector = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);

    float produit_scalaire = (gravity_vector.x()*leddar_mounting_vector[0] + gravity_vector.y()*leddar_mounting_vector[1] + gravity_vector.z()*leddar_mounting_vector[2])/ 9.81 ;
    hauteur_leddar = leddar._hauteur *10.0;

    bte_HistoriqueVal(produit_scalaire, scalar_prod_buffer, Ndata);
    bte_Mean(scalar_prod_buffer, &scalar_prod_mean, Ndata, Ndata-1);

    hauteur_leddar_corrigee = hauteur_leddar*scalar_prod_mean + 0.5*GPS._vz_gps;
    
    if(abs(hauteur_leddar_corrigee-GPS._z_gps)>30.0)
    {
      leddar._validity_flag = 0;
    }

    // lecture des angles d'Euler (deg)+ application des offset 
    imu::Vector<3> eulAng = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    
    BNO_roll = bte_ang_180(eulAng.z() - RollOffs);
    BNO_pitch = bte_ang_180(eulAng.y() - PitchOffs);
    BNO_lacet = bte_ang_180(-eulAng.x() - YawOffs);


/***************** Estimation des angles de descente, vitesse pitot filtrée...  *********************/
//

//    Serial.print(GPS._x_gps ); Serial.print("   ");
//    Serial.print(GPS._lat_0_int ); Serial.print("   ");
//    Serial.print(GPS._lat_0_dec ); Serial.print("   ");
//    Serial.print(GPS._lon_0_int ); Serial.print("   ");
//    Serial.print(GPS._lon_0_dec ); Serial.print("   ");
//    Serial.print(GPS._alti_0 ); Serial.print("   ");

//    Serial.print(remote._aileron ); Serial.print("   ");
//    Serial.print(remote._elevator ); Serial.print("   ");
//    Serial.print(remote._thrust ); Serial.print("   ");
//    Serial.print(remote._switch_C ); Serial.print("   ");
//    Serial.print(remote._switch_F ); Serial.print("   ");
//    Serial.print(remote._switch_D ); Serial.print("   ");
//    Serial.print(remote._knob ); Serial.print("   ");
    //Serial.print(produit_scalaire*10); Serial.print("   ");

//    Serial.print(BNO_wx); Serial.print("   ");
//    Serial.print(BNO_wy); Serial.print("   ");
//    Serial.print(BNO_wz); Serial.print("   ");

//    Serial.print(BNO_roll); Serial.print("   ");
//    Serial.print(BNO_pitch); Serial.print("   ");
//    Serial.print(BNO_lacet); Serial.print("   ");
//Serial.println("   ");

    if(current_target == 0 )
    {
      gps_target[0] = gps_target0[0];
      gps_target[1] = gps_target0[1];
    } else
    {
      gps_target[0] = gps_target1[0];
      gps_target[1] = gps_target1[1];
    }

/***************** Estimation de la distance à la cible et heading  *********************/

    // distance à la cible
    distance_to_target = sqrtf( (GPS._x_gps - gps_target[0])*(GPS._x_gps - gps_target[0]) + (GPS._y_gps - gps_target[1])*(GPS._y_gps - gps_target[1]));
    
    // Cap menant à la cible
    heading_to_target = atan2f((GPS._x_gps - gps_target[0]),-(GPS._y_gps - gps_target[1]))*RAD2DEG;
    heading = atan2f(-GPS._vx_gps,GPS._vy_gps)*RAD2DEG;

    longitudinal_distance = distance_to_target*cos((heading_to_target-heading_to_target_lock)*DEG2RAD);
    lateral_distance = distance_to_target*sin((heading_to_target-heading_to_target_lock)*DEG2RAD);

/***************** Paramètres de régulation par defaut*********************/
    
    // peuvent être modifiés dans les différents modes, par défaut, juste le D sur le roll
    K_Pitch = 0; KD_Pitch = 0; KI_Pitch = 0;
    K_Roll = 0; KD_Roll = 0.4;
    K_Yaw = 0; KD_Yaw = 0;
    KP_Moteur = 0; KP_Moteur = 0; Offset_gaz_reg = 0;

/***************** Modes de régulation *********************/


///////////////////////////////////////////////
// MODE SWITCH C MILLIEU ( 1 MODE STABILISE) //
///////////////////////////////////////////////
    
    // mode utilisé pour la phase de stabilisation avant le dauphin

    if (remote._switch_C != 2 ) 
    { 

      regulation_state = 1;
      
      // timer pour passer en mode 4
      time_switch = millis(); // sert quand on passe au mode appontage
      timer_mode = millis() - time_switch;  // timer_mode = 0 tt le temps.

      // paramètres mode stabilisé
      K_Pitch = 1.6; KD_Pitch = 0.8; KI_Pitch = 15.0;
      K_Roll = 7; KD_Roll = 0.4;
      K_Yaw = 1.0; KD_Yaw = 0.0, KI_Yaw = 0.15;
      //K_Yaw = 0.0; KD_Yaw = 0.0, KI_Yaw = 0.0;
      KP_Moteur = 0.05; KI_Moteur = 0.2; Offset_gaz_reg = 0.0;

      // consignes de pitch et de vitesse

      if(remote._switch_F ==0)
      {
        K_alti = 0.5;
      } else if(remote._switch_F ==1)
      {
        K_alti = 1.0;
      } else
      {
        K_alti = 2.0;
      }
      
      pitch_des = K_alti*(alti_ref-GPS._z_gps) + 20;
      
      vitesse_des = 5;
      
      yaw_des = heading_to_target;

      if(remote._switch_D ==0)
      {
        KD_Pitch = 0.0;
      } else if(remote._switch_D ==1)
      {
        KD_Pitch = 0.4;
      } else
      {
        KD_Pitch = 0.8;
      }

      if(remote._switch_C != 0)
      {
        K_Yaw = 0;
        KI_Yaw = 0;
        KD_Yaw = 0;
      }

//////// Régulation de vitesse

//      Commande_I_Moteur += -KI_Moteur * (v_horizontal_gps - vitesse_des) * dt_flt;
//      Commande_I_Moteur = constrain(Commande_I_Moteur, 0, 0.5); // sat = 200
//    
//      Commande_KP_Moteur = -KP_Moteur * (v_horizontal_gps - vitesse_des);
//      Commande_KP_Moteur = constrain(Commande_KP_Moteur, -0.5, 0.5);
//    
//      thrust = Offset_gaz_reg + Commande_KP_Moteur + Commande_I_Moteur;
//    
//      thrust = constrain(thrust, 0, 1);

      thrust = 0.35;

//////// Régulation de pitch

      // Intégrateur des flaps pour régulation du pitch
      Commande_I_flaps += -KI_Pitch * (BNO_pitch - pitch_des) * dt_flt / 360.0; // += addition de la valeur précédente
      Commande_I_flaps = constrain(Commande_I_flaps, -1.0, 1.0);

//////// Erreure de yaw pour la régulation

      err_yaw_f = bte_ang_180(heading - yaw_des);
      Commande_I_yaw += KI_Yaw * err_yaw_f * dt_flt / 360.0;
      Commande_I_yaw = constrain(Commande_I_yaw, -10,10);

      if(current_target == 0 && distance_to_target < 20.0)
      {
        current_target = 1;
      } else if (current_target != 0 && distance_to_target < 50.0)
      {
        current_target = 0;
      }
    
    }
//////////////////////////////////////// 
// SWITCH C BAS ( 2  //  MODE MANUEL) //
////////////////////////////////////////
     
    else 
    {/***Mode 1: normal, pas de moteur couple, KD sur le roll seulement.***/

      regulation_state = 0;
      
      // timer qui sera utilisé pour passer en mode couple
      time_switch = millis();
      timer_mode = millis() - time_switch;

      // paramètres mode manuel
      K_Pitch = 0; KD_Pitch = 0; KI_Pitch = 0;
      K_Roll = 0; KD_Roll = 0.4;
      K_Yaw = 0; KD_Yaw = 0;
      KP_Moteur = 0; KI_Moteur = 0; Offset_gaz_reg = 0;
      
      // Mise à 0 des commandes inutilisées pour ce mode
      Commande_I_flaps = 0;
      Commande_I_Moteur = 0;
      Commande_I_yaw = 0;
      alti_ref = GPS._z_gps;
      yaw_des = BNO_lacet;
    }

////////////////////////////////////////////
////////////////////////////////////////////
////////////////////////////////////////////

    
/*****************Commande générale avec les paramètres définis pour les différents modes *********************/

    if(abs(err_yaw_f)<90 || remote._switch_C != 0)
    {
      roll_des = - K_Yaw * err_yaw_f
                 - KD_Yaw * BNO_wz
                 - Commande_I_yaw;
    } else
    {
      roll_des = 20.0;
      Commande_I_yaw = 0;
    }

    roll_des = constrain(roll_des,-20,20);
    
    // Commande correspondant au roll, télécommande + P roll + D roll + P yaw + D yaw
    Commande_Roll = - K_Roll_remote * remote._aileron 
                    + K_Roll * (BNO_roll-roll_des) / 360.0 
                    + KD_Roll * BNO_wx  / 360.0 
                    - aileron_trim; 
    //Commande_Roll = -Commande_Roll;

    float saturation;

    if(remote._switch_C!=0 || declenchement == 0)
    {
      saturation = 0.6;
    } else
    {
      saturation = 0.3;
    }
                    
    Commande_Roll = constrain(Commande_Roll,-saturation,saturation);
    
    // Commande correspondant au pitch
    Commande_P_flaps = - K_Pitch * (BNO_pitch - pitch_des)  / 360.0;
    Commande_D_flaps = - KD_Pitch * BNO_wy  / 360.0;
    Commande_Pitch = - K_Pitch_remote * remote._elevator 
                      + Commande_dauphin  
                      + Commande_P_flaps 
                      + Commande_I_flaps 
                      + Commande_D_flaps 
                      + elevation_trim; //+ Commande_P_flapping;
    Commande_Pitch = -Commande_Pitch;

    
    // Construction de pwm servo a partir des commandes roll et pitch autour du zero des servo
    pwm_norm_R = Commande_Roll + Commande_Pitch;
    pwm_norm_L = Commande_Roll - Commande_Pitch;
    
    pwm_norm_prop = thrust; 

/***************** Coupure moteur de sécurité *********************/
       
    // Sécurité, coupure des moteurs sur le switch ou si la carte SD n'est pas présente, ou perte de signal télécommande
    if(  !OK_SDCARD || remote._perte_connection )//|| remote._rudder < 0.0 || first_GPS_data==0 )
    {
      Motor_Prop.power_off();
    } else
    {
      Motor_Prop.power_on();
    }

/*************Ecriture PWMs Servo et Moteurs*****************/

    // Saturation des pwm normalisés.
    
    pwm_norm_R = constrain(pwm_norm_R,-1.0,1);
    pwm_norm_L = constrain(pwm_norm_L,-1,1.0);
    
    Servo_R.set_normalized_pwm(pwm_norm_R);
    Servo_L.set_normalized_pwm(pwm_norm_L);

    Motor_Prop.set_normalized_pwm(pwm_norm_prop);

/*********** Log sur la carte SD ***********/

    if (OK_SDCARD) 
    {
        if (!Open) 
        {
          strcpy(filename, "log");
          sprintf(Num, "%lu", nb_file);
          strcat(filename, Num); strcat(filename, ".txt");
          
          dataFile = SD.open(filename, FILE_WRITE);
          
          nb_file++;
          Open = true;
          Closed = false;
          temps2 = millis();
       } else if (Open && !Closed) 
       {
          temps_log = millis();
          dataFile.print(temps_log); dataFile.print(";");
          
          dataFile.print(BNO_lacet * 10, 0); dataFile.print(";");
          dataFile.print(BNO_pitch * 10, 0); dataFile.print(";");
          dataFile.print(BNO_roll * 10, 0); dataFile.print(";");
          
          dataFile.print(yaw_des*10.0,0); dataFile.print(";");
          dataFile.print(pitch_des * 10, 0); dataFile.print(";");
          dataFile.print(roll_des * 10, 0); dataFile.print(";");
  
          dataFile.print(GPS._x_gps*100.0, 0); dataFile.print(";");
          dataFile.print(GPS._y_gps*100.0, 0); dataFile.print(";");
          dataFile.print(GPS._z_gps*100.0, 0); dataFile.print(";");

          dataFile.print(GPS._vx_gps*100.0, 0); dataFile.print(";");
          dataFile.print(GPS._vy_gps*100.0, 0); dataFile.print(";");
          dataFile.print(GPS._vz_gps*100.0, 0); dataFile.print(";");
  
          dataFile.print(pwm_norm_R*1000.0,0); dataFile.print(";");
          dataFile.print(pwm_norm_L*1000.0,0); dataFile.print(";");
          dataFile.print(pwm_norm_prop*1000.0,0); dataFile.print(";");

          dataFile.print(gps_target[0]*100.0, 0); dataFile.print(";");
          dataFile.print(gps_target[1]*100.0, 0); dataFile.print(";");
  
          dataFile.print(distance_to_target*1000, 0); dataFile.print(";");

          dataFile.print(remote._switch_F+10*remote._switch_D+100*remote._switch_C + declenchement*1000); dataFile.print(";");

          dataFile.print(remote._knob, 0); dataFile.print(";");
          
  
          dataFile.println(" "); // gaffe à la dernière ligne
          
          if ((millis() - temps2 > 300.0 * 1000) || (remote._switch_C != Switch_C_previous) )
          {
            dataFile.close();
            Closed = true;
            Open = false;
          }
       }
    }
   
    Switch_C_previous = remote._switch_C;

      /**********************Envoi vers PC Station******************************/
    if (first_GPS_data)
    {
       uint8_t x;
  
      if(XBEE_SERIAL.available()>1)
      {
//        Serial.print("Validation ");
//        Serial.print(XBEE_SERIAL.available());
        x = XBEE_SERIAL.read();
//        Serial.print(" ; ");
//        Serial.print(x);
        if(x==137)
        {
//          Serial.print(" OK : ");
          x = XBEE_SERIAL.read();
//          Serial.print(x);
          GPS_init_validated = (x==173);
//          Serial.print(" flag : ");
//          Serial.println(GPS_init_validated);
        }
      }
  
      if(GPS_init_validated==1)
      {
        //Serial.println("position");
        DataXB[0] = (int16_t)(BNO_roll/180.0*32768);
        DataXB[1] = (int16_t)(BNO_pitch/180.0*32768);
        DataXB[2] = (int16_t)(BNO_lacet/180.0*32768);
        
        DataXB[3] = (int16_t)(constrain(GPS._x_gps,-499,500)/500.0*32768);
        DataXB[4] = (int16_t)(constrain(GPS._y_gps,-499,500)/500.0*32768);
        DataXB[5] = (int16_t)(constrain(GPS._z_gps,-499,500)/500.0*32768);
        
        DataXB[6] = (int16_t)(constrain(GPS._vx_gps,-49,50)/50.0*32768);
        DataXB[7] = (int16_t)(constrain(GPS._vy_gps,-49,50)/50.0*32768);
        DataXB[8] = (int16_t)(constrain(GPS._vz_gps,-49,50)/50.0*32768);
    
        //Envoi des données vers le PC
        bte_sendData_int16_t(XBEE_SERIAL, 137, 173, DataXB, NbDataXB);
  
      } else
      {
        //Serial.println("initialisation");
        DataXB_float[0] = GPS._lat_0_int;
        DataXB_float[1] = GPS._lat_0_dec;
        DataXB_float[2] = GPS._lon_0_int;
        DataXB_float[3] = GPS._lon_0_dec;
        DataXB_float[4] = GPS._alti_0;
        
        bte_sendData_float(XBEE_SERIAL, 137, 173, DataXB_float, NbDataXB_float, 0);
      }
      
    }

  } // fin de boucle à 100 hz
}

void checkExist(void)
{
  do
  {
    strcpy(filename, "log");
    sprintf(Num, "%lu", counter);
    strcat(filename, Num); strcat(filename, ".txt");
    counter++;
  }
  while ( SD.exists(filename) );
  nb_file = counter - 1;
}
