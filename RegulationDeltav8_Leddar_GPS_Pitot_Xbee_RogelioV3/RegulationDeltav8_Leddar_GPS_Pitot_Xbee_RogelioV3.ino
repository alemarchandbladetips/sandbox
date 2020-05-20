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
#include "bte_GPS_pitot.h"
#include "bte_leddar.h"

/******* fréquence de fonctionnement **************/

#define DT_MICROS 10000

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
#define HAUTEUR_CABRAGE 1.5

float distance_des, alti_, K_traj, K_traj_lat;
float longitudinal_distance, lateral_distance;
float alti_offset = 0;
float alti_declenchement,distance_declenchement;

/******* Servos et moteur **************/

#define PIN_SERVO_R 2
#define PIN_SERVO_L 23
#define PIN_MOTOR_PROP 3

bte_servo Servo_R = bte_servo(PIN_SERVO_R);
bte_servo Servo_L = bte_servo(PIN_SERVO_L);

bte_motor Motor_Prop = bte_motor(PIN_MOTOR_PROP);

// PWM normalisés pour les servos et moteurs
float pwm_norm_R, pwm_norm_L, pwm_norm_prop;

/******* Télécommande **************/

bte_controller remote = bte_controller(&Serial1);

float knob2, thrust;
uint16_t Switch_C_previous;

// remplace le trim de la télécommande, exprimé en valeur de servo normalisé.
// positif vers le haut et vers la droite
float elevation_trim = 0.0;
float aileron_trim = 0.0;

/******* GPS & pitot **************/

bte_GPS_pitot GPS_pitot = bte_GPS_pitot(&Serial3);

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

float altitude_stabilisation = 60.0;

/******* leddar **************/

bte_leddar leddar = bte_leddar(&Serial2);

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
uint32_t temps_log, dt_log, timer_mode, time_switch;

/******* BN0055, orientation **************/

Adafruit_BNO055 bno = Adafruit_BNO055();

// vitesse, angle euler, acc propre
float BNO_wx, BNO_wy, BNO_wz;
float BNO_roll, BNO_pitch, BNO_lacet;
float BNO_linear_acc_norm;
const float alpha_roll = 0.04;
float BNO_roll_f = 0;

//Offsets
const float RollOffs = 0, PitchOffs = 2.0, YawOffs = 180; //en deg

/******* Variables et paramètres des régulation **************/

const float dt_flt = 0.01;
uint8_t regulation_state;
int8_t flag_dauphin_end, flag_bonsai_start, flag_leddar_track, flag_arrondi_final, flag_dauphin2, flag_first_dive;

//// régulation d'attitude et vitesse
// params
const float K_Pitch_remote = 0.6;
float KP_Pitch, KD_Pitch, KI_Pitch;
const float K_Roll_remote = -0.4;
float KP_Roll, KD_Roll;
float KP_Yaw, KD_Yaw, KI_Yaw;
float KP_Moteur, KI_Moteur;
const float alpha_stab = 0.025; // filtrage au passage en mode stabilisé
float flaps_amplitude_dauphin; 
const float flaps_offset_dauphin = 0.15;

//consignes
float pitch_des = 0, pitch_des_f = 0, yaw_des = 0, vitesse_des = 10, roll_des = 0.0;

// states
float Commande_Roll;
float Commande_Pitch, Commande_P_flaps, Commande_I_flaps, Commande_D_flaps, Commande_I_yaw;
float Commande_I_Moteur, Commande_KP_Moteur;
float err_yaw_f;

///// mode dauphin
// params
float hauteur_cabrage;
const float alpha_dauphin = 0.015; // filtrage au passage en mode dauphin
float thrust_anti_decrochage, vitesse_anti_decrochage;
float pitch_commutation_prev, pitch_commutation_prev_1;
// states
float Commande_dauphin = 0;
int8_t flap_state = 1;

// asservissement de la pente de descente
// params
float KI_slope;
const float alpha_slope = 0.008;
//consigne
float slope_des, slope_des_f_delay, slope_des_f = 0;
float slope_des_f_buffer[100];
// states
float slope_ground, slope_ground_mean;
float slope_ground_buffer[100];
float slope_aero, slope_aero_f = 0;
float Commande_I_slope;

// Asservissement de position
float gps_target[2] = {0,0};
float heading_to_target, heading_to_target_lock, yaw_to_target_lock_offset, distance_to_target;
uint8_t flag_declenchement = 0;

/******* debug **************/

//autres
uint32_t temps, temps2, dt, time_idx, i;
uint32_t dt1, dt2, dt3, dt4;

void setup()
{
  /******* ouverture des ports série **************/
  
  Serial.begin(115200);

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
  GPS_pitot.read_GPS_pitot();


/********Procédure de test manuelle pitot, ledar, GPS. Uniquement avant le démarrage du GPS***************/

  if(first_GPS_data != 1)
  {
    // test de la pitot qui faut bouger l'aile gauche quand elle sature
    if(GPS_pitot._speed_pitot>60)
    {
      Servo_L.set_normalized_pwm(-0.5);
    }
    if(GPS_pitot._speed_pitot<-60)
    {
      Servo_L.set_normalized_pwm(0.5);
    }
  
    // test du ledar qui fait bouger l'aile droite
    if(leddar._hauteur*10.0 >1.0 && leddar._hauteur *10.0<2.0)
    {
      Servo_R.set_normalized_pwm(-0.5);
    }
  
    if(GPS_pitot._x_gps != 0.0)
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
  }

/********FIN Procédure de test manuelle pitot, ledar, GPS. Uniquement avant le démarrage du GPS***************/


/********boucle principale cadencée à 100 Hz***************/

  dt = micros() - temps;
  
  if ( dt > DT_MICROS) // 
  {
    temps += dt;
    
/********Info de BNO55***************/

////// Lectures des infos gyro
    imu::Vector<3> gyro_speed = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

    // transformation en DPS
    BNO_wz = gyro_speed.z() * RAD2DEG;
    BNO_wy = -gyro_speed.x() * RAD2DEG;
    BNO_wx = gyro_speed.y() * RAD2DEG;

    // lecture de l'acc linéaire (pour log)
    imu::Vector<3> linear_acc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    BNO_linear_acc_norm = sqrtf(linear_acc.x()*linear_acc.x() + linear_acc.y()*linear_acc.y() + linear_acc.z()*linear_acc.z())/9.81;

////// lecture du vecteur gravité pour la projection de la mesure leddar (distance leddar -> altitude)
    imu::Vector<3> gravity_vector = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);

    float produit_scalaire = (gravity_vector.x()*leddar_mounting_vector[0] + gravity_vector.y()*leddar_mounting_vector[1] + gravity_vector.z()*leddar_mounting_vector[2])/ 9.81 ;
    hauteur_leddar = leddar._hauteur *10.0;

    bte_HistoriqueVal(produit_scalaire, scalar_prod_buffer, Ndata);
    scalar_prod_mean =  bte_medianFilter(produit_scalaire, scalar_prod_buffer, Ndata);
    hauteur_leddar_corrigee = hauteur_leddar*scalar_prod_mean + 0.5*GPS_pitot._vz_gps;
    
    if(abs(hauteur_leddar_corrigee-GPS_pitot._z_gps)>30.0)
    {
      leddar._validity_flag = 0;
    }

////// lecture des angles d'Euler (deg)+ application des offset 
    imu::Vector<3> eulAng = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    
    BNO_roll = bte_ang_180(-eulAng.y() - RollOffs);
    BNO_pitch = bte_ang_180(eulAng.z() - PitchOffs);
    BNO_lacet = bte_ang_180(-eulAng.x() - YawOffs);

    // Filtrage du roll pour la régulation en mode dauphin
    BNO_roll_f = (1-alpha_roll)*BNO_roll_f + alpha_roll*BNO_roll;
    BNO_roll = BNO_roll_f;

//    Serial.print(BNO_pitch); Serial.print("   ");
//    Serial.print(BNO_roll); Serial.print("   ");
//    Serial.print(BNO_pitch); Serial.print("   ");
    
//    Serial.print(BNO_wx); Serial.print("   ");
//    Serial.print(BNO_wy); Serial.print("   ");
//    Serial.print(BNO_wz); Serial.print("   ");
//
//    Serial.println("   ");

/***************** Estimation des angles de descente, vitesse pitot filtrée...  *********************/

    // filtrage moyen de la vitesse pitot sur 1s, utilisé pour la régulation de vitesse en mode dauphin
    bte_HistoriqueVal(GPS_pitot._speed_pitot, v_pitot_buffer, Ndata);
    bte_Mean(v_pitot_buffer, &v_pitot_mean, Ndata, Ndata-1);

    // vitesse pitot projeté sur le plan horizontal et filtré
    bte_HistoriqueVal(GPS_pitot._speed_pitot*cosf(BNO_pitch*DEG2RAD), vh_pitot_buffer, Ndata);
    bte_Mean(vh_pitot_buffer, &vh_pitot_mean, Ndata, Ndata-1);

    // estimation de la vitesse du vent dans l'axe de l'attérissage. utilisé a la fin du mode stabilisé pour choisir la pente de descente et adapter le point de déclanchement du dauphin
    v_wind = v_pitot_buffer[Ndata-31] - sqrtf(GPS_pitot._vx_gps*GPS_pitot._vx_gps + GPS_pitot._vy_gps*GPS_pitot._vy_gps +GPS_pitot._vz_gps*GPS_pitot._vz_gps);
    bte_HistoriqueVal(v_wind, v_wind_buffer, Ndata);
    bte_Mean(v_wind_buffer, &v_wind_mean, Ndata, Ndata-1);

    // calcul de la pente aéro
    slope_aero = atan2f(GPS_pitot._vz_gps,GPS_pitot._speed_pitot*cosf(BNO_pitch*DEG2RAD))*RAD2DEG;
    slope_aero_f = atan2f(GPS_pitot._vz_gps,vh_pitot_mean)*RAD2DEG;

    // calcul de la pente réelle + filtrage
    v_horizontal_gps = sqrtf(GPS_pitot._vx_gps*GPS_pitot._vx_gps + GPS_pitot._vy_gps*GPS_pitot._vy_gps);
    slope_ground = atan2f(GPS_pitot._vz_gps,v_horizontal_gps)*RAD2DEG;
    bte_HistoriqueVal(slope_ground, slope_ground_buffer, Ndata);
    bte_Mean(slope_ground_buffer, &slope_ground_mean, Ndata, Ndata-1);

    // retard de la pente désirée pour coller au retard du GPS.
    bte_HistoriqueVal(slope_des_f, slope_des_f_buffer, Ndata);
    slope_des_f_delay = slope_des_f_buffer[50];

/***************** Estimation de la distance/cap à la cible et heading  *********************/

    // distance à la cible
    distance_to_target = sqrtf( (GPS_pitot._x_gps - gps_target[0])*(GPS_pitot._x_gps - gps_target[0]) + (GPS_pitot._y_gps - gps_target[1])*(GPS_pitot._y_gps - gps_target[1]));
    
    // Cap menant à la cible
    heading_to_target = atan2f((GPS_pitot._x_gps - gps_target[0]),-(GPS_pitot._y_gps - gps_target[1]))*RAD2DEG;
    heading = atan2f(-GPS_pitot._vx_gps,GPS_pitot._vy_gps)*RAD2DEG;

    // projection de l'erreur sur l'axe longitudinal et latéral
    longitudinal_distance = distance_to_target*cos((heading_to_target-heading_to_target_lock)*DEG2RAD);
    lateral_distance = distance_to_target*sin((heading_to_target-heading_to_target_lock)*DEG2RAD);

/***************** Paramètres de régulation par defaut*********************/
    
    // peuvent être modifiés dans les différents modes, par défaut, juste le D sur le roll
    KP_Pitch = 0; KD_Pitch = 0; KI_Pitch = 0;
    KP_Roll = 0; KD_Roll = 0.4;
    KP_Yaw = 0; KD_Yaw = 0;
    KP_Moteur = 0; KP_Moteur = 0;

/***************** Modes de régulation *********************/


 
//////////////////////////////////////// 
// SWITCH C BAS ( 2  //  MODE MANUEL) //
////////////////////////////////////////
     
    if (remote._switch_C == 2) 
    {/***Mode 1: normal, pas de moteur couple, KD sur le roll seulement.***/

      regulation_state = 0;
      
      // timer qui sera utilisé pour passer en mode couple
      time_switch = millis();
      timer_mode = millis() - time_switch;

      // paramètres mode manuel
      KP_Pitch = 0; KD_Pitch = 0; KI_Pitch = 0;
      KP_Roll = 0; KD_Roll = 0.4;
      KP_Yaw = 0; KD_Yaw = 0;
      KP_Moteur = 0; KI_Moteur = 0;
      
      // Mise à 0 des commandes inutilisées pour ce mode
      Commande_dauphin = 0.0;
      Commande_I_flaps = 0;
      Commande_I_Moteur = 0;
      Commande_I_yaw = 0;

      // mise à 0 des flags utilisées pour le dauphin
      flag_bonsai_start = 0;
      flag_dauphin_end = 0;
      flag_declenchement = 0;
      flag_arrondi_final = 0;
      flag_leddar_track = 0;
      
      // mise à jour des consignes avec les valeurs courantes de la BNO
      yaw_des = BNO_lacet; 
      yaw_to_target_lock_offset = bte_ang_180(BNO_lacet-heading);
      pitch_des_f = BNO_pitch;
      v_wind_mean_memory = v_wind_mean;
      slope_des_f = slope_aero_f;
      alti_offset = 0;

    }



///////////////////////////////////////////////
// MODE SWITCH C MILLIEU ( 1 MODE STABILISE) //
///////////////////////////////////////////////
    
    // mode utilisé pour la phase de stabilisation avant le dauphin

    else if (remote._switch_C == 1 || (remote._switch_C == 0 && flag_declenchement == 0) ) 
    { 

      regulation_state = 1;
      
      // timer pour passer en mode 4
      time_switch = millis(); // sert quand on passe au mode appontage
      timer_mode = millis() - time_switch;  // timer_mode = 0 tt le temps.

      // paramètres mode stabilisé
      KP_Pitch = 0.75; KD_Pitch = 0.6; KI_Pitch = 10.0;
      KP_Roll = 3.5; KD_Roll = 0.4;
      KP_Yaw = 2.0; KD_Yaw = 0.3, KI_Yaw = 0.3;
      KP_Moteur = 0.05; KI_Moteur = 0.2;
      
      // mise à 0 des commandes inutilisées
      remote._elevator = 0;
      Commande_dauphin = 0;
      Commande_I_slope = 0;
      flap_state = 1;
      flag_leddar_track = 0;
      slope_des_f = slope_aero_f;
      BNO_roll_f = BNO_roll;
      flag_first_dive = 1;
      v_wind_mean_memory = v_wind_mean;
      heading_to_target_lock = heading_to_target;
      flag_dauphin2 = 0;
      pitch_commutation_prev = -50;
      pitch_commutation_prev_1 = -50;

      // consignes de pitch et de vitesse
      
      pitch_des = 4;
      vitesse_des = VITESSE_DES_DAUPHIN;
      
      yaw_des = heading_to_target;

      //filtrage du yaw pour la régukation en mode dauphin
      yaw_to_target_lock_offset = (1-alpha_roll)*yaw_to_target_lock_offset + alpha_roll*bte_ang_180(BNO_lacet-heading);

//////// Déclanchement de la phase dauphin

      // +10 et +5 pour prendre en compte le début de la courbe
      distance_des = distance_from_alti_raw(GPS_pitot._z_gps+10.0,v_wind_mean_memory);

      if( distance_to_target < distance_des+5.0 )
      {
        flag_declenchement = 1;
        alti_declenchement = GPS_pitot._z_gps;
        distance_declenchement = distance_to_target;
      }

//////// Régulation de vitesse

      Commande_I_Moteur += -KI_Moteur * (GPS_pitot._speed_pitot - vitesse_des) * dt_flt;
      Commande_I_Moteur = constrain(Commande_I_Moteur, 0, 0.4); // sat = 200
    
      Commande_KP_Moteur = -KP_Moteur * (GPS_pitot._speed_pitot - vitesse_des);
      Commande_KP_Moteur = constrain(Commande_KP_Moteur, -0.4, 0.4);
    
      thrust = Commande_KP_Moteur + Commande_I_Moteur;
    
      thrust = constrain(thrust, 0, 0.5);
      //désaturation de l'action intégrale
      Commande_I_Moteur = constrain(Commande_I_Moteur, 0, thrust-Commande_KP_Moteur);

//////// Régulation de pitch

      pitch_des_f = (1 - alpha_stab) * pitch_des_f + alpha_stab * pitch_des; //

//////// Erreure de yaw pour la régulation

      err_yaw_f = bte_ang_180(heading - yaw_des);
      err_yaw_f = constrain(err_yaw_f,-30,30);
    
    }




///////////////////////////////////////////////
//        MODE BONZAI & ARRONDI FINAL        //
///////////////////////////////////////////////

    else if (flag_bonsai_start == 1) 
    { 

      timer_mode = millis() - time_switch;  // timer_mode = 0 tt le temps.

      regulation_state = 4;
      
      // paramètres mode stabilisé
      KP_Pitch = 1.0; KD_Pitch = 0.4; KI_Pitch = 10;
      KP_Roll = 2.5; KD_Roll = 0.2;
      KP_Yaw = 1.0; KD_Yaw = 0.05; KI_Yaw = 0.3;
      KP_Moteur = 0; KI_Moteur = 0;
      K_traj_lat = 3.0;
      
      // mise à 0 des commandes inutilisées et du gaz
      remote._elevator = 0;
      Commande_dauphin=0;
      thrust = 0.0;
      flag_leddar_track = 0;
      yaw_des = yaw_to_target_lock_offset + heading_to_target_lock + K_traj_lat*lateral_distance;
      distance_des = distance_from_alti(alti_,alti_declenchement,distance_declenchement,v_wind_mean_memory);

      hauteur_cabrage = HAUTEUR_CABRAGE;

      if(hauteur_leddar_corrigee < hauteur_cabrage)// && leddar._validity_flag==1 ) // phase pré-cabrage
      {
        flag_arrondi_final = 1;
      }

      if( flag_arrondi_final ) //cabrage final
      {
        
        pitch_des = 0.8/KP_Pitch*360.0;
        pitch_des_f = pitch_des;
        regulation_state = 5;
        thrust = 0.0;
        
      } else
      {
        
        pitch_des = -atan2f(hauteur_leddar_corrigee,longitudinal_distance)*RAD2DEG;
        pitch_des = constrain(pitch_des,-25,0);
        //pitch_des = 0.0;
        
        vitesse_anti_decrochage = 12;
        pitch_des_f = pitch_des;

        thrust_anti_decrochage = 0.45;

        if(GPS_pitot._speed_pitot < vitesse_anti_decrochage && hauteur_leddar_corrigee > 5.0)
        {
          thrust = thrust_anti_decrochage;
        }
        
      }

      pitch_des_f = (1 - alpha_stab) * pitch_des_f + alpha_stab * pitch_des; //

      err_yaw_f = bte_ang_180(BNO_lacet - yaw_des);

    }


    
    
////////////////////////////////////////////
//  SWITCH C HAUT    /!\     DAUPHIN     /!\
////////////////////////////////////////////

    else 
    {

      //time_switch = millis();
      timer_mode = millis() - time_switch;  // timer_mode = 0 tt le temps.

      // paramètres mode dauphin
      KP_Pitch = 0; KD_Pitch = 0; KI_Pitch = 0;
      KP_Roll = 2.5; KD_Roll = 0.3;
      KP_Yaw = 2; KD_Yaw = 0.3; KI_Yaw = 0.3;
      KP_Moteur = 0.05; KI_Moteur = 0.2;
      KI_slope = 0.6;
      K_traj = 2.0; K_traj_lat = 3.0;

      // mises à 0 
      Commande_I_flaps = 0;
      remote._elevator = 0; // désactivation de la télécommande

      // Consignes constantes
      vitesse_des = VITESSE_DES_DAUPHIN;
      yaw_des = yaw_to_target_lock_offset + heading_to_target_lock + K_traj_lat*lateral_distance;

      
//////// Mise à jour des flags de commutation enre les différentes phases

      // détection du leddar < 20m
      if(hauteur_leddar_corrigee<HAUTEUR_LEDDAR && leddar._validity_flag==1 && hauteur_leddar_corrigee>HAUTEUR_LEDDAR/2.0)
      {
        flag_leddar_track = 1;
        alti_offset = hauteur_leddar_corrigee - GPS_pitot._z_gps;
      }

      // detection de la hauteure pour déclancher la deuxième phase de dauphin
      if(flag_leddar_track && alti_<HAUTEUR_DAUPHIN2)
      {
        flag_dauphin2 = 1;
      }

      if(hauteur_leddar_corrigee<HAUTEUR_BONZAI && leddar._validity_flag==1 && flag_leddar_track == 1)
      {
        flag_dauphin_end = 1;
      }

//////// Asservissement de la pente/trajectoire longitudinale pendant le dauphin
 
      // Consigne de pente: Changement de la pente de descente en fonction de la phase de dauphin (1 ou 2)
      if(flag_dauphin2 == 1)
      {
        slope_des = -penteGPS_from_aero(PENTE_AERO_DAUPHIN2,VITESSE_DES_DAUPHIN,v_wind_mean_memory)*RAD2DEG;
        regulation_state = 3;
      } else
      {
        slope_des = -penteGPS_from_aero(PENTE_AERO_DAUPHIN1,VITESSE_DES_DAUPHIN,v_wind_mean_memory)*RAD2DEG;
        regulation_state = 2;
      } 

      // Consigne de distance longitudinale consigne: utilisation de lidar ou gps corrigé en tant que mesure d'altitude pour le calcul de la distance.
      if (leddar._validity_flag==1 && flag_leddar_track==1)
      {
        alti_ = hauteur_leddar_corrigee;
      } else
      {
        alti_ = GPS_pitot._z_gps + alti_offset;
      }
      distance_des = distance_from_alti(alti_,alti_declenchement,distance_declenchement,v_wind_mean_memory);

      // Allumage progressif de la régulation de trajectoire sur les 3 première secondes
      if(timer_mode<3000)
      {
        K_traj = ((float)timer_mode/3000.0)*K_traj;
      }

      // correction de la consigne de pente en fonction de l'erreure longitudinale (action proportionelle seule)
      slope_des += -K_traj*(distance_des-longitudinal_distance);
      slope_des_f = (1 - alpha_slope) * slope_des_f + alpha_slope * slope_des; 

      //asservissement de la pente en fonction du pitch de commutation
      Commande_I_slope += KI_slope * (slope_des_f_delay - slope_aero_f) * dt_flt; 
      Commande_I_slope = constrain(Commande_I_slope, -25, 25);

      pitch_des = slope_des + 20 + Commande_I_slope;
      pitch_des = constrain(pitch_des,-80,5);
      pitch_des_f = pitch_des;
 
//////// Commande Dauphin

      // Au premier plongeon on limite l'amplitude des flaps
      if(flag_first_dive == 1)
      {
        flaps_amplitude_dauphin = 0.4;
        pitch_commutation_prev_1 = pitch_des_f;
      } 
      else
      {
        flaps_amplitude_dauphin = 0.7;
        pitch_des_f = constrain(pitch_des_f, pitch_commutation_prev - 7.5, pitch_commutation_prev + 7.5);
      }

      // loie de commutation
      if (BNO_pitch > pitch_des_f && flap_state!=-1 && BNO_wy > 0.0)
      {
        flap_state=-1;
        pitch_commutation_prev_1 = pitch_des_f;
      } else if (BNO_pitch < pitch_des_f && flap_state!=1 && BNO_wy < 0.0)
      {
        flap_state=1;
        flag_first_dive = 0;
        pitch_commutation_prev = pitch_commutation_prev_1;
      }

      // condition d'arrêt du dauphin
      if( BNO_pitch > (pitch_des_f) && flag_dauphin_end==1 && BNO_wy > 0.0 ) // conditions réunies pour faire le bonzai
      {
        flag_bonsai_start = 1;
        flap_state=0;
        pitch_des_f = BNO_pitch;
      }

      // commande dauphin.
      Commande_dauphin = ((float)flap_state)*flaps_amplitude_dauphin+flaps_offset_dauphin;

//////// Régulation de vitesse
      
      Commande_I_Moteur += -KI_Moteur * (v_pitot_mean - vitesse_des) * dt_flt;
      Commande_I_Moteur = constrain(Commande_I_Moteur, 0, 0.5); // sat = 200
    
      Commande_KP_Moteur = -KP_Moteur * (v_pitot_mean - vitesse_des);
      Commande_KP_Moteur = constrain(Commande_KP_Moteur, -0.5, 0.5);
    
      thrust = Commande_KP_Moteur + Commande_I_Moteur;
    
      thrust = constrain(thrust, 0, 0.5);
      //désaturation de l'action intégrale
      Commande_I_Moteur = constrain(Commande_I_Moteur, 0, thrust-Commande_KP_Moteur);

//////// Calcul de l'erreure pour la régulation du yaw

      err_yaw_f = bte_ang_180(BNO_lacet - yaw_des);  
    }

////////////////////////////////////////////
////////////////////////////////////////////
////////////////////////////////////////////

    
/*****************Commande générale avec les paramètres définis pour les différents modes *********************/

////// Commande du roll

    Commande_I_yaw += KI_Yaw * err_yaw_f * dt_flt / 360.0;
    Commande_I_yaw = constrain(Commande_I_yaw, -15,15);
      
    roll_des = - KP_Yaw * err_yaw_f
               - KD_Yaw * BNO_wz
               - Commande_I_yaw;
    
    // Commande correspondant au roll, télécommande + P roll + D roll + trim
    Commande_Roll = K_Roll_remote * remote._aileron 
                    + KP_Roll * (BNO_roll-roll_des) / 360.0 
                    + KD_Roll * BNO_wx  / 360.0 
                    + aileron_trim; 

    float saturation;

    if(remote._switch_C!=0 || flag_declenchement == 0)
    {
      saturation = 0.6;
    } else
    {
      saturation = 0.3;
    }
                    
    Commande_Roll = constrain(Commande_Roll,-saturation,saturation);

////// Commande du pitch
    
    // Intégrateur des flaps pour régulation du pitch
    Commande_I_flaps += -KI_Pitch * (BNO_pitch - pitch_des_f) * dt_flt / 360.0; // += addition de la valeur précédente
    Commande_I_flaps = constrain(Commande_I_flaps, -0.2, 0.2);

    // proportionelle
    Commande_P_flaps = - KP_Pitch * (BNO_pitch - pitch_des_f)  / 360.0;

    if(flag_arrondi_final!=1)
    {
      Commande_P_flaps = constrain(Commande_P_flaps,-0.2,0.3);
    }

    // derivee
    Commande_D_flaps = - KD_Pitch * BNO_wy  / 360.0;
    
    Commande_Pitch = - K_Pitch_remote * remote._elevator 
                      + Commande_dauphin  
                      + Commande_P_flaps 
                      + Commande_I_flaps 
                      + Commande_D_flaps 
                      + elevation_trim;
                      
    Commande_Pitch = -Commande_Pitch;

    
////// Construction de pwm servo a partir des commandes roll et pitch autour du zero des servo

    pwm_norm_R = Commande_Roll + Commande_Pitch;
    pwm_norm_L = Commande_Roll - Commande_Pitch;
    
    pwm_norm_prop = thrust; 

/***************** Coupure moteur de sécurité *********************/
       
    // Sécurité, coupure des moteurs sur le switch ou si la carte SD n'est pas présente, ou perte de signal télécommande
    if(  !OK_SDCARD || remote._perte_connection || remote._rudder < 0.0 || first_GPS_data==0 )
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
          dataFile.print(BNO_linear_acc_norm * 100, 0); dataFile.print(";");
  
          dataFile.print(GPS_pitot._x_gps*100.0, 0); dataFile.print(";");
          dataFile.print(GPS_pitot._y_gps*100.0, 0); dataFile.print(";");
          dataFile.print(GPS_pitot._z_gps*100.0, 0); dataFile.print(";");
          
          dataFile.print(v_wind_mean*100,0); dataFile.print(";");
          dataFile.print(hauteur_leddar_corrigee*100,0); dataFile.print(";");
          dataFile.print(yaw_des*10.0,0); dataFile.print(";");
  
          dataFile.print(GPS_pitot._vx_gps*100.0, 0); dataFile.print(";");
          dataFile.print(GPS_pitot._vy_gps*100.0, 0); dataFile.print(";");
          dataFile.print(GPS_pitot._vz_gps*100.0, 0); dataFile.print(";");
          dataFile.print(GPS_pitot._speed_pitot * 100.0, 0); dataFile.print(";");
  
          dataFile.print(pwm_norm_R*1000.0,0); dataFile.print(";");
          dataFile.print(pwm_norm_L*1000.0,0); dataFile.print(";");
          dataFile.print(pwm_norm_prop*1000.0,0); dataFile.print(";");
  
          dataFile.print(pitch_des_f * 10, 0); dataFile.print(";");
          dataFile.print(distance_to_target*1000, 0); dataFile.print(";");
          dataFile.print(err_yaw_f*1000, 0); dataFile.print(";");
  
          dataFile.print(slope_des_f_delay*10,0); dataFile.print(";");
          dataFile.print(distance_des*1000,0); dataFile.print(";");
          dataFile.print(slope_aero_f*10,0); dataFile.print(";");

          dataFile.print(regulation_state); dataFile.print(";");
  
          dataFile.print(remote._switch_F+10*remote._switch_D+100*remote._switch_C + flag_declenchement*1000); dataFile.print(";");

          dataFile.print(longitudinal_distance*1000,0); dataFile.print(";");
          dataFile.print(lateral_distance*1000,0); dataFile.print(";");
          dataFile.print(leddar._validity_flag); dataFile.print(";");
          
  
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
  } // fin de boucle à 100 hz
}

float penteGPS_from_aero(float pente_aero, float vitesse_aero, float vitesse_vent)
{
  return atan2(vitesse_aero*sin(pente_aero),vitesse_aero*cos(pente_aero)-vitesse_vent);
}

float distance_from_alti(float alti, float alti0, float distance0, float wind_speed)
{
  float distance_des_;
  
  if (alti0-alti<5.0)
  {
    distance_des_ = distance0 - (distance0-distance_from_alti_raw(alti0-10, wind_speed))/2 * (1/3.333) * (alti0-alti);
  } else if (alti0-alti<10.0)
  {
    distance_des_ = distance0 - (distance0-distance_from_alti_raw(alti0-10, wind_speed))/2 * (1+1/6.667*(alti0-3.333-alti)) ;
  } else
  {
    distance_des_ = distance_from_alti_raw(alti, wind_speed);
  }
  
  return distance_des_;
}

float distance_from_alti_raw(float alti, float wind_speed)
{

  float pente_dauphin1, pente_dauphin2, pente_dauphin2_1, pente_dauphin2_2,distance_des_;

  pente_dauphin1 = penteGPS_from_aero(PENTE_AERO_DAUPHIN1,VITESSE_DES_DAUPHIN,wind_speed);
  pente_dauphin2 = penteGPS_from_aero(PENTE_AERO_DAUPHIN2,VITESSE_DES_DAUPHIN,wind_speed);
  pente_dauphin2_1 = pente_dauphin2 + 4.0/5.0*(pente_dauphin1-pente_dauphin2);
  pente_dauphin2_2 = pente_dauphin2 + 1.0/5.0*(pente_dauphin1-pente_dauphin2);
    
  if (alti < HAUTEUR_BONZAI)
  {
    distance_des_ = (LONGUEUR_BONZAI-TEMPS_BONZAI*wind_speed)*(alti/HAUTEUR_BONZAI);
  } else if (alti < HAUTEUR_DAUPHIN2_2)
  {
    distance_des_ = (alti-HAUTEUR_BONZAI)/tan(pente_dauphin2_2) + (LONGUEUR_BONZAI-TEMPS_BONZAI*wind_speed);
  } else if (alti < HAUTEUR_DAUPHIN2)
  {
    distance_des_ = (alti-HAUTEUR_DAUPHIN2_2)/tan(pente_dauphin2_1) + (HAUTEUR_DAUPHIN2_2-HAUTEUR_BONZAI)/tan(pente_dauphin2_2) + (LONGUEUR_BONZAI-TEMPS_BONZAI*wind_speed);
  } else
  {
    distance_des_ = (alti-HAUTEUR_DAUPHIN2)/tan(pente_dauphin1) + (HAUTEUR_DAUPHIN2-HAUTEUR_DAUPHIN2_2)/tan(pente_dauphin2_1) + (HAUTEUR_DAUPHIN2_2-HAUTEUR_BONZAI)/tan(pente_dauphin2_2) + (LONGUEUR_BONZAI-TEMPS_BONZAI*wind_speed);
  }

  return distance_des_;
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
