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

/******* constantes fonction distance/altitude **************/

#define VITESSE_DES_DAUPHIN 10.0
#define PENTE_AERO_DAUPHIN1 45.0*DEG2RAD
#define PENTE_AERO_DAUPHIN2 20.0*DEG2RAD
#define HAUTEUR_BONZAI 11.0
#define LONGUEUR_BONZAI 42.0
#define TEMPS_BONZAI 3.0
#define HAUTEUR_DAUPHIN2_2 16.0
#define HAUTEUR_DAUPHIN2 20.0
#define HAUTEUR_PALIER 7.0
#define HAUTEUR_CABRAGE 1.5

float distance_des, alti_, K_traj;
float longitudinal_distance, lateral_distance;

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
const int16_t Ndata = 100;
uint8_t first_GPS_data = 0;
float heading;
float v_horizontal_gps;

float slope_ground_mean;
float slope_ground_buffer[100];

float altitude_stabilisation = 60.0;

/******* leddar **************/

bte_leddar leddar = bte_leddar(&Serial2);

float hauteur_leddar_corrigee;
const float leddar_mounting_vector[3] = {0.7071,0,0.7071};

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

//Offsets
float RollOffs = 0, PitchOffs = 0, YawOffs = -90; //en deg

/******* Variables et paramètres des régulation **************/

const float dt_flt = 0.01;
uint8_t regulation_state;

//// régulation d'attitude et vitesse
// params
float K_Pitch_remote = 0.3, K_Pitch = 0, KD_Pitch = 0, KI_Pitch = 0;
float K_Roll_remote = -0.3, K_Roll = 0, KD_Roll = 0.2;
float K_Yaw = 0, KD_Yaw = 0, KI_Yaw = 0;
float KP_Moteur = 0, KI_Moteur = 0, Offset_gaz_reg = 0;
float alpha_stab = 0.025; // filtrage au passage en mode stabilisé

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
int8_t arrondi_almost_ready,arrondi_ready,flap_state_mem,flap_state = 1, leddar_track = 0, arrondi_final, palier;
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
float heading_to_target, heading_to_target_lock, yaw_to_target_lock, distance_to_target;
uint8_t declanchement = 0;

/******* debug **************/

float alpha_roll = 0.04;
float BNO_roll_f = 0;

float alpha_vitesse = 0.004;

float err_yaw_f;

uint32_t stability_counter;
uint8_t stability_achieved;

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
  
  Servo_L.set_range(850,2150,1500);
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
  GPS_pitot.read_GPS_pitot();

  if(GPS_pitot._x_gps != 0.0 && first_GPS_data != 1)
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
    BNO_wy = gyro_speed.y() * RAD2DEG;
    BNO_wx = gyro_speed.x() * RAD2DEG;

    // lecture de l'acc linéaire (pour log)
    imu::Vector<3> linear_acc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    BNO_linear_acc_norm = sqrtf(linear_acc.x()*linear_acc.x() + linear_acc.y()*linear_acc.y() + linear_acc.z()*linear_acc.z())/9.81;

    // lecture du vecteur gravité pour la projection de la mesure leddar (distance leddar -> altitude)
    imu::Vector<3> gravity_vector = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);

    float produit_scalaire = (gravity_vector.x()*leddar_mounting_vector[0] + gravity_vector.y()*leddar_mounting_vector[1] + gravity_vector.z()*leddar_mounting_vector[2])/ 9.81 ;
    if(produit_scalaire > 0.2)
    {
      hauteur_leddar_corrigee = leddar._hauteur*produit_scalaire;
    }

    // lecture des angles d'Euler (deg)+ application des offset 
    imu::Vector<3> eulAng = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    
    BNO_roll = bte_ang_180(-eulAng.z() - RollOffs);
    BNO_pitch = bte_ang_180(-eulAng.y() - PitchOffs);
    BNO_lacet = bte_ang_180(-eulAng.x() - YawOffs);


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
    slope_des_f_delay = slope_des_f_buffer[0];

/***************** Estimation de la distance à la cible et heading  *********************/

    // distance à la cible
    distance_to_target = sqrtf( (GPS_pitot._x_gps - gps_target[0])*(GPS_pitot._x_gps - gps_target[0]) + (GPS_pitot._y_gps - gps_target[1])*(GPS_pitot._y_gps - gps_target[1]));
    
    // Cap menant à la cible
    heading_to_target = atan2f((GPS_pitot._x_gps - gps_target[0]),-(GPS_pitot._y_gps - gps_target[1]))*RAD2DEG;
    heading = atan2f(-GPS_pitot._vx_gps,GPS_pitot._vy_gps)*RAD2DEG;

    longitudinal_distance = distance_to_target*cos(heading_to_target-heading_to_target_lock);
    lateral_distance = distance_to_target*sin(heading_to_target-heading_to_target_lock);

/***************** Paramètres de régulation par defaut*********************/
    
    // peuvent être modifiés dans les différents modes, par défaut, juste le D sur le roll
    K_Pitch = 0; KD_Pitch = 0; KI_Pitch = 0;
    K_Roll = 0; KD_Roll = 0.2;
    K_Yaw = 0; KD_Yaw = 0;
    KP_Moteur = 0; KP_Moteur = 0; Offset_gaz_reg = 0;

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
      K_Pitch = 0; KD_Pitch = 0; KI_Pitch = 0;
      K_Roll = 0; KD_Roll = 0.2;
      K_Yaw = 0; KD_Yaw = 0;
      KP_Moteur = 0; KI_Moteur = 0; Offset_gaz_reg = 0;
      
      // Mise à 0 des commandes inutilisées pour ce mode
      Commande_dauphin = 0.0;
      Commande_I_flaps = 0;
      Commande_I_Moteur = 0;
      Commande_I_yaw = 0;
      leddar_track = 0;
      stability_achieved = 0;
      slope_des_f = slope_aero_f;
      arrondi_final = 0;
      palier = 0;

      // mise à 0 des flags utilisées pour le dauphin
      arrondi_ready = 0;
      arrondi_almost_ready = 0;
      declanchement = 0;
      
      // mise à jour des consignes avec les valeurs courantes de la BNO
      yaw_des = BNO_lacet; 
      yaw_to_target_lock = BNO_lacet;
      pitch_des_f = BNO_pitch;
      v_wind_mean_memory = v_wind_mean;

    }



///////////////////////////////////////////////
// MODE SWITCH C MILLIEU ( 1 MODE STABILISE) //
///////////////////////////////////////////////
    
    // mode utilisé pour la phase de stabilisation avant le dauphin

    else if (remote._switch_C == 1 || (remote._switch_C == 0 && declanchement == 0) ) 
    { 

      regulation_state = 1;
      
      // timer pour passer en mode 4
      time_switch = millis(); // sert quand on passe au mode appontage
      timer_mode = millis() - time_switch;  // timer_mode = 0 tt le temps.

      // paramètres mode stabilisé
      K_Pitch = 1.0; KD_Pitch = 0.4; KI_Pitch = 15.0;
      K_Roll = 2.5; KD_Roll = 0.2;
      K_Yaw = 1.44; KD_Yaw = 0.3, KI_Yaw = 0.0;
      KP_Moteur = 0.1; KI_Moteur = 0.2; Offset_gaz_reg = 0.0;

      // mise à 0 des commandes inutilisées
      Commande_dauphin = 0;
      Commande_I_slope = 0;
      Commande_I_yaw = 0;
      flap_state = 1;
      leddar_track = 0;
      slope_des_f = slope_aero_f;
      BNO_roll_f = BNO_roll;
      first_dive = 1;
      v_wind_mean_memory = v_wind_mean;
      heading_to_target_lock = heading_to_target;

      // consignes de pitch et de vitesse
      pitch_des = 4;
      vitesse_des = 10.0;
      yaw_des = heading_to_target;

      //filtrage du yaw pour la régukation en mode dauphin
      yaw_to_target_lock = (1-alpha_roll)*yaw_to_target_lock + alpha_roll*BNO_lacet;

//////// Déclanchement de la phase dauphin

      // +10 et +5 pour prendre en compte le début de la courbe
      distance_des = distance_from_alti(GPS_pitot._z_gps+10.0,v_wind_mean_memory);

      if( distance_to_target < distance_des+5.0 )
      {
        declanchement = 1;
      }

//////// Régulation de vitesse

      Commande_I_Moteur += -KI_Moteur * (GPS_pitot._speed_pitot - vitesse_des) * dt_flt;
      Commande_I_Moteur = constrain(Commande_I_Moteur, 0, 0.7); // sat = 200
    
      Commande_KP_Moteur = -KP_Moteur * (GPS_pitot._speed_pitot - vitesse_des);
      Commande_KP_Moteur = constrain(Commande_KP_Moteur, -0.5, 0.5);
    
      thrust = Offset_gaz_reg + Commande_KP_Moteur + Commande_I_Moteur;
    
      thrust = constrain(thrust, 0, 1);

//////// Régulation de pitch

      pitch_des_f = (1 - alpha_stab) * pitch_des_f + alpha_stab * pitch_des; //

      // Intégrateur des flaps pour régulation du pitch
      Commande_I_flaps += -KI_Pitch * (BNO_pitch - pitch_des_f) * dt_flt / 360.0; // += addition de la valeur précédente
      Commande_I_flaps = constrain(Commande_I_flaps, -0.4, 0.4);

//////// Erreure de yaw pour la régulation

      err_yaw_f = bte_ang_180(heading - yaw_des);
      err_yaw_f = constrain(err_yaw_f,-30,30);
      Commande_I_yaw += KI_Yaw * err_yaw_f * dt_flt / 360.0;
      Commande_I_yaw = constrain(Commande_I_yaw, -15,15);
    
    }




///////////////////////////////////////////////
//        MODE BONZAI & ARRONDI FINAL        //
///////////////////////////////////////////////

    else if (arrondi_ready == 1) 
    { 

      timer_mode = millis() - time_switch;  // timer_mode = 0 tt le temps.

      regulation_state = 4;
      
      // paramètres mode stabilisé
      K_Pitch = 1; KD_Pitch = 0.4; KI_Pitch = 15;
      K_Roll = 4.5; KD_Roll = 0.2;
      K_Yaw = 0.9; KD_Yaw = 0.05; KI_Yaw = 0.0;
      KP_Moteur = 0; KI_Moteur = 0; Offset_gaz_reg = 0.0;


      // mise à 0 des commandes inutilisées et du gaz
      Commande_dauphin=0;
      flap_state = 1;
      thrust = 0.0;
      leddar_track = 0;

      BNO_roll_f = (1-alpha_roll)*BNO_roll_f + alpha_roll*BNO_roll;
      BNO_roll = BNO_roll_f;

      if(hauteur_leddar_corrigee < HAUTEUR_CABRAGE && leddar._validity_flag==1 ) // phase pré-cabrage
      {
        arrondi_final = 1;
      }

      if(hauteur_leddar_corrigee < HAUTEUR_PALIER && leddar._validity_flag==1 ) // phase pré-cabrage
      {
        palier = 1;
      }
  
      if( arrondi_final ) //cabrage final
      {
        
        pitch_des = 80.0;
        pitch_des_f = pitch_des;
        regulation_state = 5;
        thrust = 0.0;
        
      } else if ( palier )// phase pré-cabrage
      {
        pitch_des = 0.0;
        
        vitesse_anti_decrochage = 9.5;
        pitch_des_f = pitch_des;

        thrust_anti_decrochage = 0.45;

        if(GPS_pitot._speed_pitot < vitesse_anti_decrochage)
        {
          thrust = thrust_anti_decrochage;
        }
        
      } else
      {
        
        vitesse_anti_decrochage = 10.0;
        pitch_des = -15.0;
        thrust_anti_decrochage = 0.45;

        if(GPS_pitot._speed_pitot < vitesse_anti_decrochage)
        {
          thrust = thrust_anti_decrochage;
        }
        
        pitch_des_f = pitch_des;
      }

      pitch_des_f = (1 - alpha_stab) * pitch_des_f + alpha_stab * pitch_des; //

      err_yaw_f = bte_ang_180(BNO_lacet - yaw_des);
      Commande_I_yaw += KI_Yaw * err_yaw_f * dt_flt / 360.0;
      Commande_I_yaw = constrain(Commande_I_yaw, -15,15);

      // Intégrateur des flaps pour régulation du pitch
      Commande_I_flaps += -KI_Pitch * (BNO_pitch - pitch_des_f) * dt_flt / 360.0; // += addition de la valeur précédente
      Commande_I_flaps = constrain(Commande_I_flaps, -0.5, 0.5);

    }


    
    
////////////////////////////////////////////
//  SWITCH C HAUT    /!\     DAUPHIN     /!\
////////////////////////////////////////////

    else 
    {

      time_switch = millis();
      //timer_mode = millis() - time_switch;  // timer_mode = 0 tt le temps.
      
      // paramètres mode dauphin
      K_Pitch = 0; KD_Pitch = 0; KI_Pitch = 0;
      K_Roll = 4.5; KD_Roll = 0.2;
      K_Yaw = 0.9; KD_Yaw = 0.17; KI_Yaw = 0.0;
      KP_Moteur = 0.1; KI_Moteur = 0.2; Offset_gaz_reg = 0.0;
      KI_slope = 0.3;
      K_traj = 0.0;

      // mises à 0 
      Commande_I_flaps = 0;
      remote._elevator = 0; // désactivation de la télécommande

      // Consignes constantes
      vitesse_des = VITESSE_DES_DAUPHIN;
      yaw_des = yaw_to_target_lock;
      
//////// Asservissement de la pente pendant le dauphin

      // détection du leddar < 20m
      if(hauteur_leddar_corrigee<HAUTEUR_DAUPHIN2 && leddar._validity_flag==1 )
      {
        leddar_track = 1;
      }

      if (leddar._validity_flag==1 && leddar_track==1)
      {
        alti_ = hauteur_leddar_corrigee;
      } else
      {
        alti_ = GPS_pitot._z_gps;
      }
      distance_des = distance_from_alti(alti_,v_wind_mean_memory);

      // Dauphin 1 et Dauphin 2
      if(leddar_track == 1)
      {
        slope_des = -penteGPS_from_aero(PENTE_AERO_DAUPHIN2,VITESSE_DES_DAUPHIN,v_wind_mean_memory);
        regulation_state = 3;
      } else
      {
        slope_des = -penteGPS_from_aero(PENTE_AERO_DAUPHIN1,VITESSE_DES_DAUPHIN,v_wind_mean_memory);
        regulation_state = 2;
      }

      if(time_switch>4000)
      {
        slope_des += -K_traj*(distance_des-longitudinal_distance);
      }

      slope_des_f = (1 - alpha_slope) * slope_des_f + alpha_slope * slope_des; 

      Commande_I_slope += KI_slope * (slope_des_f_delay - slope_ground_mean) * dt_flt; 
      Commande_I_slope = constrain(Commande_I_slope, -20, 20);
      
      pitch_des = slope_des + 12 + Commande_I_slope;
      pitch_des = constrain(pitch_des,-50,30);
      pitch_des_f = pitch_des;

//////// pré-déclanchement de l'arrondi, l'arrondi sera fait à la prochaine oscillation

      if(hauteur_leddar_corrigee<HAUTEUR_BONZAI && leddar._validity_flag==1 )
      {
        arrondi_almost_ready = 1;
      }

//////// Génération du dauphin

      // Au premier plongeon on limite l'amplitude des flaps
      if(first_dive == 1)
      {
        flaps_amplitude = 0.3;
      } else
      {
        flaps_amplitude = 0.5;
      }

      // loie de commutation
      if (BNO_pitch > pitch_des_f)
      {
        flap_state=-1;
      } else
      {
        flap_state=1;
      }

      // condition d'arrêt du dauphin
      if( BNO_pitch > (pitch_des_f + 10) && arrondi_almost_ready==1 && BNO_wy > 0.0 ) // conditions réunies pour faire le bonzai
      {
        arrondi_ready = 1;
        flap_state=0;
        pitch_des_f = BNO_pitch;
        // initialisation de l'action intégrale pour avoir une sortie de dauphin plus propre.
        Commande_I_flaps = 0.4;
      }

      // commande dauphin.
      Commande_dauphin = ((float)flap_state)*flaps_amplitude;

//////// Régulation de vitesse
      
      Commande_I_Moteur += -KI_Moteur * (v_pitot_mean - vitesse_des) * dt_flt;
      Commande_I_Moteur = constrain(Commande_I_Moteur, 0, 0.8); // sat = 200
    
      Commande_KP_Moteur = -KP_Moteur * (v_pitot_mean - vitesse_des);
      Commande_KP_Moteur = constrain(Commande_KP_Moteur, -0.5, 0.5);
    
      thrust = Offset_gaz_reg + Commande_KP_Moteur + Commande_I_Moteur;
    
      thrust = constrain(thrust, 0, 1);

//////// Calcul de l'erreure pour la régulation du yaw

      err_yaw_f = bte_ang_180(BNO_lacet - yaw_des);
      Commande_I_yaw += KI_Yaw * err_yaw_f * dt_flt / 360.0;
      Commande_I_yaw = constrain(Commande_I_yaw, -15,15);

//////// Filtrage du roll pour la régulation

      BNO_roll_f = (1-alpha_roll)*BNO_roll_f + alpha_roll*BNO_roll;
      BNO_roll = BNO_roll_f;
      
    }

////////////////////////////////////////////
////////////////////////////////////////////
////////////////////////////////////////////

    
/*****************Commande générale avec les paramètres définis pour les différents modes *********************/

    roll_des = - K_Yaw * err_yaw_f / 360.0
               - KD_Yaw * BNO_wz  / 360.0
               - Commande_I_yaw;
    
    // Commande correspondant au roll, télécommande + P roll + D roll + P yaw + D yaw
    Commande_Roll = K_Roll_remote * remote._aileron 
                    + K_Roll * (BNO_roll-roll_des) / 360.0 
                    + KD_Roll * BNO_wx  / 360.0 
                    + aileron_trim; 
                    
    Commande_Roll = constrain(Commande_Roll,-0.2,0.2);
    
    // Commande correspondant au pitch
    Commande_P_flaps = - K_Pitch * (BNO_pitch - pitch_des_f)  / 360.0;
    Commande_D_flaps = - KD_Pitch * BNO_wy  / 360.0;
    Commande_Pitch = - K_Pitch_remote * remote._elevator 
                      + Commande_dauphin  
                      + Commande_P_flaps 
                      + Commande_I_flaps 
                      + Commande_D_flaps 
                      + elevation_trim; //+ Commande_P_flapping;

    
    // Construction de pwm servo a partir des commandes roll et pitch autour du zero des servo
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
    pwm_norm_R = constrain(pwm_norm_R,-0.8,1);
    pwm_norm_L = constrain(pwm_norm_L,-1,0.8);
    
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
          dataFile.print(slope_ground_mean*10,0); dataFile.print(";");

          dataFile.print(regulation_state); dataFile.print(";");
  
          dataFile.print(remote._switch_F+10*remote._switch_D+100*remote._switch_C + declanchement*1000); dataFile.print(";");

          dataFile.print(longitudinal_distance*1000,0); dataFile.print(";");
          dataFile.print(lateral_distance*1000,0); dataFile.print(";");
  
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

float distance_from_alti(float alti, float wind_speed)
{

  float pente_dauphin1, pente_dauphin2, pente_dauphin2_1, pente_dauphin2_2;

  pente_dauphin1 = penteGPS_from_aero(PENTE_AERO_DAUPHIN1,VITESSE_DES_DAUPHIN,wind_speed);
  pente_dauphin2 = penteGPS_from_aero(PENTE_AERO_DAUPHIN2,VITESSE_DES_DAUPHIN,wind_speed);
  pente_dauphin2_1 = pente_dauphin2 + 4.0/5.0*(pente_dauphin1-pente_dauphin2);
  pente_dauphin2_2 = pente_dauphin2 + 1.0/5.0*(pente_dauphin1-pente_dauphin2);
    
  if (alti < HAUTEUR_BONZAI)
  {
    distance_des = (LONGUEUR_BONZAI-TEMPS_BONZAI*wind_speed)*(alti/HAUTEUR_BONZAI);
  } else if (alti < HAUTEUR_DAUPHIN2_2)
  {
    distance_des = (alti-HAUTEUR_BONZAI)/tan(pente_dauphin2_2) + (LONGUEUR_BONZAI-TEMPS_BONZAI*wind_speed);
  } else if (alti < HAUTEUR_DAUPHIN2)
  {
    distance_des = (alti-HAUTEUR_DAUPHIN2_2)/tan(pente_dauphin2_1) + (HAUTEUR_DAUPHIN2_2-HAUTEUR_BONZAI)/tan(pente_dauphin2_2) + (LONGUEUR_BONZAI-TEMPS_BONZAI*wind_speed);
  } else
  {
    distance_des = (alti-HAUTEUR_DAUPHIN2)/tan(pente_dauphin1) + (HAUTEUR_DAUPHIN2-HAUTEUR_DAUPHIN2_2)/tan(pente_dauphin2_1) + (HAUTEUR_DAUPHIN2_2-HAUTEUR_BONZAI)/tan(pente_dauphin2_2) + (LONGUEUR_BONZAI-TEMPS_BONZAI*wind_speed);
  }

  return distance_des;
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
