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
float K_Yaw = 0, KD_Yaw = 0, KI_yaw = 0;
float KP_Moteur = 0, KI_Moteur = 0, Offset_gaz_reg = 0;
float alpha_stab = 0.025; // filtrage au passage en mode stabilisé

//consignes
float pitch_des = 0, pitch_des_f = 0, yaw_des = 0, vitesse_des = 10;

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
float filtre_dauphin=0;

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
float heading_to_target, heading_to_target_lock, distance_to_target;
uint8_t declanchement = 0;

// Survey
float roll_des;
float heading_start;
uint8_t survey_state;
float max_v_wind, min_v_wind, max_heading, min_heading;
float KI_vz, KP_vz, offset_pitch_vz, Commande_I_vz;

/******* debug **************/

float alpha_roll = 0.04;
float BNO_roll_f = 0;

float alpha_vitesse = 0.004;
float vitesse_des_f;

float err_yaw_f;

uint32_t stability_counter;
uint8_t stability_achieved;

//autres
uint32_t temps, temps2, dt, time_idx;
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
    v_wind = vh_pitot_mean - sqrtf(GPS_pitot._vx_gps*GPS_pitot._vx_gps + GPS_pitot._vy_gps*GPS_pitot._vy_gps +GPS_pitot._vz_gps*GPS_pitot._vz_gps);
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
      leddar_track = 0;
      stability_achieved = 0;
      slope_des_f = slope_aero_f;
      arrondi_final = 0;
      palier = 0;
      roll_des = 0;

      // mise à 0 des valeurs utilisées pour le dauphin
      filtre_dauphin=0;
      arrondi_ready = 0;
      arrondi_almost_ready = 0;
      declanchement = 0;
      
      // mise à jour des consignes avec les valeurs courantes de la BNO
      yaw_des = BNO_lacet; 
      heading_to_target_lock = BNO_lacet;
      pitch_des_f = BNO_pitch;

    }





    
    
    ///////////////////////////////////////////////
    // MODE SWITCH C MILLIEU ( 1 MODE STABILISE) //
    ///////////////////////////////////////////////
    
    // mode utilisé pour la phase de stabilisation avant le dauphin

    else if (remote._switch_C == 1 ) 
    { 

      regulation_state = 1;
      
      // timer pour passer en mode 4
      time_switch = millis(); // sert quand on passe au mode appontage
      timer_mode = millis() - time_switch;  // timer_mode = 0 tt le temps.

      // paramètres mode stabilisé
      K_Pitch = 1.0; KD_Pitch = 0.4; KI_Pitch = 15.0;
      K_Roll = 2.5; KD_Roll = 0.2;
      K_Yaw = 3.6; KD_Yaw = 0.75;
      KP_Moteur = 0.1; KI_Moteur = 0.2; Offset_gaz_reg = 0.0;

      // mise à 0 des commandes inutilisées
      Commande_dauphin = 0;
      Commande_I_slope = 0;
      Commande_I_yaw = 0;
      flap_state = 1;
      leddar_track = 0;
      slope_des_f = slope_aero_f;
      BNO_roll_f = BNO_roll;
      vitesse_des_f = GPS_pitot._speed_pitot;
      first_dive = 1;
      v_wind_mean_memory = v_wind_mean;
      heading_start = heading;
      survey_state = 0;
      max_v_wind = 0;
      min_v_wind = 0;
      min_heading = 0;
      max_heading = 0;
      Commande_I_vz = 0;
      survey_state = 0;

      // consignes de pitch et de vitesse
      pitch_des = 4;
      roll_des = 0;
      vitesse_des = 10.0;
      
      Commande_I_Moteur += -KI_Moteur * (GPS_pitot._speed_pitot - vitesse_des) * dt_flt;
      Commande_I_Moteur = constrain(Commande_I_Moteur, 0, 0.7); // sat = 200
    
      Commande_KP_Moteur = -KP_Moteur * (GPS_pitot._speed_pitot - vitesse_des);
      Commande_KP_Moteur = constrain(Commande_KP_Moteur, -0.5, 0.5);
    
      thrust = Offset_gaz_reg + Commande_KP_Moteur + Commande_I_Moteur;
    
      thrust = constrain(thrust, 0, 1);
      
      err_yaw_f = bte_ang_180(BNO_lacet - yaw_des);

      err_yaw_f = constrain(err_yaw_f,-30,30);

      pitch_des_f = (1 - alpha_stab) * pitch_des_f + alpha_stab * pitch_des; //

      // Intégrateur des flaps pour régulation du pitch
      Commande_I_flaps += -KI_Pitch * (BNO_pitch - pitch_des_f) * dt_flt / 360.0; // += addition de la valeur précédente
      Commande_I_flaps = constrain(Commande_I_flaps, -0.4, 0.4);
    
    }


    
    
    ////////////////////////////////////////////
    //  SWITCH C HAUT    /!\     SURVEY     /!\
    ////////////////////////////////////////////

    else 
    {

      time_switch = millis();
      timer_mode = millis() - time_switch;  // timer_mode = 0 tt le temps.
      // paramètres mode stabilisé
      K_Pitch = 1.0; KD_Pitch = 0.4; KI_Pitch = 15.0;
      K_Roll = 2.5; KD_Roll = 0.2;
      K_Yaw = 3.6; KD_Yaw = 0.75;
      KP_Moteur = 0.1; KI_Moteur = 0.2; Offset_gaz_reg = 0.0;
      KP_vz = 0; KI_vz = 0; offset_pitch_vz = 5;
      K_Yaw = 0; KD_Yaw = 0;

//      if (remote._switch_D==2)
//      {
//        offset_pitch_vz = 18;
//      } else if(remote._switch_D==1)
//      {
//        offset_pitch_vz = 15;
//      } else
//      {
//        offset_pitch_vz = 12.5;
//      }

      offset_pitch_vz = 18;

      // mise à 0 des commandes inutilisées
      Commande_dauphin = 0;
      Commande_I_slope = 0;
      Commande_I_yaw = 0;
      BNO_roll_f = BNO_roll;

      // consignes de pitch et de vitesse
      Commande_I_vz += -KI_vz * GPS_pitot._vz_gps * dt_flt;
      pitch_des = -KP_vz*GPS_pitot._vz_gps + Commande_I_vz + offset_pitch_vz;
      vitesse_des = 10.0;

      roll_des = 10.0;

      if(survey_state == 0 && bte_ang_360(heading-heading_start)>180 && bte_ang_360(heading-heading_start)<270)
      { // 1/2 tour effectué
        survey_state = 1;
        K_Yaw = 0; KD_Yaw = 0;
      } else if (survey_state == 1 && bte_ang_360(heading-heading_start)<90)
      { // 1 tour effectué
        survey_state = 2;
        K_Yaw = 0; KD_Yaw = 0;
      } /*else if (survey_state == 2)
      { // Calcul de la direction du vent et de la cible GPS pour le début du virage
        K_Yaw = 0; KD_Yaw = 0;
        max_heading = bte_ang_180(bte_ang_180(bte_ang_180(min_heading+180)-max_heading)/2+max_heading);
        min_heading = bte_ang_180(max_heading+180);
        // target à 300m dos au vent + 20m sur la droite pour pouvoir faire le virage
        gps_target[0] = 300.0*cosf(min_heading*PI/180.0);//+20.0*cosf((min_heading+90.0)*PI/180.0);
        gps_target[1] = 300.0*sinf(min_heading*PI/180.0);//+20.0*sinf((min_heading+90.0)*PI/180.0);
        survey_state = 3;
      } 
      else if (survey_state == 3)
      { // on rejoint le point de virage
        yaw_des = heading_to_target;
        vitesse_des = 15.0;
        roll_des = 0.0;
      } else if (survey_state == 3 && distance_to_target < 20.0)
      { // met à jour la cible GPS
        gps_target[0] = 0.0;
        gps_target[1] = 0.0;
        survey_state = 4;
        roll_des = 0.0;
      } else if (survey_state == 4 && abs(bte_ang_180(heading_to_target-heading)) > 30)
      {
        K_Yaw = 0; KD_Yaw = 0;
      } else if (survey_state == 4 && abs(bte_ang_180(heading_to_target-heading)) < 30)
      {
        survey_state = 5;
        roll_des = 0.0;
      }*/

      //if(survey_state<2)
      {
        if(v_wind > max_v_wind)
        {
          max_v_wind = v_wind;
          max_heading = heading;
        }
        if(v_wind < min_v_wind)
        {
          min_v_wind = v_wind;
          min_heading = heading;
        }
      }
      
//      Commande_I_Moteur += -KI_Moteur * (GPS_pitot._speed_pitot - vitesse_des) * dt_flt;
//      Commande_I_Moteur = constrain(Commande_I_Moteur, 0, 0.7); // sat = 200
//    
//      Commande_KP_Moteur = -KP_Moteur * (GPS_pitot._speed_pitot - vitesse_des);
//      Commande_KP_Moteur = constrain(Commande_KP_Moteur, -0.5, 0.5);
    
//      thrust = Offset_gaz_reg + Commande_KP_Moteur + Commande_I_Moteur;

      //thrust = 1.2*Commande_I_Moteur;
      
      thrust = constrain(thrust, 0, 1);
      
      err_yaw_f = bte_ang_180(heading - yaw_des);

      err_yaw_f = constrain(err_yaw_f,-30,30);

      pitch_des_f = (1 - alpha_stab) * pitch_des_f + alpha_stab * pitch_des; //

      // Intégrateur des flaps pour régulation du pitch
      Commande_I_flaps += -KI_Pitch * (BNO_pitch - pitch_des_f) * dt_flt / 360.0; // += addition de la valeur précédente
      Commande_I_flaps = constrain(Commande_I_flaps, -0.4, 0.4);
      
    }
  
   // */

////////////////////////////////////////////
////////////////////////////////////////////
////////////////////////////////////////////

    
/*****************Commande générale avec les paramètres définis pour les différents modes *********************/
    
    // Commande correspondant au roll, télécommande + P roll + D roll + P yaw + D yaw
    Commande_Roll = K_Roll_remote * remote._aileron 
                    + K_Roll * (BNO_roll-roll_des) / 360.0 
                    + KD_Roll * BNO_wx  / 360.0
                    + K_Yaw * err_yaw_f / 360.0
                    + KD_Yaw * BNO_wz  / 360.0
                    + Commande_I_yaw 
                    + aileron_trim; 
    
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
          
          dataFile.print(v_wind*100,0); dataFile.print(";");
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
  
          dataFile.print(max_v_wind*100,0); dataFile.print(";");
          dataFile.print(min_v_wind*100,0); dataFile.print(";");
          dataFile.print(max_heading*100,0); dataFile.print(";");
          dataFile.print(min_heading*100,0); dataFile.print(";");
  
          dataFile.print(survey_state); dataFile.print(";");
  
          dataFile.print(remote._switch_F+10*remote._switch_D+100*remote._switch_C + declanchement*1000); dataFile.print(";");

          dataFile.print(gps_target[0]*100,0); dataFile.print(";");
          dataFile.print(gps_target[1]*100,0); dataFile.print(";");
  
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
  //Serial.println(filename);
}
