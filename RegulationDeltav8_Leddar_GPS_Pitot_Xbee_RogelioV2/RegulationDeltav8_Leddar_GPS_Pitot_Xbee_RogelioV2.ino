//Mode QUADRI, ventousse  plus deplacement lateral, plus pixy
#include <SD.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Wire.h>
#include "Servo.h"
#include "Regulation.h"
#include "bte_servo.h"
#include "bte_motor.h"
#include "bte_controller.h"

#define UART0 Serial
#define UART1 Serial1
#define UART2 Serial2
#define UART3 Serial3

// SD Card
File dataFile;
bool Open = false, Closed = true;
String dataString;
char *filename;
uint32_t nb_file = 0, counter = 0;
bool  OK_SDCARD = false;
char Num[8];
uint32_t temps_log, dt_log, timer_mode, time_switch;

// appel IMU
Adafruit_BNO055 bno = Adafruit_BNO055();
//constantes
const int H = 50;

//Info BNO55
float BNO_wx, BNO_wy, BNO_wz;
float BNO_roll, BNO_pitch, BNO_lacet;
float WZ[H];
// flitrage data
float BNO_wx_f = 0, BNO_wy_f = 0, BNO_wz_f = 0;
float alpha_BNO_f = 1;

//Servos
Servo Servo_R, Servo_L;
Servo Motor_Prop, Motor_Inclinaison, Motor_Couple;

bte_servo myservo(7);
bte_motor mymotor(12);


uint8_t pin_Servo_R = 2, pin_Servo_L = 23;
uint8_t pin_Mot_Prop = 3, pin_Motor_Inclinaison = 5, pin_Motor_Couple = 22;

uint16_t min_Servo_R = 500, min_Servo_L = 500;
uint16_t max_Servo_R = 1000, max_Servo_L = 1000;
uint16_t zero_Servo_R = 750, zero_Servo_L = 750;

uint16_t min_Mot_Prop = 1000, min_Motor_Inclinaison = 1000, min_Mot_Couple = 1000;
uint16_t max_Mot_Prop = 2000, max_Motor_Inclinaison = 2000, max_Mot_Couple = 2000;
uint16_t zero_Mot_Prop = 1500, zero_Motor_Inclinaison = 1500, zero_Mot_Couple = 1050;

//initialisation
uint16_t pwm_Servo_R = min_Servo_R, pwm_Servo_L = min_Servo_L;
uint16_t pwm_Mot_Prop = min_Mot_Prop, pwm_Motor_Inclinaison = min_Motor_Inclinaison, pwm_Motor_Couple = min_Mot_Couple;

// sera réecrit dans chaque phase de vol.
// Regulation Delta
float K_Roll_commande = 175, K_Roll = 5, KD_Roll = 10, K_Yaw, KD_Yaw;
float K_Pitch_commande = 375, K_Pitch = 0, KD_Pitch = 10, K_Pitch_moteur = 0, KD_Pitch_moteur = 0, K_Pitch_moteur_f = 0, KI_Pitch_moteur, KI_Pitch;

// regulation vitesse
float Commande_I_Moteur;
float Commande_KP_Moteur;
float KI_Moteur = 0.5;
float KP_Moteur = 100;
float vitesse_des = 10;
float sat_KI_Moteur = 200; // saturation
float Offset_gaz_reg;
float Gaz_t;

float vitesse_des_f, vitesse_des_32, vitesse_des_31, Offset_gaz_reg_33, Offset_gaz_reg_32, Offset_gaz_reg_31 ;
float pitch_des_33,  pitch_des_32,  pitch_des_31 , Gaz_init;
float alpha_1sec = 0.01;
//float alpha_3sec = 0.003;
float alpha_3sec = 0.015; // on teste
float alpha_05sec = 0.05;


float BNO_pitch_f=0;
float Commande_P_flapping=0;

// 25-> 120bmp -> 1 er test/ oscilations très grandes
// 30 // bon.... jusqu'à 104 
// 31 // oscile un peu chelou.... jusqu'à 104
// 33-> 90 bmp ->1,5Hz
// 40-> 80 bmp ->1,33Hz
// 50-> 60 bmp ->1Hz
//

// test 1) tsin = 30 // kp = 0.4 // pitch des = 25, 30 35   // réponse : 35° on a encoredes accoups

// test 2) tsin = 30 // kp = 0.4,6,8 // pitch des = 35   // réponse : 04 cool 08 et 06 cambrent ouf, on a encore des accelerations

// test 3) tsin = 30 // kp = 0.4 // pitch des = 40,45,50   // réponse : cool on a 50° de pente @ 50 de pitch desnavec plein de oups de gaz; 7.5 m/s moy

// test 4) tsin = 30 // kp flapping = 0.4 //  TEST: alpha_3sec = 0.005 & pitch des = 55/60/65;  REPONSE : 65 OK // filtre OK

// test 5) tsin = 30 // kp flapping = 0.4 // pitch des = 65; alpha_3sec = 0.005 //  TEST: 3 gaz differents   REPONSE : gaz Millieu

// gaz proportionnel avec valeurs plus faibles

// gaz constant 


float kp_plapping;

const int t_sin = 30; // demi période /////////////////////////////////////////////////////////////////
int32_t counter_cycle;
int8_t arrondi_ready,flap_state_mem,flap_state = 1;
float flaps_amplitude = 250.0, hyst_width = 0;
const int n_filtre=2*t_sin;

float filtre[n_filtre];
float total_filtre=0;
int Rn=1;
int R2,R1;
int n_kp_flaps, n_kpi_flaps, n_ki_flaps;
float i_flaps;
int t_sin_f;

int zero_un=0;
float zero_un_f=0;


float R = 0, sign=1;

float zero_un_1sec = 0 , zero_un_2sec = 0, zero_un_3sec = 0;
float Cas = 100;

float alpha_K = 0.015, alpha_K_stab = 0.025,  alpha_K_stab2 = 0.001, filter_n; // filtres des transitions  n=1,8/tau

// Commande = dernière étape avant PWM
float Commande_Roll, Commande_Pitch, Commande_Pitch_moteur, Commande_sin;
float Commande_P_Pitch_moteur, Commande_D_Pitch_moteur, Commande_I_Pitch_moteur, Commande_P_flaps, Commande_I_flaps, Commande_D_flaps;
float moteur_couple_moy, moteur_couple_moy_f;


// des = consigne
float pitch_des = 0, pitch_des_f=0;
float yaw_des = 0;
float offset_motor_couple = 0, offset_motor_couple_f = 0;

// angles flaps selon chaque config
float Flaps_offset = 0, Flaps_offset_f = 0; // angle des flaps en microsecondes

float flaps_moyen_33, flaps_moyen_32, flaps_moyen_31, flaps_moyen, flaps_moyen_0;

float pwm_Motor_Couple_f=1000;

// valeur télécommande normalisées
float roll_t, pitch_t, Gaz, knob, knob2;
//knob entre 0 et 1


//Régulation Assiette

uint16_t Switch_C_previous;


//Offsets
float RollOffs = 0, PitchOffs = 0, YawOffs = 0; //en deg
uint8_t last_Switch_F = 1;

//Envoi vers PC via XBee
const uint8_t NbDataXB = 11;
int16_t DataXB[NbDataXB];

int16_t DataXB1[10];
int16_t DataXB2[10];
int16_t DataXB3[3];
uint8_t count_XB = 1;

//autres
uint32_t temps, temps2, dt, time_idx;
uint16_t buffer_uint16;
uint8_t *ptr_buffer_uint16 = ( uint8_t *)&buffer_uint16;
float roll = 0, pitch = 0;
uint32_t temps_tmp, dt1, dt2, dt3, dt4;






void setup()
{
  Serial.begin(115200);
  UART1.begin(115200); // lecture radio télécommande
  UART2.begin(38400); // Lecture Leddar
  UART3.begin(57600); //Lecture GPS

  /*******Initialisation BNO**************/
  if ( !bno.begin() )
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  temps = micros();
  while ( (micros() - temps) < 1000000 ) {};
  bno.setExtCrystalUse(true);
  /***************************************/

  /*******Initialisation Moteurs************/
  Servo_R.attach(pin_Servo_R);
  Servo_R.writeMicroseconds(zero_Servo_R);

  Servo_L.attach(pin_Servo_L);
  Servo_L.writeMicroseconds(zero_Servo_L);

  Motor_Prop.attach(pin_Mot_Prop);
  Motor_Prop.writeMicroseconds(min_Mot_Prop);

  Motor_Couple.attach(pin_Motor_Couple);
  Motor_Couple.writeMicroseconds(min_Mot_Couple);

  Motor_Inclinaison.attach(pin_Motor_Inclinaison);
  Motor_Inclinaison.writeMicroseconds(min_Motor_Inclinaison);
  /*****************************************/

  myservo.set_range(500,1500);
  mymotor.set_range(1000,2000);

  //Logger
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

  }

  temps = micros();
  time_idx = 0;
  PitchOffs = 2.0;

  //temps_pitot = micros();
}






void loop()
  {
  lectureRadio();
  lectureLeddar();
  lectureGPS();
  
  dt = micros() - temps;
  
  if ( dt > 10000){
    
    //        Serial.print(dt);Serial.print(" ");
    temps += dt;
    
    
    
    /********Info de la télécommande***************/
    
    // références normalisées
    roll_t  = (100.0 - Roll) / 100.0;
    pitch_t = (Pitch - 100.0) / 100.0;
    Gaz_t = (200.0 - Collectif) / 200.0; /// a changé de Gaz à gaz_t
    knob = constrain(Trim / 100.0, 0, 1);
    knob2 = constrain((129.0 - Queue) / 59.0, 0, 1);
    
    /********Info de BNO55***************/
    
    temps_tmp = micros();
    
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    
    HistoriqueVal(euler.z()*RAD2DEG, WZ);
    Mean(WZ, &BNO_wz, 20, H - 1);
    BNO_wz = euler.z() * RAD2DEG;
    BNO_wy = euler.y() * RAD2DEG;
    BNO_wx = euler.x() * RAD2DEG;
    
    // filtrage du gyro ( ne fait rien )
    BNO_wx_f = (1 - alpha_BNO_f) * BNO_wx_f + alpha_BNO_f * BNO_wx;
    BNO_wy_f = (1 - alpha_BNO_f) * BNO_wy_f + alpha_BNO_f * BNO_wy;
    BNO_wz_f = (1 - alpha_BNO_f) * BNO_wz_f + alpha_BNO_f * BNO_wz;
    
    imu::Vector<3> eulAng = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    
    //RollOffs = -4*knob+2; // offset roll sur le knob .  zero de la centrale sur roll
    
    BNO_roll = ang_180(-eulAng.z() - RollOffs);
    BNO_pitch = ang_180(-eulAng.y() - PitchOffs);
    BNO_lacet = ang_180(-eulAng.x() - YawOffs);
    
    roll = -BNO_roll * DEG2RAD; // variables utilisées par le Leddar ( pas faire gaffe à roll et pitch )
    pitch = -BNO_pitch * DEG2RAD;
    
    
    
    /*****************Paramètres de régulation par defaut*********************/
    // peuvent être modifiés dans les différents modes, par défaut, juste le D sur le roll
    
    K_Pitch = 0;
    K_Roll = 0;
    KD_Roll = 0.25;
    KD_Pitch = 0;
    //KI_Pitch = 0;   /////
    K_Yaw = 0;
    KD_Yaw = 0;
    Flaps_offset = 0; // offsets sur proffondeur
    pitch_des = 0;
    K_Pitch_moteur = 0;
    KD_Pitch_moteur = 0;
    
    /*****************Modes de régulation *********************/
    
    
    
    
    ///////////////////////////////////// 214 250 411
    /////////////////////////////////////
    /////////////////////////////////////
    // SWITCH C BAS ( 3  //  MODE MANUEL)
    /////////////////////////////////////
    /////////////////////////////////////
    /////////////////////////////////////
    
    
    if (Switch_C == 1) {/***Mode 1: normal, pas de moteur couple, KD sur le roll seulement.***/
      
      // timer qui sera utilisé pour passer en mode couple
      time_switch = millis();
      timer_mode = millis() - time_switch;
      
      // offset sur les flaps (remplace le trim en pitch)
      Flaps_offset_f = 20;

      
      // sauvegarde des données qui serons utilisé comme valeur de référence lors des changement de mode
      pwm_Motor_Couple = min_Mot_Couple;
      offset_motor_couple_f = 0;
      Commande_I_flaps = 0;
      Commande_I_Moteur = 0;
      yaw_des = BNO_lacet;
      pitch_des_f = BNO_pitch; // commande proportionelle = 0 ( à peu près équivalent à kp=0 )
      Gaz = Gaz_t;
      pwm_Motor_Couple_f=1080;
      Commande_P_flapping=0;
      BNO_pitch_f=BNO_pitch;
      
      zero_un=0;
      zero_un_f=0;
      arrondi_ready = 0;
    }
    
    
    
    ////////////////////////////////////////////
    ////////////////////////////////////////////
    ////////////////////////////////////////////
    // MODE SWITCH C MILLIEU   ( MODE STABILISE)
    ////////////////////////////////////////////
    ////////////////////////////////////////////
    ////////////////////////////////////////////
    
    
    else if (Switch_C == 2  || arrondi_ready == 1) { /** Mode 2: comme mode 1 + sécurité sur la vitesse **/
      
      // timer pour passer en mode 4
      time_switch = millis(); // sert quand on passe au mode appontage
      timer_mode = millis() - time_switch;  // timer_mode = 0 tt le temps.
      
      
      // sauvegarde des données qui serons utilisé comme valeur de référence lors des changement de mode
      Flaps_offset = 20;
      offset_motor_couple_f = 0;
      pwm_Motor_Couple = min_Mot_Couple;
      Commande_I_Pitch_moteur = 0;
      
      
      // REGULATION ROLL & YAW
      
      // Régulation P sur le roll ajouté au D
      K_Roll = 5;
      
      // PD sur le yaw (option)
      K_Yaw = 0; //3.75;
      KD_Yaw = 0; //0.375;
      
      
      // REGULATION FLAPS PITCH
      
      //pitch_des = 20*knob2-10;  // valeur écrite sur écran divisée par 10
      pitch_des = 4;
      
      pitch_des_f = (1 - alpha_K_stab) * pitch_des_f + alpha_K_stab * pitch_des; //
      
      // régulation PID pitch
      K_Pitch = 1.5;
      KD_Pitch = 0.2;
      KI_Pitch = 0.25;
      
      // Intégrateur des flaps pour régulation du pitch
      Commande_I_flaps += -KI_Pitch * (BNO_pitch - pitch_des_f); // += addition de la valeur précédente
      Commande_I_flaps = constrain(Commande_I_flaps, -150, 150);
      
      // filtrage de l'offset de flaps, partie intégrale non filtrée
      Flaps_offset_f = (1 - alpha_K_stab) * Flaps_offset_f + alpha_K_stab * Flaps_offset;
      
      
      // Commande Gaz Rogelio
      
      // Valeurs ok pour 10 m/s
      KI_Moteur = 1.5;  // 1.5 ???
      KP_Moteur = 300; // 200 => +500ms avec delta v de 2,5
      vitesse_des = 10;

//      if(Switch_F==1)
//      {
//        vitesse_des = 10;
//      } else if(Switch_F==2)
//      {
//        vitesse_des = 8;
//      } else
//      {
//        vitesse_des = 6;
//      }
      
      Offset_gaz_reg = 240;
      
      Commande_I_Moteur += -KI_Moteur * (Speed_Avion_med - vitesse_des);
      Commande_I_Moteur = constrain(Commande_I_Moteur, -200, 200); // sat = 200
      
      Commande_KP_Moteur = -KP_Moteur * (Speed_Avion_med - vitesse_des);
      Commande_KP_Moteur = constrain(Commande_KP_Moteur, -500, 500);
      
      Gaz = (Offset_gaz_reg + Commande_KP_Moteur + Commande_I_Moteur) / 1000;
      
      Gaz = constrain(Gaz, 0, 1);
      
      // limites
      if (Speed_Avion_med < 7.5) {Gaz = 1;}
      if (Speed_Avion_med > 12.5) {Gaz = 0.1;}
      
      //Gaz = 0;
      R=0;
      pwm_Motor_Couple_f=1080;
      
      //Commande_I_flaps=0;
      BNO_pitch_f=BNO_pitch;
      Commande_P_flapping=0;

      zero_un=0;
      zero_un_f=0;

      counter_cycle = 0;
      flap_state = 1;
    
    }
    
    
    
    

 
    ////////////////////////////////////////////
    ////////////////////////////////////////////
    ////////////////////////////////////////////
    //  SWITCH C HAUT    /!\    T W E R K    /!\
    ////////////////////////////////////////////
    ////////////////////////////////////////////
    ////////////////////////////////////////////
    
 //   /*
    
    else{
      
      // Mises à zero
      K_Pitch = 0;
      KD_Pitch = 0;
      KI_Pitch = 0;
      pwm_Motor_Couple_f = 1000;
      Commande_I_flaps =0;
      Commande_P_flaps =0;

      K_Roll = 10*knob;

      pitch_des = -10;
      hyst_width = 5;
            
       if (Switch_F == 3){
        
        flaps_amplitude = 220.0;
        
       } else if (Switch_F == 2){
        
        flaps_amplitude = 240.0;
        
      } else {
        
        flaps_amplitude = 250.0;
        
      }

      //REGULATIO YAW
      //if( knob2 > 0.5) { K_Yaw = 3.75; KD_Yaw = 0.375;} else {K_Yaw = 0; KD_Yaw = 0;}


      // REGULATION VITESSE
      /*
      if(Speed_Avion_med>8) {Gaz = 0;}
      if(Speed_Avion_med<8) {Gaz = 0.2;}
      if(Speed_Avion_med<7.5) {Gaz = 0.3;}
      if(Speed_Avion_med<7) {Gaz = 0.4;}
      if(Speed_Avion_med<6.5) {Gaz = 0.5;}
*/
//   Gaz = 0.2;
      
      
      // REGULATION PITCH FLAPS

      pitch_des_f = (1 - alpha_3sec) * pitch_des_f + alpha_3sec * pitch_des; // filtrage consigne ok, ca tend bien vers pitch des
      //pitch_des_f=-20;
      
      // REGULATION initiale amplitude FLAPS
      zero_un_f = (1 - alpha_3sec) * zero_un_f + alpha_3sec * 1;
      //zero_un_f =zero_un_f+0.01;
      zero_un_f =constrain(zero_un_f,0,1);
      
//      if(zero_un_f < 0.9)
//      {
//        counter_cycle++;
//        if(counter_cycle > 30)
//        {
//          flap_state *= -1;
//          counter_cycle = 0;
//        }
//      } else
//      {
//        counter_cycle = 0;
//        if( flap_state=-1 && (BNO_pitch < (pitch_des_f + hyst_width)) )
//        {
//          flap_state=1;
//        } else if( flap_state=1 && (BNO_pitch > (pitch_des_f - hyst_width)) )
//        {
//          flap_state=-1;
//        }
//      }

      if (BNO_pitch > (pitch_des_f + hyst_width))
      {
        flap_state=-1;
        flap_state_mem = -1;
        
        
      } else if (BNO_pitch < (pitch_des_f - hyst_width))
      {
        flap_state=1;
        flap_state_mem = 1;
        
      } else
      {
        if(flap_state_mem == -1 && flap_state==-1)
        {
          flap_state=1;
          //Serial.println("top1");
        }
        if(flap_state_mem == 1 && flap_state==1)
        {
          flap_state=-1;
          counter_cycle++;
          if(counter_cycle>=30)
          {
            arrondi_ready = 1;
          }
          //Serial.println("top2");
        }
      }

      Flaps_offset_f = zero_un_f*((float)flap_state)*flaps_amplitude;

          Serial.print(flap_state);Serial.print("\t");
          Serial.print(BNO_pitch);Serial.print("\t");
          Serial.print(pitch_des);Serial.print("\t");
          Serial.print((pitch_des + hyst_width));Serial.print("\t");
          Serial.print(zero_un_f);Serial.print("\t");
          Serial.println(Flaps_offset_f);
      
//
//      // filtre moyenne sur un cycle de sinusoide
//      total_filtre = total_filtre + BNO_pitch - filtre[Rn]; 
//      filtre[Rn]=BNO_pitch;
//      Rn=Rn+1; if (Rn==n_filtre+1) {Rn=1;}
//      BNO_pitch_f= total_filtre/(n_filtre) ;
//
//      
//      // Calculs commande proportionelle
//      n_kp_flaps = round(kp_plapping*(BNO_pitch_f - pitch_des_f));
//      n_kp_flaps=constrain(n_kp_flaps,-t_sin+3,t_sin-3);       // saturation à .03 sec près
//      //n_kp_flaps =0;
//
//      
//      // Calcul commande intégrale
//      i_flaps +=  0.01*knob* (BNO_pitch_f - pitch_des_f);
//      i_flaps=constrain(i_flaps,-t_sin*.3,t_sin*.3);
//      n_ki_flaps= round(i_flaps);
//      n_ki_flaps= 0;
//
//      n_kpi_flaps=n_ki_flaps+n_kp_flaps;
//      n_kpi_flaps=constrain(n_kpi_flaps,-t_sin + 3, t_sin-3);
//      
//      
//      // Calculs commande min/max
//      // débatements max servos : +-250
//      
//      R=R+1;
//      R1=R1+1;
//      R2=R2+1;
//      
//      if (R==t_sin) {       // calcul cycles de base
//        sign=sign*-1; R=0;
//        if (sign==1) {R1=0;} else {R2=0;}
//      }
//      
//      if ((R1 < t_sin-n_kpi_flaps) && ((R2<R1) || (sign==1))) {// calcul PWM
//        Flaps_offset_f= 300*zero_un_f;} 
//        else  {
//          Flaps_offset_f= -300*zero_un_f;
//          }

    }
  
   // */

////////////////////////////////////////////
////////////////////////////////////////////
////////////////////////////////////////////

    
    /*****************Commande générale avec les paramètres définis pour les différents modes *********************/
    
    // Commande correspondant au roll, télécommande + P roll + D roll + P yaw + D yaw
    Commande_Roll = K_Roll_commande * roll_t  + K_Roll * BNO_roll + KD_Roll * BNO_wx_f + K_Yaw * ang_180(BNO_lacet - yaw_des) + KD_Yaw * BNO_wz_f; //
    
    // Commande correspondant au pitch (intégrateur est dans "Flaps_offset_tt"
    Commande_P_flaps = - K_Pitch * (BNO_pitch - pitch_des_f);
    Commande_D_flaps = - KD_Pitch * BNO_wy_f;
    Commande_Pitch = Flaps_offset_f - K_Pitch_commande * pitch_t + Commande_P_flaps +Commande_I_flaps + Commande_D_flaps; //+ Commande_P_flapping;
    
    // Construction de pwm servo a partir des commandes roll et pitch autour du zero des servo
    pwm_Servo_R = zero_Servo_R + Commande_Roll + Commande_Pitch;
    pwm_Servo_L = zero_Servo_L + Commande_Roll - Commande_Pitch;
    
    //pwm_Mot_Prop = (max_Mot_Prop - min_Mot_Prop) * Gaz + min_Mot_Prop;
    
    pwm_Mot_Prop = 1000 + 1000 * Gaz; // nvelle formule + offset gaz ?
    
    pwm_Motor_Inclinaison = min_Motor_Inclinaison;  // C'est quoi ?
    
    // Sécurité, coupure des moteurs sur le switch ou si la carte SD n'est pas présente, ou perte de signal télécommande
//    /*
    if(Switch_D !=3 || !OK_SDCARD || perteConection){
      pwm_Motor_Inclinaison = min_Motor_Inclinaison;
      pwm_Motor_Couple = min_Mot_Couple;
      pwm_Mot_Prop = min_Mot_Prop;
    }
//    */
    /*****************saturation des pwm ***********************/
    
    pwm_Motor_Inclinaison = constrain(pwm_Motor_Inclinaison, min_Motor_Inclinaison, max_Motor_Inclinaison);
    
    pwm_Motor_Couple = constrain(pwm_Motor_Couple_f, min_Mot_Couple, max_Mot_Couple);
    pwm_Servo_R = constrain(pwm_Servo_R,min_Servo_R,max_Servo_R);
    pwm_Servo_L = constrain(pwm_Servo_L,min_Servo_L,max_Servo_L);

    if(Switch_D==1)
    {
      pwm_Servo_R = 2000;
      pwm_Servo_L = 2000;
    }
    
    pwm_Mot_Prop = constrain(pwm_Mot_Prop, min_Mot_Prop, max_Mot_Prop);
   // pwm_Mot_Prop=1000;
    /********************************************************/
    
    
    /*************Ecriture PWMs Moteurs*****************/
    Servo_R.writeMicroseconds(pwm_Servo_R);
    Servo_L.writeMicroseconds(pwm_Servo_L);
    Motor_Prop.writeMicroseconds(pwm_Mot_Prop);
    Motor_Inclinaison.writeMicroseconds(pwm_Motor_Inclinaison);
    Motor_Couple.writeMicroseconds(pwm_Motor_Couple);
    
    Serial.println(hauteur_leddar);
    /*** Log sur la carte SD ***/
    
    if (OK_SDCARD) {
        if (!Open) {
          strcpy(filename, "log");
          sprintf(Num, "%lu", nb_file);
          strcat(filename, Num); strcat(filename, ".txt");
          
          dataFile = SD.open(filename, FILE_WRITE);
          
          //                Serial.print(Switch_D);
          //                Serial.print(" Opened : ");
          //                Serial.println(filename);
          
          nb_file++;
          Open = true;
          Closed = false;
          temps2 = millis();
        }
        
      else if (Open && !Closed) {
        temps_log = millis();
        dataFile.print(temps_log); dataFile.print(";");
        
        dataFile.print(BNO_roll * 10, 0); dataFile.print(";");
        dataFile.print(BNO_pitch * 10, 0); dataFile.print(";");
        dataFile.print(BNO_lacet * 10, 0); dataFile.print(";");
        
        dataFile.print(knob); dataFile.print(";");
        dataFile.print(knob2); dataFile.print(";");
        
        dataFile.print(Flaps_offset_f , 0); dataFile.print(";");
        dataFile.print(pitch_des_f * 10, 0); dataFile.print(";");
        
        dataFile.print(vx_gps, 0); dataFile.print(";");
        dataFile.print(vy_gps, 0); dataFile.print(";");
        dataFile.print(vz_gps, 0); dataFile.print(";");
        
        dataFile.print(pwm_Servo_R); dataFile.print(";");  // pwm_Servo_R est un entier, ne pas préciser aucun chiffre après virgule "  ,0 . "
        dataFile.print(pwm_Servo_L); dataFile.print(";");
        dataFile.print(hauteur_leddar,0); dataFile.print(";");
        dataFile.print(Speed_Avion_med * 100.0, 0); dataFile.print(";");
        dataFile.print(pwm_Motor_Couple); dataFile.print(";");
        
        dataFile.print(Commande_Roll, 0); dataFile.print(";");
        dataFile.print(Commande_Pitch, 0); dataFile.print(";");
        
        // seules trucs à changer
        
        //dataFile.print(Commande_P_flaps,0); dataFile.print(";");   // si c'est un entier, ne pas mettre le ,0
        //dataFile.print(Commande_D_flaps,0); dataFile.print(";");
        //dataFile.print(Commande_I_flaps,0);
        /*
        dataFile.print(t_sin*10, 0); dataFile.print(";");
        //dataFile.print(Commande_I_flaps, 0); dataFile.print(";");
        dataFile.print(BNO_pitch_f*10, 0); dataFile.print(";");
        dataFile.print(BNO_pitch*10, 0);
*/
        
        dataFile.print(BNO_pitch_f*10,0); dataFile.print(";");  //bleu
        dataFile.print(Flaps_offset_f/10,0); dataFile.print(";");  // vert
        dataFile.print(n_kp_flaps*10);        // rouge 

        /*
        dataFile.print(n_kp_flaps*10); dataFile.print(";");
        //dataFile.print(Commande_I_flaps, 0); dataFile.print(";");
        dataFile.print(Flaps_offset_f, 0); dataFile.print(";");
        dataFile.print(R*10, 0);
        */
               
        dataFile.println(";"); // gaffe à la dernière ligne
        
        
        if ((millis() - temps2 > 300.0 * 1000) || (Switch_C != Switch_C_previous) ){
          dataFile.close();
          Closed = true;
          Open = false;
        }
      
      }
    }
    
    Switch_C_previous = Switch_C;
    
    /************************************************************************/
    /**********************Envoi vers PC Station******************************/
    
//    DataXB[0] = BNO_roll * 100 ;
//    DataXB[1] = BNO_pitch * 100;
//    DataXB[2] = BNO_lacet * 10;
//    
//    DataXB[3] = x_gps / 100.0;
//    DataXB[4] = y_gps / 100.0;
//    DataXB[5] = z_gps / 100.0;
//    
//    DataXB[6] = vx_gps;
//    DataXB[7] = vy_gps;
//    DataXB[8] = vz_gps;
//    
//    DataXB[9] = pwm_Servo_R - zero_Servo_R;
//    DataXB[10] = pwm_Servo_L - zero_Servo_L;
//    DataXB[11] = pwm_Mot_Prop - min_Mot_Prop;
//    DataXB[12] = pwm_Motor_Couple - zero_Mot_Couple;
//    DataXB[13] = pitch_des_f * 10;
//    
//    DataXB[14] = hauteur_leddar;
//    DataXB[15] = Speed_Avion_med * 100;
//    DataXB[16] = Switch_C * 100;
//    DataXB[17] = Switch_D * 100;
//    DataXB[18] = (int16_t)(dt / 100);
//    DataXB[19] = 0;
//    DataXB[20] = 0;
//    DataXB[21] = 0;
//    DataXB[NbDataXB - 1] = 0;

    DataXB[0] = BNO_roll * 100 ;
    DataXB[1] = BNO_pitch * 100;
    DataXB[2] = BNO_lacet * 10;
    
    DataXB[3] = Commande_Pitch;
    DataXB[4] = Flaps_offset_f;
    DataXB[5] = z_gps / 100.0;
    
    DataXB[6] = vx_gps;
    DataXB[7] = vy_gps;
    DataXB[8] = vz_gps;
    
    DataXB[9] = pitch_des_f * 10;
    DataXB[NbDataXB - 1] = Speed_Avion_med * 100; 
    
    
    
    //Envoi des données vers le PC
    
    
        UART2.write(137);
        for (int i = 0; i < NbDataXB ; i++)
        {
            send_in16_t(UART2, DataXB + i);
        }
        
        UART2.write(173);
    
  
  
      /************************************************************************/
      /*********************Debugging****************/
  
      //    Serial.print(x_gps);Serial.print("\t");
      //    Serial.print(y_gps);Serial.print("\t");
      //    Serial.print(z_gps);Serial.print("\t");
      //    Serial.print(vx_gps);Serial.print("\t");
      //    Serial.print(vy_gps);Serial.print("\t");
      //    Serial.println(vz_gps);
  
      //    Serial.print(BNO_roll);Serial.print("\t");
      //    Serial.print(BNO_pitch);Serial.println("\t");
  
      //Serial.println(Speed_Avion_med);
  
  
      //    Serial.print(BNO_roll);Serial.print("\t");
      //    Serial.print(BNO_pitch);Serial.println("\t");
  

  }

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









// PROGRAMME EN 2 TEMPS: 1)PHASE STABILISATION    2) PHASE COUPLE


// PHASE 2 ( virer if(0)
/*

        if(0)// Mode 4: au bout de X secondes en mode 3, on passe en mode 4 (couple), idem mode 3 + moteur couple + sortie flaps
        //if(Switch_C==3 && timer_mode>3000 )
        {
          // timer de mode
          timer_mode = millis() - time_switch;

          // offset moteur (0-1000)
          offset_motor_couple = 220+48*Speed_Avion_med;

           // régulation PID pitch
          K_Pitch = 1.5;
          KD_Pitch = 0.2;
          KI_Pitch = 0.25;

          // Régulation P sur le roll ajouté au D
          K_Roll = 5;

          // PD sur le yaw (option)
          K_Yaw = 0; //3.75;
          KD_Yaw = 0; //0.375;

          // consigne de pitch
          pitch_des = 20*knob2-20;

          // différentes valeur de sortie des flaps sur le switch de droite
          if(Switch_F==1)
          {
            Flaps_offset = -150;
          } else if(Switch_F==2)
          {
            Flaps_offset = -175;
          } else
          {
            Flaps_offset = -200;
          }

          // filtrage de la consigne d'angle, de l'offset des flaps et de l'offset moteur couple
          pitch_des_f = (1-alpha_K)*pitch_des_f + alpha_K*pitch_des;
          Flaps_offset_f = (1-alpha_K)*Flaps_offset_f + alpha_K*Flaps_offset;
          offset_motor_couple_f = (1-alpha_K)*offset_motor_couple_f + alpha_K*offset_motor_couple;

          // Pwm moteur couple saturé
          pwm_Motor_Couple = offset_motor_couple_f+zero_Mot_Couple;
          pwm_Motor_Couple = constrain(pwm_Motor_Couple,zero_Mot_Couple,max_Mot_Couple);

          // intégrateur sur les flaps
          Commande_I_flaps += -KI_Pitch*(BNO_pitch-pitch_des_f);
          Commande_I_flaps = constrain(Commande_I_flaps,-150,150);

          // filtrage de l'offset de flaps, partie intégrale non  +remontée des flaps avec la télécommande pour pouvoir faire l'arrondie final
          Flaps_offset_f = (1-alpha_K_stab)*Flaps_offset_f + alpha_K_stab*Flaps_offset;

          Flaps_offset_tt = Flaps_offset_f + Commande_I_flaps - 600*pitch_t ;

          // on met la valeur de la télécommande à 0 pour qu'il ne soit pas repris en compte dans la commande générale (déja utilisé au dessus)
          pitch_t = 0;


        }

          // démarage transition linéaire
            if(Cas!=33){ Cas=33; R=0; Gaz_init=Gaz;}

            // Transitions
            R=R+1;
            zero_un_1sec=R/100; if (zero_un_1sec>1){ zero_un_1sec=1;}
            zero_un_2sec=R/200; if (zero_un_2sec>1){ zero_un_2sec=1;}
            zero_un_3sec=R/300; if (zero_un_3sec>1){ zero_un_3sec=1;}

          // timer pour passer en mode 4
          timer_mode = millis() - time_switch;



            if(Cas!=32){Cas=32; R=0; }

            if(Cas!=31){ Cas=31; R=0;}

            R=R+1;
            zero_un_1sec=R/100; if (zero_un_1sec>1){ zero_un_1sec=1;}
            zero_un_2sec=R/200; if (zero_un_2sec>1){ zero_un_2sec=1;}
            zero_un_3sec=R/300; if (zero_un_3sec>1){ zero_un_3sec=1;}
            Gaz=zero_un_3sec;


          R=R+1;

            zero_un_1sec=R/100; if (zero_un_1sec>1){ zero_un_1sec=1;}
            zero_un_2sec=R/200; if (zero_un_2sec>1){ zero_un_2sec=1;}
            zero_un_3sec=R/300; if (zero_un_3sec>1){ zero_un_3sec=1;}

*/




    ////////////////////////////////////////////
    ////////////////////////////////////////////
    ////////////////////////////////////////////
    //  SWITCH C HAUT    /!\   APPONTAGE    /!\
    ////////////////////////////////////////////
    ////////////////////////////////////////////
    ////////////////////////////////////////////
    
 /*
    
    else{  // Mode 3: Stabilisation PID sur le pitch, PD sur le roll, PD sur le yaw (option)
      
      // Régulation P sur le roll ajouté au D
      K_Roll = 5;
      
      // PD sur le yaw (option)
      K_Yaw = 0; //3.75;
      KD_Yaw = 0; //0.375;
      
      // sauvegarde des données qui serons utilisé comme valeur de référence lors des changement de mode
      Flaps_offset = 20;
      //offset_motor_couple_f = 0;
      //pwm_Motor_Couple = min_Mot_Couple;
      Commande_I_Pitch_moteur = 0;
      
      
      //////////////////////////////////
      //////////////////////////////////
      //////////////////////////////////
      
      if (Switch_F == 3){
        
        // roll & yaw
        
        // Moteur proulsion
        vitesse_des= 9.5;
        KI_Moteur = 1.5;
        KP_Moteur = 300;
        Offset_gaz_reg = 240;
        
        // Pitch
        pitch_des=20 * knob2 - 20; // de -10 à 10 
        //pitch_des=20 * knob2 - 30; // de -30 à -10 
        //pitch_des=20 * knob2 - 50; // de -50 à -30 
        
        Flaps_offset = .3*300;  // max = ~300 ms
        
        // Moteur couple
        pwm_Motor_Couple = 1350;
      
      }
      
      //////////////////////////////////
      //////////////////////////////////
      //////////////////////////////////
      
      else if (Switch_F == 2){
        
        
        // roll & yaw
        
        // Moteur proulsion
        vitesse_des= 9.5;
        KI_Moteur = 1.5;
        KP_Moteur = 300;
        Offset_gaz_reg = 240;
        
        // Pitch
        pitch_des=20 * knob2 - 20; // de -10 à 10 
        //pitch_des=20 * knob2 - 30; // de -30 à -10 
        //pitch_des=20 * knob2 - 50; // de -50 à -30 
        
        Flaps_offset = .4*300;  // max = ~300 ms
        
        // Moteur couple
        pwm_Motor_Couple = 1400;
      }
       
      //////////////////////////////////
      //////////////////////////////////
      //////////////////////////////////
      
      else {
        
        // roll & yaw
        
        // Moteur proulsion
        vitesse_des= 9.5;
        KI_Moteur = 1.5;
        KP_Moteur = 300;
        Offset_gaz_reg = 240;
        
        // Pitch
        pitch_des=20 * knob2 - 20; // de -10 à 10 
        //pitch_des=20 * knob2 - 30; // de -30 à -10 
        //pitch_des=20 * knob2 - 50; // de -50 à -30 
        
        Flaps_offset = .6*300;  // max = ~300 ms
        
        // Moteur couple
        pwm_Motor_Couple = 1500;

        
      }
      
      
      // REGULATION VITESSE
      
      // filtrages
      vitesse_des_f=(1-alpha_3sec)*vitesse_des_f + alpha_3sec*vitesse_des;  
      
      
      Commande_I_Moteur += -KI_Moteur*(Speed_Avion_med-vitesse_des_f);
      Commande_I_Moteur=constrain(Commande_I_Moteur,-200,200); // sat = 200
      
      Commande_KP_Moteur = -KP_Moteur*(Speed_Avion_med-vitesse_des_f);
      Commande_KP_Moteur=constrain(Commande_KP_Moteur,-500,500);
      
      Gaz = (Offset_gaz_reg + Commande_KP_Moteur + Commande_I_Moteur)/1000;
      
      Gaz=constrain(Gaz,0,1);
      
      // limites
      if(Speed_Avion_med<8.5) {Gaz = 1;}
      if(Speed_Avion_med>12.5) {Gaz = 0.1;}
      
      //Gaz=0;
      
      
      
      // REGULATION MOTEUR COUPLE 
      
      // filtrages
      
        pwm_Motor_Couple_f=(1-alpha_3sec)*pwm_Motor_Couple_f + alpha_3sec*pwm_Motor_Couple;
      
      // REGULATION PITCH FLAPS
     // pitch_des = 20 * knob2 - 10; // valeur écrite sur écran divisée par 10
     // pitch_des = 0;
      
      
      pitch_des_f = (1 - alpha_3sec) * pitch_des_f + alpha_3sec * pitch_des;
      
      // régulation PID pitch
      K_Pitch = 1.5;
      KD_Pitch = 0.2;
      KI_Pitch = 0.25;
      
      // Intégrateur des flaps pour régulation du pitch
      Commande_I_flaps += -KI_Pitch * (BNO_pitch - pitch_des_f); // += addition de la valeur précédente
      Commande_I_flaps = constrain(Commande_I_flaps, -300, 300);
      
      // filtrage de l'offset de flaps, partie intégrale non filtrée
      Flaps_offset_f = (1 - alpha_3sec) * Flaps_offset_f + alpha_3sec * Flaps_offset;
      
    }

 */



///////
