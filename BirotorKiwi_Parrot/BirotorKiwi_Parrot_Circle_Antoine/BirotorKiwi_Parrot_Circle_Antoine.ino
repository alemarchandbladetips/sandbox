
#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"
#include "utility/imumaths.h"
#include "Adafruit_INA260.h"
#include "Wire.h"
#include "GlobalVariables.h"
#include "RadioSignals.h"
#include "Functions.h"
#include "decodageGPS.h"

  /******Definitions Motor*************/
  MyServo S1, S2, M1, M2;
  
  const int WIDTH_PWM_S = 4000; //periode du pwm pour les servos
  const int WIDTH_PWM = 2500;   // periode du pwm pour les moteurs
  uint16_t const max_servo = 200;// max debattement pour les servos   

  float zero_servo1 = 1518;
  float zero_servo2 = 1582;
  
  //  pin, pwmMin, pwmMax, pwmInit, pwm, control, rot
  motor Servo1  {5, zero_servo1-max_servo, zero_servo1+max_servo, zero_servo1, zero_servo1, 0, 1}; 
  motor Servo2  {4, zero_servo2-max_servo, zero_servo2+max_servo, zero_servo2, zero_servo2, 0, 1};
  
  float pwminit_motor1 = 1080; //pwm initial du motor 1 
  float pwminit_motor2 = 1080;
  
  //  pin, pwmMin, pwmMax, pwmInit, pwm, control, rot
  // gaz limité à 1850 us
  motor Motor1  {3, 1000, 1850, pwminit_motor1, pwminit_motor1, 0, -1}; // Atention! pins des moteurs doivent pas avoir meme timer que servos
  motor Motor2  {2, 1000, 1850, pwminit_motor2, pwminit_motor2, 0,  1};
  
/********* BNO085 *********/
  // Appel IMU BNO055
  Adafruit_BNO055 myIMU = Adafruit_BNO055(Wire1);
  adafruit_bno055_offsets_t  OffsetsBNO;
  imu::Vector<3> gyro, eul_ang;
  imu::Quaternion quater;
  imu::Quaternion aux_quater;  
  imu::Quaternion quater0; 
  imu::Quaternion quat_rate;
  
  vect3   angle_IMU{0, 0, 0}; // roll, pitch, yaw
  vect3   ang_offset{0, 0, 0}; // offset de l'IMU

  ValMean e_angle_z_f(4); // instance de la valuer moyenne sur 4 données pour l'erreur yaw 
  
  /******Radios params*************/
  bool ALL_OFF = true;
  bool SET_OFFSET_YAW = false;
  uint8_t last_Switch_F = 2;
  
  /*****Régulation Assiette**********/
  //  gain kp, gain kd, gain ki, valuer initiale pour integrale int_0
  Pid regRoll(75,25,0,0);
  Pid regPitch(13.0,3.0,0,0);
  Pid regYaw(6.25,1.875,0,0);
  Pid regZ(0,0,0,0);        // régulation pour la hauteur
        
  vect3  rate{0, 0, 0}; // wx, wy, wz
  vect3  angle{0, 0, 0}; // roll, pitch, yaw
  vect3  angle_d{0, 0, 0}; // angle desiré
  vect3  e_angle{0, 0, 0}; // erreur de l'angle
  vect3  rate_d{0, 0, 0}; // vitesse angulaire desirée
  vect3  e_rate{0, 0, 0}; // erreur de la vitesse angulaire 
  
  float n_p = 0.70; // % dedié pour le pitch de la commande max de servo = 0.70*max_servo
  //variables auxiliaire pour la commande sur pitch, yaw et servos
  float aux_pitch = 0;
  float aux_yaw = 0; 
  float aux_control_S1 = 0;
  float aux_control_S2 = 0;
        
  /**********Capteur V I *********/
  // Mesures Courant, Voltage, Power
  Adafruit_INA260 ina260 = Adafruit_INA260();
  float Courant, Voltage, Power;
  bool INA260_OK = false;
  float I_f, V_f, P_f, f_ina = 1.8/50; //f_ina = facteur equivalent à filtre moyenne sur 50 valeurs
  
  
  /**** XBEE DATA ****************/
  RxTxSerial XBeeComm(Serial2,true); // true active la checksum des données reçues
  uint8_t START1 = 137, START2 = 157, STOP = 173;
  
  const int NbXB_RX = 60; // max nombres des octets du buffer reception dès Station Xbee
  int16_t DATA_XBEE_RX[NbXB_RX];
  
  const int NbXB_TX = 29+3; //DATA + 3 params pour confirmer données reçues depuis Station Xbee
  int16_t DATA_XBEE_TX[NbXB_TX];
  int GAIN[9]={0};

  /*****Hovering**************************/
  float ez_gps = 0;
  float e_vz_gps = 0;

  uint8_t last_switch_C = 2;
  uint8_t last_switch_D = 2; 

  vect3 angHov_ref{0,0,0};
  vect3 rateHov_ref{0,0,0};
  
  /***** Mode Circle *********************/
  bool SET_P0 = false;
  bool SET_P1 = false;
  bool CIRCLE_ON = false;
  vect3 Center{0,0,0};
  vect3 P0{0,0,0};
  vect3 P1{0,0,0};
  float R_circle = 1; //mts
  float Gamma = 0; // angle du vecteur du centre à la position du drone
  float Gamma0 = 0; // angle initial du vecteur centre-position_drone sur répère GPS EstNordUp
  float prop_roll_circle = 0; // partie proportionnelle de la reference roll en mode cercle
  float int_roll_circle = 0; // partie intégrale de la reference roll en mode cercle

  float Vxy_gps = 0;     //vitesse gps horizontal 
  float kw_yaw = 1;      // gain vitesse pour yaw en mode cercle
  float kr_yaw = 1;      // gain proportionnel pour yaw en mode cercle
  float r_drone= 1;      // distance entre le centre et la position du drône
  float V_gamma = 0;     // vitesse en direction centre- position_drone
  float kr_roll = 0;     // gain proportionnel pour roll en mode cercle
  float kw_roll = 0;     // gain vitesse pour roll en mode cercle
  float ki_roll = 0;     // gain intégral pour roll en mode cercle
  
  vect3 angCircle_ref{0,0,0};
  vect3 rateCircle_ref{0,0,0};

  
  

  /*********Timers***************/
  uint32_t dt, tt;
  uint32_t dt_bno, tt_bno;
  uint32_t dt_reg, tt_reg;
  uint32_t tt_ina, dt_ina;  
  uint32_t tt_data, dt_data;    

  //timers pour transition entre mode cercle et hovering
  uint32_t const dt_yaw=2000; // en millis
  uint32_t const dt_pitch=1000;
  uint32_t const dt_roll=1000;
  float f_angle1s = 2.0/100; // filtre correspondant à 1 seg
  float f_angle2s = 2.0/200; // filtre correspondant à 2 seg
  uint32_t dt_transition=0;
  uint32_t tt_angle_hov=0;
  uint32_t tt_angle_cir=0;
  
  /*********Others ***************/

  
  float input=0;
  
void setup()
{
   Serial.begin(230400);
   Serial1.begin(57600); // GPS  increase buffer rx for reading gps
   Serial2.begin(111111); // XBees  
   Serial3.begin(115200); // Radio Telecommande
   
   //Initialisation des Servos(Moteurs)   
   S1.set_resol(12);
   S2.set_resol(12);

   S1.attach_pin_us(Servo1.pin,Servo1.pwmMin,Servo1.pwmMax,WIDTH_PWM_S);
   S2.attach_pin_us(Servo2.pin,Servo2.pwmMin,Servo2.pwmMax,WIDTH_PWM_S);
   
   S1.write_us(Servo1.pwmInit);
   S2.write_us(Servo2.pwmInit);

   M1.set_resol(12);
   M2.set_resol(12);

   M1.attach_pin_us(Motor1.pin,Motor1.pwmMin,Motor1.pwmMax,WIDTH_PWM);
   M2.attach_pin_us(Motor2.pin,Motor2.pwmMin,Motor2.pwmMax,WIDTH_PWM);
   
   M1.write_us(Motor1.pwmMin);
   M2.write_us(Motor2.pwmMin);
   
  // Initialisation BNO_055
   while ( !myIMU.begin() )
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  }
  tt = micros();
  while ( (micros() - tt) < 1.0*1E6 ) {}; 
  myIMU.setExtCrystalUse(true);
  tt = micros();
  while ( (micros() - tt) < 1.0*1E6 ) {}; //pour éviter BNO donne des valuers NAN
  
  // Initial rotation's quaternion YAW-PITCH-ROLL
  float ang0_yaw = -135*DEG2RAD;
  float ang0_pitch = 0*DEG2RAD;
  float ang0_roll = 0*DEG2RAD;
  
  imu::Quaternion quater0_yaw( cosf(ang0_yaw/2.0), 0, 0, sinf(ang0_yaw/2.0) );
  imu::Quaternion quater0_pitch( cosf(ang0_pitch/2.0), 0, sinf(ang0_pitch/2.0), 0 );
  imu::Quaternion quater0_roll( cosf(ang0_roll/2.0), sinf(ang0_roll/2.0), 0, 0 );
  quater0 = quater0_yaw*quater0_pitch*quater0_roll; // attitude initiale de l'IMU par rapport au cap du drône
  
   // Initialisation Capteur INA260
   INA260_OK = ina260.begin(0x40,&Wire);
   
   //Limites regulation asssiette
   regRoll.set_int_lim(100);
   regPitch.set_int_lim(100);
   regYaw.set_int_lim(100);
   regZ.set_int_lim(100);
   regZ.set_reg_lim(300);

   
   // timers
   tt_ina = micros();
   tt_reg = micros();
   tt_bno = micros();
   dt_data = micros();
   
   tt_angle_hov = millis();
   tt_angle_cir = millis();
}


void loop()
{   

   
    /********Lecture du Radio********************/ 
    lectureRadio(Serial3);

    /*************Lecture GPS *******************/
    decodageGPS(Serial1); //augmenter taille buffer RX1 à 1023
    
    /********** Récéption données dès le Xbee PC**************/
   if ( XBeeComm.getData_int16_2( START1, START2, STOP, DATA_XBEE_RX) > 0 )
   {   // on reçoit trois valeurs
       int gainPos = DATA_XBEE_RX[0];
       int gainVal = DATA_XBEE_RX[1];
       int param =   DATA_XBEE_RX[2]; //paramètre optionnel à recevoir dès Station Xbee

       DATA_XBEE_TX[NbXB_TX-3] = gainPos;
       DATA_XBEE_TX[NbXB_TX-2] = gainVal;
       DATA_XBEE_TX[NbXB_TX-1] = param;

       if ( DATA_XBEE_RX[0] > 0 ) // condition indicant on a reçu un gain dès Station Xbee
       { 
         GAIN[ gainPos-1] =  gainVal;
       }
   }

     /******** Lecture BNO_055 à 100 Hz **************/
    dt_bno = micros() - tt_bno;
    if ( dt_bno > 10000)
    {
      tt_bno += dt_bno;

      //Info de BNO55  
        // getting attitude in quaternions
        aux_quater   = myIMU.getQuat();
        quater = aux_quater*quater0.conjugate();
        eul_ang = quater.toEuler(); // en rad

        angle_IMU.x = eul_ang.z() - ang_offset.x*DEG2RAD ;
        angle_IMU.y = eul_ang.y() - ang_offset.y*DEG2RAD ; 
        angle_IMU.z = ang_pi( eul_ang.x() - ang_offset.z*DEG2RAD ); //on applique le module pi

        //angular velocities
        gyro = myIMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE); // en deg/sec
        imu::Quaternion aux_quat_rate(0,gyro); // transformant vecteur gyro en quaternion
        quat_rate = quater0*aux_quat_rate*quater0.conjugate(); // quaternion gyro dans le sytème du cap du drône
        
        rate.x =  quat_rate.x()*DEG2RAD;
        rate.y =  quat_rate.y()*DEG2RAD;
        rate.z =  quat_rate.z()*DEG2RAD;

    }


    dt_ina = micros() - tt_ina;
    if (dt_ina > 10'000)
    {
        tt_ina += dt_ina;
        if ( INA260_OK ) // si le capteur VI a été détecté au démarrage
        { 
          Courant = ina260.readCurrent()/1000.0;
          Voltage = ina260.readBusVoltage()/1000.0;
          Power   = Courant*Voltage;
        }

        I_f += f_ina*(Courant-I_f);
        V_f += f_ina*(Voltage-V_f);
        P_f += f_ina*(Power-P_f);
        
    }

    /************Boucle de régulation********************/
    dt_reg = micros() - tt_reg;
    if (dt_reg > 10'000)
    {
        tt_reg += dt_reg;

        /*********Régulation du Drone*****************************/
        // Etat de la Radio Telecommande
        ALL_OFF = (switch_F==2); // si vrai tout est éteint
        SET_OFFSET_YAW = (switch_F != 2 && last_Switch_F == 2);
        last_Switch_F = switch_F;
        
        //Offset Yaw
        if (SET_OFFSET_YAW) 
        {
          ang_offset.z = RAD2DEG*ang_pi( eul_ang.x() );
        }
  
        if (ALL_OFF)
        { // reset des offset et des valeurs initales des intégrales
          ang_offset.z = 0;
            
          regRoll.reset_int();
          regPitch.reset_int();
          regYaw.reset_int();

          regZ.reset_int();
          int_roll_circle = 0; 
        }

        // Points et centre du cercle
        if (last_switch_C==2 && switch_C==1)
        {
          SET_P0 = true; //point 0 du cercle sauvegardé
          P0.x = x_gps;
          P0.y = y_gps;
          P0.z = z_gps;
        }
        else if (last_switch_C==1 && switch_C==0 && SET_P0)
        {
          SET_P1 = true; // point 1 du cercle  sauvegardé
          P1.x = x_gps;
          P1.y = y_gps;
          P1.z = z_gps;
          
          // Calcul du centre du cercle
          Center.x = (P0.x + P1.x)/2.0;
          Center.y = (P0.y + P1.y)/2.0;
          Center.z = (P0.z + P1.z)/2.0;

          R_circle = 0.5*getMod(P1.y-P0.y,P1.x-P0.x); //PO et P1 donne le diamètre du cercle
          Gamma0 = atan2f(P1.y-P0.y,P1.x-P0.x);       // angle vecteur centre_position_drone
        }
        else if (last_switch_D==2 && switch_D==1 && SET_P0 && SET_P1) // declenche le mode circle
        {
          CIRCLE_ON = true; // mode cercle activé
        }
        
        if (switch_C == 2) 
        {
          SET_P0 = false;
          SET_P1 = false;
        }
        if (switch_D == 2) // Arrêt mode Circle
        {
          CIRCLE_ON = false;
          int_roll_circle = 0;
          regZ.reset_int();
        }

        //mise à jour des valeur des last switch
        last_switch_C = switch_C;
        last_switch_D = switch_D;

        /******** Angles et vitesse angulaires de référence pour mode cercle et hovering**********/
        if (CIRCLE_ON) // mode circle
        { 
          Vxy_gps = getMod(vy_gps, vx_gps );                    //vitesse gps horizontale gps
          r_drone = getMod( y_gps - Center.y, x_gps - Center.x); //distance centre-position_drone
          Gamma = atan2f( y_gps - Center.y, x_gps - Center.x);  // Angle vecteur centre-position_drone
          V_gamma = -cosf(Gamma)*vx_gps - sinf(Gamma)*vy_gps;   // Vitesse sur direction radial

          // gains pour la partie roll
          kr_roll = GAIN[4-1]/20.0;
          kw_roll = GAIN[5-1]/50.0;
          ki_roll = GAIN[6-1]/100.0;

          //  gains pour la partie yaw
          kr_yaw = GAIN[7-1]/50;
          kw_yaw = GAIN[8-1]/100.0;

          // calcul partie proportionnel et intégrale pour roll
          prop_roll_circle = kr_roll*(R_circle - r_drone);
          sat (prop_roll_circle,-10,10);                         //saturé à 10 deg

          int_roll_circle += ki_roll*(R_circle - r_drone)*0.010; // intégré chaque  10 ms
          sat(int_roll_circle,-5,5);                             // saturé à  5 deg

          //angles références en mode cercle
          angCircle_ref.x = (prop_roll_circle + int_roll_circle)*DEG2RAD;
          angCircle_ref.y = GAIN[3-1]*DEG2RAD;                        // fixer depuis meguno
          angCircle_ref.z = Gamma + d90 + kr_yaw*(r_drone-R_circle)*DEG2RAD; // angle référence du drône sur repère gps

          // vitesses angulaires références
          rateCircle_ref.x = kw_roll*V_gamma*DEG2RAD;
          rateCircle_ref.y = 0;
          rateCircle_ref.z = kw_yaw * (Vxy_gps/R_circle); // estimation de w_yaw = Vxy/R = rad/seg, valide si drône fait le cercle
          
          angle.x = angle_IMU.x;             // angle roll du drône
          angle.y = angle_IMU.y;             // angle pitch du drône
          angle.z = angle_IMU.z + Gamma0 ;    // angle yaw du drône sur repère gps

          // gains pour régulation hauteur reglé précedemment
          // regler si c'est pas fait
          regZ.set_kp( 0 ) ;
          regZ.set_kd( 80 );
          regZ.set_ki( 8.5 );
          
        }
        else // mode Hovering
        { // angles référence en mode hoverinf
          angHov_ref.x = aileron*30.0*DEG2RAD;
          angHov_ref.y = elevator*30.0*DEG2RAD;
          angHov_ref.z = rudder*60.0*DEG2RAD;
        
          angle.x = angle_IMU.x;      // angle roll du drône
          angle.y = angle_IMU.y;      // angle pitch du drône
          angle.z = angle_IMU.z;      // angle yaw dur drône

          rateHov_ref.x = 0;
          rateHov_ref.y = 0;
          rateHov_ref.z = 0;

          regZ.set_kp( 0 );
          regZ.set_kd( 0 ) ;
          regZ.set_ki( 0 ) ;
          
        }

       /***********Transition des références entre mode Cercle et Hovering*******************/
       if (CIRCLE_ON)  // transition ver mode cercle
       { 
          dt_transition = millis() - tt_angle_hov;
          // transtion du yaw
          if (dt_transition < dt_yaw)
          {
            angle_d.z += f_angle1s*(angCircle_ref.z-angle_d.z) ;
            angle_d.y = angHov_ref.y;
            angle_d.x = angHov_ref.x;

            rate_d.z += f_angle1s*(rateCircle_ref.z - rate_d.z) ;
            rate_d.y = rateHov_ref.y;
            rate_d.x = rateHov_ref.x;

          }
          else if ( dt_transition < (dt_yaw+dt_pitch) )
          { // transtion du pitch
            angle_d.z = angCircle_ref.z;
            angle_d.y += f_angle1s*(angCircle_ref.y-angle_d.y) ;
            angle_d.x = angHov_ref.x;

            rate_d.z = rateCircle_ref.z;
            rate_d.y += f_angle1s*(rateCircle_ref.y-rate_d.y);
            rate_d.x = rateHov_ref.x;
          }
          else if (dt_transition < (dt_yaw +dt_pitch+dt_roll) )
          { // transtion du roll
            angle_d.z = angCircle_ref.z;
            angle_d.y = angCircle_ref.y;
            angle_d.x += f_angle1s*(angCircle_ref.x-angle_d.x) ;

            rate_d.z = rateCircle_ref.z;
            rate_d.y = rateCircle_ref.y;
            rate_d.x += f_angle1s*(rateCircle_ref.x-rate_d.x);
          }
          else
          { // transtion du hovering vers mode cercle terminée
            angle_d.z = angCircle_ref.z;
            angle_d.y = angCircle_ref.y;
            angle_d.x = angCircle_ref.x;

            rate_d.z = rateCircle_ref.z;
            rate_d.y = rateCircle_ref.y;
            rate_d.x = rateCircle_ref.x;
 
          }
          
          tt_angle_cir = millis();

        }
        else  // transition vers mode Hovering
        {
          dt_transition = millis() - tt_angle_cir;
          // transtion du roll
          if ( dt_transition < dt_roll )
          {  
             angle_d.x += f_angle1s*(angHov_ref.x-angle_d.x);
             angle_d.y = angCircle_ref.y;
             angle_d.z = angCircle_ref.z;

             rate_d.x += f_angle1s*(rateHov_ref.x - rate_d.x);
             rate_d.y = rateCircle_ref.y;
             rate_d.z = rateCircle_ref.z ;

          }
          else if( dt_transition < (dt_roll + dt_pitch ) )
          { // transtion du pitch
            angle_d.x = angHov_ref.x;
            angle_d.y += f_angle1s*(angHov_ref.y-angle_d.y);
            angle_d.z = angCircle_ref.z;

            rate_d.x = rateHov_ref.x;
            rate_d.y += f_angle1s*(rateHov_ref.y - rate_d.y);
            rate_d.z = rateCircle_ref.z ;
            
          }
          else if ( dt_transition < (dt_roll + dt_pitch + dt_yaw ) )
          { // transtion du yaw
            angle_d.x = angHov_ref.x;
            angle_d.y = angHov_ref.y;
            angle_d.z += f_angle1s*(angHov_ref.z-angle_d.z);

            rate_d.x = rateHov_ref.x;
            rate_d.y = rateHov_ref.y;
            rate_d.z += f_angle1s*(rateHov_ref.z - rate_d.z) ;
          }
          else
          { // transition du mode cercle vers hovering terminée
            angle_d.x = angHov_ref.x;
            angle_d.y = angHov_ref.y;
            angle_d.z = angHov_ref.z;

            rate_d.x = rateHov_ref.x;
            rate_d.y = rateHov_ref.y;
            rate_d.z = rateHov_ref.z;
          }
          
          tt_angle_hov = millis();  
        }
        
        //Erreurs des angles 
        e_angle.x = angle_d.x - angle.x;
        e_angle.y = angle_d.y - angle.y;
        e_angle.z = e_angle_z_f.getMean( ang_pi( angle_d.z - angle.z ) );

        //Erreurs des vitesses angulaires
        e_rate.x = rate_d.x - rate.x;
        e_rate.y = rate_d.y - rate.y;
        e_rate.z = rate_d.z - rate.z;

        // erreur de la vitesse de la hauteur
        ez_gps = Center.z-z_gps;
        e_vz_gps = 0 - vz_gps;
        
        //Gains fixé précédemment en mode hovering pour l'attitude
        // modifier si c'est pas fait
//        regPitch.set_kp( 130.0/10.0);
//        regPitch.set_kd( 30.0/10.0 );
//        
//        regRoll.set_kp( 75 );
//        regRoll.set_kd( 25 );
//       
//        regYaw.set_kp( 250.0/40.0 );
//        regYaw.set_kd( 75/40.0 );
        
        // feeding PIDs pour l'attitude(erreur de l'angle, erreur de vitesse)
        regRoll.feed_pid(e_angle.x, e_rate.x);   //cela fait le calcul de la regulation PID
        regPitch.feed_pid(e_angle.y, e_rate.y);
        regYaw.feed_pid(e_angle.z, e_rate.z);

        // alimentation PID hauteur (erreur de position, erreur de vitesse)
        // en mode hovering les gains valent zeros
        regZ.feed_pid(ez_gps , e_vz_gps);
        
        /****************Ecriture des pwm aux Servos**********************/
        aux_pitch =  9.5149*regPitch.get_reg(); // scalage en microseg de la régulation  en pitch
        sat(aux_pitch,-n_p*max_servo,n_p*max_servo); //saturation au 70% du debattement max des servos
        
        aux_yaw =  9.5149*regYaw.get_reg();
        sat(aux_yaw, -(max_servo-abs(aux_pitch)), max_servo-abs(aux_pitch) ); // saturation au debattement disponible >= 30% pour yaw
             
        aux_control_S1 = aux_pitch - aux_yaw;
        
        aux_control_S2 = aux_pitch + aux_yaw;
        
        Servo1.control =  - sat(aux_control_S1,-max_servo,max_servo) ;  //saturation de sécurité pour ne pas depassar le debattement max servos
        Servo2.control =    sat(aux_control_S2,-max_servo,max_servo) ;

        // pwms for servos and motors

        Servo1.pwm = Servo1.pwmInit + Servo1.control; // pwm du servo autour de sa position initiale
        Servo2.pwm = Servo2.pwmInit + Servo2.control;
        
        Motor1.control = regRoll.get_reg() + regZ.get_reg(); 
        Motor1.pwm = Motor1.pwmInit + 800*thrust + Motor1.control;

        Motor2.control =  - regRoll.get_reg() + regZ.get_reg(); 
        Motor2.pwm = Motor2.pwmInit + 800*thrust + Motor2.control;

        // Sécurité des moteurs
        if (ALL_OFF  )
        {
          Servo1.pwm = Servo1.pwmInit;
          Servo2.pwm = Servo2.pwmInit;

          Motor1.pwm = Motor1.pwmMin;
          Motor2.pwm = Motor2.pwmMin;
        }

//      Ecriture des  pwms to servo and motors
        S1.write_us(Servo1.pwm);
        S2.write_us(Servo2.pwm);

        M1.write_us(Motor1.pwm);
        M2.write_us(Motor2.pwm);
                
    }
    
    // Envoi des données vers la Station Xbee
    dt_data = micros() - tt_data;
    if (dt_data > 10'000)
    {   
        tt_data += dt_data;    

        /********XBEE Send Data to Station***************/
        DATA_XBEE_TX[0] = angle.x*RAD2DEG*10;           DATA_XBEE_TX[1] = angle.y*RAD2DEG*10;            DATA_XBEE_TX[2] = angle.z*RAD2DEG*10;
        DATA_XBEE_TX[3] = rate.x*RAD2DEG*10;            DATA_XBEE_TX[4] = rate.y*RAD2DEG*10;             DATA_XBEE_TX[5] = rate.z*RAD2DEG*10;
        DATA_XBEE_TX[6] = I_f*1000;                     DATA_XBEE_TX[7] =  V_f*1000;                     DATA_XBEE_TX[8] = P_f*10;
        DATA_XBEE_TX[9] = Servo1.pwm - Servo1.pwmInit;  DATA_XBEE_TX[10] = rate.y*RAD2DEG*10;            DATA_XBEE_TX[11] = Servo2.pwm - Servo2.pwmInit;
        DATA_XBEE_TX[12] = e_angle.z*RAD2DEG*10;        DATA_XBEE_TX[13] = 0;                            DATA_XBEE_TX[14] = 0;
        DATA_XBEE_TX[15] = angle_d.x*RAD2DEG*10;        DATA_XBEE_TX[16] = angle_d.y*RAD2DEG*10;         DATA_XBEE_TX[17] = angle_d.z*RAD2DEG*10;
        DATA_XBEE_TX[18] = regRoll.get_reg();           DATA_XBEE_TX[19] = aux_pitch;                    DATA_XBEE_TX[20] = aux_yaw;
        DATA_XBEE_TX[21] = CIRCLE_ON;                   DATA_XBEE_TX[22] = x_gps*10;                     DATA_XBEE_TX[23] = y_gps*10;
        DATA_XBEE_TX[24] = z_gps*10;                    DATA_XBEE_TX[25] = vx_gps*10;                    DATA_XBEE_TX[26] = vy_gps*10;
        DATA_XBEE_TX[27] = vz_gps*10;                   DATA_XBEE_TX[28] = thrust*10;
        
        XBeeComm.sendData_int16_2(START1, START2, STOP, DATA_XBEE_TX, NbXB_TX);   


    }
    
}
