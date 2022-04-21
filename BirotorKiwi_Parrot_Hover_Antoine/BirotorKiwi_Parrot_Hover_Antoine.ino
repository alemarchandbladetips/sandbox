//increase buffer rx for gps

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
  
  vect3   ang_offset{0, 0, 0}; // offset de l'IMU

  ValMean e_angle_z_f(4); // instance de la valuer moyenne sur 4 données pour l'erreur yaw 
  
  /******Radios params*************/
  bool ALL_OFF = true;
  bool SET_OFFSET_YAW = false;
  uint8_t last_Switch_F = 2;
  
  /*****Régulation Assiette**********/
  //  gain kp, gain kd, gain ki, valuer initiale pour integrale int_0
  Pid regRoll(0,0,0,0);
  Pid regPitch(0,0,0,0);
  Pid regYaw(0,0,0,0);
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
  float e_vz_gps = 0;

  
  /*********Timers***************/
  uint32_t dt, tt;
  uint32_t dt_bno, tt_bno;
  uint32_t dt_reg, tt_reg;
  uint32_t tt_ina, dt_ina;  
  uint32_t tt_data, dt_data;    
  
  /*********Others ***************/

  
  float input=0;
  
void setup()
{
   Serial.begin(230400);
   Serial1.begin(57600); // GPS 
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

        angle.x = eul_ang.z() - ang_offset.x*DEG2RAD ;
        angle.y = eul_ang.y() - ang_offset.y*DEG2RAD ; 
        angle.z = ang_pi( eul_ang.x() - ang_offset.z*DEG2RAD ); //on applique le module pi

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
        }
          // roll, pitch, et yaw desirés
          angle_d.x = aileron*30.0*DEG2RAD;//+- 30 deg max pour roll
          angle_d.y = elevator*30.0*DEG2RAD; //+- 30 deg max pour pitch
          angle_d.z = rudder*60.0*DEG2RAD; //+- 60 deg max pour yaw
          
          // vitesses angulaires desirées
          rate_d.x = 0; // w_roll 
          rate_d.y = 0; // w_pitch
          rate_d.z = 0; // w_yaw

        //Erreurs des angles 
        e_angle.x = angle_d.x - angle.x;
        e_angle.y = angle_d.y - angle.y;
        e_angle.z = e_angle_z_f.getMean( ang_pi( angle_d.z - angle.z ) );

        //Erreurs des vitesses angulaires
        e_rate.x = rate_d.x - rate.x;
        e_rate.y = rate_d.y - rate.y;
        e_rate.z = rate_d.z - rate.z;

        // erreur de la vitesse de la hauteur
        e_vz_gps = 0 - vz_gps;
        
        //Setting params of PIDs pour l'attitude
        regPitch.set_kp( GAIN[1-1]/10.0);
        regPitch.set_kd( GAIN[2-1]/10.0 );
        
        regRoll.set_kp( GAIN[4-1] );
        regRoll.set_kd( GAIN[5-1] );
        
        regYaw.set_kp( GAIN[7-1]/40.0 );
        regYaw.set_kd( GAIN[8-1]/40.0 );

        //Gains pour la régulation de hauteur
        regZ.set_kp( 0 ); 
        regZ.set_kd( 0 ) ;
        
        // feeding PIDs pour l'attitude(erreur de l'angle, erreur de vitesse)
        regRoll.feed_pid(e_angle.x, e_rate.x);   //cela fait le calcul de la regulation PID
        regPitch.feed_pid(e_angle.y, e_rate.y);
        regYaw.feed_pid(e_angle.z, e_rate.z);

        // alimentation PID hauteur (erreur de position, erreur de vitesse)
        regZ.feed_pid( 0-z_gps, e_vz_gps);
        
        
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
//        Servo1.pwm = Servo1.pwmInit + Servo1.control - GAIN[3-1] + GAIN[9-1]; // 
//        Servo2.pwm = Servo2.pwmInit + Servo2.control + GAIN[3-1] + GAIN[9-1]; //GAIN[3-1] = compensation sur pitch, GAIN[9-1] = comp. sur yaw

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
        DATA_XBEE_TX[21] = 0;                           DATA_XBEE_TX[22] = x_gps*10;                     DATA_XBEE_TX[23] = y_gps*10;
        DATA_XBEE_TX[24] = z_gps*10;                    DATA_XBEE_TX[25] = vx_gps*10;                    DATA_XBEE_TX[26] = vy_gps*10;
        DATA_XBEE_TX[27] = vz_gps*10;                   DATA_XBEE_TX[28] = thrust*10;
        
        XBeeComm.sendData_int16_2(START1, START2, STOP, DATA_XBEE_TX, NbXB_TX);   


    }
    
}
