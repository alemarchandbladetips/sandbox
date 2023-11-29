// include bladetips
#include "GlobalVariables.h"
#include "Functions.h"
#include "RadioSignals.h"

// include externe
#include "Adafruit_INA260.h"
//#include "Adafruit_MLX90614.h"

/****** Paramètres principaux de la procédure de test *************/

#define PWM_START_PROCEDURE 1000
#define PWM_STOP_PROCEDURE 1600
#define PWM_STEP_PROCEDURE 30
#define TEMPS_MESURE_PROCEDURE 6000000 //7000000

/****** Gains des capteurs de poids/couple (procédure de calib) *************/

#define GAIN_POIDS 0.3708
//0.377686
//0.3785
#define GAIN_COUPLE 1.374
//-1.354722
//-1.358637

/******Capteur Poids/couple *************/

// timers de calibration pour la tare poids couple
uint32_t dt_calib = 10000000;
uint32_t tt_calib;
uint32_t dt_calib_WIP = 1000000;
uint32_t tt_calib_WIP;

////////// Poids
int pinPoids = 20;

///// mesure et filtrage
// variables
int mesure_poids;
float mesure_poids_f; 
float poids,poids_f;
float var_poids_f, sigma_poids_f;
// param
float aplha_mesure_poids = 0.001;

///// Gain et offset calib
float gain_poids = GAIN_POIDS;
float offset_poids = 0.0;

////////// Couple

int pinCouple = 14;

///// mesure et filtrage
// variables
int mesure_couple;
float mesure_couple_f; 
float couple,couple_f;
float var_couple_f, sigma_couple_f;
// param
float aplha_mesure_couple = 0.001;

///// Gain et offset calib
float gain_couple = GAIN_COUPLE;
float offset_couple = 0.0;

/**********Capteur Tension courant *********/

// Capteur
Adafruit_INA260 ina260 = Adafruit_INA260();
int8_t ina260_init = false;

// Variables mesures
float courant_ina, tension_ina, puissance_ina;
// Variables filtrées
float courant_ina_f, tension_ina_f, puissance_ina_f;
// param filtre
float alpha_ina = 0.02;

/**********Telecommande *********/

int last_switch_C = 0;
uint8_t motor_ON = 0;
uint8_t mode_manuel = 1;

/******Moteur*************/

MyServo M1;
//            pin, pwmMin, pwmMax, pwmInit, pwm, control, rot
motor Motor1  {23,1000, 2000, 1023, 1023, 0, 1};
const int WIDTH_PWM = 4000;

/******Capteur RPM*************/

// timers capteur rpm

uint32_t dt_wait, tt_wait;

uint32_t const mindtwait=1000, maxdtwait=60'000; 
uint32_t temps2wait = maxdtwait;
uint16_t value_portLED=0;
boolean EtatA=HIGH, last_EtatA=HIGH;
float rpm=0;
float dt_filt=1000000;
int nb_pales = 3;//2;
int portLED = 15;
ValMean dt_RPM(100);
float dt_rpm_mean = 1;
float rpm_mean=0;

/***********Reception RPM via port Serie*********/
RxTxSerial SpeedMot(Serial2,0);
float speed_rpm = 0;
/******Capteur température*************/
/*
Adafruit_MLX90614 mlx90614 = Adafruit_MLX90614();
float motor_temp, ambiant_temp;
int8_t mlx_init;
*/
/******Commandes Sérial*************/

// Pour lecture serial
char caractere;

/******Gestion procédure automatique*************/

// Timers procedure de test
uint32_t tt_start_phase;
uint32_t dt_init_filtres = 500000;//500000;
uint32_t dt_mesure = TEMPS_MESURE_PROCEDURE;

uint16_t pwm_procedure;
uint16_t pwm_step_procedure = PWM_STEP_PROCEDURE;
uint16_t pwm_start_procedure = PWM_START_PROCEDURE;
uint16_t pwm_stop_procedure = PWM_STOP_PROCEDURE;
uint8_t init_filtres = 0;
uint8_t procedure_state = 0;

// Regulation thrust

bool reg_thrust = false;
float max_thrust_ref = 1000;
float thrust_ref = 0;
float poids_ref = 0;
float f_p = 0.003;
Pid regThrust(0,0,0,0); //kp,kd,ki,int0
uint32_t tt_reg_thrust;
uint32_t dt_reg_thrust = 1'000'000;
uint32_t max_dt_reg_thrust = 60'000'000;
uint32_t counter_dt = 0;

/******Affichage*************/

// temps d'affichage
uint32_t dt_plot = 10000;
uint32_t tt_plot;
uint8_t enable_plot = 0;
uint8_t enable_timing_error = 1;

/******Gestion timing*************/

// timer d'acquisition poids couple
uint32_t dt_acquisition_PC_rpm = 1000;
uint32_t tt_acquisition_PC_rpm;

//// Downsampling de la boucle de commande moteur, lecture INA et MLXpar rapport à la fréquence d'acquisition poids couple rpm
uint32_t downsampling_counter = 0;
uint32_t downsampling = 10;


/******debug*************/
uint32_t tt_debug;

void setup()
{
  Serial.setTimeout(0);
  Serial.print( "///////////////// SETUP ////////////////" );Serial.print("\n");
  
  //Seria Reception RPM 
  Serial2.begin(115200);

  ///// initialisation serial
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial.print( "Serial OK" );Serial.print("\n");

  ///// Initialisation du moteur
  M1.set_resol(12);
  M1.attach_pin_us(Motor1.pin,Motor1.pwmMin,Motor1.pwmMax,WIDTH_PWM);
  Serial.print( "Moteur OK" );Serial.print("\n");

  ///// résolution des pin analog
  analogReadResolution(12);

  ///// Initialisation INA
  ina260_init = ina260.begin(0x40,&Wire);

  ///// Initialisation MLX
  //mlx_init = mlx90614.begin(MLX90614_I2CADDR,&Wire1);
  
  ///// init filtres
  if(ina260_init)
  { 
    courant_ina_f = ina260.readCurrent()/1000.0; // en Ampère
    tension_ina_f = ina260.readBusVoltage()/1000.0; // en Volts
    Serial.print( "INA260 OK" );Serial.print("\n");
  } else
  {
    Serial.print( "/!\\ INA260 défaillant" );Serial.print("\n");
  }

  ///// Initialisation des filtres poids couple
  reset_PC_filters();
  ///// tare des capteurs
  tare_all();

  ///// Initialisation des timers
  tt_acquisition_PC_rpm = micros();
  downsampling_counter = 0;
  tt_calib = micros();
  tt_plot = micros();
  
  // pid reg thrust
  regThrust.set_kp(3);
  regThrust.set_ki(1.75);
  regThrust.set_reg_lim(700);

  ///////////////////////////////////////////////////////////////////
  Serial.print( "Setup OK" );Serial.print("\n");Serial.print("\n");
  Serial.print( "///////////////////////////////////////////" );Serial.print("\n");
  Serial.print( "Commandes disponible dans le Serial :");Serial.print("\n");
  Serial.print( "'t' tare des capteurs de couple et de poids");Serial.print("\n");
  Serial.print( "'p' affichage des mesures courantes");Serial.print("\n");
  Serial.print( "'e' enable/disable debug plot");Serial.print("\n");
  Serial.print( "'cXX' set thrust target to XXg");Serial.print("\n");Serial.print("\n");
}


void loop()
{  
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
////// Lecture radio
  
  SpeedMot.getData_float(1,137,173,&rpm_mean);
  lectureRadio(Serial1);
  
  if(switch_F == 2)
  {
    motor_ON = 0;
    procedure_state = 0;
    reg_thrust = false;

  } 
  else
  {
    motor_ON = 1;
  }
  
  if( (last_switch_C == 2) && (switch_C !=2 ) && (motor_ON == 1) )
  {
    mode_manuel = 0;
    procedure_state = 1;
    pwm_procedure = pwm_start_procedure;
    init_filtres = 0;
    Serial.print( "///////////////////////////////////////////" );Serial.print("\n");
    Serial.print( "Début de la procédure de test moteur automatique" );Serial.print("\n");Serial.print("\n");
    //tare_all();
    print_output_header();
    tt_start_phase = micros();
    tt_reg_thrust = micros();
    regThrust.reset_int();
    poids_ref = 0;

  } else if ( switch_C==2 )
  {
    if(procedure_state == 1)
    {
      procedure_state = 2;
    }
  }
  last_switch_C = switch_C;


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
////// Boucle principale cadencée à 1000 Hz 
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

  if ( (micros() - tt_acquisition_PC_rpm) > dt_acquisition_PC_rpm )
  { 
    if( ((micros() - tt_acquisition_PC_rpm) > dt_acquisition_PC_rpm+200) && enable_timing_error)
    {
      Serial.print( "/!\\ Timing acquisition poids couple rpm non respecté " );Serial.print((micros() - tt_acquisition_PC_rpm));Serial.print("\n");  
    }

    tt_acquisition_PC_rpm += dt_acquisition_PC_rpm;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
////// lecture et filtrage des capteurs poids et couple 

    // Aquisition, filtrage, correction, ecart type de la mesure de poids
    mesure_poids = analogRead(pinPoids);

    mesure_poids_f = (1.0-aplha_mesure_poids)*mesure_poids_f+aplha_mesure_poids*(float)mesure_poids;

    poids = gain_poids*(mesure_poids - offset_poids);
    poids_f = gain_poids*(mesure_poids_f - offset_poids);

    var_poids_f = (1.0-aplha_mesure_poids)*var_poids_f+aplha_mesure_poids*(poids_f-poids)*(poids_f-poids);
    sigma_poids_f = sqrtf(var_poids_f);

    // Aquisition, filtrage, correction, ecart type  de la mesure de couple
    mesure_couple = analogRead(pinCouple);
    
    mesure_couple_f = (1.0-aplha_mesure_couple)*mesure_couple_f+aplha_mesure_couple*(float)mesure_couple;
    
    couple_f = gain_couple*(mesure_couple_f - offset_couple);
    couple = gain_couple*(mesure_couple - offset_couple);

    var_couple_f = (1.0-aplha_mesure_couple)*var_couple_f+aplha_mesure_couple*(couple_f-couple)*(couple_f-couple);
    sigma_couple_f = sqrtf(var_couple_f);
    

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///// Commande des moteurs

    downsampling_counter++;
    
    if ( downsampling_counter >= downsampling )
    {   
      downsampling_counter = 0;

      if (mode_manuel) // Mode manuel
      {
        Motor1.control = 1000*thrust;
        Motor1.pwm = Motor1.pwmMin + Motor1.control;
        
      } else if(procedure_state == 1) // Procédure de mesure automatique
      {
        

        if (reg_thrust)
        { 
          
          poids_ref += f_p*(thrust_ref-poids_ref);

          if (thrust_ref <0)
          {
            regThrust.feed_pid(poids_f-poids_ref,0);
          }
          else
          {
            regThrust.feed_pid(poids_ref-poids_f,0);
          }
          

          Motor1.control = 0.8*abs(poids_ref) + regThrust.get_reg();
          Motor1.pwm = Motor1.pwmMin + Motor1.control;

          if( micros() - tt_reg_thrust > dt_reg_thrust)
          {
            print_output_line();
            tt_reg_thrust += dt_reg_thrust;
            tt_start_phase = micros();
            init_filtres = 0;

            counter_dt += dt_reg_thrust;
            if (counter_dt > max_dt_reg_thrust)
            {
              procedure_state = 2;
              reg_thrust = false;
              counter_dt = 0;
              regThrust.reset_int();
              poids_ref = 0;
            }
          }
        }
        else
        {
          Motor1.pwm = pwm_procedure;
          
          if( ((micros() - tt_start_phase) > dt_init_filtres) && !init_filtres) // Reset des filtre au bout de 'dt_init_filtres' pour accélérer la convergence
          {
            reset_PC_filters();
            init_filtres = 1;
          }

          if( (micros() - tt_start_phase) > dt_mesure ) // Prise de la mesure et initialisation pour la phase suivante
          {
            print_output_line();
            pwm_procedure += pwm_step_procedure;
            tt_start_phase = micros();
            init_filtres = 0;
            if (pwm_procedure > pwm_stop_procedure) // Condition d'arrêt de la procédure de test
            {
              procedure_state = 2;
            }
          }
        }

       
      } else if(procedure_state == 2) // Ralentissement doux du moteur  
      {
        Motor1.pwm -= 5;
        counter_dt = 0;
        if( Motor1.pwm < Motor1.pwmMin)
        {
          Motor1.pwm = Motor1.pwmMin;
          procedure_state = 3;
          reset_PC_filters();
          tt_start_phase = micros();
          tt_reg_thrust = micros();
          regThrust.reset_int();
          poids_ref = 0;
        }
      } else if(procedure_state == 3) // Affichage de la sortie pour vérification de l'offset
      {
        if( (micros() - tt_start_phase) > dt_mesure )
        {
          rpm_mean = 0;
          print_output_line();
          procedure_state = 0;
          mode_manuel = 1;
          Serial.print("\n");Serial.print( "Fin de la procédure de test moteur automatique" );Serial.print("\n");
          Serial.print( "///////////////////////////////////////////" );Serial.print("\n");Serial.print("\n");
          reg_thrust = false;
          regThrust.reset_int();
          poids_ref = 0;
        }
      }
  
      // Sécurité sur les moteurs, toujours garder juste avant l'écriture des PWM
      if (!motor_ON)
      {
        Motor1.pwm = Motor1.pwmMin;
      }
      // Ecriture des pwms
      M1.write_us(Motor1.pwm);
      
    }
  
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///// Capteur Puissance
    
    if (downsampling_counter == 1)
    {
      if ( ina260_init )
      { 
        courant_ina = ina260.readCurrent()/1000.0; // en Ampère
      }
    }
    if (downsampling_counter == 2)
    {
      if ( ina260_init )
      { 
        tension_ina = ina260.readBusVoltage()/1000.0; // en Volts
      }
      
      puissance_ina   = courant_ina*tension_ina; // En Watts
      courant_ina_f = (1-alpha_ina)*courant_ina_f+alpha_ina*courant_ina;
      tension_ina_f = (1-alpha_ina)*tension_ina_f+alpha_ina*tension_ina;
      puissance_ina_f = (1-alpha_ina)*puissance_ina_f+alpha_ina*puissance_ina;
    }


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///// Capteur Température
/*
    if (downsampling_counter == 3)
    {
      if(mlx_init)
      {
        motor_temp = mlx90614.readObjectTempC();
      }
    }
    if (downsampling_counter == 4)
    {
      if(mlx_init)
      {
        ambiant_temp = mlx90614.readAmbientTempC();
      }
    }
*/
      
  }


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///// Affichage à la fréquence choisie

  if ( (micros() - tt_plot) > dt_plot )
  {   

    tt_plot += dt_plot;
    
    
    // Serial.println(rpm_mean);
    // Serial.println(speed_rpm);

    // Serial.print(poids_ref);Serial.print("\t");
    // Serial.print(poids_f);Serial.print("\t");
    // Serial.print(Motor1.pwm);Serial.print("\t");
    // Serial.print(Motor1.control);Serial.print("\t");
    // Serial.print(knob);Serial.print("\n");

    if(enable_plot)
    {
      Serial.print( poids );Serial.print("\t");
      Serial.print( poids_f );Serial.print("\t");
//      //Serial.print( poids_f+2*sigma_poids_f );Serial.print("\t");
//      //Serial.print( poids_f-2*sigma_poids_f );Serial.print("\t");
      Serial.print( couple );Serial.print("\t");
      Serial.print( couple_f );Serial.print("\t");
      //Serial.print( couple_f+2*sigma_couple_f );Serial.print("\t");
      //Serial.print( couple_f-2*sigma_couple_f );Serial.print("\t");


//      Serial.print( motor_temp );Serial.print("\t");
//      Serial.print( ambiant_temp );Serial.print("\t");

      //Serial.print( courant_ina );Serial.print("\t");
      //Serial.print( courant_ina_f );Serial.print("\t");
      //Serial.print( tension_ina );Serial.print("\t");
      //Serial.print( tension_ina_f );Serial.print("\t");
      //Serial.print( puissance_ina );Serial.print("\t");
      //Serial.print( puissance_ina_f );Serial.print("\t");
      
      //Serial.print( Motor1.pwm );Serial.print("\t");

      //Serial.print( rpm_mean );Serial.print("\t");

//      Serial.print( switch_F );Serial.print("\t");
//      Serial.print( switch_C );Serial.print("\t");
//      Serial.print( thrust );Serial.print("\t");

      Serial.print("\n");
    }
  }


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///// Gestion des commandes du Serial

  if (Serial.available())
  {
    caractere = Serial.read();
    if(caractere == 't' && procedure_state == 0)
    {
      tare_all();
      Serial.print("\n");
    }
    if(caractere == 'p' && procedure_state == 0)
    {
      print_output_header();
      print_output_line();
      Serial.print("\n");
    }
    if(caractere == 'e' && procedure_state == 0)
    {
      enable_plot = !enable_plot;
    }
    if (caractere == 'c')
    { 
      thrust_ref = Serial.parseFloat();
      if (thrust_ref < 0)
      {
        sat(thrust_ref, -max_thrust_ref, 0);
      }
      else
      {
        sat(thrust_ref,0,max_thrust_ref);
      }
      
      Serial.print("thrust_ref = "); Serial.println(thrust_ref); 
      reg_thrust = true;
      counter_dt = 0;
      print_output_line();
      Serial.print("\n");
    }
    while(Serial.available())
    {
      Serial.read();
    }
  }

}

void tare_all()
{
  int32_t dt_acquisition_PC_tmp;
  float aplha_mesure_poids_tmp;
  float aplha_mesure_couple_tmp;

  dt_acquisition_PC_tmp = dt_acquisition_PC_rpm;
  aplha_mesure_poids_tmp = aplha_mesure_poids;
  aplha_mesure_couple_tmp = aplha_mesure_couple;
  
  dt_acquisition_PC_rpm = 1000;
  aplha_mesure_poids = 0.001;
  aplha_mesure_couple = 0.001;
  
  reset_PC_filters();
  tt_calib_WIP = micros();
  tt_calib = micros();
  tt_acquisition_PC_rpm = micros();

  Serial.print("\n");
  Serial.print("Tare des capteur de couple et de poids en cours ");
    
  while ((micros() - tt_calib) < dt_calib )
  {
    if ( (micros() - tt_acquisition_PC_rpm) > dt_acquisition_PC_rpm )
    {   
      tt_acquisition_PC_rpm += dt_acquisition_PC_rpm;
      
      mesure_poids = analogRead(pinPoids);
      mesure_poids_f = (1-aplha_mesure_poids)*mesure_poids_f+aplha_mesure_poids*(float)mesure_poids;

      mesure_couple = analogRead(pinCouple);
      mesure_couple_f = (1-aplha_mesure_couple)*mesure_couple_f+aplha_mesure_couple*(float)mesure_couple;
    }
    if ( (micros() - tt_calib_WIP) > dt_calib_WIP )
    { 
      tt_calib_WIP += dt_calib_WIP;
      Serial.print(".");
    }
  }

  offset_poids = mesure_poids_f;
  offset_couple = mesure_couple_f;
  
  dt_acquisition_PC_rpm = dt_acquisition_PC_tmp;
  aplha_mesure_poids = aplha_mesure_poids_tmp;
  aplha_mesure_couple = aplha_mesure_couple_tmp;

  Serial.print("\n");Serial.print("Tare OK");Serial.print("\n");Serial.print("\n");
  
  return;

}

void reset_PC_filters()
{
  mesure_poids_f = (float)analogRead(pinPoids);
  mesure_couple_f = (float)analogRead(pinCouple);
  var_poids_f = 0;
  var_couple_f = 0;
}

void print_output_line()
{
  Serial.print(Motor1.pwm);Serial.print("\t");
  Serial.print(tension_ina_f);Serial.print("\t");
  Serial.print(courant_ina_f,4);Serial.print("\t");
  Serial.print(poids_f);Serial.print("\t");
  Serial.print(sigma_poids_f);Serial.print("\t");
  Serial.print(couple_f);Serial.print("\t");
  Serial.print(sigma_couple_f);Serial.print("\t");
  Serial.print(rpm_mean);Serial.print("\t");
  Serial.print(0);Serial.print("\t");
  Serial.print(poids_f/(tension_ina_f*courant_ina_f));Serial.print("\n");
}

void print_output_header()
{
  Serial.print( "PWM (us): " );Serial.print("\t");
  Serial.print( "Tension (V): " );Serial.print("\t");
  Serial.print( "Courant (A): " );Serial.print("\t");
  Serial.print( "Thrust (g): " );Serial.print("\t");
  Serial.print( "SigmaT (g): " );Serial.print("\t");
  Serial.print( "Couple (g.cm): " );Serial.print("\t");
  Serial.print( "SigmaC (g.cm): " );Serial.print("\t");
  Serial.print( "rpm (RPM): " );Serial.print("\t");
  Serial.print( "Motor temp (°C): " );Serial.print("\t");
  Serial.print( "g/W: " );Serial.print("\n");
}
