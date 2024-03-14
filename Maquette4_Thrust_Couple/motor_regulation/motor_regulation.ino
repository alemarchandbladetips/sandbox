// include bladetips
#include "GlobalVariables.h"
#include "Functions.h"
#include "RadioSignals.h"


#define Serial_Radio Serial1
/****** Paramètres principaux de la procédure de test *************/
#define PWM_START_PROCEDURE 1100
#define PWM_STOP_PROCEDURE 1900//2000
#define PWM_STEP_PROCEDURE 100
#define TEMPS_MESURE_PROCEDURE 7000000 //7000000

/****** Gains des capteurs de poids/couple (procédure de calib) *************/
#define GAIN_POIDS 0.3708
#define GAIN_COUPLE 1.352


/******Moteur*************/

MyServo M1;
//            pin, pwmMin, pwmMax, pwmInit, pwm, control, rot
motor Motor1  {23,1000, 2000, 1023, 1023, 0, 1};
const int WIDTH_PWM = 2500;
// motor Motor1  {23,125, 250, 125, 125, 0, 1};
// const int WIDTH_PWM = 250;

/******Capteur Poids/couple *************/
// timers de calibration pour la tare poids couple
uint32_t dt_calib = 10000000;
uint32_t tt_calib;
uint32_t dt_calib_WIP = 1000000;
uint32_t tt_calib_WIP;

////////// Poids
int pinPoids = 21;

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

int pinCouple = 20;

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

/**********Telecommande *********/
int last_switch_C = 0;
uint8_t motor_ON = 0;
uint8_t mode_manuel = 1;



/***********Reception RPM via port Série*********/
int START_BYTE = 137;
int STOP_BYTE = 173;
// RxTxSerial SpeedMot(Serial2,0);
RxTxSerial Serial_RPM(Serial2,0);
uint32_t temps_rpm; 
uint16_t rpm_mot=0;

int const  NB_DATA_RPM = 3;// temps_rpm(uint32_t)+rpm_mot(uint16_t) = 3*int16_t
int16_t serial_data_rpm[NB_DATA_RPM];

/**********Reception puissance via port Série ****************/
RxTxSerial Serial_VI(Serial4,0);
uint32_t temps_vi;
float courant_ina[2];
float tension_ina[2];
float courant_ina_f, tension_ina_f;
float current_mot, tension_mot;
float puissance_ina, puissance_ina_f;
float alpha_ina = 0.02/10;

int const  NB_DATA_VI = 6;// temps_vi(uint32_t)+2*courant(uint16_t)+2*tension(uint16_t) = 6*int16_t
int16_t serial_data_vi[NB_DATA_VI];

/******Commandes Sérial*************/
// Pour lecture serial
char caractere;

/******Gestion procédure automatique*************/
int16_t PWM_tab[11] = {1000 , 1100 , 1200 , 1300 , 1350 , 1400 , 1450 , 1500 , 1600 , 1800 , 2000};
#define N_PWM 11
uint16_t i_PWM = 0;

// Timers procedure de test
uint32_t tt_start_phase;
uint32_t dt_init_filtres = 1000000;//500000;
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


/*****square cycle**********/
float pwm_low = 1200;
float pwm_high = 1350;
uint32_t tt_cycle, dt_cycle;
uint32_t PERIOD_CYCLE = 2500000;

//helice lente
//x7t5
// float pwm_twr[5] ={1415,1550, 1665, 1785, 1910};
// x7t6
// float pwm_twr[5] ={1455,1590, 1715, 1845, 1985};
// x7t7
// float pwm_twr[5] ={1535,1690, 1835, 1980, 2000};
//helice rapide
//x07-17
// float pwm_twr[5] ={1525,1695, 1850, 2000, 2000};
//x07-19
// float pwm_twr[5] ={1460,1605, 1745, 1890, 2000};
//x07-25
float pwm_twr[5] ={1355,1470, 1590, 1715, 1845};


float pwm_twr1=1000;
float pwm_twr15 = 1000;
float pwm_twr2 = 1000;
float pwm_twr25 = 1000;
float pwm_twr3 = 1000;

/******Affichage*************/

// temps d'affichage
uint32_t dt_plot = 10000;
uint32_t period_plot = 10000;
uint32_t tt_plot;
uint8_t enable_plot = 1;
uint8_t enable_timing_error = 1;
bool print_data_on = false;
/******Gestion timing*************/

// timer d'acquisition poids couple
uint32_t dt_acquisition_PC_rpm;
uint32_t tt_acquisition_PC_rpm;
uint32_t period_sequence = 2000;

//// Downsampling de la boucle de commande moteur, lecture INA et MLXpar rapport à la fréquence d'acquisition poids couple rpm
uint32_t downsampling_counter = 0;
uint32_t downsampling = 1;


/******debug*************/
uint32_t tt_debug;

void setup()
{ 
    ///// initialisation serial
  // Serial.begin(230400);
  Serial.begin(500000);
  Serial.setTimeout(0);
  Serial.print( "///////////////// SETUP ////////////////" );Serial.print("\n");
  Serial.print( "Serial OK" );Serial.print("\n");
  
  //Seria Reception RPM & VI
  Serial_RPM.begin(230400);
  Serial_VI.begin(230400);
  Serial_Radio.begin(115200);

  ///// Initialisation du moteur
  M1.set_resol(12);
  M1.attach_pin_us(Motor1.pin,Motor1.pwmMin,Motor1.pwmMax,WIDTH_PWM);
  Serial.print( "Moteur OK" );Serial.print("\n");
  
  ///// résolution des pin analog
  analogReadResolution(12);

  ///// Initialisation des filtres poids couple
  reset_PC_filters();
  ///// tare des capteurs
  tare_all();

  ///// Initialisation des timers
  tt_acquisition_PC_rpm = micros();
  downsampling_counter = 0;
  tt_calib = micros();
  tt_plot = micros();
  tt_cycle = micros();
  
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
  Serial.print( "'e' enable/disable plot");Serial.print("\n");Serial.print("\n");
}


void loop()
{  
  ////// Lecture RPM from teensy speedmeter
  if ( Serial_RPM.getData_int16_t(NB_DATA_RPM,START_BYTE,STOP_BYTE,serial_data_rpm) )
  {
    int16_t *ptr16_t= (int16_t*)(&temps_rpm);
    *ptr16_t     = serial_data_rpm[0];
    *(ptr16_t+1) = serial_data_rpm[1]; 

    rpm_mot = (uint16_t)serial_data_rpm[2];

    // Serial.print(temps_rpm);Serial.print("\t");
    // print(rpm_mot,"\n");

  }
  ////// Lecture power from teensy V-I-P
  if (Serial_VI.getData_int16_t(NB_DATA_VI, START_BYTE, STOP_BYTE, serial_data_vi))
  { 
    int16_t *ptr16_t= (int16_t*)(&temps_vi);
    *ptr16_t     = serial_data_vi[0];
    *(ptr16_t+1) = serial_data_vi[1]; 

    courant_ina[0] = serial_data_vi[2]/1000.0;
    tension_ina[0] = serial_data_vi[3]/1000.0;
    courant_ina[1] = serial_data_vi[4]/1000.0;
    tension_ina[1] = serial_data_vi[5]/1000.0; 

    // Serial.print(temps_vi);Serial.print("\t");
    // print(courant_ina[0], tension_ina[0],"\t");
    // print(courant_ina[1], tension_ina[1],"\n");

  }
  ////// Lecture radio
  lectureRadio(Serial_Radio);
  
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
    i_PWM = 0;
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
  dt_acquisition_PC_rpm = micros() - tt_acquisition_PC_rpm;
  if ( dt_acquisition_PC_rpm > period_sequence )
  // if (elapsed_us(tt_acquisition_PC_rpm,period_sequence) )
  { 
    // if( ((micros() - tt_acquisition_PC_rpm) > dt_acquisition_PC_rpm+200) && enable_timing_error)
    // {
    //   Serial.print( "/!\\ Timing acquisition poids couple rpm non respecté " );Serial.print((micros() - tt_acquisition_PC_rpm));Serial.print("\n");  
    // }

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
        // Motor1.control = 125*thrust;
        Motor1.pwm = Motor1.pwmMin + Motor1.control;
        
      } 
      else if(procedure_state == 1) // Procédure de mesure automatique
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
          // Motor1.pwm = pwm_procedure;
          Motor1.pwm = PWM_tab[i_PWM];
          
          if( ((micros() - tt_start_phase) > dt_init_filtres) && !init_filtres) // Reset des filtre au bout de 'dt_init_filtres' pour accélérer la convergence
          {
            reset_PC_filters();
            init_filtres = 1;
          }
          if( (micros() - tt_start_phase) > dt_mesure ) // Prise de la mesure et initialisation pour la phase suivante
          {
            print_output_line();
            // pwm_procedure += pwm_step_procedure;
            i_PWM++;
            tt_start_phase = micros();
            init_filtres = 0;
            // if (pwm_procedure > pwm_stop_procedure) // Condition d'arrêt de la procédure de test
            // {
            //   procedure_state = 2;
            // }
             if (i_PWM > N_PWM-1) // Condition d'arrêt de la procédure de test
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
          rpm_mot = 0;
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
      
      /**procedure for square wave */
      if (switch_D < 2 )
      { 
        // dt_cycle = micros() - tt_cycle;
        // if ( dt_cycle < PERIOD_CYCLE)
        // {
        //   Motor1.pwm = pwm_low;
        // }
        // else if (dt_cycle < 2*PERIOD_CYCLE)
        // {
        //   Motor1.pwm = pwm_high;
        // }
        // else
        // {
        //   tt_cycle += dt_cycle;
        //   Motor1.pwm = pwm_low;
        // }

        dt_cycle = micros() - tt_cycle;
        if ( dt_cycle < PERIOD_CYCLE)
        {
          Motor1.pwm = Motor1.pwmMin;
        }
        else if (dt_cycle < 2*PERIOD_CYCLE)
        {
          Motor1.pwm = pwm_twr[0];//pwm_twr1;
        }
        else if (dt_cycle < 3*PERIOD_CYCLE)
        {
          Motor1.pwm = pwm_twr[1];  //1.5
        }
        else if (dt_cycle < 4*PERIOD_CYCLE)
        {
          Motor1.pwm = pwm_twr[0];// pwm_twr1;  
        }
        else if (dt_cycle < 5*PERIOD_CYCLE)
        {
          Motor1.pwm = pwm_twr[2];//pwm_twr2;  
        }
        else if (dt_cycle < 6*PERIOD_CYCLE)
        {
          Motor1.pwm = pwm_twr[0];//pwm_twr1;  
        }
        else if (dt_cycle < 7*PERIOD_CYCLE)
        {
          Motor1.pwm = pwm_twr[3];//pwm_twr25;  
        }
        else if (dt_cycle < 8*PERIOD_CYCLE)
        {
          Motor1.pwm = pwm_twr[0];//pwm_twr1;  
        }
        else if (dt_cycle < 9*PERIOD_CYCLE)
        {
          Motor1.pwm = pwm_twr[4];//pwm_twr3;  
        }
        else if (dt_cycle < 10*PERIOD_CYCLE)
        {
          Motor1.pwm = pwm_twr[0];//pwm_twr1;  
        }
        else
        {
          tt_cycle += dt_cycle;
          Motor1.pwm = Motor1.pwmMin;
        }
        
        Serial.print(tt_acquisition_PC_rpm);Serial.print("\t");
        Serial.print(poids);Serial.print("\t");
        Serial.print(couple);Serial.print("\t");
        Serial.print(Motor1.pwm);Serial.print("\t");
        Serial.print(rpm_mot);Serial.print("\t");
        Serial.print(current_mot);Serial.print("\t");
        Serial.print(tension_mot);Serial.print("\n");
      }
      else
      {
        tt_cycle = micros();
      } 

      // filter for current and voltage
      current_mot = courant_ina[0] + courant_ina[1];
      tension_mot = (tension_ina[0]+tension_ina[1])/2.0;
      puissance_ina   = current_mot*tension_mot; // En Watts
      courant_ina_f = (1-alpha_ina)*courant_ina_f+alpha_ina*current_mot;
      tension_ina_f = (1-alpha_ina)*tension_ina_f+alpha_ina*tension_mot;
      puissance_ina_f = (1-alpha_ina)*puissance_ina_f+alpha_ina*puissance_ina;

      // Sécurité sur les moteurs, toujours garder juste avant l'écriture des PWM
      if (!motor_ON)
      {
        Motor1.pwm = Motor1.pwmMin;
      }
      // Ecriture des pwms
      M1.write_us(Motor1.pwm);

    }
  
      
}




//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///// Affichage à la fréquence choisie
  if (enable_plot)
  { 
     // if (dt_acquisition_PC_rpm > 1200)
     // {
     //  Serial.print("dt_acquisition_PC_rpm");Serial.print("\t");
     //  Serial.println(dt_acquisition_PC_rpm);
     // }
           /*******Print Values***********/
      if (print_data_on)
      {
        Serial.print(tt_acquisition_PC_rpm);Serial.print("\t");
        Serial.print(poids);Serial.print("\t");
        Serial.print(couple);Serial.print("\t");
        Serial.print(Motor1.pwm);Serial.print("\t");
        Serial.print(temps_rpm);Serial.print("\t");
        Serial.print(rpm_mot);Serial.print("\t");
        Serial.print(temps_vi);Serial.print("\t");
        Serial.print(courant_ina[0]);Serial.print("\t");
        Serial.print(tension_ina[0]);Serial.print("\t");
        Serial.print(courant_ina[1]);Serial.print("\t");
        Serial.print(tension_ina[1]);Serial.print("\n");
      }

    if (elapsed_us(tt_plot,period_plot))
    {
    //   Serial.println(tt_acquisition_PC_rpm);
      // Serial.println(speed_rpm);

      // Serial.print(poids_ref);Serial.print("\t");

      Serial.print(Motor1.pwm);Serial.print("\t");
      Serial.print(poids_f);Serial.print("\t");
      Serial.print( couple_f );Serial.print("\t");
      Serial.print(tension_ina_f);Serial.print("\t");
      Serial.print(courant_ina_f);Serial.print("\t");
      Serial.print(rpm_mot);Serial.print("\n");


      
      // Serial.print(Motor1.control);Serial.print("\t");
      // Serial.print(knob);Serial.print("\n");


  //      Serial.print( poids );Serial.print("\t");
  //      Serial.print( poids_f );Serial.print("\t");
  //      //Serial.print( poids_f+2*sigma_poids_f );Serial.print("\t");
  //      //Serial.print( poids_f-2*sigma_poids_f );Serial.print("\t");
        // Serial.print( couple );Serial.print("\t");
        // Serial.print( couple_f );Serial.print("\t");
        // Serial.print( couple_f+2*sigma_couple_f );Serial.print("\t");
        // Serial.print( couple_f-2*sigma_couple_f );Serial.print("\t");
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
    if(caractere == 'u')
    { 
      char temp_char = Serial.read();
      if ( temp_char == 'L')
      {
        pwm_low = Serial.parseFloat();
        sat(pwm_low,1000,2000);
        Serial.print("pwm_low set to  ");Serial.println(pwm_low);
      }
      else if (temp_char == 'H')
      {
        pwm_high = Serial.parseFloat();
        sat(pwm_high,1000,2000);
        Serial.print("pwm_high set to  ");Serial.println(pwm_high);
      }
      else if (temp_char == 'T')
      { 
        char temp_char_T = Serial.read();
        if (temp_char_T == '1')
        {
          pwm_twr1 = Serial.parseFloat();
          sat(pwm_twr1,1000,2000);
          Serial.print("pwm_twr1 set to  ");Serial.println(pwm_twr1);
        }
        else if (temp_char_T == '2')
        {
          pwm_twr15 = Serial.parseFloat();
          sat(pwm_twr15,1000,2000);
          Serial.print("pwm_twr15 set to  ");Serial.println(pwm_twr15);
        }
        else if (temp_char_T == '3')
        {
           pwm_twr2 = Serial.parseFloat();
          sat(pwm_twr2,1000,2000);
          Serial.print("pwm_twr2 set to  ");Serial.println(pwm_twr2);
        }
        else if (temp_char_T == '4')
        {
          pwm_twr3 = Serial.parseFloat();
          sat(pwm_twr3,1000,2000);
          Serial.print("pwm_twr3 set to  ");Serial.println(pwm_twr3);
        }
      }
    }
    if (caractere == 'x')
    {
      print_data_on = !print_data_on;
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
  Serial.print(rpm_mot);Serial.print("\t");
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
