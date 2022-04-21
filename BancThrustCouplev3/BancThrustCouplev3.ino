//maquette thurst couple fil
#include "GlobalVariables.h"
#include "Functions.h"
#include "RadioSignals.h"

#include <HX711_ADC.h>
#include "Adafruit_INA260.h"

#define RAD2DEG  180/PI
#define DEG2RAD  PI/180

/******Definitions Motor*************/
MyServo M1;
//           pin, pwmMin, pwmMax, pwmInit, pwm, control, rot
motor Motor1  {7,1000, 2000, 1023, 1023, 0, 1};
const int WIDTH_PWM = 4000;

 //Capteurs pour thrust
const int B1_sck  = 25; //mcu > HX711 sck pin
const int B1_dout = 26; //mcu > HX711 dout pin

const int B2_sck  = 27; //mcu > HX711 sck pin
const int B2_dout = 28; //mcu > HX711 dout pin

const int B3_sck  = 29; //mcu > HX711 sck pin
const int B3_dout = 30; //mcu > HX711 dout pin

// cpateur pour couple
const int B4_sck  = 31; //mcu > HX711 sck pin
const int B4_dout = 32; //mcu > HX711 dout pin


const int stabilizingtime = 2000; //en ms
bool _tare =true;

//float calibrationValues[4]={499.95, 459.32, 450.37, 1065.89}; //1066.12
float calibrationValues[4]={499.95, 459.32, 450.37, 1065.89};

HX711_ADC BALANCES[4]={HX711_ADC(B1_dout, B1_sck),HX711_ADC(B2_dout, B2_sck),HX711_ADC(B3_dout, B3_sck),HX711_ADC(B4_dout, B4_sck)};

//Mesures des balances
float poids[4]={0};

float poids_B1 = 0;
float poids_B2 = 0;
float poids_B3 = 0;
float poids_B4 = 0;
float poids_tot = 0;
float poids_mean = 0;

float Mod_torque_tot = 0;

//Filter factor balances
float N =10;
float k_filter = 2/(N+1);

int Nb_data = 20;

//ValMean POIDS[4]={ValMean(Nb_data),ValMean(Nb_data),ValMean(Nb_data),ValMean(Nb_data)};
ValMean POIDS[4](Nb_data);
ValMean poids_torque(Nb_data);
float torque=0;
float aux_poids[4];
float poids3_mean=0;
float torque_nofilt=0;

// Position Balances
float l_B = 10; // en cm

/**********Capteur V I *********/
Adafruit_INA260 ina260 = Adafruit_INA260();
float Courant, Voltage, Power;
bool INA260_OK = false;
float I_f, V_f, P_f, f_ina = 1.8/50;

//Radio Télécommande
bool ALL_OFF = false;

//Paramètres mesure de RPM
//1000 us => -60'000'000/(1000*nb_pales)=30'000 rpm & 60'000 us => 60'000'000/(60'000*nb_pales)= 500 rpm
uint32_t const mindtwait=1000, maxdtwait=60'000; 
uint32_t temps2wait = maxdtwait;

uint16_t value_portLED=0;
boolean EtatA=HIGH, last_EtatA=HIGH;
float rpm=0;
float dt_filt=1000000;
int nb_pales = 2;
int portLED = A3;
ValMean dt_RPM(100);
float dt_rpm_mean = 1;
float rpm_mean=0;

//Regulation thrust des moteurs
//          kp, kd, ki, int_e
Pid Reg_Mot(1.0, 0, 0, 0);
float KP = 1.0, KI=0;
float e_thrust=0;
float thrust_ref=0;
int const DURATION_SEG=5;
uint16_t phase_mot=0;
int const MAX_PHASE = 20;
bool PROC_PWM_ON = false; //déclenche procédure d'échelonnage pwm
int last_switch_C = 2 ;// Bouton en bas de la télécommande
float f_back_zero = 2.0/(1+400);
bool BACK_ZERO = false;

//Timers
uint32_t dt, tt;
uint32_t dt_ina, tt_ina;
uint32_t dt_mot, tt_mot;
uint32_t dt_rpm, tt_rpm;
uint32_t dt_wait, tt_wait;
uint32_t dt_pwm, tt_pwm;

void setup()
{
  Serial.begin(115200);
  Serial1.begin(115200);

   pinMode(portLED,INPUT_PULLUP);
   // Initialisation Capteur INA260
   INA260_OK = ina260.begin(0x40,&Wire);

   // Setting the motors
   M1.set_resol(12);
   M1.attach_pin_us(Motor1.pin,Motor1.pwmMin,Motor1.pwmMax,WIDTH_PWM);

   //Limites Pid
   Reg_Mot.set_prop_lim(300); // en us
   Reg_Mot.set_int_lim(150); // en us
   Reg_Mot.set_reg_lim(400); // en us
   
  //Taring les balances
  for (int i=0 ; i<4; i++)
  {
    BALANCES[i].begin();
    BALANCES[i].start(stabilizingtime,_tare);
    if (BALANCES[i].getTareTimeoutFlag())
    {
      Serial.print("Something wrong with balance B");Serial.println(i+1);
    }
    else
    { 
      BALANCES[i].setCalFactor(calibrationValues[i]);
      BALANCES[i].setSamplesInUse(1);
      Serial.print("Balance B");Serial.print(i+1);Serial.println(" has been tared");
    }
  }
 
 
  // Timers
  tt = micros();
  tt_ina = tt;
  tt_mot = tt;
  tt_rpm = tt;
  tt_wait = tt;
  tt_pwm = millis();
  
  Serial.print( "PWM: " );Serial.print("\t");
  Serial.print( "Tension: " );Serial.print("\t");
  Serial.print( "Courant: " );Serial.print("\t");
  Serial.print( "Thrust: " );Serial.print("\t");
  Serial.print( "Couple: " );Serial.print("\t");
  Serial.print( "rpm: " );Serial.print("\n");
//  Serial.print( "Power: " );Serial.print("\n");
}


void loop()
{  
  // Lecture radio
  lectureRadio (Serial1);

  // Mesure de RPM du moteur
  dt_rpm = micros() - tt_rpm;
  if (dt_rpm > 1000 )
  {   
      tt_rpm += dt_rpm;
      dt_wait = micros() - tt_wait; 
       
      if ( dt_wait > temps2wait)
      {
          value_portLED = analogRead(portLED);
//          Serial.println(value_portLED);
          if (value_portLED > 700) 
          {
            EtatA = HIGH; // en face de surface réflective
          }
          else
          {
            EtatA = LOW;
          }

          if ( last_EtatA == LOW && EtatA == HIGH ) //Front ascendant
          {
              tt_wait += dt_wait;

              dt_filt += 0.03*(dt_wait-dt_filt);
              rpm = 60'000'000.0/(nb_pales*dt_filt);
              
              dt_rpm_mean = dt_RPM.getMean(dt_wait);
              rpm_mean = 60'000'000.0/(nb_pales*dt_rpm_mean);
           
              temps2wait = constrain(0.250*dt_wait,mindtwait,maxdtwait);
                
          }
          else if (dt_wait > maxdtwait )
          {   
              tt_wait += dt_wait;
              rpm = 0.5*rpm; // si pas de détection donc ca converge à vitesse zero 
              temps2wait = maxdtwait;
          }
          
          last_EtatA = EtatA;
      
      }
  }   

  // Decryptage d'info du port serie
   if(Serial.available() > 1 )
   {
      char input[2]={Serial.read(),Serial.read()};
      if (input[0]  == 't')
      { 
        if      (input[1] == '1') tare(1);
        else if (input[1] == '2') tare(2);
        else if (input[1] == '3') tare(3);
        else if (input[1] == '4') tare(4);
        else if (input[1] == 't') 
        { 
          for (int i=1; i<5;i++)
          {
            tare(i);
          }
        }
        else Serial.println("No tare");
      }
      else if (input[0]  == 'c')
      { 
        if      (input[1] == '1') calibration(1);
        else if (input[1] == '2') calibration(2);
        else if (input[1] == '3') calibration(3);
        else if (input[1] == '4') calibration(4);
        else if (input[1] == 'c') calibration();
      }
      else if (input[0]  == 'p')
      {
          Serial.print( Motor1.pwm );Serial.print("\t");          
          Serial.print( V_f );Serial.print("\t");
          Serial.print( I_f );Serial.print("\t");
          Serial.print( poids_tot);Serial.print("\t");
          Serial.print( Mod_torque_tot );Serial.print("\t");
          Serial.print( rpm );Serial.print("\n");
    
      }
   }

     // Lecture capteurs de poids
   dt = micros() - tt;
   if (dt > 10'000)
   {
      tt += dt;
     
      //Reading data from sensors
      for (int i=0; i<4 ; i++)
      { 
        if( BALANCES[i].update() )
        { 
          poids[i] = POIDS[i].getMean( BALANCES[i].getData() );
//          aux_poids[i] = BALANCES[i].getData();
//          poids[i] += k_filter*(aux_poids[i] - poids[i]);
//          poids[i] += k_filter*(BALANCES[i].getData() - poids[i]);

//          if (i==3)  poids3_mean = poids_torque.getMean(aux_poids[3]);
          
        }
      }
      
      poids_tot = poids[0]+poids[1]+poids[2];

      Mod_torque_tot = poids[3]*l_B;
//      torque = poids3_mean*l_B;
//      torque_nofilt = aux_poids[3]*l_B;
      
      
//    Serial.print( Motor1.pwm );Serial.print("\t");
//    Serial.print( V_f );Serial.print("\t");
//    Serial.print( I_f );Serial.print("\t");
    Serial.print( poids_tot);Serial.print("\n");
//    Serial.print( Mod_torque_tot );Serial.print("\t");
//    Serial.print( rpm );Serial.print("\n");
   }
   
// Capteur Puissance
   dt_ina = micros() - tt_ina;
    if (dt_ina > 10'000)
    {
        tt_ina += dt_ina;
//        uint32_t tt_test = micros();
        if ( INA260_OK )
        { 
          Courant = ina260.readCurrent()/1000.0; // en Ampère
          Voltage = ina260.readBusVoltage()/1000.0; // en Volts
          Power   = Courant*Voltage; // En Watts
        }

        I_f += f_ina*(Courant-I_f);
        V_f += f_ina*(Voltage-V_f);
        P_f += f_ina*(Power-P_f);
    }

    //Regulation thrust des moteurs
    dt_mot = micros() - tt_mot;
    if ( dt_mot > 10'000 )
      {  
          tt_mot += dt_mot;
          
          ALL_OFF = (switch_F==2);
          
          if (switch_C == 1 && last_switch_C == 2)
          {
            PROC_PWM_ON = true;
          }
          last_switch_C = switch_C;
          
          if (!ALL_OFF && PROC_PWM_ON) // start procédure sinon control normal
          {
            if (!BACK_ZERO)
            { 
              dt_pwm = millis() - tt_pwm;
              if( dt_pwm > phase_mot*DURATION_SEG*1000)
              { 
                printValues();
                if (++phase_mot > MAX_PHASE)
                {
                  
                  //tt_pwm += dt_pwm;
                  BACK_ZERO = true;
                }  
              }
              Motor1.control = phase_mot*50;
            }
            else
            { 
              Motor1.control += (f_back_zero)*(0-Motor1.control);
              //Serial.println(Motor1.control);
              if (Motor1.control < 10) 
              {
                BACK_ZERO = false;
                PROC_PWM_ON = false;
                tt_pwm += dt_pwm;
                phase_mot = 0;     
              }
              
            }
          }
          else
          { 
            tt_pwm = millis();
            PROC_PWM_ON = false;
            phase_mot = 0;
            Motor1.control = 1000*thrust + 100*(2*knob-1);
            BACK_ZERO = false;
          }

          Motor1.pwm = Motor1.pwmMin + Motor1.control;

          if (ALL_OFF)
          {
            Motor1.pwm = Motor1.pwmMin;
          }
          
          // Ecriture des pwms
          M1.write_us(Motor1.pwm);

//          Serial.print( poids_tot);Serial.print("\t");
//          Serial.print( Mod_torque_tot);Serial.print("\n");
//          Serial.print( torque_nofilt );Serial.print("\n");

//          Serial.print( Mod_torque_tot);Serial.print("\n");
//          Serial.println(Motor1.pwm);

      }
}

/*******************Functions************************************************************/
/****************************************************************************************/
void printValues(void)
{
    Serial.print( Motor1.pwm );Serial.print("\t");
    
    Serial.print( V_f );Serial.print("\t");
    Serial.print( I_f );Serial.print("\t");
    Serial.print( poids_tot);Serial.print("\t");
    Serial.print( Mod_torque_tot );Serial.print("\t");
    Serial.print( rpm_mean );Serial.print("\n");
//    Serial.print( P_f );Serial.print("\n");
}

void tare(int B_i)
{ 
    BALANCES[B_i-1].tareNoDelay();
    while(BALANCES[B_i-1].getTareStatus() == false){BALANCES[B_i-1].update();}

    Serial.print("Taring done on balance ");Serial.println(B_i);
  
}
void calibration(int B_i)
{ 
      tare(B_i);
      bool WAITING_MASS = true;
      float known_mass;
      Serial.println("Put the known mass on the balance");
      while (WAITING_MASS)
      { 
        BALANCES[B_i-1].update();
        
        if( Serial.available()>0 )
        {  
           known_mass = Serial.parseFloat();
           WAITING_MASS = false;
        } 
      }
     
      bool CALIB_OK = false;
      while (!CALIB_OK)
      { 
        if (BALANCES[B_i-1].update())
        {
            float newCalibrationValue = BALANCES[B_i-1].getNewCalibration(known_mass);
            BALANCES[B_i-1].setCalFactor(newCalibrationValue);
            Serial.print("New calibration value for B");Serial.print(B_i);Serial.print(" is : ");
            Serial.println(newCalibrationValue);
            CALIB_OK = true;
        }
      }
  
}

//calibration des 3 balances pour le mesurer le thrust
void calibration(void)
{ 
  for(int i=1; i<4; i++)
  {
    tare(i);
  }
  
  bool WAITING_MASS = true;
  float known_mass;
  Serial.println("Put the known mass on the balance");
  while (WAITING_MASS)
  { 
    for (int i=0; i<3; i++)
    {
      BALANCES[i].update();
    }
    if( Serial.available()>0 )
    {  
       known_mass = Serial.parseFloat()/3.0;
       WAITING_MASS = false;
    } 
  }

  for (int i=0; i<3; i++)
  {
    bool CALIB_OK = false;
     while (!CALIB_OK)
     {  
        if (BALANCES[i].update() )
        {
            float newCalibrationValue = BALANCES[i].getNewCalibration(known_mass);
            BALANCES[i].setCalFactor(newCalibrationValue);
            Serial.print("New calibration value for B");Serial.print(i+1);Serial.print(" is : ");
            Serial.println(newCalibrationValue);
            CALIB_OK = true;
        }
      }  
   }
   
}

void rot_z( float *X ,float ang)
{
  float aux_X[2] = {X[0],X[1]};
  float _ang = ang*PI/180.0;
  X[0] = aux_X[0]*cosf(_ang) - aux_X[1]*sinf(_ang);
  X[1] = aux_X[0]*sinf(_ang) + aux_X[1]*cosf(_ang);
  
}

void cross(float *X, float *F, float* M)
{
  M[0] = X[1]*F[2]-X[2]*F[1];
  M[1] = X[2]*F[0]-X[0]*F[2];
  M[2] = X[0]*F[1]-X[1]*F[0];
}
