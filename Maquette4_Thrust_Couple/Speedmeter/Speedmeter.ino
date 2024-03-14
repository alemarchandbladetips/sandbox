#include "GlobalVariables.h"
#include "Functions.h"

// timers capteur rpm
uint32_t dt_wait, tt_wait;

uint32_t const mindtwait=100, maxdtwait=60'000; 
uint32_t temps2wait = maxdtwait;
uint16_t value_portLED=0;
boolean EtatA=HIGH, last_EtatA=HIGH;
float rpm=0;
float dt_filt=1000000;
int nb_pales = 3;//2;
int portLED = 15;
ValMean dt_RPM(1);
float dt_rpm_mean = 1;
float rpm_mean=0;
float valmin= 500;
float valmax= 4095;

ValMean val_analog(2000); 
float val_mean_analog = 4000;

ValMean dt_RPM_2(300);
float dt_rpm_mean_2 = 0;
float rpm_mean_2=0;

//Serial Send speed
RxTxSerial Serial_FT(Serial4,0);
RxTxSerial Serial_VI(Serial5,0);

int START_BYTE = 137;
int STOP_BYTE = 173;

int const NB_DATA_RPM = 3; //temps_rpm(unint32_t) + rpm_mot (int16_t) = 3*uint16_t
int16_t serial_data_rpm[NB_DATA_RPM];

// int const NB_SERIAL_DATA = 6;
// int16_t serial_data[NB_SERIAL_DATA];

// float courant_ina[2];
// float tension_ina[2];
// uint32_t temps_VI;

//timers
uint32_t tt, dt;
uint32_t tt_rpm;
uint32_t period_rpm=2000;


void setup()
{
	Serial.begin(115200);

  Serial_FT.begin(230400);
  Serial_VI.begin(230400);

	 ///// résolution des pin analog
  analogReadResolution(12);

  ///// Initialisation des timers
  tt= micros();
}

void loop()
{ 

	dt_wait = micros() - tt_wait; 
     
    if ( dt_wait > temps2wait)
    {
      value_portLED = analogRead(portLED);

      val_mean_analog = val_analog.getMean(value_portLED);
      sat(val_mean_analog, valmin,valmax);

      // Serial.print(value_portLED); Serial.print("\t");
      // Serial.println(val_mean_analog*0.95);
      
      // if (value_portLED > val_mean_analog*0.94) //3300
      if (value_portLED < 1000) //3300
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

          dt_rpm_mean = dt_RPM.getMean(dt_wait);
          rpm_mean = 60'000'000.0/(nb_pales*dt_rpm_mean);
          if (rpm_mean > 30'000) rpm_mean = 0;

          dt_rpm_mean_2 = dt_RPM_2.getMean(dt_wait);
          rpm_mean_2 = 60'000'000.0/(nb_pales*dt_rpm_mean_2);
          if (rpm_mean_2 > 30'000) rpm_mean_2 = 0;
       
          temps2wait = constrain(0.250*dt_wait,mindtwait,maxdtwait); //0.25
      
      }
      else if (dt_wait > maxdtwait )
      {   
          tt_wait += dt_wait;
          rpm_mean = 0.5*rpm_mean;
          temps2wait = maxdtwait;

          rpm_mean_2 = 0.5*rpm_mean_2;
      }
      last_EtatA = EtatA;
    }

    if ( elapsed_us(tt_rpm,period_rpm) )
    {
      serial_data_rpm[0] = *((int16_t*) (&tt_rpm));
      serial_data_rpm[1] = *((int16_t*) (&tt_rpm) + 1);
      serial_data_rpm[2] = rpm_mean_2;
      // serial_data_rpm[2] = rpm_mean;
      
      Serial_FT.sendData_int16_t(START_BYTE, STOP_BYTE, serial_data_rpm, NB_DATA_RPM);
    }

    // dt = micros() - tt;
    // if (dt > 3'000)
    // {	
    // 	tt += dt;

    //   // Serial.print(rpm_mean);Serial.print("\t");
    //   // Serial.println(rpm_mean_2);
    //   rpm_mean = 1.1;

    //   Serial_FT.sendData_float(137,173,&rpm_mean,1);
    // }
}