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
ValMean dt_RPM(100);
float dt_rpm_mean = 1;
float rpm_mean=0;
float valmin= 500;
float valmax= 4095;

ValMean val_analog(2000); 
float val_mean_analog = 4000;

//Serial Send speed
RxTxSerial SpeedMot(Serial1,0);

uint32_t tt, dt;

void setup()
{
	Serial.begin(115200);
  Serial1.begin(115200);

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
      // Serial.println(val_mean_analog*0.96-25);
      
      if (value_portLED > val_mean_analog*0.96) //3300
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
       
          temps2wait = constrain(0.250*dt_wait,mindtwait,maxdtwait); //0.25
      
      }
      else if (dt_wait > maxdtwait )
      {   
          tt_wait += dt_wait;
          rpm = 0.5*rpm; // si pas de détection donc ca converge à vitesse zero
          rpm_mean = 0.5*rpm_mean;
          temps2wait = maxdtwait;
      }
      last_EtatA = EtatA;
    }

    dt = micros() - tt;
    if (dt > 10'000)
    {	
    	tt += dt;

      Serial.println(rpm_mean);
      SpeedMot.sendData_float(137,173,&rpm_mean,1);
    }
}