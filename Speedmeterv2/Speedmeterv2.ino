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
float valmean= 4000;
float f_mean = 0.003;

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
      valmean += f_mean*(value_portLED - valmean);
      sat(valmean, valmin,valmax);
       Serial.print(value_portLED); Serial.print("\t");
       Serial.println(valmean);

      if (value_portLED > valmean) //3300
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
          if (rpm > 30'000) rpm = 0;
          

          // dt_rpm_mean = dt_RPM.getMean(dt_wait);
          // rpm_mean = 60'000'000.0/(nb_pales*dt_rpm_mean);
       
          temps2wait = constrain(0.250*dt_wait,mindtwait,maxdtwait); //0.25
      
      }
      else if (dt_wait > maxdtwait )
      {   
          tt_wait += dt_wait;
          rpm = 0.5*rpm; // si pas de détection donc ca converge à vitesse zero 
          temps2wait = maxdtwait;
      }
      last_EtatA = EtatA;
    }

    dt = micros() - tt;
    if (dt > 10'000)
    {	
    	tt += dt;
      // rpm = 100.5;
    	// Serial.println(rpm);
      SpeedMot.sendData_float(137,173,&rpm,1);
    }
}