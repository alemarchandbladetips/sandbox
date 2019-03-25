#include <Wire.h>
#include "SDP6x.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define RAD2DEG 57.29577951

int Input =0, Mesure=0;
int pinWind = A1;

uint32_t dt, temps;
float outSensor, Pressure;
float ZeroPressure = 2.1;
const int H =50;
float P[H], meanP;
//float resol=4095;
float resol=1023;
float Vref=5;
float last_meanP;
//float Vref=3.3;

// Capteur Difference Pression
float difPressure;
float angle;
float DifP[H], meanDifP;

#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055(55);
uint8_t sys, gyr, accel, mag = 0;
float omega[3], quaternion[4], rpy[3], proper_acc[3];

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  difPressure = 0.0;
  temps = micros();

  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  while(gyr!=3)
  {
    bno.getCalibration(&sys, &gyr, &accel, &mag);
  }
  //bno.setExtCrystalUse(true);
}

void loop()
{  
   dt = micros() - temps;
   if ( dt >= 20000 ) 
   {  
      temps += dt;

      //Capteur Pitot Analogique
      Mesure = analogRead(pinWind);
      
      outSensor = (Mesure*Vref/resol);

      Pressure = (outSensor - ZeroPressure) >0 ? (outSensor - ZeroPressure)*62/(4-ZeroPressure) : (outSensor - ZeroPressure)*62/(ZeroPressure-0.25); // En Pascal

      HistoriqueVal(Pressure, P);
      Mean(P, &meanP,10, H-1);

      

    //  if( abs(meanP-last_meanP) > 0.01 ){Serial.println(meanP);}
      last_meanP = meanP;

      //Capteur differentiel     
      difPressure = SDP6x.GetPressureDiff()/3.0;
      
      HistoriqueVal(difPressure, DifP);
      Mean(DifP, &meanDifP,10, H-1);

      imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      imu::Vector<3> gyro=bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
      imu::Vector<3> proper_acc=bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
      

      //Calcul angle
      if ( max(abs(meanDifP),abs(meanP))>0.1 )//&& abs(meanDifP)>0.1 )
      {
        angle = atan2(meanDifP,meanP)*RAD2DEG;
      }
      else
      {
        angle = 0;
      }
       
      // Data vers port série
      //Serial.print(dt);Serial.print(" ");
      //Serial.print(Pressure,3);Serial.print(" ");
      //Serial.println(Mesure);
      //Serial.println(outSensor);
      //Serial.print(meanP);Serial.print("\t");
      //Serial.print(difPressure,3);Serial.print(" ");
      //Serial.print(euler.x()-186.0,3);Serial.print(" ");
      //Serial.print(gyro.z(),3);Serial.print(" ");
      //Serial.print(proper_acc.x(),3);Serial.print(" ");
      Serial.println((int16_t)(angle/180.0*16384,3);

//      float scale;
//      if (meanP!=0) {scale=meanDifP/meanP;}
//      
//      if ( abs(scale) <5) {Serial.println(scale);}
      

      
      
   }
   
   
}

/**********  FONCTIONS  *******************************/
//Filtre moyen
void Mean(float A[H], float* prom,int NbData, int posf)
 {
      float sum=0;
      int vali=posf+1-NbData;
      int valf=posf+1;
      for(int k=vali;k<valf;k++){
        sum+=A[k];
        }
      *prom=sum/NbData;      
 }

void HistoriqueVal(float val, float A[H])
{
      for(int k=0;k<(H-1);k++){
        A[k]=A[k+1];
        
      }
      A[H-1]=val;
}
