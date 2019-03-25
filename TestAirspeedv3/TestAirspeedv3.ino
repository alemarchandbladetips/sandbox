#include <Wire.h>
#include "SDP6x.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define RAD2DEG 57.29577951
#define START_BYTE 137
#define STOP_BYTE 173

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

int16_t buffer_int16;
unsigned char *ptr_buffer_int16 = (unsigned char *)&buffer_int16;

Adafruit_BNO055 bno = Adafruit_BNO055(55);
uint8_t sys, gyr, accel, mag = 0;
float omega[3], quaternion[4], rpy[3], proper_acc[3];

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  difPressure = 0.0;
  temps = micros();

//  if(!bno.begin())
//  {
//    /* There was a problem detecting the BNO055 ... check your connections */
//    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
//    while(1);
//  }
//  while(gyr!=3)
//  {
//    bno.getCalibration(&sys, &gyr, &accel, &mag);
//  }
  //bno.setExtCrystalUse(true);
}

void loop()
{  
  int8_t j;
   dt = micros() - temps;
   if ( dt >= 10000 ) 
   {  
      temps += dt;

      //Capteur Pitot Analogique
      //Mesure = analogRead(pinWind);
      
      outSensor = (Mesure*Vref/resol);

      Pressure = (outSensor - ZeroPressure) >0 ? (outSensor - ZeroPressure)*62/(4-ZeroPressure) : (outSensor - ZeroPressure)*62/(ZeroPressure-0.25); // En Pascal

      HistoriqueVal(Pressure, P);
      Mean(P, &meanP,20, H-1);

      

    //  if( abs(meanP-last_meanP) > 0.01 ){Serial.println(meanP);}
      last_meanP = meanP;

      //Capteur differentiel     
      difPressure = SDP6x.GetPressureDiff()/3.0;
      
      HistoriqueVal(difPressure, DifP);
      Mean(DifP, &meanDifP,20, H-1);

//      imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
//      imu::Vector<3> gyro=bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
//      imu::Vector<3> proper_acc=bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
      

      //Calcul angle
      if ( max(abs(meanDifP),abs(meanP))>0.1 )//&& abs(meanDifP)>0.1 )
      {
        angle = atan2(meanDifP,3*meanP)*RAD2DEG;
      }
      else
      {
        angle = 0;
      }
       
      // Data vers port série
      //Serial.print(dt);Serial.println(" ");
      //Serial.print(Pressure);Serial.print("\t");
      //Serial.println(Mesure);
      //Serial.println(outSensor);
      //Serial.print(meanDifP);Serial.print("\t");
      //Serial.print(meanP);Serial.print("\t");
      Serial.print(difPressure);Serial.println("\t");
      //Serial.print(angle/10.0);Serial.println(" ");
//      Serial.print(euler.x()-186.0,3);Serial.print(" ");
      //Serial.print(gyro.z(),3);Serial.print(" ");
      //Serial.print(proper_acc.x(),3);Serial.print(" ");
      
//      Serial.write(START_BYTE);
//      buffer_int16 = (int16_t)(-angle/180.0*16384);
//      for(j=0;j<2;j++)
//      {
//        Serial.write(ptr_buffer_int16[j]);
//      }
//      Serial.write(STOP_BYTE);    
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
