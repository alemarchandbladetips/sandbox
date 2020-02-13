#include <Wire.h>
#include "SDP6x.h"

#define START_BYTE 137
#define STOP_BYTE 173

uint32_t dt, temps;

// Capteur Difference Pression
float difPressure;

int16_t buffer_int16;
unsigned char *ptr_buffer_int16 = (unsigned char *)&buffer_int16;

void setup()
{
  Serial.begin(115200);
  Serial1.begin(115200);
  Wire.begin();
  difPressure = 0.0;
  temps = micros();
}

void loop()
{  
  int8_t j;
   dt = micros() - temps;
   if ( dt >= 10000 ) 
   {  
      temps += dt;
      
      //Capteur differentiel     
      difPressure = SDP6x.GetPressureDiff()/3.0;
      
      Serial.print((int16_t)(difPressure*1200.0));Serial.println("\t");

      Serial1.write(START_BYTE);
      buffer_int16 = (int16_t)(difPressure*1200.0);
      for(j=0;j<2;j++)
      {
        Serial1.write(ptr_buffer_int16[j]);
      }
      Serial1.write(STOP_BYTE);    
   }
}
