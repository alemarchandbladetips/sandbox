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
  Wire.begin();
  difPressure = 0.0;
  temps = micros();
}

void loop()
{  
  int8_t j;
   dt = micros() - temps;
   if ( dt >= 100000 ) 
   {  
      temps += dt;
      
      //Capteur differentiel     
      difPressure = SDP6x.GetPressureDiff();
      
      Serial.print(difPressure*10);Serial.print(" 2 4 6 8 10 12 14 16 18 20 -2 -4 -6 -8 -10 -12 -14 -16 -18 -20 0 ");Serial.println("\t");
   
   }
}
