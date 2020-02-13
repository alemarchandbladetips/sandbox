#include "LeddarReading.h"
#include "stdint.h"
#include "math.h"

//Lecture Leddar
uint8_t DataLeddar[2], DataLeddarRaw[3];
float hauteur_leddar=0;
float Hauteur_med_leddar=10;
const int Nfiltmed =20;
uint16_t HistoLeddar[Nfiltmed];



static uint8_t START, STOP;

//Fonctions

///Filtre median avec int16
uint16_t medianFilter16(uint16_t val, uint16_t XX[], uint8_t Elem)
{
  uint8_t i, j;
  uint16_t S[Elem];
  //Historique des valeurs stocké dans X
  for (i = 0; i < (Elem - 1) ; i++) {
    XX[i] = XX[i + 1];
  }
  XX[Elem - 1] = val;

  for (i = 0; i < Elem ; i++) {
    S[i] = XX[i];
  }

  //Rangement des valeurs (min au max)
  uint16_t temp;
  for ( i = 0; i < (Elem-1); i++)
  {
    boolean change = false;

    for ( j = 1; j < (Elem - i); j++)
    {
      temp = S[j - 1];
      if ( temp > S[j] )
      {
        S[j - 1] = S[j];
        S[j] = temp;
        change = true;
      }
    }
    if (change == false) {
        i=Elem;
        //break;  //ceci implique que tous les vals sont ordonnées
    }
  }

  return S[(Elem - 1) / 2]; // valeur au millieu
}

//Lecture Leddar
void lectureLeddar(void)
{   
   if(Serial2.available() > 3)
   {    
        START = Serial2.read();
        
        if( START == 137)
        {   
            Serial2.readBytes(DataLeddarRaw,3);
            if( DataLeddarRaw[2] == 173)
            {   
                DataLeddar[0] = DataLeddarRaw[0];
                DataLeddar[1] = DataLeddarRaw[1];
                
                *ptr_buffer_uint16 = DataLeddar[0];
                *(ptr_buffer_uint16+1) = DataLeddar[1];
                
                //hauteur_leddar = (buffer_uint16/10.0)*cosf(roll)*cosf(pitch); // en cm
                hauteur_leddar = (buffer_uint16/10.0); // en cm

                Hauteur_med_leddar =  medianFilter16(buffer_uint16, HistoLeddar, Nfiltmed)/10.0*cosf(roll)*cosf(pitch);
//                Hauteur = (buffer_uint16/10.0); // en cm
//                Hauteur_med =  medianFilter16(buffer_uint16, HistoLeddar, Nfiltmed)/10.0;                  
            }
               
        }
    
   }
}
