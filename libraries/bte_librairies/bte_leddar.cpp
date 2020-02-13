#include "stdint.h"
#include "math.h"
#include "bte_leddar.h"

//Lecture Leddar
uint8_t DataLeddar[2], DataLeddarRaw[3];


static uint8_t START, STOP;

bte_leddar::bte_leddar(HardwareSerial *serial)
{
  _hauteur = 0;
  _validity_flag = 0;
  _serial = serial;
  (*_serial).begin(38400); 
  ptr_buffer_uint16 = (uint8_t *)&buffer_uint16;
}

//Lecture Leddar
void bte_leddar::read_leddar(void)
{   
  
   if((*_serial).available() > 3)
   {    
        START = (*_serial).read();
        if( START == 137)
        {   
            (*_serial).readBytes(DataLeddarRaw,3);
            if( DataLeddarRaw[2] == 173)
            {   
                DataLeddar[0] = DataLeddarRaw[0];
                DataLeddar[1] = DataLeddarRaw[1];
                *ptr_buffer_uint16 = DataLeddar[0];
                *(ptr_buffer_uint16+1) = DataLeddar[1];
                _hauteur = (buffer_uint16/10.0);

                if( (buffer_uint16>15000) || buffer_uint16 == HistoLeddar[BTE_LEDDAR_NFILT_MED-20] && HistoLeddar[BTE_LEDDAR_NFILT_MED-3]==HistoLeddar[BTE_LEDDAR_NFILT_MED-2] && HistoLeddar[BTE_LEDDAR_NFILT_MED-2]==HistoLeddar[BTE_LEDDAR_NFILT_MED-1] )
                {
                  _validity_flag = 0;
                } else
                {
                  _validity_flag = 1;
                }
                //Hauteur_med_leddar =  medianFilter16(buffer_uint16, HistoLeddar, Nfiltmed)/10.0*cosf(roll)*cosf(pitch);
                Hauteur_med_leddar =  bte_medianFilter16(buffer_uint16, HistoLeddar, BTE_LEDDAR_NFILT_MED)/10.0;
//                Hauteur = (buffer_uint16/10.0); // en cm
//                Hauteur_med =  medianFilter16(buffer_uint16, HistoLeddar, Nfiltmed)/10.0;                  
            }
               
        }
    
   }
}
