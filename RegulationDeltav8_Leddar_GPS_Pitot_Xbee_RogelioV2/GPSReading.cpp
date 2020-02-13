#include "GPSReading.h"
#include "stdint.h"

#define NB_DATA_RX_GPS 31

float x_gps, y_gps, z_gps, vx_gps, vy_gps , vz_gps;
float Speed_Avion_med = 0;
uint8_t GPS_mode=0;


void lectureGPS(void)
{    
    if ( Serial3.available()> (NB_DATA_RX_GPS-1) )
    {    
          if (Serial3.read() == 0xAA)
          {   
              uint8_t temp_data_gps[NB_DATA_RX_GPS-1];
              Serial3.readBytes(temp_data_gps, NB_DATA_RX_GPS - 1);
              if ( temp_data_gps[NB_DATA_RX_GPS-2] == 0x55 )
              {   
                  float val_float;
                  uint8_t *ptr_float = (uint8_t *) &val_float;
                  for (int j=0; j<7; j++)
                  {
                        for (int i=0; i<4 ;i++)
                        {
                            *(ptr_float + i) = temp_data_gps[4*j+i];
                        }
                        if(j==0) y_gps = val_float;
                        if(j==1) x_gps = val_float;
                        if(j==2) z_gps = -val_float;
                        if(j==3) vy_gps = val_float;
                        if(j==4) vx_gps = val_float;
                        if(j==5) vz_gps = -val_float;
                        if(j==6) Speed_Avion_med = val_float;
                  }
                  
                  GPS_mode = temp_data_gps[NB_DATA_RX_GPS-3];
                  
              }
          }
    
    }

}
