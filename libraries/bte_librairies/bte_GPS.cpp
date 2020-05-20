#include "bte_GPS.h"
#include "stdint.h"

#define NB_DATA_RX_GPS 47

bte_GPS::bte_GPS(HardwareSerial *serial)
{
  _x_gps = 0;
  _y_gps = 0;
  _z_gps = 0;
  _vx_gps = 0;
  _vy_gps = 0;
  _vz_gps = 0;
  _serial = serial;
  (*_serial).begin(57600); 
}

void bte_GPS::read_GPS(void)
{    
    if ( (*_serial).available()> (NB_DATA_RX_GPS-1) )
    {    
          if ((*_serial).read() == 0xAA)
          {   
              uint8_t temp_data_gps[NB_DATA_RX_GPS-1];
              (*_serial).readBytes(temp_data_gps, NB_DATA_RX_GPS - 1);
              if ( temp_data_gps[NB_DATA_RX_GPS-2] == 0x55 )
              {   
                  float val_float;
                  uint8_t *ptr_float = (uint8_t *) &val_float;
                  for (int j=0; j<11; j++)
                  {
                        for (int i=0; i<4 ;i++)
                        {
                            *(ptr_float + i) = temp_data_gps[4*j+i];
                        }
                        if(j==0) _y_gps = val_float/100.0;
                        if(j==1) _x_gps = val_float/100.0;
                        if(j==2) _z_gps = val_float/100.0;
                        if(j==3) _vy_gps = val_float/100.0;
                        if(j==4) _vx_gps = val_float/100.0;
                        if(j==5) _vz_gps = -val_float/100.0;
                        if(j==6) _lat_0_int = val_float;
                        if(j==7) _lat_0_dec = val_float;
                        if(j==8) _lon_0_int = val_float;
                        if(j==9) _lon_0_dec = val_float;
                        if(j==10) _alti_0 = val_float;
                  }
                  
              }
          }
    
    }

}
