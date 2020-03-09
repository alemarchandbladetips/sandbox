#include "bte_GPS_pitot.h"
#include "stdint.h"

#define NB_DATA_RX_GPS 31

bte_GPS_pitot::bte_GPS_pitot(HardwareSerial *serial)
{
  _x_gps = 0;
  _y_gps = 0;
  _z_gps = 0;
  _vx_gps = 0;
  _vy_gps = 0;
  _vz_gps = 0;
  _speed_pitot = 0;
  _GPS_mode = 0;
  _serial = serial;
  (*_serial).begin(57600); 
}

void bte_GPS_pitot::read_GPS_pitot(void)
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
                  for (int j=0; j<7; j++)
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
                        if(j==6) _speed_pitot = val_float;
                  }
                  
                  _GPS_mode = temp_data_gps[NB_DATA_RX_GPS-3];
                  
              }
          }
    
    }

}
