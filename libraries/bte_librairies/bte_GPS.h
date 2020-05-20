#ifndef _BTE_GPS_PITOT_H
#define _BTE_GPS_PITOT_H

#include "Arduino.h"

class bte_GPS
{

  public:

  bte_GPS(HardwareSerial *serial);
  void read_GPS(void);

  float _x_gps;
  float _y_gps;
  float _z_gps;
  float _vx_gps;
  float _vy_gps;
  float _vz_gps;
  float _lat_0_int;
  float _lat_0_dec;
  float _lon_0_int;
  float _lon_0_dec;
  float _alti_0;
  uint8_t _GPS_mode;

  private:

  HardwareSerial *_serial;
  
};

#endif