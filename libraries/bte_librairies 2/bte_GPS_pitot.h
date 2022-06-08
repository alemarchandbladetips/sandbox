#ifndef _BTE_GPS_PITOT_H
#define _BTE_GPS_PITOT_H

#include "Arduino.h"

class bte_GPS_pitot
{

  public:

  bte_GPS_pitot(HardwareSerial *serial);
  void read_GPS_pitot(void);

  float _x_gps;
  float _y_gps;
  float _z_gps;
  float _vx_gps;
  float _vy_gps;
  float _vz_gps;
  float _speed_pitot;
  uint8_t _GPS_mode;

  private:

  HardwareSerial *_serial;
  
};

#endif