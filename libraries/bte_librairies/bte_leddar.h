#ifndef _BTE_LEDDAR_H
#define _BTE_LEDDAR_H


#include "Arduino.h"
#include "bte_functions.h"

#define BTE_LEDDAR_NFILT_MED 20

class bte_leddar
{
  public:

  bte_leddar(HardwareSerial *serial);
  void read_leddar(void);

  float _hauteur;
  uint8_t _validity_flag;

  private:

  HardwareSerial *_serial;
  uint16_t HistoLeddar[BTE_LEDDAR_NFILT_MED];
  uint16_t Hauteur_med_leddar;
  uint16_t buffer_uint16;
  uint8_t *ptr_buffer_uint16;
  
};

#endif