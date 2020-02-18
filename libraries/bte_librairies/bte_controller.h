#ifndef _BTE_CONTROLLER_H
#define _BTE_CONTROLLER_H

#include "Arduino.h"
#include <Stream.h>

#define BTE_CONTROLLER_NB_DATA_RADIO 32
#define BTE_CONTROLLER_NF 3

class bte_controller
{

  public:

  bte_controller(HardwareSerial *controller_serial);
  void update_controller(void);
  void set_connection_lost_time_millis(uint32_t connection_lost_time_millis);

  float _thrust;
  float _elevator;
  float _rudder;
  float _aileron;
  float _knob;
  uint8_t _switch_C;
  uint8_t _switch_D;
  uint8_t _switch_F;
  uint8_t _perte_connection;

  private:

  int switchState(int Sw, int a, int b, int c);
  uint8_t medianFilter(int val, uint8_t X[], uint8_t Elem);

  uint8_t DataRadio[BTE_CONTROLLER_NB_DATA_RADIO - 3];
  uint32_t last_millis;
  uint32_t temps_radio;
  uint32_t _connection_lost_time_millis;
  HardwareSerial *_controller_serial;


  /// Variables  Radio Telecommande ///
  uint8_t Trim;
  uint8_t Queue;
  uint8_t Pitch;
  uint8_t Roll;
  uint8_t Collectif;
  uint8_t Switch_C;
  uint8_t Switch_D;
  uint8_t Switch_F;
  uint8_t Switch;
  uint8_t IdSerial1; 
  uint8_t IdSerial2;
  
  
  uint8_t auxTrim;
  uint8_t auxQueue;
  uint8_t auxCollectif;
  uint8_t auxPitch;
  uint8_t auxRoll;
  uint8_t auxSwitch;
  uint8_t auxSwitch_C;
  uint8_t auxSwitch_D;
  uint8_t auxSwitch_F;
  
  
  uint8_t lastTrim;
  uint8_t lastQueue;
  uint8_t lastCollectif;
  uint8_t lastPitch;
  uint8_t lastRoll;
  uint8_t lastSwitch;
  uint8_t lastSwitch_C;
  uint8_t lastSwitch_D;
  uint8_t lastSwitch_F;
  
  //MedianFiltre pour Données dès la telecommande
  uint8_t TrimHisto[BTE_CONTROLLER_NF];
  uint8_t QueueHisto[BTE_CONTROLLER_NF];
  uint8_t CollectifHisto[BTE_CONTROLLER_NF];
  uint8_t PitchHisto[BTE_CONTROLLER_NF];
  uint8_t RollHisto[BTE_CONTROLLER_NF];
  uint8_t SwitchC_Histo[BTE_CONTROLLER_NF];
  uint8_t SwitchD_Histo[BTE_CONTROLLER_NF];
  uint8_t SwitchF_Histo[BTE_CONTROLLER_NF];
  uint8_t Switch_Histo[BTE_CONTROLLER_NF];

};

#endif