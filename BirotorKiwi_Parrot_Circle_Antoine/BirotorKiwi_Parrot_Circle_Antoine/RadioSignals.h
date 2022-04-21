#include "Arduino.h"

#ifndef RADIO_SIGNAL_H
  #define RADIO_SIGNAL_H
  
extern float aileron;
extern float elevator;
extern float rudder;
extern float knob;
extern float thrust;
extern uint8_t switch_D;
extern uint8_t switch_F;
extern uint8_t switch_C;

extern uint8_t perteRadio;
//Fonctions
//void lectureRadio (void);
void lectureRadio (Stream &mySerial);

void print_Serial(uint8_t *X, int NbData, char *S);
void clean_Serial(Stream &mySerial);
uint8_t identPack(uint8_t *P);
float normVal(float val, float minVal, float maxVal);
float normVal_pn(float val, float minVal, float maxVal);
uint8_t switch_state(uint16_t sw_val, int val0, int  val1, int  val2);

#endif
