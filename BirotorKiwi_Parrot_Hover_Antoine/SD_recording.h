#include "Arduino.h"

extern bool OK_SDCARD;
//Fonctions
void checkExist(void);
void init_SD_record(void);
void SD_record(float *DATA, int NbData, uint16_t secs, uint8_t sw_Val);
