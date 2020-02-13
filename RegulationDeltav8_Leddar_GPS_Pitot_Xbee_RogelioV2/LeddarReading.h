#include "GlobalVariables.h"
#include "Arduino.h"

extern float hauteur_leddar, Hauteur_med_leddar;

// Fonctions
uint16_t medianFilter16(uint16_t val, uint16_t XX[], uint8_t Elem);
void lectureLeddar(void);
