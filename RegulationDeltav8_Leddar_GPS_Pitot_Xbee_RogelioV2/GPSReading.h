#include "Arduino.h"

#define R_TERRE 6366000.0

extern float x_gps, y_gps, z_gps;
extern float vx_gps, vy_gps, vz_gps;
extern float Speed_Avion_med;
extern uint8_t GPS_mode;

//Fonctions
void lectureGPS(void);
