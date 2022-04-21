#include "Arduino.h"

extern float x_gps, y_gps, z_gps;
extern float vx_gps, vy_gps, vz_gps;
extern int8_t GPS_init_done;
void decodageGPS(Stream &myserial);
