#include "stdint.h"

#define R_TERRE 6366000.0
#define RAD2DEG 57.29577951308
#define DEG2RAD 0.0174532925

#ifndef GLOBAL_H
  #define GLOBAL_H
  extern const float pi;
  
  //extern float roll, pitch;
  
  extern int16_t buffer_int16;
  extern uint16_t buffer_uint16;
  extern uint8_t *ptr_buffer_int16;
  extern uint8_t *ptr_buffer_uint16;
  
  extern const int H;
  
  typedef struct VECT3
  {
      float x;
      float y;
      float z;
  }vect3;

  typedef struct VECT2
  {
      float mod;
      float ang;
      float lim;
  }vect2;

typedef struct motor{
  int      pin;
  uint16_t pwmMin;
  uint16_t pwmMax;
  uint16_t pwmInit;
  float    pwm;
  float    control;
  int      rot;
}motor;

typedef struct REG{
   float KP;
   float KD;
   float KI;
   float P;
   float D;
   float I;
   float control;
} reg;

#endif
