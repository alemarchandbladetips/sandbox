#include "stdint.h"

#define RAD2DEG 57.29577951308
#define DEG2RAD 0.0174532925

extern const float pi;

extern float roll, pitch;

extern int16_t buffer_int16;
extern uint16_t buffer_uint16;
extern uint8_t *ptr_buffer_int16;
extern uint8_t *ptr_buffer_uint16;

extern const int H;

//extern float w_rotor; // en deg/seg
//extern float wz_imu; // en rad/s
//extern float yaw_Base, yaw_Base_est;
//extern uint16_t last_pwm_M1;
//extern float dpwm, DPWM[];
