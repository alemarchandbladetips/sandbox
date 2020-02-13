#ifndef _BTE_MOTOR_H
#define _BTE_MOTOR_H


#include "Servo.h"

class bte_motor
{
  
	public:

  bte_motor(uint8_t pin_motor);
	int8_t set_range(uint16_t min_motor,uint16_t max_motor);
	int8_t set_normalized_pwm(float pwm_norm);
	int8_t set_pwm(uint16_t pwm_int);

  private:
  
	uint16_t _min_motor;
	uint16_t _max_motor;
  Servo my_motor;
	
};
	
#endif