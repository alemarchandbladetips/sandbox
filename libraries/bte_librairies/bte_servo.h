#ifndef _BTE_SERVO_H
#define _BTE_SERVO_H

#include "Servo.h"

class bte_servo
{
  
	public:

  	bte_servo(uint8_t pin_servo);
	int8_t set_range(uint16_t min_servo,uint16_t max_servo);
	int8_t set_range(uint16_t min_servo,uint16_t max_servo, uint16_t zero_servo);
	int8_t set_normalized_pwm(float pwm_norm);
	int8_t set_pwm(uint16_t pwm_int);
	uint16_t get_current_pwm(void);

  	private:
  
	uint16_t _min_servo;
	uint16_t _max_servo;
  	uint16_t _zero_servo;
  	uint16_t _current_pwm;
  	Servo my_servo;
	
};
	
#endif