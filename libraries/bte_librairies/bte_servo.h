#ifndef _BTE_SERVO_H
#define _BTE_SERVO_H

#include "Servo.h"

class bte_servo
{
  
	public:

  	bte_servo(uint8_t pin_servo);
  	
	int8_t set_range(uint16_t min_servo,uint16_t max_servo);
	int8_t set_range(uint16_t min_servo,uint16_t max_servo, uint16_t zero_servo);
	int8_t set_angle_range(float angle_range);
	
	int8_t set_normalized_pwm(float pwm_norm);
	int8_t set_pwm(uint16_t pwm_int);
	int8_t set_angle(float angle);
	
	uint16_t get_current_pwm(void);
	
	void power_on(void);
	void power_off(void);

  	private:
  
	uint16_t _min_servo;
	uint16_t _max_servo;
  	uint16_t _zero_servo;
  	float _angle_range;
  	float _pwm_to_angle;
  	
  	uint16_t _current_pwm;
  	uint8_t _power_flag;
  	
  	Servo my_servo;
	
};
	
#endif