#include "bte_servo.h"


bte_servo::bte_servo(uint8_t pin_servo)
{
  	my_servo.attach(pin_servo);
  	_power_flag = 0;
}

int8_t bte_servo::set_range(uint16_t min_servo,uint16_t max_servo)
{
  	if ( min_servo > max_servo)
	{
		return -1;
	}
	
  	_min_servo = min_servo;
  	_max_servo = max_servo;
  	_zero_servo = (max_servo+min_servo)/2;
  	
  	return 0;
}

int8_t bte_servo::set_range(uint16_t min_servo,uint16_t max_servo, uint16_t zero_servo)
{

  	if ( min_servo > max_servo || zero_servo < min_servo || zero_servo > max_servo)
	{
		return -1;
	}
	
  	_min_servo = min_servo;
  	_max_servo = max_servo;
  	_zero_servo = zero_servo;
  	
  	return 0;
}

int8_t bte_servo::set_normalized_pwm(float pwm_norm)
{
  	float pwm_tmp;
  	
  	if( pwm_norm > 1 )
  	{
  		set_pwm(_max_servo);
    	return -1;
    	
  	} else if ( pwm_norm < -1 )
  	{
    	set_pwm(_min_servo);
    	return -1;
    	
  	} else if(pwm_norm>0)
  	{
    	pwm_tmp = 1.0*_zero_servo + pwm_norm*(1.0*_max_servo-1.0*_zero_servo);
    	return set_pwm((uint16_t)pwm_tmp);
    	
  	} else if (pwm_norm<0)
  	{
    	pwm_tmp = 1.0*_zero_servo + pwm_norm*(1.0*_zero_servo-1.0*_min_servo);
    	return set_pwm((uint16_t)pwm_tmp);
    	
  	} else
  	{
    	return set_pwm(_zero_servo);
  	}
}

int8_t bte_servo::set_pwm(uint16_t pwm_int)
{
	if(_power_flag == 0)
	{
		my_servo.writeMicroseconds(_zero_servo);
	}
	
  	if( pwm_int > _max_servo )
  	{
  		_current_pwm = _max_servo;
    	my_servo.writeMicroseconds(_current_pwm);
    	return -1;
  	} 	else if ( pwm_int < _min_servo )
  	{
  		_current_pwm = _min_servo;
    	my_servo.writeMicroseconds(_current_pwm);
    	return -1;
  	}
  	
  	_current_pwm = pwm_int;
  	my_servo.writeMicroseconds(_current_pwm);
  	
  	return 0;
}

uint16_t bte_servo::get_current_pwm(void)
{
	return _current_pwm;
}

void bte_servo::power_on(void)
{
	_power_flag = 1;
}

void bte_servo::power_off(void)
{
	_power_flag = 0;
}
