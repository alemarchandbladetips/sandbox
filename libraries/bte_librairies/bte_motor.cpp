#include "bte_motor.h"


bte_motor::bte_motor(uint8_t pin_motor)
{
  my_motor.attach(pin_motor);
  my_motor.writeMicroseconds(0);
}

int8_t bte_motor::set_range(uint16_t min_motor,uint16_t max_motor)
{
  if ( min_motor > max_motor)
	{
		return -1;
	}
	
  _min_motor = min_motor;
  _max_motor = max_motor;
  	
  return 0;
}

int8_t bte_motor::set_normalized_pwm(float pwm_norm)
{
  float pwm_tmp;
  if(pwm_norm>1 || pwm_norm<0)
  {
    my_motor.writeMicroseconds(_min_motor);
  	return -1;
  }
  pwm_tmp = 1.0*_min_motor + pwm_norm * (1.0*_max_motor-1.0*_min_motor);
  return set_pwm((uint16_t)pwm_tmp);
}

int8_t bte_motor::set_pwm(uint16_t pwm_int)
{
  if( pwm_int>_max_motor || pwm_int<_min_motor )
  {
    my_motor.writeMicroseconds(_min_motor);
  	return -1;
  }
  my_motor.writeMicroseconds(pwm_int);
  return 0;
}
