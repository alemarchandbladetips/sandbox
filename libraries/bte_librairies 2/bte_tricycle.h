#ifndef _BTE_TRICYCLE_H
#define _BTE_TRICYCLE_H

#include <arduino.h>

#define BTE_TRICYCLE_DEFAULT_REVERSE_DELAY_MS 200
#define BTE_TRICYCLE_DEFAULT_ZERO_THRESHOLD 0.01f
#define BTE_TRICYCLE_DEFAULT_MOTOR_GAIN 115.0f
#define BTE_TRICYCLE_DEFAULT_MOTOR_OFFSET 65.0f

class bte_tricycle_controller
{
  
	public:

  	bte_tricycle_controller(uint8_t acc_pin, uint8_t brake_pin, uint8_t reverse_pin );
	
	int8_t set_zero_threshold(float zero_threshold);
	int8_t set_reverse_delay_ms(uint32_t reverse_delay_ms);
	int8_t set_motor_gain(float motor_gain);

	int8_t update_reference(float reference);
	int8_t brake(uint8_t brake_flag);
	int8_t constant_cruise(void);

  	private:
  
	uint8_t _acc_pin;
	uint8_t _brake_pin;
	uint8_t _reverse_pin;
	
	uint32_t _speed_int;
	float _speed_flt;
	int8_t _direction;
	float _zero_threshold;
	
	uint32_t _reverse_delay_ms;
	uint32_t _reverse_counter_ms;
	
	uint32_t _motor_zero;
	float _motor_gain;
	
};

float reference_function(float reference);
	
#endif