#include "bte_tricycle.h"

bte_tricycle_controller::bte_tricycle_controller(uint8_t acc_pin, uint8_t brake_pin, uint8_t reverse_pin )
{
	_acc_pin = acc_pin;
	_brake_pin = brake_pin;
	_reverse_pin = reverse_pin;
	_speed_flt = 0;
	_speed_int = 0;
	_direction = 1;
	_zero_threshold = BTE_TRICYCLE_DEFAULT_ZERO_THRESHOLD;
	_reverse_delay_ms = BTE_TRICYCLE_DEFAULT_REVERSE_DELAY_MS;
	_motor_gain = BTE_TRICYCLE_DEFAULT_MOTOR_GAIN;
	_motor_zero = BTE_TRICYCLE_DEFAULT_MOTOR_OFFSET;
	_reverse_counter_ms = millis();
	
	pinMode(reverse_pin,OUTPUT);
	pinMode(brake_pin,OUTPUT);
	pinMode(acc_pin,OUTPUT);
	
	digitalWrite(reverse_pin,LOW);
	analogWrite(brake_pin,0);
	analogWrite(acc_pin,_speed_int);
}

int8_t bte_tricycle_controller::set_zero_threshold(float zero_threshold)
{
	if(zero_threshold<0 || zero_threshold > 1)
	{
		return -1;
	}
	
	_zero_threshold = zero_threshold;
	return 0;
}

int8_t bte_tricycle_controller::set_reverse_delay_ms(uint32_t reverse_delay_ms)
{
	_reverse_delay_ms = reverse_delay_ms;
	return 0;
}

int8_t bte_tricycle_controller::set_motor_gain(float motor_gain)
{
	_motor_gain = motor_gain;
	
	return 0;
}

int8_t bte_tricycle_controller::update_reference(float reference)
{
	float reference_tmp;
	
	// prèsfiltrage de la consigne de vitesse
	reference_tmp = reference_function(reference);
	
	//////////////////////////////////////////////////////
	// REVERSE : test du changement de direction avec la zone morte
	if( (_direction > 0) && (reference_tmp < -_zero_threshold) )
    {
      _direction = -1;
      digitalWrite(_reverse_pin,HIGH);
	  _reverse_counter_ms = millis();
    } else if ( (_direction < 0) && (reference_tmp > _zero_threshold) )
    {
      _direction = 1;
      digitalWrite(_reverse_pin,LOW);
	  _reverse_counter_ms = millis();
    }
	
	// temps d'attente après le changement de direction qui empèche d'appliquer de nouvelle commande
	if( millis() - _reverse_counter_ms < _reverse_delay_ms)
	{
		analogWrite(_acc_pin,0);
		return 1;
	}
	
	////////////////////////////////////////////////////////////
	// 
	_speed_flt = reference_tmp;
	
	///////////////////////////////////////////////////////
	// Conversion de la consigne en int puis application
	if(abs(_speed_flt) < _zero_threshold )
	{
		analogWrite(_acc_pin,0);
	} else
	{
		_speed_int = (int32_t)(_motor_gain*(_direction*_speed_flt)+_motor_zero);
		analogWrite(_acc_pin,_speed_int);
	}
	
	return 0;
}

int8_t bte_tricycle_controller::brake(uint8_t brake_flag)
{
	analogWrite(_brake_pin,brake_flag);
	
	return 0;
}

float reference_function(float reference)
{
	return reference; //min(1.0f,max(-1.0f,reference));
}