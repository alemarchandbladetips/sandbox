#ifndef BTE_WIND_TUNNEL_MOTOR_H_
  #define BTE_WIND_TUNNEL_MOTOR_H_

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>


class bte_wind_tunnel_motor{
  /**
   * \class bte_wind_tunnel_motor
   * \brief class used to instantiate one motor of the wind tunnel designed by Bladetips Energy
   */
   
  public:
  
	  bte_wind_tunnel_motor();
	  /**
       * \fn bte_wind_tunnel_motor()
	   * \brief Default constructor for the bte_wind_tunnel_motor class
       */
       
	  int8_t set_all_params(uint8_t motor_position, Adafruit_PWMServoDriver* pwm_driver,uint16_t min_motor,uint16_t max_motor);
	  /**
       * \fn set_all_params(uint8_t position_motor, Adafruit_PWMServoDriver pwm_driver,uint16_t min_motor,uint16_t max_motor)
	   * \brief Set all the params of the wind tunnel motors
	   * @param (uint8_t)motor_position motor plug position on the driver (0-15)
	   * @param (Adafruit_PWMServoDriver)pwm_driver Pwm driver on which the motor is plug
	   * @param (uint16_t)min_motor min PWM of the motor
	   * @param (uint16_t)max_motor max PWM of the motor
	   * @return -1 if the function fails, 0 otherwise
       */
       
	  int8_t set_range(uint16_t min_motor,uint16_t max_motor);
	  /**
       * \fn set_range(uint16_t min_motor,uint16_t max_motor)
	   * \brief Set all the PWM limits for the motor
	   * @param (uint16_t)min_motor min PWM of the motor
	   * @param (uint16_t)max_motor max PWM of the motors
	   * @return -1 if the function fails, 0 otherwise
       */
       
	  int8_t set_pwm(uint16_t pwm_int);
	  /**
       * \fn set_pwm(uint16_t pwm_int)
	   * \brief Set the pwm 
	   * @param (uint16_t)pwm_int pwm to set on the motor
	   * @return -1 if the function fails (motor off or pwm out of bounds), 0 otherwise
       */
       
	  int8_t set_wind_speed(float wind_speed);
	  /**
       * \fn set_wind_speed(float wind_speed)
	   * \brief Set the wind speed for the motor
	   * @param (float)wind_speed desired wind speed for the motor
	   * @return -1 if the function fails (motor off or pwm corresponding to the wind speed out of bounds), 0 otherwise
       */
       
      uint16_t get_current_pwm(void);
      /**
       * \fn get_current_pwm(void)
	   * \brief Get the current PWM of the motor
	   * @return the current PWM
       */
      
      float get_current_power(void);
      /**
       * \fn get_current_power(void)
	   * \brief Get the current power of the motor
	   * @return the current power (W)
       */
	  
	  void power_on(void);
	  /**
       * \fn power_on(void)
	   * \brief Power on the motor (off by default)
       */
       
	  void power_off(void);
	  /**
       * \fn power_off(void)
	   * \brief Power off the motor (off by default)
       */
  

  private:
	  uint8_t _motor_position;
	  uint8_t _power_flag = 0;
	  Adafruit_PWMServoDriver* _pwm_driver;
	  uint16_t _min_motor;
	  uint16_t _max_motor;
	  uint16_t _pwm_int = 0;
  
  };
  
float get_power_from_pwm(uint16_t pwm_int);
	/**
	* \fn get_power_from_pwm(uint16_t pwm_int)
	* \brief convert the PWM into power (identification done on test bench). Does not apply the PWM.
	* @param (uint16_t)pwm_int pwm of the motor
	* @return the power corresponding to the PWM of the motor
	*/
       
uint16_t get_pwm_from_wind_speed(float wind_speed);
	/**
	* \fn get_pwm_from_wind_speed(float wind_speed)
	* \brief computes the PWM to apply to the motor to reach the desired wind speed (identification done on test bench). Does not apply the PWM.
	* @param (float)wind_speed desired wind speed
	* @return the PWM corresponding to the desired wind speed
	*/
	
#endif