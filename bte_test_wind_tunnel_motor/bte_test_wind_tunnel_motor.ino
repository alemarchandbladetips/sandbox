#include "bte_wind_tunnel_motor.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm_driver1 = Adafruit_PWMServoDriver(0x40); //nothing
Adafruit_PWMServoDriver pwm_driver2 = Adafruit_PWMServoDriver(0x41); //A0 soldered

bte_wind_tunnel_motor motor1;
bte_wind_tunnel_motor motor2;

int8_t ret_val;
uint16_t pwm_ret;
float power_ret;

void setup() {
  Serial.begin(115200);
  Serial.println("//////////////////////////////////////////");
  Serial.println("Test for the bte_wind_tunnel_motor");
  Serial.println("//////////////////////////////////////////");

  pwm_driver1.begin();
 
  pwm_driver1.setOscillatorFrequency(27000000);
  pwm_driver1.setPWMFreq(50);  // Analog servos run at ~50 Hz updates

  pwm_driver2.begin();
 
  pwm_driver2.setOscillatorFrequency(27000000);
  pwm_driver2.setPWMFreq(50);  // Analog servos run at ~50 Hz updates

  Serial.print("Set param with min>max");
  ret_val = motor1.set_all_params(0,&pwm_driver1,2000,1000);
  if(ret_val==-1)
  {
    Serial.println(" Test OK");
  } else
  {
    Serial.println(" Test FAILED");
  }


  delay(1);
  
  Serial.print("Set param with motor pin < 0");
  ret_val = motor1.set_all_params(-1,&pwm_driver1,1000,2000);
  if(ret_val==-1)
  {
    Serial.println(" Test OK");
  } else
  {
    Serial.println(" Test FAILED");
  }


  delay(1);
  
  Serial.print("Set param with motor pin > 15");
  ret_val = motor1.set_all_params(16,&pwm_driver1,1000,2000);
  if(ret_val==-1)
  {
    Serial.println(" Test OK");
  } else
  {
    Serial.println(" Test FAILED");
  }


  delay(1);
  
  Serial.print("Set param with good parameters");
  ret_val = motor1.set_all_params(0,&pwm_driver1,1000,2000);
  if(ret_val==0)
  {
    Serial.println(" Test OK");
  } else
  {
    Serial.println(" Test FAILED");
  }


  delay(1);
  
  Serial.print("Set param with good parameters");
  ret_val = motor2.set_all_params(5,&pwm_driver2,1000,1500);
  if(ret_val==0)
  {
    Serial.println(" Test OK");
  } else
  {
    Serial.println(" Test FAILED");
  }


  delay(1);
  
  Serial.print("Set pwm in bound with motor off");
  ret_val = motor2.set_pwm(1000);
  if(ret_val==-1)
  {
    Serial.println(" Test OK");
  } else
  {
    Serial.println(" Test FAILED");
  }

  motor2.power_on();
  motor1.power_on();


  delay(1);
  
  Serial.print("Set pwm out of bound");
  ret_val = motor2.set_pwm(999);
  if(ret_val==-1)
  {
    Serial.println(" Test OK");
  } else
  {
    Serial.println(" Test FAILED");
  }


  delay(1);
  
  Serial.print("Set pwm in bound");
  ret_val = motor1.set_pwm(1000);
  if(ret_val==0)
  {
    Serial.println(" Test OK");
  } else
  {
    Serial.println(" Test FAILED");
  }

  
  delay(1);

  Serial.print("Set pwm out of bound");
  ret_val = motor2.set_pwm(1501);
  if(ret_val==-1)
  {
    Serial.println(" Test OK");
  } else
  {
    Serial.println(" Test FAILED");
  }


  delay(1);
  
  Serial.print("Set pwm in bound");
  ret_val = motor2.set_pwm(1500);
  if(ret_val==0)
  {
    Serial.println(" Test OK");
  } else
  {
    Serial.println(" Test FAILED");
  }


  delay(1);
  
  Serial.print("get pwm");
  pwm_ret = motor2.get_current_pwm();
  if(pwm_ret==1500)
  {
    Serial.println(" Test OK");
  } else
  {
    Serial.println(" Test FAILED");
  }


  delay(1);
  
  Serial.print("Set pwm in bound");
  ret_val = motor2.set_pwm(1000);
  if(ret_val==0)
  {
    Serial.println(" Test OK");
  } else
  {
    Serial.println(" Test FAILED");
  }


  delay(1);
  
  Serial.print("Set pwm in bound");
  ret_val = motor1.set_pwm(2000);
  if(ret_val==0)
  {
    Serial.println(" Test OK");
  } else
  {
    Serial.println(" Test FAILED");
  }


  delay(1);
  
  motor1.power_off();
  Serial.print("Set pwm after power off");
  ret_val = motor1.set_pwm(2000);
  if(ret_val==-1)
  {
    Serial.println(" Test OK");
  } else
  {
    Serial.println(" Test FAILED");
  }


  delay(1);
  
  Serial.print("Get pwm after power off");
  pwm_ret = motor1.get_current_pwm();
  if(pwm_ret==1000)
  {
    Serial.println(" Test OK");
  } else
  {
    Serial.println(" Test FAILED");
  }


  delay(1);
  
  Serial.print("Set pwm limits wrong values");
  ret_val = motor2.set_range(2000,1000);
  if(ret_val==-1)
  {
    Serial.println(" Test OK");
  } else
  {
    Serial.println(" Test FAILED");
  }


  delay(1);
  
  Serial.print("Set pwm limits");
  ret_val = motor2.set_range(1200,1300);
  if(ret_val==0)
  {
    Serial.println(" Test OK");
  } else
  {
    Serial.println(" Test FAILED");
  }


  delay(1);
  
  Serial.print("Set pwm out of bound");
  ret_val = motor2.set_pwm(1199);
  if(ret_val==-1)
  {
    Serial.println(" Test OK");
  } else
  {
    Serial.println(" Test FAILED");
  }


  delay(1);
  
  Serial.print("Set pwm out of bound");
  ret_val = motor2.set_pwm(1301);
  if(ret_val==-1)
  {
    Serial.println(" Test OK");
  } else
  {
    Serial.println(" Test FAILED");
  }


  delay(1);
  
  Serial.print("Set pwm in bound");
  ret_val = motor2.set_pwm(1250);
  if(ret_val==0)
  {
    Serial.println(" Test OK");
  } else
  {
    Serial.println(" Test FAILED");
  }


  delay(1);
  
  ret_val = motor2.set_range(1000,2000);
  Serial.print("Set wind speed in bound");
  ret_val = motor2.set_wind_speed(10);
  if(ret_val==0)
  {
    Serial.println(" Test OK");
  } else
  {
    Serial.println(" Test FAILED");
  }


  delay(1);
  
  Serial.print("Set wind speed out of bound");
  ret_val = motor2.set_wind_speed(100);
  if(ret_val==-1)
  {
    Serial.println(" Test OK");
  } else
  {
    Serial.println(" Test FAILED");
  }


  delay(1);
  
  Serial.print("Set wind speed out of bound");
  ret_val = motor2.set_wind_speed(-5);
  if(ret_val==-1)
  {
    Serial.println(" Test OK");
  } else
  {
    Serial.println(" Test FAILED");
  }

  delay(1);

    Serial.println("//////////////////////////////////////////");
  Serial.println("Test finished");
  Serial.println("//////////////////////////////////////////");
  
}

// You can use this function if you'd like to set the pulse length in seconds
// e.g. setServoPulse(0, 0.001) is a ~1 millisecond pulse width. It's not precise!


void loop() {

}
