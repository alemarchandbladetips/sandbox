//increase buffer rx for gps

#include "Wire.h"
#include "RadioSignals.h"
#include "Functions.h"
#include "GlobalVariables.h"

#define SAMPLING_TIME_US 10000
#define WIDTH_PWM 2500   // periode du pwm pour les moteurs  

  /******Definitions Motor*************/
  MyServo M1, M2, M3, M4;
  
  float pwminit_motor1 = 1080; //pwm initial du motor 1 
  float pwminit_motor2 = 1080;
  float pwminit_motor3 = 1080;
  float pwminit_motor4 = 1080;

  float pwmmax_motor = 1850;
  
  //  pin, pwmMin, pwmMax, pwmInit, pwm, control, rot
  // gaz limité à 1850 us
  motor Motor1  {2, 1000, pwmmax_motor, pwminit_motor1, pwminit_motor1, 0, -1};
  motor Motor2  {3, 1000, pwmmax_motor, pwminit_motor2, pwminit_motor2, 0,  1};
  motor Motor3  {4, 1000, pwmmax_motor, pwminit_motor3, pwminit_motor3, 0, -1};
  motor Motor4  {5, 1000, pwmmax_motor, pwminit_motor4, pwminit_motor4, 0,  1};
  
  /******Filtrage pwm*************/

  float alpha_pwm = 0.02;
  float decay_max = 5;
  float pwm_tmp1, pwm_tmp2, pwm_tmp3, pwm_tmp4;
  float pwm_filt1, pwm_filt2, pwm_filt3, pwm_filt4 = 1000.0;
  
  
  /*********Timers***************/
  uint32_t dt, tt;   
  
  /*********Others ***************/
  
void setup()
{
   Serial.begin(115200);
   Serial1.begin(115200); // Radio Telecommande
   
   //Initialisation des Moteurs

   M1.set_resol(12);
//   M2.set_resol(12);
//   M3.set_resol(12);
//   M4.set_resol(12);

   M1.attach_pin_us(Motor1.pin,Motor1.pwmMin,Motor1.pwmMax,WIDTH_PWM);
//   M2.attach_pin_us(Motor2.pin,Motor2.pwmMin,Motor2.pwmMax,WIDTH_PWM);
//   M3.attach_pin_us(Motor3.pin,Motor3.pwmMin,Motor3.pwmMax,WIDTH_PWM);
//   M4.attach_pin_us(Motor4.pin,Motor4.pwmMin,Motor4.pwmMax,WIDTH_PWM);
   
   M1.write_us(Motor1.pwmMin);
//   M2.write_us(Motor2.pwmMin);
//   M1.write_us(Motor3.pwmMin);
//   M2.write_us(Motor4.pwmMin);

   tt = micros();
}


void loop()
{   

   
    /********Lecture du Radio********************/ 
    lectureRadio(Serial1);

    /************Boucle de régulation********************/
    dt = micros() - tt;
    if (dt > SAMPLING_TIME_US)
    {
        tt += dt;

        pwm_tmp1 = Motor1.pwmInit + 800*thrust;
        pwm_tmp2 = Motor2.pwmInit + 800*thrust;
        pwm_tmp3 = Motor3.pwmInit + 800*thrust;
        pwm_tmp4 = Motor4.pwmInit + 800*thrust;

        if(pwm_tmp1 < pwm_filt1)
        {
          pwm_filt1 = max(pwm_tmp1,pwm_filt1-decay_max);//(1-alpha_pwm)*pwm_filt1 + alpha_pwm*pwm_tmp1;
        } else
        {
          pwm_filt1 = pwm_tmp1;
        }
        Motor1.pwm = pwm_filt1;

        if(pwm_tmp2 < pwm_filt2)
        {
          pwm_filt2 = (1-alpha_pwm)*pwm_filt2 + alpha_pwm*pwm_tmp2;
        } else
        {
          pwm_filt2 = pwm_tmp2;
        }
        Motor2.pwm = pwm_filt2;

        if(pwm_tmp3 < pwm_filt3)
        {
          pwm_filt3 = (1-alpha_pwm)*pwm_filt3 + alpha_pwm*pwm_tmp3;
        } else
        {
          pwm_filt3 = pwm_tmp3;
        }
        Motor3.pwm = pwm_filt3;

        if(pwm_tmp4 < pwm_filt4)
        {
          pwm_filt4 = (1-alpha_pwm)*pwm_filt4 + alpha_pwm*pwm_tmp4;
        } else
        {
          pwm_filt4 = pwm_tmp4;
        }
        Motor4.pwm = pwm_filt4;


        // Sécurité des moteurs
        if (switch_F == 2 )
        {
          Motor1.pwm = Motor1.pwmMin;
          Motor2.pwm = Motor2.pwmMin;
          Motor3.pwm = Motor3.pwmMin;
          Motor4.pwm = Motor4.pwmMin;
        }

        M1.write_us(Motor1.pwm);
//        M2.write_us(Motor2.pwm);
//        M3.write_us(Motor3.pwm);
//        M4.write_us(Motor4.pwm);

        Serial.println(Motor1.pwm);
                
    }

    
}
