#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include "Servo.h"

#define DT_US 10000
#define DT_S 0.01

// Pins à connecter de BMP388 SDA = SDI,  SCK = SCL, en teensy 3.5 SCL0 = pin 19 & SDA0 = pin18
Adafruit_BMP3XX bmp; // I2C


const float alpha_d = 0.1; // 0.1s de temps de réponse à 100hz. Diminuer alpha pour augmenter le temps de réponse du filtre et réduire le bruit sur la dérivée. Si besoin
float pression_des = 990.0; //mbar

float Commande_P, Commande_D, Commande_I, Commande_II;
float Kp, Kd, Ki,Kii;
float d_pression, d_pression_f, pression, pression_prev;

uint32_t pwm_servo;
Servo my_servo;
uint32_t pwm_0 = 1100;
const uint32_t max_servo = 1300, min_servo = 1000;
const int pin_servo = 16;

float p_0 = 991.0;

float step_p = 4.0;
float step_p_close = 20.0;

uint32_t temps_us, dt_us, counter, i;

void setup() {

  Serial.begin(115200);
  while (!Serial);
  Serial.println("BMP388 test");

  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_100_HZ);

  while(!bmp.performReading())
  {
    delay(1);
  }

  //delay(2000);
  my_servo.attach(pin_servo);

  d_pression_f = 0.0;
  temps_us = micros();
  Commande_I = 0;
  Commande_II = 0.0;
  counter = 0;
  pression_des = pression_prev;

  for(i=0;i<500;i++)
  {
    my_servo.writeMicroseconds(min_servo);
    delay(10);
  }

  bmp.performReading();
  pression_prev = bmp.pressure/100.0;
  p_0 = bmp.pressure/100.0;

  Kp = 35.0;
  Ki = 25.0;
  Kd = 2.5;

  pression_des = p_0;
  //Commande_I = 50/Ki;
  Commande_I = 0.0;
  
}

void loop() {
  // put your main code here, to run repeatedly:
  
  dt_us = micros() - temps_us;
  
  if ( dt_us > DT_US) // boucle principale cadencée à 100 Hz
  {
    temps_us += dt_us;
    counter ++;

    bmp.performReading();
    pression = bmp.pressure/100.0; //in mbar

    // dérivée filtrée de la pression
    d_pression = (pression - pression_prev)/DT_S;
    // le filtrage necessaire dépend du bruit du capteur, pour commencer, temps de réponse de 0.1s -> quasiment pas de filtrage.
    d_pression_f = (1-alpha_d)*d_pression_f + alpha_d*d_pression;
    

    if(counter<600)
    {
      pression_des = p_0;
      pwm_servo = min_servo;
    } else if (counter == 600)
    {
      p_0 = pression;
    } else if(counter<800)
    {
      pression_des = p_0 + step_p;
      Commande_I += (pression_des-pression)*DT_S;
  
      Commande_I = constrain(Commande_I,0,300/Ki);
  
      pwm_servo = (uint32_t)(Kp*(pression_des-pression) 
                            + Ki * Commande_I
                            + (float)pwm_0 );
    } else
    {
      Commande_I = 0;
      counter = 0;  
    }

   

   

    //pwm_servo = constrain(pwm_servo,min_servo,max_servo);
    my_servo.writeMicroseconds(pwm_servo);
  
    pression_prev = pression;

    //Serial.print(Kp*(pression_des-pression));Serial.print(" ");
    //Serial.print(Ki*Commande_I);Serial.print(" ");
    //Serial.print((pwm_servo));Serial.print(" ");
    Serial.print(pression_des);Serial.print(" ");
    Serial.print(pression);Serial.print(" ");
    //Serial.print(pwm_servo);Serial.print(" ");

    Serial.println(" ");

  }

}
