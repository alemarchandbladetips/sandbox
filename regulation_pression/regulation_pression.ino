#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include "Servo.h"

#define DT_US 10000
#define DT_S 0.01

// Pins à connecter de BMP388 SDA = SDI,  SCK = SCL, en teensy 3.5 SCL0 = pin 19 & SDA0 = pin18
Adafruit_BMP3XX bmp; // I2C


const float alpha_d = 0.2; // 0.1s de temps de réponse à 100hz. Diminuer alpha pour augmenter le temps de réponse du filtre et réduire le bruit sur la dérivée. Si besoin
float pression_des = 990.0; //mbar

float Commande_P, Commande_D, Commande_I, Commande_II;
float Kp, Kd, Ki,Kii;
float d_pression, d_pression_f, pression, pression_prev;

uint32_t pwm_servo;
Servo my_servo;
const uint32_t pwm_0 = 1125;
const uint32_t max_servo = 1500, min_servo = 750;
const int pin_servo = 3;

float p_0 = 994.0;

float step_p = 5.0;
float step_p_close = 20.0;

uint32_t temps_us, dt_us, counter;

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
  bmp.performReading();
  pression_prev = bmp.pressure/100.0;
  p_0 = bmp.pressure/100.0;

  d_pression_f = 0.0;
  temps_us = micros();
  Commande_I = 0;
  Commande_II = 0.0;
  counter = 0;
  pression_des = pression_prev;
}

void loop() {
  // put your main code here, to run repeatedly:
  
  dt_us = micros() - temps_us;
  
  if ( dt_us > DT_US) // boucle principale cadencée à 100 Hz
  {
    temps_us += dt_us;
    counter ++;

    if(counter<50)
    {
      pression_des = p_0-100;
      //Commande_I = 0;
      //Commande_II = 0.0;
    } else if (counter == 50)
    {
      pression_des = p_0+step_p;
      Commande_I = 0;
    }
    else if(counter<100)
    {
      pression_des = p_0+step_p;
      //Commande_I = 0;
      //Commande_II = 0.0;
    } else if (counter<200)
    {
      pression_des = p_0+step_p_close;
    } else
    {
      counter = 0;
    }

    bmp.performReading();
    pression = bmp.pressure/100.0; //in mbar

    Kp = 20;
    Ki = 150.0;
    Kii = 0.0;
    Kd = 0.9;

    // dérivée filtrée de la pression
    d_pression = (pression - pression_prev)/DT_S;
    // le filtrage necessaire dépend du bruit du capteur, pour commencer, temps de réponse de 0.1s -> quasiment pas de filtrage.
    d_pression_f = (1-alpha_d)*d_pression_f + alpha_d*d_pression;

    Commande_I += (pression-pression_des)*DT_S;
    Commande_II += Commande_I*DT_S;

    Commande_I = constrain(Commande_I,-365/Ki,375/Ki);
    Commande_II = constrain(Commande_II,-375.0/Kii,375.0/Kii);

    pwm_servo = (uint32_t)(Kp*(pression-pression_des) 
                          + Kd*d_pression_f
                          + Ki*Commande_I 
                          + Kii*Commande_II );

    pwm_servo += pwm_0;
    pwm_servo = constrain(pwm_servo,min_servo,max_servo);
    my_servo.writeMicroseconds(pwm_servo);
  
    pression_prev = pression;

    //Serial.print(Commande_P);Serial.print(" ");
    //Serial.print(Ki*Commande_I);Serial.print(" ");
    Serial.print(pression_des);Serial.print(" ");
    Serial.print(pression);Serial.print(" ");
    //Serial.print(pwm_servo);Serial.print(" ");

    Serial.println(" ");

  }

}
