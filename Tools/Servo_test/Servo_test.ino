// PGM pour generer un PWM pour contrôler un moteur. 

// le pin 3 a la valeur de PWM de 1000 ms ( MIN )
// le pin 5 a la valeur de PWM de 1200 ms ( Valeur utilisable )
// le pin 6 a la valeur de PWM de 1400 ms ( Valeur utilisable )
// le pin 9 a la valeur de PWM de 1600 ms ( Valeur utilisable )
// le pin 10 a la valeur de PWM de 1800 ms ( Valeur utilisable )
// le pin 11 a la valeur de PWM de 1900 ms ( MAX )

// Si le variateur a été débranché,pour allumer le moteur:
// 1)connecter le câble sur le pin 13 jusqu'à ce que quelques bips retentissent
// 2)connecter le câble sur le pin 10 jusqu'à ce que quelques bips retentissent
// 3)connecter le câble sur le pin 10 jusqu'à ce que le moteur se mette à tourner

#include <Servo.h>

int pwm_min=1000;
int X=200;

Servo myservo_min;
Servo myservo_1200;
Servo myservo_1400;
Servo myservo_1600;
Servo myservo_1800;
Servo myservo_max;

void setup(void)
{
  // Pins des pwm
  pinMode(3, OUTPUT);     
 Serial.begin(115200);

  myservo_min.attach(3);


}


void loop(void)
{
 myservo_min.writeMicroseconds(450);
 delay(1000);
 myservo_min.writeMicroseconds(800);
 delay(1000);
 myservo_min.writeMicroseconds(1085);
 delay(1000);
}

