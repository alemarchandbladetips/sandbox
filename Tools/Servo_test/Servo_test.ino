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

#define SERVO_IMPULSE_MAX 950
#define SERVO_IMPULSE_MIN 450
#define SERVO_IMPULSE_ZERO 700
#define SERVO_IMPULSE_FOLDED 1200

#include <Servo.h>

int pwm_min=1000;
int X=200;

Servo myservo;

void setup(void)
{
  // Pins des pwm
 pinMode(10, OUTPUT);     
 Serial.begin(115200);

  myservo.attach(10);
}

// angle in +- 38deg
int16_t deg2us(float angle)
{
  int16_t retval;

  retval = (int16_t)(angle*6.6 + SERVO_IMPULSE_ZERO);
  retval = min(retval,SERVO_IMPULSE_MAX);
  retval = max(retval,SERVO_IMPULSE_MIN);
  
  return retval;
}

void loop(void)
{
// myservo.writeMicroseconds(450);
// delay(1000);
// myservo.writeMicroseconds(800);
// delay(1000);
// myservo.writeMicroseconds(1085);
// delay(1000);

  myservo.writeMicroseconds(deg2us(32));
  delay(2000);
  myservo.writeMicroseconds(deg2us(0));
  delay(2000);
  myservo.writeMicroseconds(deg2us(-32));
  delay(2000);
  myservo.writeMicroseconds(deg2us(0));
  delay(2000);
}

// perpendiculaire = 733; -40 = 1000 ; 35 = 500

