#include <Adafruit_BNO055.h>

#define PIN_PLUS 7
#define PIN_MINUS 8

#define DT_US 10000

#define HYST_ANGLE 5

#define ALPHA_ANGLE 0.2
#define M_ANGLE 0.85

float k = 1/(1+2*M_ANGLE/ALPHA_ANGLE+(1/ALPHA_ANGLE)*(1/ALPHA_ANGLE));
float a1 = 2*M_ANGLE/ALPHA_ANGLE+2*(1/ALPHA_ANGLE)*(1/ALPHA_ANGLE);
float a2 = -(1/ALPHA_ANGLE)*(1/ALPHA_ANGLE);

int8_t motor_direction = 0;

Adafruit_BNO055 bno = Adafruit_BNO055();

float angle, angle_f, angle_f1, angle_f2 = 0;

uint32_t timer;


void setup() {
  // put your setup code here, to run once:
  pinMode(PIN_PLUS,"output");
  pinMode(PIN_MINUS,"output");

  digitalWrite(PIN_PLUS,0);
  digitalWrite(PIN_MINUS,0);

  if ( !bno.begin() )
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  timer = micros();

}

void loop() {
   if( micros()-timer > DT_US)
  {
    timer += DT_US;

    imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    

    //Serial.print(acc.x());Serial.print("\t");
    //Serial.print(acc.y());Serial.print("\t");
    //Serial.print(acc.z());Serial.print("\t");

    angle = atan2(acc.y(),acc.z())*180/3.14;

    angle_f2 = angle_f1;
    angle_f1 = angle_f;

    angle_f = k*(angle + a1*angle_f1 + a2*angle_f2); 

    Serial.print(angle);Serial.print("\t");
    Serial.print(angle_f);Serial.print("\t");

    if(motor_direction == 0)
    {
      if(angle_f > HYST_ANGLE)
      {
        motor_direction = 1;
      }
      if(angle_f < -HYST_ANGLE )
      {
        motor_direction = -1;
      }
    } else if(motor_direction > 0)
    {
      if(angle_f < 0)
      {
        motor_direction = 0;
      }
      
    } else if(motor_direction < 0)
    {
      if(angle_f > 0)
      {
        motor_direction = 0;
      }
    }
    digitalWrite(PIN_PLUS,motor_direction > 0);
    digitalWrite(PIN_MINUS,motor_direction < 0);
    Serial.print(motor_direction);Serial.print("\t");

    Serial.println("\t");
  }
}
