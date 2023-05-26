#include <Adafruit_BNO055.h>
#include <Servo.h>

uint32_t timer;
uint32_t dt_us = 20000;

Adafruit_BNO055 bno = Adafruit_BNO055();

Servo myservo;

float angle = 0;
uint16_t consigne_servo;
float consigne_servoI = 0;

void setup() {
  Serial.begin(115200);
  
  if ( !bno.begin() )
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  myservo.attach(14,1000,2000);

  timer = micros();

}

void loop() {

  if( micros()-timer > dt_us)
  {
    timer += dt_us;

    imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    

    Serial.print(acc.x());Serial.print("\t");
    Serial.print(acc.y());Serial.print("\t");
    Serial.print(acc.z());Serial.print("\t");

    angle = 0.03*atan2(acc.x(),-acc.y())*180/3.14+0.97*angle;

    Serial.print(angle);Serial.print("\t");

    consigne_servoI -= 0.3*angle;
    consigne_servo = 1500-5*angle +consigne_servoI;

    Serial.print(consigne_servoI);Serial.print("\t");
    Serial.print(consigne_servo);Serial.print("\t");

    myservo.writeMicroseconds(consigne_servo);

    Serial.println("\t");

  }
}
