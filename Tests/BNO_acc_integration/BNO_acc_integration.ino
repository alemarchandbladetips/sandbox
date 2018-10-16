//A2 prend les info du BNO et les transmet via serial ver le Main

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Wire.h>

///VARIABLES BNO
float wz, wy, wx, angx=0, angy, angz, accx, accy, accz, x, y, z, vx, vy, vz;
uint8_t calib_status_sys, calib_status_acc, calib_status_gyr, calib_status_mag;

long sampling_period_ms;

long last_time;

// appel IMU
Adafruit_BNO055 bno = Adafruit_BNO055();
//Timer mon_timer;

void setup() {
  

  Serial.begin(115200);
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);
    /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(true);

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");

  sampling_period_ms = 10;

  x = 0;
  y = 0;
  z = 0;
  vx = 0;
  vy = 0;
  vz = 0;

}


void loop() {

  while(millis()-last_time<sampling_period_ms){
    delayMicroseconds(1);
    last_time += sampling_period_ms;
  }

// Calibration status
    bno.getCalibration(&calib_status_sys, &calib_status_gyr, &calib_status_acc, &calib_status_mag);

//  linear accelerations  
    imu::Vector<3> linear_acc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    accx=linear_acc.x();
    accy=linear_acc.y();
    accz=linear_acc.z();

    vx += accx*sampling_period_ms/1000;
    vy += accy*sampling_period_ms/1000;
    vz += accz*sampling_period_ms/1000;
    x += vx*sampling_period_ms/1000;
    y += vy*sampling_period_ms/1000;
    z += vz*sampling_period_ms/1000;
 
Serial.print(vx);Serial.print("  ");
Serial.print(vy);Serial.print("  ");
Serial.print(vz);Serial.print("  ");
Serial.print(x);Serial.print("  ");
Serial.print(y);Serial.print("  ");
Serial.print(z);Serial.println("  ");

}  



