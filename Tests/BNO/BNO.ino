//A2 prend les info du BNO et les transmet via serial ver le Main

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Wire.h>

///VARIABLES BNO
float wz, wy, wx, angx=0, angy, angz, accx, accy, accz, x, y, z, vx, vy, vz;
uint8_t calib_status_sys, calib_status_acc, calib_status_gyr, calib_status_mag;

long sampling_period;

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

  x = 0;
  y = 0;
  z = 0;
  vx = 0;
  vy = 0;
  vz = 0;

}


void loop() {

  while(millis()-last_time<sampling_period){
    delayMicroseconds(1);
  }
 delay(10);
 
//millis();
//delayMicroseconds(1);
   // mon_timer.update();
  
//  vitesses
//    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);  // Prise d'infos
//    wz=euler.z();
//    wy=euler.y();
//    wx=euler.x();

//  angles    
//    imu::Vector<3> eulAng=bno.getVector(Adafruit_BNO055::VECTOR_EULER);
//    angz=eulAng.x();
//    angy=-eulAng.y();
//    angx=-eulAng.z();

// Calibration status
    bno.getCalibration(&calib_status_sys, &calib_status_gyr, &calib_status_acc, &calib_status_mag);

//  linear accelerations  
    imu::Vector<3> linear_acc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    accx=linear_acc.x();
    accy=linear_acc.y();
    accz=linear_acc.z();

    vx += accx*0.01;
    x+= vx*0.01;
 
Serial.print(accx);Serial.print("  ");
Serial.print(vx);Serial.print("  ");
//Serial.print(accz);Serial.print("  ");
Serial.print(x);Serial.println("  ");

//Serial.print(angz);Serial.print("  ");
//Serial.print(angy);Serial.print("  ");
//Serial.print(angx);Serial.print("  ");
//Serial.print(calib_status_sys);Serial.println("  ");

//Serial.print(wz);Serial.print("  ");
//Serial.print(wy);Serial.print("  ");
//Serial.print(wx);Serial.print("  ");
//Serial.print(calib_status_gyr);Serial.println("  ");

}  



