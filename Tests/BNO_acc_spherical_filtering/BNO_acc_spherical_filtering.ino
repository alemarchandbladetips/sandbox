//A2 prend les info du BNO et les transmet via serial ver le Main

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>
#include <Wire.h>

///VARIABLES BNO
float wz, wy, wx, accx, accy, accz;

float acc_world_filt[3], integrated_quat[4], vertical_world[3];

long sampling_period_ms, last_time;

int first_update;

// appel IMU
Adafruit_BNO055 bno = Adafruit_BNO055();
//Timer mon_timer;

void setup() {
  

  Serial.begin(115200);
  //Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);
    /* Display the current temperature */
  //int8_t temp = bno.getTemp();
  //Serial.print("Current Temperature: ");
  //Serial.print(temp);
  //Serial.println(" C");
  //Serial.println("");

  bno.setExtCrystalUse(true);

  //Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");

  sampling_period_ms = 10;
  integrated_quat[0] = 1;
  integrated_quat[1] = 0;
  integrated_quat[2] = 0;
  integrated_quat[3] = 0;
  acc_world_filt[0] = 0;
  acc_world_filt[1] = 0;
  acc_world_filt[2] = 1;
  vertical_world[0] = 0;
  vertical_world[1] = 0;
  vertical_world[2] = 1;
}

void quat_mult(float q1[4], float q2[4], float qout[4]){
  qout[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
  qout[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
  qout[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1];
  qout[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0];
}

void vect_mult(float v1[3], float v2[3], float vout[3]){
  vout[0] = v1[1]*v2[2] - v1[2]*v2[1];
  vout[1] = v1[2]*v2[0] - v1[0]*v2[2];
  vout[2] = v1[0]*v2[1] - v1[1]*v2[0];
}

void quat_conj(float qin[4], float qout[4]) {
  qout[0] = qin[0];
  qout[1] = -qin[1];
  qout[2] = -qin[2];
  qout[3] = -qin[3];
}

void quat_to_euler(float qin[4], float ypr[3]) {
  ypr[0] = atan2((2*qin[0]*qin[3] + 2*qin[1]*qin[2]),(1 - 2*qin[2]*qin[2] - 2*qin[3]*qin[3]))*57.295779513082;
  ypr[1] = asin((2*qin[0]*qin[2] - 2*qin[1]*qin[3]))*57.295779513082;
  ypr[2] = atan2((2*qin[0]*qin[1] + 2*qin[2]*qin[3]),(1 - 2*qin[2]*qin[2] - 2*qin[1]*qin[1]))*57.295779513082;
}

void quat_vect_rot(float qin[4], float vin[3], float vout[3]) {
  vout[0] = (1 - 2*qin[2]*qin[2] - 2*qin[3]*qin[3])*vin[0] + (2*qin[1]*qin[2] - 2*qin[3]*qin[0])*vin[1]     + (2*qin[1]*qin[3] + 2*qin[2]*qin[0])*vin[2];
  vout[1] = (2*qin[1]*qin[2] + 2*qin[3]*qin[0])*vin[0]     + (1 - 2*qin[1]*qin[1] - 2*qin[3]*qin[3])*vin[1] + (2*qin[2]*qin[3] - 2*qin[1]*qin[0])*vin[2];
  vout[2] = (2*qin[1]*qin[3] - 2*qin[2]*qin[0])*vin[0]     + (2*qin[2]*qin[3] + 2*qin[1]*qin[0])*vin[1]     + (1 - 2*qin[1]*qin[1] - 2*qin[2]*qin[2])*vin[2];
}

void loop() {

  int i;
  float acc_world[3], quaternion[4], quaternion_conj[4], acc_tmp[3], acc_world_filt_norm, acc_filt[3], scaled_gyr[4], quat_increment[4], quat_norm, vertical[3], vertical_diff[3], ypr[3];
  uint8_t calib_status_sys, calib_status_acc, calib_status_gyr, calib_status_mag;
  
  while((millis() - last_time) < sampling_period_ms){
    delayMicroseconds(1);
  }
  last_time += sampling_period_ms;

// quaternion
    //imu::Quaternion quat = bno.getQuat();
    //quaternion[0] = quat.w();
    //quaternion[1] = quat.x();
    //quaternion[2] = quat.y();
    //quaternion[3] = quat.z();
    
// Calibration status
    bno.getCalibration(&calib_status_sys, &calib_status_gyr, &calib_status_acc, &calib_status_mag);
    
//  linear accelerations  
    imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    acc_tmp[0] = acc.x()/9.81;
    acc_tmp[1] = acc.y()/9.81;
    acc_tmp[2] = acc.z()/9.81;
    
//  rotations  
    imu::Vector<3> gyr = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    scaled_gyr[0] = 0;
    scaled_gyr[1] = 0.5*gyr.x()*sampling_period_ms/1000;
    scaled_gyr[2] = 0.5*gyr.y()*sampling_period_ms/1000;
    scaled_gyr[3] = 0.5*gyr.z()*sampling_period_ms/1000;

//// quaternion integration
    quat_norm = 0;
    quat_mult(integrated_quat,scaled_gyr,quat_increment);
    for(i=0;i<4;i++) {
      integrated_quat[i] += quat_increment[i];
      quat_norm += integrated_quat[i]*integrated_quat[i];
    }
    // normalization after integration
    quat_norm = sqrtf(quat_norm);
    for(i=0;i<4;i++) {
      integrated_quat[i] = integrated_quat[i]/quat_norm;
    }

//// Acc filtering
// projection of acc in world frame
    quat_vect_rot(integrated_quat,acc_tmp,acc_world);

// filtering acc
    acc_world_filt_norm = 0;
    for(i=0;i<3;i++) {
      acc_world_filt[i] = 0.01 * acc_world[i] + 0.99 * acc_world_filt[i];
      acc_world_filt_norm += acc_world_filt[i]*acc_world_filt[i];
    }

// normalization
    acc_world_filt_norm = sqrtf(acc_world_filt_norm);
    for(i=0;i<3;i++) {
      acc_world_filt[i] = acc_world_filt[i]/acc_world_filt_norm;
    }

// reprojection in sensor frame
    quat_conj(integrated_quat,quaternion_conj);
    quat_vect_rot(quaternion_conj,acc_world_filt,acc_filt);

//// Corection of quaternion
    quat_vect_rot(quaternion_conj,vertical_world,vertical);
    vect_mult(acc_filt,vertical,vertical_diff);

    scaled_gyr[0] = 0;
    scaled_gyr[1] = 0.5*vertical_diff[0]*0.01;
    scaled_gyr[2] = 0.5*vertical_diff[1]*0.01;
    scaled_gyr[3] = 0.5*vertical_diff[2]*0.01;

    quat_norm = 0;
    quat_mult(integrated_quat,scaled_gyr,quat_increment);
    for(i=0;i<4;i++) {
      integrated_quat[i] += quat_increment[i];
      quat_norm += integrated_quat[i]*integrated_quat[i];
    }
    // normalization after integration
    quat_norm = sqrtf(quat_norm);
    for(i=0;i<4;i++) {
      integrated_quat[i] = integrated_quat[i]/quat_norm;
    }
    quat_conj(integrated_quat,quaternion_conj);
    quat_to_euler(integrated_quat,ypr);

//Serial.print(acc_tmp[0]);Serial.print("  ");
//Serial.print(acc_tmp[1]);Serial.print("  ");
//Serial.print(acc_tmp[2]);Serial.print("  ");
//Serial.print(acc_filt[0]);Serial.print("  ");
//Serial.print(acc_filt[1]);Serial.print("  ");
//Serial.print(acc_filt[2]);Serial.print("  ");
//Serial.print(acc_world_filt[0]-3);Serial.print("  ");
//Serial.print(acc_world_filt[1]-3);Serial.print("  ");
//Serial.print(acc_world_filt[2]-3);Serial.print("  ");
//Serial.print(vertical_diff[0]-6);Serial.print("  ");
//Serial.print(vertical_diff[1]-6);Serial.print("  ");
//Serial.print(vertical_diff[2]-6);Serial.println("  ");
Serial.print(ypr[0]);Serial.print("  ");
Serial.print(ypr[1]);Serial.print("  ");
Serial.print(ypr[2]);Serial.println("  ");
//Serial.print(acc[0]);Serial.println("  ");
//Serial.print(y);Serial.print("  ");
//Serial.print(z);Serial.println("  ");

}  



