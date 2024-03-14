/*

*/

#include <Adafruit_BNO055.h>
#include <Wire.h>
#include "Servo.h"
#include <SparkFun_VL53L5CX_Library.h> //http://librarymanager/All#SparkFun_VL53L5CX
#include "Functions.h"

#define ENABLE_ADAPTATIVE_REG 0
#define MAX_ADAPTATIVE_ANGLE 10

// Conversion radian/degrès
#define RAD2DEG 57.2957795
#define DEG2RAD 0.01745329252

//// Servo regulation ////
#define SERVO_PIN 23

Servo my_servo;

const float gain_servo = 6.41; // us/deg
int16_t pwm_servo = 0;
int16_t pwm_angle0_servo, pwm_servo_prev;
int16_t pwm0_servo = 1032;
float alpha = 0.03;

float gimbal_angle0 = 120.0;

float gimbal_angle;

//// Liddar ////
#define LIDDAR_FREQ 50

SparkFun_VL53L5CX myImager;
VL53L5CX_ResultsData measurementData; // Result data class structure, 1356 byes of RAM

const int imageResolution = 16; 
const int imageWidth = 4; 

const float regression_weights[4] = {0.411421 , 0.13165 , 0.13165 , 0.411421};
const float regression_signs[4] = {1.0 , 1.0 , -1.0 , -1.0};
float pitch_angle, yaw_angle;

float mean_range;
float mean_row[4];
float mean_col[4];

//// Camera ////
#define TRIGGER_PIN 9

uint32_t camera_counter, trigger_counter;

//// BNO ////
Adafruit_BNO055 BNO_gimbal = Adafruit_BNO055(-1,BNO055_ADDRESS_A,&Wire1);

float  BNO_roll, BNO_pitch, BNO_yaw;

//// Communnication Carte mère ////
#define SERIAL_PACKAGE_INI 137
#define SERIAL_PACKAGE_END 173
#define SERIAL_PACKAGE_TRIGGER 345
#define SERIAL_PACKAGE_ANGLE 567

RxTxSerial CM_Serial(Serial1,0);

int16_t serialData[2];

void setup()
{
  Serial.begin(115200);
  Serial1.begin(115200);

  pinMode(TRIGGER_PIN,OUTPUT);
  digitalWrite(TRIGGER_PIN,0);

  Wire.begin(); //This resets to 100kHz I2C
  Wire.setClock(400000); //Sensor has max I2C freq of 400kHz 
  
  Serial.println("Initializing liddar sensor board. This can take up to 10s. Please wait.");
  while (myImager.begin() == false)
  {
    Serial.println(F("Sensor not found - check your wiring. Freezing"));
    delay(1000);
  }
  Serial.println("Liddar initialized");

  while ( !BNO_gimbal.begin() )
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("BNO_gimbal  failed to initialize");
    delay(1000);
  }
  Serial.println("BNO_gimbal initialized");
  
  myImager.setResolution(imageResolution); //Enable 16 pads

  bool response = myImager.setRangingFrequency(LIDDAR_FREQ);
  if (response == true)
  {
    int frequency = myImager.getRangingFrequency();
    if (frequency > 0)
    {
      Serial.print("Ranging frequency set to ");
      Serial.print(frequency);
      Serial.println(" Hz.");
    }
    else
      Serial.println(F("Error recovering ranging frequency."));
  }
  else
  {
    Serial.println(F("Cannot set ranging frequency requested. Freezing..."));
    while (1) ;
  }

  myImager.startRanging();

  my_servo.attach(SERVO_PIN);

  camera_counter = millis();
  trigger_counter = millis();
  pwm_angle0_servo = (int16_t)(gain_servo*gimbal_angle0+pwm0_servo);
  pwm_servo = pwm_angle0_servo;
}

void loop()
{

  if(CM_Serial.getData_int16_t(2, SERIAL_PACKAGE_INI, SERIAL_PACKAGE_END, serialData))
  {
    if(serialData[0] == SERIAL_PACKAGE_TRIGGER)
    {
      trigger_counter = millis();
      digitalWrite(TRIGGER_PIN,1);
    } else if(serialData[0] == SERIAL_PACKAGE_ANGLE)
    {
      if(serialData[1]>=0 && serialData[1]<=135)
      {
        gimbal_angle0 = (float)serialData[1];
        CM_Serial.sendData_int16_t(SERIAL_PACKAGE_INI, SERIAL_PACKAGE_END, serialData, 2);
      } else
      {
        serialData[1] = -1;
        CM_Serial.sendData_int16_t(SERIAL_PACKAGE_INI, SERIAL_PACKAGE_END, serialData, 2);
      }
    }
  }
/*
  if((millis()-camera_counter)>10000)
  {
    camera_counter = millis();
    if(gimbal_angle0>45.0)
    {
      gimbal_angle0 = 0.0;
    } else
    {
      gimbal_angle0 = 90.0;
    }
    CM_Serial.sendData_int16_t(SERIAL_PACKAGE_INI, SERIAL_PACKAGE_END, serialData, 2);
  }
*/
  if((millis()-trigger_counter)>500) // Cam trigger on for 500ms
  {
    digitalWrite(TRIGGER_PIN,0);
    serialData[0] = SERIAL_PACKAGE_TRIGGER;
    serialData[1] = 0;
    CM_Serial.sendData_int16_t(SERIAL_PACKAGE_INI, SERIAL_PACKAGE_END, serialData, 2);
  }
  

  //Poll sensor for new data
  if (myImager.isDataReady() == true)
  {
    imu::Vector<3> eulAng = BNO_gimbal.getVector(Adafruit_BNO055::VECTOR_EULER);
    
    //RollOffs = -4*knob+2; // offset roll sur le knob .  zero de la centrale sur roll
    
    BNO_roll = eulAng.z();
    BNO_pitch = eulAng.y();
    BNO_yaw = eulAng.x();

    mean_range = 0.0f;
    for(int i = 0; i<imageWidth ; i++)
    {
      mean_row[i] = 0.0f;
      mean_col[i] = 0.0f;
    }
    if (myImager.getRangingData(&measurementData)) //Read distance data into array
    {
      //The ST library returns the data transposed from zone mapping shown in datasheet
      //Pretty-print data with increasing y, decreasing x to reflect reality
      for (int y = 0 ; y < imageWidth ; y++)
      {
        for (int x = 0 ; x < imageWidth ; x++)
        {
          mean_range += (float)measurementData.distance_mm[x + y*imageWidth] / (float)imageResolution;
          mean_row[y] += (float)measurementData.distance_mm[x + y*imageWidth] / (float)imageWidth;
          mean_col[x] += (float)measurementData.distance_mm[x + y*imageWidth] / (float)imageWidth;
        }
      }
      pitch_angle = 0;
      yaw_angle = 0;
      for(int i = 0; i<imageWidth ; i++)
      {
        pitch_angle += regression_signs[i]*atan2(mean_col[i]-mean_range,mean_range*regression_weights[i])*RAD2DEG/((float)imageWidth);
        yaw_angle += regression_signs[i]*atan2(mean_row[i]-mean_range,mean_range*regression_weights[i])*RAD2DEG/((float)imageWidth);
      }

      gimbal_angle -= alpha*constrain(pitch_angle,-MAX_ADAPTATIVE_ANGLE,MAX_ADAPTATIVE_ANGLE);
      gimbal_angle = constrain(gimbal_angle,-MAX_ADAPTATIVE_ANGLE,MAX_ADAPTATIVE_ANGLE);

      pwm_servo_prev = pwm_servo;
      pwm_angle0_servo = (int16_t)(gain_servo*gimbal_angle0+pwm0_servo);

      if(ENABLE_ADAPTATIVE_REG)
      {
        gimbal_angle -= alpha*constrain(pitch_angle,-MAX_ADAPTATIVE_ANGLE,MAX_ADAPTATIVE_ANGLE);
        gimbal_angle = constrain(gimbal_angle,-MAX_ADAPTATIVE_ANGLE,MAX_ADAPTATIVE_ANGLE);   
      } else
      {
        gimbal_angle = 0;
      }

      pwm_servo = (int16_t)(gain_servo*gimbal_angle)+pwm_angle0_servo;
      // limitation vitesse servo
      pwm_servo = constrain(pwm_servo,pwm_servo_prev-6,pwm_servo_prev+6);
      pwm_servo = constrain(pwm_servo,1000,2000);
      
      my_servo.writeMicroseconds(pwm_servo);

      Serial.print(mean_range/10.0);Serial.print("\t");
      Serial.print(pitch_angle);Serial.print("\t");
      Serial.print(gimbal_angle);Serial.print("\t");
      Serial.print(pwm_servo);Serial.print("\t");
      Serial.print(yaw_angle);Serial.print("\t");
      Serial.print(BNO_roll);Serial.print("\t");
      Serial.print(BNO_pitch);Serial.print("\t");
      Serial.print(BNO_yaw);Serial.print("\t");
      //Serial.print("\t");Serial.print(mean_row[0]);Serial.print("\t");Serial.print(mean_row[1]);Serial.print("\t");Serial.print(mean_row[2]);Serial.print("\t");Serial.println(mean_row[3]);
      //Serial.print("\t");Serial.print(mean_col[0]);Serial.print("\t");Serial.print(mean_col[1]);Serial.print("\t");Serial.print(mean_col[2]);Serial.print("\t");Serial.println(mean_col[3]);
      Serial.println();

    
    }
  }
  

  //delay(5); //Small delay between polling
}
