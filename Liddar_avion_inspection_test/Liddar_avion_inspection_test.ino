/*
  Read an 8x8 array of distances from the VL53L5CX
  By: Nathan Seidle
  SparkFun Electronics
  Date: October 26, 2021
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to read all 64 distance readings at once.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/18642

*/

#include <Wire.h>
#include "Servo.h"
#include <SparkFun_VL53L5CX_Library.h> //http://librarymanager/All#SparkFun_VL53L5CX

// Conversion radian/degrès
#define RAD2DEG 57.2957795
#define DEG2RAD 0.01745329252

#define SERVO_PIN 23

Servo my_servo;

float gimbal_angle;
float gain_servo = 28.6;
int16_t pwm_servo = 1500;
float alpha = 0.01;

SparkFun_VL53L5CX myImager;
VL53L5CX_ResultsData measurementData; // Result data class structure, 1356 byes of RAM

int imageResolution = 16; //Used to pretty print output
int imageWidth = 4; //Used to pretty print output

const float regression_weights[4] = {0.411421 , 0.13165 , 0.13165 , 0.411421};
const float regression_signs[4] = {1.0 , 1.0 , -1.0 , -1.0};
float pitch_angle, yaw_angle;

float mean_range;
float mean_row[4];
float mean_col[4];

void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("SparkFun VL53L5CX Imager Example");

  Wire.begin(); //This resets to 100kHz I2C
  Wire.setClock(400000); //Sensor has max I2C freq of 400kHz 
  
  Serial.println("Initializing sensor board. This can take up to 10s. Please wait.");
  if (myImager.begin() == false)
  {
    Serial.println(F("Sensor not found - check your wiring. Freezing"));
    while (1) ;
  }
  
  myImager.setResolution(imageResolution); //Enable 16 pads

    bool response = myImager.setRangingFrequency(50);
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
}

void loop()
{
  

  //Poll sensor for new data
  if (myImager.isDataReady() == true)
  {
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


      gimbal_angle += alpha*pitch_angle;
      gimbal_angle = constrain(gimbal_angle,-35,35);
      pwm_servo = (int16_t)(gain_servo*gimbal_angle)+1500;
      pwm_servo = constrain(pwm_servo,1000,2000);

      my_servo.writeMicroseconds(pwm_servo);


      Serial.print(mean_range/10.0);Serial.print("\t");
      Serial.print(pitch_angle);Serial.print("\t");
      //Serial.print(gimbal_angle);Serial.print("\t");
      //Serial.print(pwm_servo);Serial.print("\t");
      Serial.print(yaw_angle);Serial.print("\t");
      //Serial.print("\t");Serial.print(mean_row[0]);Serial.print("\t");Serial.print(mean_row[1]);Serial.print("\t");Serial.print(mean_row[2]);Serial.print("\t");Serial.println(mean_row[3]);
      //Serial.print("\t");Serial.print(mean_col[0]);Serial.print("\t");Serial.print(mean_col[1]);Serial.print("\t");Serial.print(mean_col[2]);Serial.print("\t");Serial.println(mean_col[3]);
      Serial.println();

    
    }
  }
  

  //delay(5); //Small delay between polling
}
