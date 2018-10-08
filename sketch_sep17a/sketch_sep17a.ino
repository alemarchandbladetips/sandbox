#include <Arduino.h>
#include "wiring_private.h" // pinPeripheral() function
#include <SPI.h>
#include <SD.h>

#define BUFFER_SIZE 25000

// variables for the log
const int chipSelect = 4;
String filename;
int log_state = 0;
int long_num = 0;
File dataFile;

const int led_pin = 19;
const int32_t Ts=1000;
uint8_t input;
uint32_t timer, timer_tmp;
uint32_t dt;

uint8_t current_log[BUFFER_SIZE];
//float current_dt[10000];
uint16_t idx;

float Courant=0;
const float gain = 1.0;
const float offset = 0.0;

void setup() {
  // put your setup code here, to run once:

  
  Serial.begin(115200);
  Serial1.begin(115200);

  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  } else
  {
    Serial.println("SD card ready");
  }
  
  while(Serial1.available())
  {
    Serial1.read();
  }

}

void loop() {
  uint16_t i;
  // put your main code here, to run repeatedly:
  if(Serial1.available())
  {
    timer_tmp = micros();
    dt = timer_tmp - timer;
    timer = timer_tmp;
    input = Serial1.read();
    //Serial.println(dt);
    if(log_state == 0) // Opening the file on first Sample
    {
      //Serial.print("Opening file ... ");
      filename = "logc";
      filename = filename + long_num;
      filename = filename + ".txt";
      //Serial.println(filename);
      dataFile = SD.open(filename, FILE_WRITE);
      
      if(dataFile)
      {
        //Serial.println("file openend");
        log_state = 1;
        long_num++;
        idx = 0;
      }else
      {
        //Serial.println("Opening failed");
      }
    } 
    if(log_state == 1)
    {
      //current_dt[idx] = dt;
      current_log[idx] = input;
      idx++;
      //Serial.println(micros()-timer);
    }
  }

  if((log_state == 1) && (idx>BUFFER_SIZE-2))
  {
    //Serial.println("printing file");
    for (i=0;i<idx;i++)
    {
      //Serial.println(i);
      dataFile.println(current_log[i]);
    }
    dataFile.close();
    //Serial.print("Opening file ... ");
    filename = "logc";
    filename = filename + long_num;
    filename = filename + ".txt";
    //Serial.println(filename);
    dataFile = SD.open(filename, FILE_WRITE);
    if(dataFile)
    {
      //Serial.println("file openend");
      log_state = 1;
      long_num++;
      idx = 0;
      timer = micros();
    }else
    {
      //Serial.println("Opening failed");
    }
  }
  
  if((log_state == 1) && ((micros() - timer) > 500000))
  {
    //Serial.println("printing file");
    for (i=0;i<idx;i++)
    {
      //Serial.println(i);
      dataFile.println(current_log[i]);
    }
    
    dataFile.close();
    log_state = 0;
    //Serial.println("file closed");
  }
}
