#include <SPI.h>
#include <SD.h>
#include <MPU6050_tockn.h>
#include <Wire.h>

MPU6050 mpu6050(Wire);

const int chipSelect = 4;
String filename;
int state, log_state, state2;
long click_time,timer;
File dataFile;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(14,OUTPUT);
  pinMode(18,INPUT);
  state = 0;
  state2 = 0;
  log_state = 0;

  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
  
}

void loop() {
  // put your main code here, to run repeatedly:

  if((state == 0) && (digitalRead(18) == 0))
  {
    state = 1;
    click_time = millis();
  }

  if(digitalRead(18) == 1)
  {
    state = 0;
    state2 = 0;
  }

  if((state == 1) && ((millis()-click_time) > 1000) && (state2 == 0))
  {
    state2 = 1;
    if(log_state == 0)
    {
      Serial.print("Opening file ... ");
      filename = "log";
      filename = filename + click_time;
      filename = filename + ".txt";
      Serial.println(filename);
      dataFile = SD.open(filename, FILE_WRITE);
      if(dataFile)
      {
        Serial.println("file openend");
        log_state = 1;
      }else
      {
        Serial.println("Opening failed");
      }
      
      dataFile.print("COUCOU ");
      timer = micros();

    } else
    {
      Serial.print("Closing file ... ");
      dataFile.close();
      Serial.println("file closed");
      log_state = 0;
    }
  }
  digitalWrite(14,log_state);

   mpu6050.update();

  if( (micros() - timer > 5000) && (log_state == 1)){
    
    timer = micros();
    
    Serial.print(mpu6050.getAccX());Serial.print("\t");
    Serial.print(mpu6050.getAccY());Serial.print("\t");
    Serial.print(mpu6050.getAccZ());Serial.print("\t");
  
    Serial.print(mpu6050.getGyroX());Serial.print("\t");
    Serial.print(mpu6050.getGyroY());Serial.print("\t");
    Serial.println(mpu6050.getGyroZ());

    dataFile.print(mpu6050.getAccX());dataFile.print("\t");
    dataFile.print(mpu6050.getAccY());dataFile.print("\t");
    dataFile.print(mpu6050.getAccZ());dataFile.print("\t");
  
    dataFile.print(mpu6050.getGyroX());dataFile.print("\t");
    dataFile.print(mpu6050.getGyroY());dataFile.print("\t");
    dataFile.println(mpu6050.getGyroZ());
  }

  
}
