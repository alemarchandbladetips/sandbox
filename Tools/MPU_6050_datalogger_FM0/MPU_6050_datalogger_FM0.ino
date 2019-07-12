#include <SPI.h>
#include <SD.h>
#include <MPU6050_tockn.h>
#include <Wire.h>

MPU6050 mpu6050(Wire);

const int chipSelect = 4;
String filename;
int state, log_state, state2;
long click_time,timer,log_start_time;
File dataFile;
int long_num = 1;

const int button_pin = 14;
const int led_pin = 15;

long t0,t1;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(led_pin,OUTPUT);
  pinMode(button_pin,INPUT);
  state = 0;
  state2 = 0;
  log_state = 0;

  Wire.begin();
  
//  while (!Serial) {
//    ; // wait for serial port to connect. Needed for native USB port only
//  }

  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
//  if (!SD.begin(chipSelect)) {
//    Serial.println("Card failed, or not present");
//    // don't do anything more:
//    return;
//  }
//  Serial.println("card initialized.");

  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  delay(5000);
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  digitalWrite(led_pin,1);
  delay(250);
  digitalWrite(led_pin,0);
  delay(250);
  digitalWrite(led_pin,1);
  delay(250);
  digitalWrite(led_pin,0);
  delay(250);
  digitalWrite(led_pin,1);
  delay(250);
  digitalWrite(led_pin,0);
  delay(250);
  digitalWrite(led_pin,1);
  delay(250);
  digitalWrite(led_pin,0);
}

void loop() {
  // put your main code here, to run repeatedly:

  if((state == 0) && (digitalRead(button_pin) == 0))
  {
    state = 1;
    click_time = millis();
  }

  if(digitalRead(button_pin) == 1)
  {
    state = 0;
    state2 = 0;
  }

  if((state == 1) && ((millis()-click_time) > 1000) && (state2 == 0))
  {
    state2 = 1;
    if(log_state == 0)
    {
      log_state = 1;
//      Serial.print("Opening file ... ");
//      filename = "logmpu";
//      filename = filename + long_num;
//      filename = filename + ".txt";
//      Serial.println(filename);
//      dataFile = SD.open(filename, FILE_WRITE);
//      if(dataFile)
//      {
//        Serial.println("file openend");
//        log_state = 1;
//        long_num++;
//        log_start_time = millis();
//      }else
//      {
//        Serial.println("Opening failed");
//      }
//      
//      dataFile.println("Timestamp(us) GyroX(dps) GyroY(dps) GyroZ(dps)");
//      timer = micros();
//
//    } else
//    {
//      Serial.print("Closing file ... ");
//      dataFile.close();
//      Serial.println("file closed");
//      log_state = 0;
      }
  }
  digitalWrite(led_pin,log_state);

//  if((log_state == 1) && ((millis() - log_start_time) > 120000))
//  {
//    dataFile.close();
//    Serial.print("Opening file ... ");
//    filename = "logmpu";
//    filename = filename + long_num;
//    filename = filename + ".txt";
//    Serial.println(filename);
//    dataFile = SD.open(filename, FILE_WRITE);
//    if(dataFile)
//    {
//      Serial.println("file openend");
//      log_state = 1;
//      long_num++;
//      log_start_time = millis();
//    }else
//    {
//      Serial.println("Opening failed");
//    }
//    
//    dataFile.println("Timestamp(us) GyroX(dps) GyroY(dps) GyroZ(dps)");
//    timer = micros();
//
//  }

   mpu6050.update();

  if( (micros() - timer > 10000) && (log_state == 1)){
    
    timer = micros();

    //Serial.print(timer);Serial.print("\t");
    Serial.print(1000*mpu6050.getAccX());Serial.print("\t");
    Serial.print(1000*mpu6050.getAccY());Serial.print("\t");
    Serial.print(1000*mpu6050.getAccZ());Serial.print("\t");
  
    Serial.print(mpu6050.getGyroX());Serial.print("\t");
    Serial.print(mpu6050.getGyroY());Serial.print("\t");
    Serial.println(mpu6050.getGyroZ());

//    dataFile.print(timer);dataFile.print("\t");
//    dataFile.print(1000*mpu6050.getAccX());dataFile.print(" ");
//    dataFile.print(1000*mpu6050.getAccY());dataFile.print(" ");
//    dataFile.print(1000*mpu6050.getAccZ());dataFile.print(" ");
//  
//    dataFile.print(mpu6050.getGyroX());dataFile.print(" ");
//    dataFile.print(mpu6050.getGyroY());dataFile.print(" ");
//    dataFile.println(mpu6050.getGyroZ());

    //Serial.println(micros()-t0);

    
  }

  
}
