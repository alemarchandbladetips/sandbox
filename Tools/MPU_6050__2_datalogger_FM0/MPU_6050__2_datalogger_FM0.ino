#include <SPI.h>
#include <SD.h>
#include <MPU6050_tockn.h>
#include <Wire.h>
#include "wiring_private.h" // pinPeripheral() function

// WARNING !!!! 2-10K pullup resistors are required on SDA and SCL, both go to 3.3V! You can use your oscilloscope to see the data traces

///////////////// declaration of I2C on pin 11(SDA) and 13(SCL) /////////////////////////
TwoWire myWire(&sercom1, 11, 13);

#define MCP4725_CMD_WRITEDAC            (0x40)
#define MCP4725_ADDR                    (0x62)

//////////////////////////////////////////////////////////////////

MPU6050 mpu6050(Wire);
MPU6050 mpu6050_sat(myWire);

const int chipSelect = 4;
String filename;
int state, log_state, state2;
long click_time,timer,log_start_time,timestamp;
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
  myWire.begin();

   // Assign pins 13 & 11 to SERCOM functionality
  pinPeripheral(11, PIO_SERCOM);
  pinPeripheral(13, PIO_SERCOM);

  
//  while (!Serial) {
//    ; // wait for serial port to connect. Needed for native USB port only
//  }

  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");

  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
Serial.println("1");
  mpu6050_sat.begin();
  mpu6050_sat.calcGyroOffsets(true);
Serial.println("2");
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
Serial.println("3");
  mpu6050_sat.begin();
  mpu6050_sat.calcGyroOffsets(true);
Serial.println("4");
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
      Serial.print("Opening file ... ");
      filename = "logmpu";
      filename = filename + long_num;
      filename = filename + ".txt";
      Serial.println(filename);
      dataFile = SD.open(filename, FILE_WRITE);
      if(dataFile)
      {
        Serial.println("file openend");
        log_state = 1;
        long_num++;
        log_start_time = millis();
      }else
      {
        Serial.println("Opening failed");
      }
      
      dataFile.println("Timestamp(us) GyroX(dps) GyroY(dps) GyroZ(dps)");
      timer = micros();

    } else
    {
      Serial.print("Closing file ... ");
      dataFile.close();
      Serial.println("file closed");
      log_state = 0;
    }
  }
  digitalWrite(led_pin,log_state);

  if((log_state == 1) && ((millis() - log_start_time) > 120000))
  {
    dataFile.close();
    Serial.print("Opening file ... ");
    filename = "logmpu";
    filename = filename + long_num;
    filename = filename + ".txt";
    Serial.println(filename);
    dataFile = SD.open(filename, FILE_WRITE);
    if(dataFile)
    {
      Serial.println("file openend");
      log_state = 1;
      long_num++;
      log_start_time = millis();
    }else
    {
      Serial.println("Opening failed");
    }
    
    dataFile.println("Timestamp(us) GyroX(dps) GyroY(dps) GyroZ(dps)");
    timer = micros();

  }

   mpu6050.update();
   mpu6050_sat.update();

  if( (micros() - timer > 10000) && (log_state == 1)){
    
    timer = micros();
    timestamp = millis();

//    Serial.print(timer);Serial.print("\t");
//    Serial.print(mpu6050_sat.getGyroX());Serial.print("\t");
//    Serial.print(mpu6050_sat.getGyroY());Serial.print("\t");
//    Serial.print(mpu6050_sat.getGyroZ());Serial.print("\t");
//  
//    Serial.print(mpu6050.getGyroX());Serial.print("\t");
//    Serial.print(mpu6050.getGyroY());Serial.print("\t");
//    Serial.println(mpu6050.getGyroZ());

    dataFile.print(timestamp);dataFile.print("\t");
    dataFile.print(mpu6050_sat.getGyroX());dataFile.print("\t");
    dataFile.print(mpu6050_sat.getGyroY());dataFile.print("\t");
    dataFile.print(mpu6050_sat.getGyroZ());dataFile.print("\t");
  
    dataFile.print(mpu6050.getGyroX());dataFile.print("\t");
    dataFile.print(mpu6050.getGyroY());dataFile.print("\t");
    dataFile.println(mpu6050.getGyroZ());

    //Serial.println(micros()-t0);

    
  }

  
}
