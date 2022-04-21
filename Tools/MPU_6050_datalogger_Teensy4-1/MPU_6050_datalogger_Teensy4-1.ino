#include <SPI.h>
//#include <SD.h>
#include <MPU6050_tockn.h>
#include <Wire.h>
#include "SdFat1.h"

MPU6050 mpu6050(Wire);

const int chipSelect = 4;

int state, log_state, state2, new_log;
int32_t click_time,timer,log_start_time, timer_led;
int32_t led_period, led_state;


SdFat sd;
char Num[8];
bool  OK_SDCARD = false;
File dataFile;
int long_num = 1;
String dataString;
char *filename;
uint32_t nb_file = 0, counter = 0;

const int button_pin = 38;
const int led_pin = 13;

long t0,t1;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(led_pin,OUTPUT);
  pinMode(button_pin,INPUT);
  state = 0;
  state2 = 0;
  log_state = 0;
  led_period = 0;
  led_state = 0;
  new_log = 0;

  Wire.begin();
  
//  while (!Serial) {
//    ; // wait for serial port to connect. Needed for native USB port only
//  }

  Serial.print("Initializing SD card...");

  if (!sd.begin(SdioConfig(FIFO_SDIO)))
  {
    Serial.println("Card failed, or not present");
    while(1)
    {
      digitalWrite(led_pin,1);
      delay(50);
      digitalWrite(led_pin,0);
      delay(50);
    }
    //return;
  }
  else
  {
    Serial.println("card initialized.");
    filename = (char*) malloc( strlen("logIMU6050_") + strlen(Num) + strlen(".txt") + 1) ;
    checkExist(); //vérifie existence du fichier et l'écriture commence à partir du numéro de fichier que n'existe pas

  }
 // Serial.println("card initialized.");

  mpu6050.begin();
  //mpu6050.calcGyroOffsets(true);

  delay(3000);
  mpu6050.begin();
//  mpu6050.calcGyroOffsets(true);

  led_state = 1;
  digitalWrite(led_pin,led_state);
}

void loop() {
  // put your main code here, to run repeatedly:

//Serial.println(digitalRead(button_pin));
//Serial.println("e");
  if((state == 0) && (digitalRead(button_pin) == 1))
  {
    state = 1;
    click_time = millis();
    Serial.println("1");
  }

  if(digitalRead(button_pin) == 0)
  {
    state = 0;
    state2 = 0;
    //Serial.println("0");
  }

  if( ((state == 1) && ((millis()-click_time) > 500) && (state2 == 0)) || (new_log==1) )
  {
    state2 = 1;
    Serial.println("2");
    if(log_state == 0 || new_log==1)
    {
      log_state = 1;
      new_log = 0;
      led_period = 1000;
      timer_led = millis();
      led_state = 0;
      digitalWrite(led_pin,led_state);
      Serial.print("Opening file ... ");


      strcpy(filename, "logIMU_");
      sprintf(Num, "%lu", nb_file);
      strcat(filename, Num); strcat(filename, ".txt");
      
      dataFile = sd.open(filename, FILE_WRITE);
      
      nb_file++;

      if(dataFile)
      {
        Serial.println("file openend");
        log_start_time = millis();
      }else
      {
        Serial.println("Opening failed");
      }

      Serial.println("a");
      //dataFile.println("Timestamp(us) GyroX(dps) GyroY(dps) GyroZ(dps)");
      timer = micros();
      Serial.println("z");

    } else
    {
        Serial.print("Closing file ... ");
        dataFile.close();
        Serial.println("file closed");
        log_state = 0;
        led_period = 0;
    }
  }


   
  if( (millis()-timer_led>led_period/2) && (led_period!=0) )
  {
    timer_led = millis();
    if(led_state == 1)
    {
      led_state = 0;
    } else
    {
      led_state = 1;
    }
  } else if (led_period==0)
  {
    led_state = 1;
  }
  digitalWrite(led_pin,led_state);

   mpu6050.update();

  if( (micros() - timer > 1000) && (log_state == 1)){

    //Serial.println(micros() - timer);
    
    timer = micros();

//    Serial.print(timer);Serial.print("\t");
    Serial.print(1000*mpu6050.getAccX(),0);Serial.print("\t");
    Serial.print(1000*mpu6050.getAccY(),0);Serial.print("\t");
    Serial.print(1000*mpu6050.getAccZ(),0);Serial.print("\t");
  
    Serial.print(mpu6050.getGyroX());Serial.print("\t");
    Serial.print(mpu6050.getGyroY());Serial.print("\t");
    Serial.println(mpu6050.getGyroZ());

    dataFile.print(timer);dataFile.print(";");
    dataFile.print(1000*mpu6050.getAccX(),0);dataFile.print(";");
    dataFile.print(1000*mpu6050.getAccY(),0);dataFile.print(";");
    dataFile.print(1000*mpu6050.getAccZ(),0);dataFile.print(";");
  
    dataFile.print(mpu6050.getGyroX()*100,0);dataFile.print(";");
    dataFile.print(mpu6050.getGyroY()*100,0);dataFile.print(";");
    dataFile.print(mpu6050.getGyroZ()*100,0);dataFile.print(";");
    
    dataFile.println(" ");

    //Serial.println(micros()-t0);
  }

  if(millis()-log_start_time >120000)
  {
    Serial.print("Closing file ... ");
    dataFile.close();
    Serial.println("file closed");
    new_log = 1;
  }
  
}

void checkExist(void)
{
  do
  {
    strcpy(filename, "logIMU_");
    sprintf(Num, "%lu", counter);
    strcat(filename, Num); strcat(filename, ".txt");
    counter++;
  }
  while ( sd.exists(filename) );
  nb_file = counter - 1;
}
