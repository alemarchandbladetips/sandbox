#include <SPI.h>
#include <SD.h>
#include <Wire.h>

#define PACKAGE_SIZE 50

const int chipSelect = 4;
String filename;
int state, log_state, state2;
long click_time,timer,log_start_time;
File dataFile;
int long_num = 1;

const int button_pin = 14;
const int led_pin = 13;

long t0,t1;

float buffer_float;
unsigned char *ptr_buffer = (unsigned char *)&buffer_float;
uint8_t package_buffer[PACKAGE_SIZE];

int16_t i,j;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial1.begin(115200);
  pinMode(led_pin,OUTPUT);
  pinMode(button_pin,INPUT);
  state = 0;
  state2 = 0;
  log_state = 0;

  Wire.begin();

  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
  
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
  uint8_t x;

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

  if((log_state == 1) && ((millis() - log_start_time) > 300000))
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

  if(Serial1.available()>PACKAGE_SIZE-1)
  {
    x = Serial1.read();
    Serial.print(x);Serial.println(" ");
    if(x == 137)
    {
      Serial1.readBytes(package_buffer,PACKAGE_SIZE-1);
      
      if( package_buffer[PACKAGE_SIZE-2]==173 && (log_state == 1)){

        for(i=0;i<12;i++)
        {
          for(j=0;j<4;j++)
          {
            *(ptr_buffer+j) = package_buffer[4*i+j];
          }
          Serial.print(buffer_float);Serial.print("\t");
          dataFile.print(buffer_float);dataFile.print("\t");
        }
        dataFile.println(" ");
        Serial.println(" ");
      }
    }
    
  }

  
}
