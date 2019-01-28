#include <Arduino.h>
#include "wiring_private.h" // pinPeripheral() function
#include <SPI.h>
#include <SD.h>

#define SERIAL1_DATA_SIZE 33
#define SERIAL1_SPEED 115200
#define START_BYTE 137
#define STOP_BYTE 173

// variables for the log
const int chipSelect = 4;
String filename;
int log_state = 0;
int long_num = 0;
File dataFile;

const int32_t Ts=1000;
uint8_t input;
uint32_t timer, timer_tmp;

long click_time,log_start_time,timestamp;
uint32_t dt;

float Courant=0;
const float gain = 1.0;
const float offset = 0.0;

int state, state2;
long blink_timer;
long blink_duration;
int8_t led_status;

const int button_pin = 14;
const int led_pin = 13;
const int vcc_pin = 15;

uint8_t serial_data[SERIAL1_DATA_SIZE];

int16_t buffer_int16;
unsigned char *ptr_buffer_int16 = (unsigned char *)&buffer_int16;

void setup() {

  Serial.begin(115200);
  Serial1.begin(SERIAL1_SPEED);
  // put your setup code here, to run once:

  delay(2000);
  
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  } else
  {
    Serial.println("SD card ready");
  }

  delay(2000);
  while(Serial1.available())
  {
    Serial1.read();
  }

  pinMode(button_pin,INPUT);
  pinMode(led_pin,INPUT);
  pinMode(vcc_pin,OUTPUT);

  digitalWrite(vcc_pin,HIGH);

  blink_timer = millis();
  digitalWrite(led_pin,0);
  led_status = 0;
  blink_duration = 250;

  state = 0;
}

void loop() {
  uint8_t x,i,j;

  
  if(Serial1.available()>SERIAL1_DATA_SIZE-1)
  {
    timer_tmp = micros();
    dt = timer_tmp - timer;
    timer = timer_tmp;

    x = Serial1.read();
    
    if(x == START_BYTE)
    {
     
    Serial1.readBytes(serial_data,SERIAL1_DATA_SIZE-1);

      if(serial_data[SERIAL1_DATA_SIZE-2] == STOP_BYTE)
      {
        Serial.print(log_state);Serial.print("\t");
        Serial.print(dt);Serial.print("\t");
        Serial.print(serial_data[0]);Serial.print("\t"); // pertes signal
        Serial.print(serial_data[1]);Serial.print("\t"); // tension 4S
        Serial.print(serial_data[2]);Serial.print("\t"); // tension 2S

        for (i = 0; i < 3; i++)
        {
          for (j = 0; j < 2; j++)
          {
            *(ptr_buffer_int16 + j) = serial_data[2 * i + j + 3];
          }
          Serial.print((float)buffer_int16*180.0/16384);Serial.print("\t"); // YPR
        }

        for (i = 0; i < 3; i++)
        {
          for (j = 0; j < 2; j++)
          {
            *(ptr_buffer_int16 + j) = serial_data[2 * i + j + 9];
          }
          Serial.print((float)buffer_int16*2000.0/16384);Serial.print("\t"); // Wxyz
        }

        Serial.print(serial_data[15]);Serial.print("\t"); // tension 4S
        Serial.print(serial_data[16]);Serial.print("\t"); // tension 2S
        Serial.print(serial_data[17]);Serial.print("\t"); // tension 4S
        Serial.print(serial_data[18]);Serial.print("\t"); // tension 2S
        Serial.print(serial_data[19]);Serial.print("\t"); // tension 4S
        Serial.print(serial_data[20]);Serial.print("\t"); // tension 2S

        for (i = 0; i < 5; i++)
        {
          for (j = 0; j < 2; j++)
          {
            *(ptr_buffer_int16 + j) = serial_data[2 * i + j + 21];
          }
          Serial.print(buffer_int16);Serial.print("\t"); // PWMs
        }
        

        
        if(log_state == 1)
        {
          
        }
         
        Serial.println(" ");
      }
    }
  }

  // log closure
  if((log_state == 1) && ((millis()-click_time) > 1000) && state == 1 && state2 == 0)
  {
    dataFile.close();
    log_state = 0;
    Serial.println("file closed");
    blink_duration = 250;
    state2 = 1;
  }

  // log openning
  if((state == 1) && ((millis()-click_time) > 1000) && (log_state == 0) && state2==0 ) // Opening the file on first Sample
  {
    do{
      Serial.print("Opening file ... ");
      filename = "logc";
      filename = filename + long_num;
      filename = filename + ".txt";
      Serial.println(filename);
      long_num++;
    } while(SD.exists(filename));
    dataFile = SD.open(filename, FILE_WRITE);
    
    if(dataFile)
    {
      blink_duration = 50;
      Serial.println("file openend");
      log_state = 1;
      state2 = 1;
      
    }else
    {
      Serial.println("Opening failed");
    }
  }

  // led blink
  if ((millis()-blink_timer > blink_duration) && (blink_duration!=0))
  {
    blink_timer = millis();
    led_status = !led_status;
    digitalWrite(led_pin,led_status);
  }

  
  if((state == 0) && (digitalRead(button_pin) == 1))
  {
    state = 1;
    click_time = millis();
  }

  if(digitalRead(button_pin) == 0)
  {
    state = 0;
    state2 = 0;
  }
}
