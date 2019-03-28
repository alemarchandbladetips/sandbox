#include <Arduino.h>
#include "wiring_private.h" // pinPeripheral() function
#include <SPI.h>
#include <SD.h>

#define SERIAL1_DATA_SIZE 41
#define SERIAL1_SPEED 115200
#define START_BYTE 137
#define STOP_BYTE 173

// variables for the log
const int chipSelect = 4;
String filename;
int log_state = 0;
int long_num = 0;
File dataFile;

float courant,tension,puissance, puissance_filt, tension_filt, courant_filt = 0.0;

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

  
   if(Serial1.available()>3)
  {
    timer_tmp = micros();
    dt = timer_tmp - timer;
    timer = timer_tmp;

    x = Serial1.read();
    
    if(x == 0xAA)
    {
     
    Serial1.readBytes(serial_data,3);

      if(serial_data[2] == 0x55)
      {
        

        courant = ((serial_data[0]-(76.0))*8.0/46.0);
        tension = (0.96*(5.0*serial_data[1]/256.0)*(24.1/3.03));
        puissance = courant*tension;
        puissance_filt = 0.95*puissance_filt + 0.05*puissance;
        courant_filt = 0.98*courant_filt + 0.02*courant;
        tension_filt = 0.95*tension_filt + 0.05*tension;

        Serial.print(dt/1000000.0); Serial.print(" ");
        Serial.print(tension); Serial.print(" ");
        Serial.print(courant); Serial.print(" ");
        //Serial.print(puissance); Serial.print(" ");
        Serial.print(puissance_filt/10);
        Serial.print("20"); Serial.print(" ");
        Serial.print("24"); Serial.print(" ");
        Serial.print("22"); Serial.print(" ");
        Serial.print("26"); Serial.print(" ");
        Serial.print("30"); Serial.print(" ");
        Serial.print("40"); Serial.print(" ");
        Serial.print("50"); Serial.print(" ");
        Serial.println(" ");
        
        if(log_state == 1)
        {
     
          dataFile.print(dt); dataFile.print(" "); 
          
          dataFile.print(tension); dataFile.print(" "); 
          dataFile.print(puissance_filt); dataFile.print(" "); 
          dataFile.println(courant);
          
        }  
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
