#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Arduino.h>
#include "wiring_private.h" // pinPeripheral() function

#define SERIAL1_DATA_SIZE 48
#define SERIAL1_SPEED 115200

#define SERIAL2_DATA_SIZE 25
#define SERIAL2_SPEED 38400

const int print_data = 0;
const int print_timing = 0;

const int chipSelect = 4;
String filename;
int state, log_state, state2;
long click_time, timer, log_start_time;
File dataFile;
int long_num = 1;

const int button_pin = 14;
const int led_pin = 13;

long t0, t1;

float buffer_float;
unsigned char *ptr_buffer = (unsigned char *)&buffer_float;
int16_t buffer_int16;
unsigned char *ptr_buffer_int16 = (unsigned char *)&buffer_int16;


uint8_t serial1_buffer[SERIAL1_DATA_SIZE + 1];
uint8_t serial2_buffer[SERIAL2_DATA_SIZE + 1];

uint32_t timer_tmp;

int16_t i, j;

///////////////// declaration of Serial2 /////////////////////////
Uart Serial2 (&sercom1, 11, 10, SERCOM_RX_PAD_0, UART_TX_PAD_2);

void SERCOM1_Handler()
{
  Serial2.IrqHandler();
}
//////////////////////////////////////////////////////////////////

void setup() {

  // openning serials
  Serial.begin(115200);
  Serial1.begin(SERIAL1_SPEED);
  Serial2.begin(SERIAL2_SPEED);

  // Assign pins 10 & 11 SERCOM functionality (Serial 2)
  pinPeripheral(10, PIO_SERCOM);
  pinPeripheral(11, PIO_SERCOM);

  // assigning pins
  pinMode(led_pin, OUTPUT);
  pinMode(button_pin, INPUT);


  delay(1000);

  // see if the card is present and can be initialized:
  if (print_data) {
    Serial.print("Initializing SD card...");
  }
  if (!SD.begin(chipSelect)) {
    if (print_data) {
      Serial.println("Card failed, or not present");
    }
    // don't do anything more:
    return;
  }
  if (print_data) {
    Serial.println("card initialized.");
  }

  // vars init
  state = 0;
  state2 = 0;
  log_state = 0;
  for (i = 0; i < SERIAL2_DATA_SIZE + 1; i++)
  {
    serial2_buffer[i] = 0;
  }
  for (i = 0; i < SERIAL1_DATA_SIZE + 1; i++)
  {
    serial1_buffer[i] = 0;
  }

  // LED blink at the end of init
  digitalWrite(led_pin, 1);
  delay(250);
  digitalWrite(led_pin, 0);
  delay(250);
  digitalWrite(led_pin, 1);
  delay(250);
  digitalWrite(led_pin, 0);
  delay(250);
  digitalWrite(led_pin, 1);
  delay(250);
  digitalWrite(led_pin, 0);
  delay(250);
  digitalWrite(led_pin, 1);
  delay(250);
  digitalWrite(led_pin, 0);
}

void loop() {
  // put your main code here, to run repeatedly:
  uint8_t x;

  // button = 1 when not pressed and 0 when pressed
  // button handeling: start timer at falling edge
  if ((state == 0) && (digitalRead(button_pin) == 0))
  {
    state = 1;
    click_time = millis();
  }

  // reseting falling edge detection when state = 0
  if (digitalRead(button_pin) == 1)
  {
    state = 0;
    state2 = 0;
  }

  // if pressed for 1s, we start or close the log
  if ((state == 1) && ((millis() - click_time) > 1000) && (state2 == 0))
  {
    state2 = 1;
    if (log_state == 0)
    {
      if (print_data) {
        Serial.print("Opening file ... ");
      }
      filename = "log2s";
      filename = filename + long_num;
      filename = filename + ".txt";
      if (print_data) {
        Serial.println(filename);
      }
      dataFile = SD.open(filename, FILE_WRITE);
      if (dataFile)
      {
        if (print_data) {
          Serial.println("file openend");
        }
        log_state = 1;
        long_num++;
        log_start_time = millis();
      } else
      {
        if (print_data) {
          Serial.println("Opening failed");
        }
      }
    } else
    {
      if (print_data) {
        Serial.print("Closing file ... ");
      }
      dataFile.close();
      if (print_data) {
        Serial.println("file closed");
      }
      log_state = 0;
    }
  }
  digitalWrite(led_pin, log_state);

  // if we log for more than X s, we start a new file.
  if ((log_state == 1) && ((millis() - log_start_time) > 300000))
  {
    dataFile.close();
    if (print_data) {
      Serial.print("Opening file ... ");
    }
    filename = "log2s";
    filename = filename + long_num;
    filename = filename + ".txt";
    if (print_data) {
      Serial.println(filename);
    }
    dataFile = SD.open(filename, FILE_WRITE);
    if (dataFile)
    {
      if (print_data) {
        Serial.println("file openend");
      }
      log_state = 1;
      long_num++;
      log_start_time = millis();
    } else
    {
      if (print_data) {
        Serial.println("Opening failed");
      }
    }
  }

  // reading serial1, data writting is also done here
  if (Serial1.available() > SERIAL1_DATA_SIZE + 1)
  {

    x = Serial1.read();
    //if(print_data){Serial.print("S1=> ");Serial.print(x);Serial.println(" ");}
    if (x == 137)
    {
      timer_tmp = micros();
      Serial1.readBytes(serial1_buffer, SERIAL1_DATA_SIZE + 1);

      if ( serial1_buffer[SERIAL1_DATA_SIZE] == 173 && (log_state == 1))
      {

        for (i = 0; i < 12; i++)
        {
          for (j = 0; j < 4; j++)
          {
            *(ptr_buffer + j) = serial1_buffer[4 * i + j];
          }
          //if(print_data){Serial.print(buffer_float);Serial.print("\t");}
          dataFile.print(buffer_float); dataFile.print("\t");
        }

        // data of serial2
        for (i = 0; i < 6; i++)
        {
          for (j = 0; j < 2; j++)
          {
            *(ptr_buffer_int16 + j) = serial2_buffer[2 * i + j];
          }
          if (print_data) {
            Serial.print(buffer_int16);
            Serial.print("\t");
          }
          dataFile.print(buffer_int16); dataFile.print("\t");
        }
        if (print_data) {
          Serial.print(serial2_buffer[12]);
          Serial.print("\t");
        }
        dataFile.print(serial2_buffer[12]); dataFile.print("\t");
        for (i = 0; i < 5; i++)
        {
          for (j = 0; j < 2; j++)
          {
            *(ptr_buffer_int16 + j) = serial2_buffer[2 * i + j + 13];
          }
          if (print_data) {
            Serial.print(buffer_int16);
            Serial.print("\t");
          }
          dataFile.print(buffer_int16); dataFile.print("\t");
        }
        if (print_data) {
          Serial.print(serial2_buffer[23]);
          Serial.print("\t");
        }
        dataFile.print(serial2_buffer[23]); dataFile.print("\t");
        if (print_data) {
          Serial.print(serial2_buffer[24]);
          Serial.print("\t");
        }
        dataFile.print(serial2_buffer[24]); dataFile.print("\t");

        dataFile.println(" ");
        if (print_data) {
          Serial.println(" ");
        }
      } //else
      //      {
      //        Serial.print("wrong stop");Serial.print(" ");
      //        for(i=0;i<SERIAL1_DATA_SIZE+1;i++)
      //        {
      //          Serial.print(serial1_buffer[i]);Serial.print(" ");
      //        }
      //
      //        Serial.println(" ");
      //      }
      if (print_timing) {
        Serial.print("S1: ");
        Serial.println(micros() - timer_tmp);
      }
    } // else
    //    {
    //      Serial.print("wrong start");Serial.println(" ");
    //    }
  }

  // reading serial2, data writting is also done here
  if (Serial2.available() > SERIAL2_DATA_SIZE + 1)
  {
    x = Serial2.read();
    //if(print_data){Serial.print("S2=> ");Serial.print(x);Serial.println(" ");}
    if (x == 137)
    {
      timer_tmp = micros();
      Serial2.readBytes(serial2_buffer, SERIAL2_DATA_SIZE + 1);

      if ( serial2_buffer[SERIAL2_DATA_SIZE] != 173 || (log_state != 1))
      {
        for (i = 0; i < SERIAL2_DATA_SIZE + 1; i++)
        {
          serial2_buffer[i] = 0;
        }
      }
      if (print_timing) {
        Serial.print("S2: ");
        Serial.println(micros() - timer_tmp);
      }
    }
  }


}
