#include <Arduino.h>
#include "wiring_private.h" // pinPeripheral() function
#include <SPI.h>
#include <SD.h>

#define GIMBAL_PACKET_SIZE 33 // number of bytes to be recieved from 
#define PACKET_START 0xAA // starting char of package
#define PACKET_STOP 0x55 // starting char of package

#define MEGA_PACKET_SIZE 18 // number of bytes to be recieved from 
#define PACKET_START_MEGA 137 // starting char of package
#define PACKET_STOP_MEGA 173 // starting char of package

// constant used to choose printed data
const int8_t print_rpy = 0;
const int8_t print_rpy_p = 0;
const int8_t print_NED = 0;
const int8_t print_NED_speed = 0;
const int8_t print_flag = 0;

const float pi = 3.14159265359;

// variables for the serial read an data recomposition
float all_data[8];
float rpy[3];
float rpy_p[3];
int32_t NED_coordinates[3];
float NED_coordinates_flt[3];
int16_t NED_speed[3];
uint8_t sanity_flag;

uint8_t raw_data[100];
float buffer_float;
unsigned char *ptr_buffer = (unsigned char *)&buffer_float;
int16_t buffer_int16;
unsigned char *ptr_buffer_int16 = (unsigned char *)&buffer_int16;
int32_t buffer_int32;
unsigned char *ptr_buffer_int32 = (unsigned char *)&buffer_int32;

const float rad_to_deg = 57.2957795; // 180/pi
long t0;

// variables for the log
const int chipSelect = 4;
String filename;
int log_state = 0;
long click_time,timer,log_start_time;
File dataFile;
const int led_pin = 19;
int long_num = 1;

/*** PWMs les moteurs ***/
int U_MP1, U_MP2, U_MP3, U_ME, U_A1=700, U_A2=700, U_A3=700;
 

/*****Angle pour LEDs ****/
uint8_t Angle_Leds = 0, Kintens;

///////////////// declaration of Serial2 /////////////////////////
Uart Serial2 (&sercom1, 11, 10, SERCOM_RX_PAD_0, UART_TX_PAD_2);

void SERCOM1_Handler()
{
  Serial2.IrqHandler();
}
//////////////////////////////////////////////////////////////////
 
void setup() {

    
  
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial2.begin(115200);

// Assign pins 10 & 11 SERCOM functionality (Serial 2)
  pinPeripheral(10, PIO_SERCOM);
  pinPeripheral(11, PIO_SERCOM);
  
  Serial.println("Start");
  t0 = micros();
  pinMode(led_pin,OUTPUT);

  Serial.print("Initializing SD card...");

  delay(2000);
    
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
          
}

//////////////////////////////////////////////////////////////////////
 
void loop() {
int i,j,x;

  // Reading the Serial from IMU
  if (Serial2.available() > GIMBAL_PACKET_SIZE-1) // Number of data corresponding to the IMU packet size is waiting in the biffer of serial
  { 
    x = Serial2.read(); // read first data
    //Serial.print(x); Serial.print(" ");
    if(x == PACKET_START) // check that first data correspond to start char
    {
      
      t0 = micros();
      Serial2.readBytes(raw_data,GIMBAL_PACKET_SIZE-1); // Reading the IMU packet

      if(raw_data[GIMBAL_PACKET_SIZE-2] == PACKET_STOP) // check taht the last data correspond to the packet end char
      {

        if(log_state == 0) // Opening the file on first Sample
        {
          Serial.print("Opening file ... ");
          filename = "logc";
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
        }

        if(log_state == 1)
        {
          dataFile.print(micros()); dataFile.write(9); dataFile.print(1); dataFile.write(9);
          for(i=0;i<6;i++) // decode YPR data
          {
            for(j=0;j<2;j++) // filling the 4 bytes of the float with the data of serial port
            {
              ptr_buffer_int16[j] = raw_data[2*i+j];
            }
            if(i<3)
            { // roll, pitch, tips0 angle
              rpy[i] = (float)buffer_int16*180.0/32768.0;
              if(print_rpy){ Serial.print(rpy[i]); Serial.write(9); }
              dataFile.print(rpy[i]); dataFile.write(9);
            } else
            { // derivatives of roll, pitch, tips0 angle
              rpy_p[i-3] = (float)buffer_int16*2000.0/32768.0;
              //if(i<5)
              {
              if(print_rpy_p){ Serial.print(rpy_p[i-3]); Serial.write(9);}
              dataFile.print(rpy_p[i-3]); dataFile.write(9);
              }
            }
          }
           /// GPS DATA
          for (i = 0; i < 3; i++)
          {
            for (j = 0; j < 4; j++)
            {
              ptr_buffer_int32[j] = raw_data[4*i+j+12];
            }
            NED_coordinates[i] = buffer_int32;
            NED_coordinates_flt[i] = (float)NED_coordinates[i]/100.0;
            if(print_NED){ Serial.print(NED_coordinates_flt[i]); Serial.write(9);}
          }
          for (i = 0; i < 3; i++)
          {
            for (j = 0; j < 2; j++)
            {
              ptr_buffer_int16[j] = raw_data[2*i+j+24];
            }
            NED_speed[i] = buffer_int16;
            if(print_NED_speed){ Serial.print(NED_speed[i]); Serial.write(9);}
          }
  
          sanity_flag = raw_data[30];
          dataFile.println(raw_data[30]);
          if(print_flag){ Serial.print(sanity_flag); Serial.write(9);}
          Serial.println(" ");
        }
      }
    }
  }

  if (Serial1.available() > MEGA_PACKET_SIZE-1) // Number of data corresponding to the IMU packet size is waiting in the biffer of serial
  { 
    x = Serial1.read(); // read first data
    //Serial.print(x); Serial.print(" ");
    if(x == PACKET_START_MEGA) // check that first data correspond to start char
    {
      
      t0 = micros();
      Serial1.readBytes(raw_data,MEGA_PACKET_SIZE-1); // Reading the IMU packet

      if(raw_data[MEGA_PACKET_SIZE-2] == PACKET_STOP_MEGA) // check taht the last data correspond to the packet end char
      {
        if(log_state == 1)
        {
        dataFile.print(micros()); dataFile.write(9); dataFile.print(2); dataFile.write(9);
         *ptr_buffer_int16 = raw_data[0];
        *(ptr_buffer_int16 + 1) = raw_data[1];
        U_MP1 = buffer_int16;
        dataFile.print(U_MP1); dataFile.write(9);
        if(print_flag){ Serial.print(U_MP1); Serial.write(9);}
        *ptr_buffer_int16 = raw_data[2];
        *(ptr_buffer_int16 + 1) = raw_data[3];
        U_MP2 = buffer_int16;
        dataFile.print(U_MP2); dataFile.write(9);
        if(print_flag){ Serial.print(U_MP2); Serial.write(9);}
        *ptr_buffer_int16 = raw_data[4];
        *(ptr_buffer_int16 + 1) = raw_data[5];
        U_MP3 = buffer_int16;
        dataFile.print(U_MP3); dataFile.write(9);
        if(print_flag){ Serial.print(U_MP3); Serial.write(9);}
        *ptr_buffer_int16 = raw_data[6];
        *(ptr_buffer_int16 + 1) = raw_data[7];
        U_ME = buffer_int16; //pwm pour moteur Exterieur
        dataFile.print(U_ME); dataFile.write(9);
        if(print_flag){ Serial.print(U_ME); Serial.write(9);}
        Angle_Leds = raw_data[8];
        dataFile.print(Angle_Leds); dataFile.write(9);
        if(print_flag){ Serial.print(Angle_Leds); Serial.write(9);}
        Kintens = raw_data[9];
        dataFile.print(Kintens); dataFile.write(9);
        if(print_flag){ Serial.print(Kintens); Serial.write(9);}
        *ptr_buffer_int16 = raw_data[10];
        *(ptr_buffer_int16 + 1) = raw_data[11];
        U_A1 = buffer_int16;
        dataFile.print(U_A1); dataFile.write(9);
        if(print_flag){ Serial.print(U_A1); Serial.write(9);}
        *ptr_buffer_int16 = raw_data[12];
        *(ptr_buffer_int16 + 1) = raw_data[13];
        U_A2 = buffer_int16;
        dataFile.print(U_A2); dataFile.write(9);
        if(print_flag){ Serial.print(U_A2); Serial.write(9);}
        *ptr_buffer_int16 = raw_data[14];
        *(ptr_buffer_int16 + 1) = raw_data[15];
        U_A3 = buffer_int16;
        dataFile.println(U_A3);
        if(print_flag){ Serial.println(U_A3); }
        }
      }
    }
  }

  if((log_state == 1) && ((millis() - log_start_time) > 120000))
  {
    dataFile.close();
    Serial.print("Opening file ... ");
    filename = "logc";
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
    
    dataFile.println("Timestamp(us) AccX(mg) AccY(mg) AccZ(mg) GyroX(dps) GyroY(dps) GyroZ (dps)");
    timer = micros();

  }
  
  if(((micros() - t0) > 1000000) && (log_state==1))
  {
    dataFile.close();
    log_state = 0;
    Serial.println("File closed");
  }
  digitalWrite(led_pin,log_state);
}


