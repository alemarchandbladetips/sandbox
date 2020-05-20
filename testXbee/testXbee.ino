#include "bte_functions.h"

/******* Xbee **************/

#define XBEE_SERIAL Serial1

const uint8_t NbDataXB = 9;
int16_t DataXB[NbDataXB];

int16_t DataXB1[10];
int16_t DataXB2[10];
int16_t DataXB3[3];
uint8_t count_XB = 1;

float DataXB_float[5];
uint8_t NbDataXB_float = 5;

uint8_t GPS_init_validated = 0;
uint8_t reception_position;

/************** ****************/

float BNO_ypr[3];
float gps_position[3];
float gps_speed[3];

int8_t x,i;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  XBEE_SERIAL.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:

  if(GPS_init_validated)
  {
    reception_position = bte_recep_int16_t(XBEE_SERIAL, NbDataXB, 137, 173, DataXB, DataXB);
    if(reception_position)
    {
      for(i=0;i<9;i++)
      {
        if(i<3)
        {
          BNO_ypr[i] = DataXB[i] * 180.0 / 32768.0;
          Serial.print(BNO_ypr[i]);
        }else if (i<6)
        {
          gps_position[i-3] = DataXB[i] * 500.0 / 32768.0;
        } else
        {
          gps_speed[i-6] = DataXB[i] * 50.0 / 32768.0;
        }
      }
      Serial.println("");
    }
  } else
  {
    GPS_init_validated = bte_recep_float(XBEE_SERIAL, NbDataXB_float, 137, 173, DataXB_float, 0);
    XBEE_SERIAL.write(1);
    if(GPS_init_validated)
    {
      Serial.print("init : ");
      Serial.println(GPS_init_validated);
    }
  }
}
