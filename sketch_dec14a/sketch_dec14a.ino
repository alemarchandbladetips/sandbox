#include <Arduino.h>   // required before wiring_private.h
#include "wiring_private.h" // pinPeripheral() function

#define POZYX_PACKET_SIZE 9 // number of bytes to be recieved from 
#define POZYX_PACKET_START 137 // starting char of package
#define POZYX_PACKET_STOP 173 // starting char of package

int8_t raw_data[100];

Uart Serial2 (&sercom1, 11, 10, SERCOM_RX_PAD_0, UART_TX_PAD_2);

void SERCOM1_Handler()
{
  Serial2.IrqHandler();
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(38400);
  Serial1.begin(115200);
  
  // Assign pins 10 & 11 SERCOM functionality
  pinPeripheral(10, PIO_SERCOM);
  pinPeripheral(11, PIO_SERCOM);
}

uint8_t i=0;
void loop() {
  int8_t x,i;
  if (Serial2.available() > POZYX_PACKET_SIZE) // Number of data corresponding to the IMU packet size is waiting in the biffer of serial
  { 
    x = Serial2.read(); // read first data
    Serial.print(x); Serial.print(" ");
    if(x == POZYX_PACKET_START) // check that first data correspond to start char
    {
      for(i=0;i<POZYX_PACKET_SIZE;i++)
      {
        raw_data[i] = Serial2.read(); // Reading the IMU packet
      }
      //Serial.print(raw_data[GIMBAL_PACKET_SIZE-1]);
      if(raw_data[POZYX_PACKET_SIZE-1] == POZYX_PACKET_STOP) // check taht the last data correspond to the packet end char
      {
        Serial.println(" ");
      }
    }
  }
}
