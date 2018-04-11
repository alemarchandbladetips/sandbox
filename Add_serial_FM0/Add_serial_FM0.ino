
#include <Arduino.h>
#include "wiring_private.h" // pinPeripheral() function

///////////////// declaration of Serial2 /////////////////////////
Uart Serial2 (&sercom1, 11, 10, SERCOM_RX_PAD_0, UART_TX_PAD_2);

void SERCOM1_Handler()
{
  Serial2.IrqHandler();
}
//////////////////////////////////////////////////////////////////

void setup() {
  // put your setup code here, to run once:
  // Assign pins 10 & 11 SERCOM functionality (Serial 2)
  pinPeripheral(10, PIO_SERCOM);
  pinPeripheral(11, PIO_SERCOM);

  Serial.begin(115200);
  Serial2.begin(38400);
  Serial1.begin(38400);
}

void loop() {
  int8_t x;
  // put your main code here, to run repeatedly:
 if (Serial2.available() > 0) // Number of data corresponding to the IMU packet size is waiting in the biffer of serial
  { 
    x = Serial2.read(); // read first data
    Serial.print(x);
  }
  if (Serial1.available() > 0) // Number of data corresponding to the IMU packet size is waiting in the biffer of serial
  { 
    x = Serial1.read(); // read first data
    Serial.print(x);
  }
}
