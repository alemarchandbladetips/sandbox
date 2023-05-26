/*
 * echo - echo characters back to bluetooth device
 *
 * Waits for a connection and then echos each charater received.
 *
 * Debugging is enabled, so if you open the 'Serial Monitor' you can see
 * the search for the HC05 baud and the wait for the BT connection.
 */
#include <Arduino.h>
#include "HC05.h"
#include <SoftwareSerial.h>

HC05 btSerial = HC05(3, 2, 0, 1);  // cmd, state, rx, tx
HC05 btSerial2 = HC05(10, 9, 7, 8);  // cmd, state, rx, tx

void setup()
{
  DEBUG_BEGIN(115200);
  Serial1.begin(115200);
  Serial2.begin(115200);
  btSerial.findBaud();
  btSerial2.findBaud();
}

void loop()
{
  btSerial.println("Echo Server- type something");

  while (btSerial.connected())
  {
    if (btSerial.available())
    {
      btSerial.write(btSerial.read());
    }
  }
}
