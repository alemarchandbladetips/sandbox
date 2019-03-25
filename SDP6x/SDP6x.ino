/****************************************************************
 * ReadSDP6x
 *  An example sketch that reads the sensor and prints the
 *  Differential Pressure to the PC's serial port
 *
 *  Tested with:
 *    - SDP600/610-125Pa (bought from FutureElectronics.com)
 ***************************************************************/

#include <Wire.h>
#include "SDP6x.h"
//#include "SDP6x1.h"

float difPressure;
float difPressure1;
//TwoWire Wire1;

void setup()
{
//  Wire1.setSDA(30);
//  Wire1.setSCL(29);
//  Wire1.begin();
//  Wire1.setClock(400000);
  Wire.begin();
  Serial.begin(115200);
  difPressure = 0.0;
}

void loop()
{
  //Serial.print("Pressure Differential (Pa): ");
  difPressure = SDP6x.GetPressureDiff();
  //difPressure1 = SDP6x1.GetPressureDiff();
  Serial.print(difPressure);
  Serial.print(" ");
  //Serial.print(difPressure1);
  Serial.print("\n");
  delay(50);
}
