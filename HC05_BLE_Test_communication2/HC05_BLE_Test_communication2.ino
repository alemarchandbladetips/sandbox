#include <WiiChuck.h>

#define BLE_serial Serial1
#define BLE_START1 0x42
#define BLE_START2 0x24

Accessory nunchuck;

uint32_t timer;
uint32_t dt_us = 100000;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  nunchuck.begin();
	if (nunchuck.type == Unknown) {
		nunchuck.type = NUNCHUCK;
	}
  BLE_serial.begin(38400);
  timer = micros();

}

void loop() {

  if( micros()-timer > dt_us)
  {
    timer += dt_us;

    nunchuck.readData();

    BLE_serial.write(BLE_START1);
    BLE_serial.write(BLE_START2);
    BLE_serial.write(nunchuck.getJoyX());
    Serial.print(nunchuck.getJoyX());Serial.print("\t");
    BLE_serial.write(nunchuck.getJoyY());
    Serial.print(nunchuck.getJoyY());Serial.print("\t");
    BLE_serial.write(nunchuck.getButtonC());
    Serial.print(nunchuck.getButtonC());Serial.print("\t");
    BLE_serial.write(nunchuck.getButtonZ());
    Serial.print(nunchuck.getButtonZ());Serial.println("\t");
  }
}
