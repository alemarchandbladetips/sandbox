

#define BLE_serial_Master Serial1
#define BLE_serial_Slave Serial2

int x;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  BLE_serial_Master.begin(115200);
  BLE_serial_Slave.begin(115200);

}

void loop() {
  // put your main code here, to run repeatedly:
  if(BLE_serial_Slave.available())
  {
    Serial.write(BLE_serial_Slave.read());
  }

  if(Serial.available())
  {
    BLE_serial_Master.write(Serial.read());
  }
  
}
