void setup() {
  // put your setup code here, to run once:
Serial.begin(19200);
Serial.print("Start");

}

void loop() {
  // put your main code here, to run repeatedly:
  uint8_t x;
  if(Serial.available()>0)
  {
    Serial.print(Serial.available()); Serial.print(" ");
    x = Serial.read();
    Serial.print(x);Serial.println(" ");
  }
}
