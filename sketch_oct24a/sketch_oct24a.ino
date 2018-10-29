void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial1.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:

  uint8_t x;

  if(Serial1.available())
  {
    Serial.print(Serial1.read());Serial.print(" ");
  }
}
