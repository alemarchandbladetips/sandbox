void setup() {
  Serial.begin(112500);
}

void loop() {

uint8_t x;
  
  if(Serial.available())
  {
    x = Serial.read();
    Serial.write(x);
  }
  

}
