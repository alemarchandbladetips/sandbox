void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(18,INPUT_PULLUP);
}

void loop() {
  // put your main code here, to run repeatedly:
  uint8_t x;
  
  delay(100);
  x = analogRead(18);
  Serial.println(x);
}
