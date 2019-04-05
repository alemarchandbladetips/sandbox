void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(18,INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  int16_t x=analogRead(18);
  Serial.println(x);
  delay(10);
}
