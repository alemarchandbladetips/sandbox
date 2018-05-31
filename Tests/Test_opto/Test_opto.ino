void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Start");
}

void loop() {
  int x;
  // put your main code here, to run repeatedly:
  delay(200);
  Serial.write(24);
}

