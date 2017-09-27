void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  Serial1.begin(57600);

  Serial.println("Start Serial");
  Serial.println("Start Serial1");

}

void loop() {
  // put your main code here, to run repeatedly:

  Serial.write(1);
  Serial1.write(2);

  delay(1000);
}
