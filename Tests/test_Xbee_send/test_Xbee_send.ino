void setup() {
  // put your setup code here, to run once:
  Serial1.begin(38400);
}

void loop() {
  // put your main code here, to run repeatedly:

  delay(500);
  Serial1.write(54);
  Serial1.write(1);
  delay(500);
  Serial1.write(54);
  Serial1.write(2);
  delay(500);
  Serial1.write(54);
  Serial1.write(1);
  delay(1200);
  Serial1.write(54);
  Serial1.write(2);
}
