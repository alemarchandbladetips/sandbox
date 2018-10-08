void setup() {
  Serial1.begin(115200);

}

void loop() {
  // put your main code here, to run repeatedly:

  delay(1000);
  Serial1.write(0xAA);
  Serial1.write(23);
  Serial1.write(45);
  Serial1.write(0x55);
}
