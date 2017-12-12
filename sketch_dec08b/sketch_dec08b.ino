

const float pi = 3.14159265359;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(112500);
}

void loop() {
  int i;
  delay(10000);
  for(i=0;i<1080;i++)
  {
    Serial.print((uint8_t)(sin(i/3*pi/180)*100+127)),Serial.print(", ");
  }
  // put your main code here, to run repeatedly:
  delay(60000);
}
