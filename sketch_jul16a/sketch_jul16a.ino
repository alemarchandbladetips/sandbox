int i;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial1.begin(115200);
  i = 0;
}

void loop() {
  // put your main code here, to run repeatedly:

  if(Serial1.available())
  {
    Serial.print(Serial1.read());
    Serial.print(" ");
    i++;
    if(i==52)
    {
      Serial.println(" ");
      i = 0;
    }
  }
}
