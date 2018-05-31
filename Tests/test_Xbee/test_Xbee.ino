
long last_time;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(38400);
  last_time = millis();
}

void loop() {
  int x;
  // put your main code here, to run repeatedly:
  if(Serial.available() >=2)
  {
    x = Serial.read();
    Serial.print(x);Serial.print(" ");
    if(x == 54)
    {
      x = Serial.read();
      Serial.println(x);
      last_time = millis();
    }
  }
  if((millis() - last_time) > 1000)
  {
    Serial.println("Time Out");
    delay(200);
  }
}
