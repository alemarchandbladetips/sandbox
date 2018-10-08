void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
  
  pinMode(14,INPUT);
  pinMode(15,OUTPUT);

  digitalWrite(15,HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:

  delay(100);

  Serial.println(digitalRead(14));
  
}
