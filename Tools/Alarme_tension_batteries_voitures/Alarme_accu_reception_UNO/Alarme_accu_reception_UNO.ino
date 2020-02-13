int pinAlarme=13;
int MeanTension;
long last_time;

void setup()
{
    pinMode(pinAlarme,OUTPUT);
    Serial.begin(38400);
    digitalWrite(pinAlarme,LOW);
    last_time = millis();
}

void loop()
{
  int x;
  if(Serial.available() >=2)
  {
    x = Serial.read();
    Serial.print(x);Serial.print(" ");
    if(x == 54)
    {
      MeanTension = Serial.read();
      Serial.println(MeanTension);
      last_time = millis();
    }
  }
  if((millis() - last_time) > 2000)
  {
    Serial.println("Time Out");
    MeanTension = 0;
  }

 if(MeanTension > 1 )
 {
    digitalWrite(pinAlarme,LOW);
 }else if (MeanTension == 0 )
 {
    digitalWrite(pinAlarme,LOW);
    delay(200);
    digitalWrite(pinAlarme,HIGH);
    delay(200);
 }else
 {
    digitalWrite(pinAlarme,HIGH);
 }
 //Serial.println(MeanTension);
}

