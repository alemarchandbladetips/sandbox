int8_t pinTension= A0;
int16_t Tension=0;
float tension_filt = 0;
float tension_prev = 0;

void setup()
{
    pinMode(pinTension,INPUT);
    Serial.begin(38400);
}

void loop()
{
  delay(10);
  
 Tension=analogRead(pinTension);
 tension_filt = 0.05*(float(Tension*49.6/900.0))+0.95*tension_prev;
 tension_prev = tension_filt;
 
   if(tension_filt<44.0)
   {
    Serial.write(54);Serial.write(1);
   } else
   {
    Serial.write(54);Serial.write(2);
   }
}

