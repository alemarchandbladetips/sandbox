
uint32_t t0,t1,dt = 5000000;



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  t0 = 5000000;
}

void loop() {
  // put your main code here, to run repeatedly:

  t0 += 1000000;
  dt = t0-t1;
  Serial.print(t0);
  Serial.print(" "); 
  Serial.print(t1);
  Serial.print(" "); 
  Serial.println(dt); 
  if(t0<5000000)
  {
    delay(1000);
  }
  t1 = t0;

}
