uint8_t angle_252 = 0;
uint8_t intensity = 100;

int8_t default_ = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(default_==0)
  {
    Serial.write(angle_252);
  }
  default_ = 0;
  Serial.write(intensity);
  delay(10);
  
  angle_252 += 3;
  
  if (angle_252 > 252)
  {
    angle_252 -= 252;
    //intensity += 10;
  }
/*
  if(intensity >100)
  {
    intensity -=50;
    //default_ = 1;
  }*/

}
