uint8_t input;
uint32_t timer, timer_tmp;
uint32_t dt;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial1.begin(115200);
  timer = micros();
}

void loop() {
  // put your main code here, to run repeatedly:
  //Serial.println(Serial1.available());
  if(Serial1.available())
  {
    timer_tmp = micros();
    dt = timer_tmp - timer;
    timer = timer_tmp;
    input = Serial1.read();
    Serial.println(dt);
  }
}

