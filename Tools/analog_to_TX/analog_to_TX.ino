uint16_t input;
uint8_t output;
uint32_t timer, timer_tmp;
uint32_t dt;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  timer = micros();
}

void loop() {
  timer_tmp = micros();
  if(timer_tmp-timer >= 100000)
  {
    // starting byte
    Serial1.write(0xAA);
    
    Serial.println(timer_tmp-timer);
    timer = timer_tmp;

    // read current
    input = analogRead(19);
    output = (uint8_t)(input);
    Serial.println(output);
    // send to addalogger
    Serial1.write(output);

    // read voltage
    input = analogRead(18);
    output = (uint8_t)(input>>2);
    Serial.println(output);
    // send to addalogger
    Serial1.write(output);

    // ending byte
    Serial1.write(0x55);
  }
}
