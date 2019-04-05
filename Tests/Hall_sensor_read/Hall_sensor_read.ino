

#define ANALOG_PIN 18

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(ANALOG_PIN,INPUT_PULLUP);
}

void loop() {
  // put your main code here, to run repeatedly:
  uint16_t x;
  
  x = analogRead(ANALOG_PIN);
  Serial.print(100*(x<10)); Serial.print(" ");Serial.println(x);
  delay(1);
}
