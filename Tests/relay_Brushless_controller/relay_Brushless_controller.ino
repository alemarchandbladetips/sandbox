

const int EN1 = 2;   // pin enable bls
const int IN1 = 3;    //pins des pwm


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  // pin Enable
  pinMode(EN1, OUTPUT); 
  digitalWrite(EN1, HIGH);

  pinMode(IN1, OUTPUT); 
  analogWrite(IN1, 0);

}

void loop() {
  // put your main code here, to run repeatedly:

  delay(1000);
  analogWrite(IN1, 255);
  digitalWrite(EN1, HIGH);
  Serial.println("high");
  delay(1000);
  analogWrite(IN1, 0);
  digitalWrite(EN1, LOW);
  Serial.println("low");

}
