
#define GM_JOYX_PIN 32
#define GM_JOYY_PIN 33

uint16_t joyX_int, joyY_int;

uint32_t timer;
uint32_t dt_us = 10000;

void setup() {
  // put your setup code here, to run once:
  //Serial.begin(115200);

  //pinMode(GM_JOYX_PIN,INPUT);
  //pinMode(GM_JOYY_PIN,INPUT);
}

void loop() {
  if( micros()-timer > dt_us)
  {
    timer += dt_us;

    //joyX_int = analogRead(GM_JOYX_PIN);
    //joyY_int = analogRead(GM_JOYY_PIN);

    //Serial.print(joyX_int);Serial.print("\t");
    //Serial.print(joyY_int);Serial.print("\t");


    //Serial.println("\t");

  }

}
