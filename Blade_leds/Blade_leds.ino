/*
 * TEST
 * 
  1) Le pgm de contrôle tourne
  à partir du moment ou une trame d'angle est reçue de la part du main.
  (il est en attente de la récéption de la trame / 100Hz)
  2) Une fois la trame reçue, il estime quand est ce qu'il faut lancer une trame de flash
  3) la partie "prédiction" peut être activée maisl ca complique un peu les choses:
  - il faut que l'ensemble du fash et le tem
  ps d'attente soit plys rapide que 10 ms pour ne pas faire sauter le cycle.

Ca marche OK pour les 3 ! 
*/
int temps_led= 300; // durée du flash
uint32_t t0, enlapsed_time;
uint8_t light_on = 0;

uint8_t print_data = 0;

int16_t angle_252;
int16_t angle_offset = 0; // Blade 3
//int16_t angle_offset = 84; // Blade 1 (120deg in 252_angle)
//int16_t angle_offset = 168; // Blade 2 (240deg in 252_angle)

uint8_t intensity_100;
float intensity_256;

int ok3 = 0;   // sert à éviter d'avoir des doubles flash 
int ok1 = 0;
int ok2 = 0;

// R: cable jaune
int Couleur1 = 9;
// G: cable rouge
int Couleur2 = 10;
// B: cable marron
int Couleur3 = 11;

/////////////////////////////////////////////////////////////

void light_white(uint8_t intensity)
{
  float intensity_tmp = intensity*0.78;
  
  analogWrite(Couleur1,(uint8_t)intensity_tmp);
  analogWrite(Couleur2,intensity);
  analogWrite(Couleur3,(uint8_t)intensity_tmp);
}

void light_blue(uint8_t intensity)
{
  analogWrite(Couleur1,0);
  analogWrite(Couleur2,0);
  analogWrite(Couleur3,intensity);
}

void light_red(uint8_t intensity) // Roxanne
{
  analogWrite(Couleur1,intensity);
  analogWrite(Couleur2,0);
  analogWrite(Couleur3,0);
}

void light_off()
{
  analogWrite(Couleur1,0);
  analogWrite(Couleur2,0);
  analogWrite(Couleur3,0);
}

/////////////////////////////////////////////////////////////

void setup() {
// LEDS 
  pinMode(Couleur1,OUTPUT);  
  pinMode(Couleur2,OUTPUT); 
  pinMode(Couleur3,OUTPUT); 

// COMUNICATION
  Serial.begin(115200);    // reception angle

}

void loop() {

  uint8_t dummy;
  
  enlapsed_time = millis()-t0;

  if ((enlapsed_time > temps_led) && (light_on == 1))
  {
    light_off();
    light_on = 0;
  }

// DECLENCHEMENT BOUCLE
  if (Serial.available() != 0)
  { // début de boucle dès la réception via serie de l'angle et des commandes   

    // assigner les valeurs
    angle_252 = Serial.read(); // current rotor angle
    angle_252 = constrain(angle_252,0,252);
    

    delayMicroseconds(100);
    
    if (Serial.available() == 1)  
    {
      intensity_100 = Serial.read();
      intensity_100 = constrain(intensity_100,0,100);
      intensity_256 = (float)intensity_100*2.55;

      if(print_data)
      {
        Serial.print(angle_252);Serial.print(" ");
        Serial.print((uint8_t)intensity_100);Serial.print(" ");
        Serial.print(100*ok1);Serial.print(" ");
        Serial.print(100*ok2);Serial.print(" ");
        Serial.print(100*ok3);Serial.print(" ");
        Serial.print(100*light_on);Serial.println(" ");
      }

      angle_252 += angle_offset;
      if (angle_252 > 252)
      {
        angle_252 -= 252;
      }

      if (angle_252>0 && angle_252<42)
      {
        if (ok1 == 0)
        {
          ok1 = 1;
          ok2 = 0;
          ok3 = 0;
          light_blue((uint8_t)intensity_256);
          t0 = millis();
          light_on = 1;
        }
      }

      if (angle_252>84 && angle_252<84+42)
      {
        if (ok2 == 0)
        {
          ok1 = 0;
          ok2 = 1;
          ok3 = 0;
          light_white((uint8_t)intensity_256);
          t0 = millis();
          light_on = 1;
        }
      }

      if (angle_252>168 && angle_252<168+42)
      {
        if (ok3 == 0)
        {
          ok1 = 0;
          ok2 = 0;
          ok3 = 1;
          light_red((uint8_t)intensity_256);
          t0 = millis();
          light_on = 1;
        }
      }
    }
  }
}
