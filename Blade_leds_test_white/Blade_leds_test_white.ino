/*
 * TEST
 * 
  1) Le pgm de contrôle tourne
  à partir du moment ou une trame d'angle est reçue de la part du main.
  (il est en attente de la récéption de la trame / 100Hz)
  2) Une fois la trame reçue, il estime quand est ce qu'il faut lancer une trame de flash
  3) la partie "prédiction" peut être activée maisl ca complique un peu les choses:
  - il faut que l'ensemble du fash et le temps d'attente soit plys rapide que 10 ms pour ne pas faire sauter le cycle.

Ca marche OK pour les 3 ! 
*/
int temps_led= 50; // durée du flash

int16_t angle_252;
int16_t angle_offset = 0; // Blade 1
//int16_t angle_offset = 84; // Blade 2 (120deg in 252_angle)
//int16_t angle_offset = 168; // Blade 3 (240deg in 252_angle)

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
analogWrite(Couleur1,200);
  analogWrite(Couleur2,255);
  analogWrite(Couleur3,200);
}
