
/******Capteur Poids*************/

int pinPoids = 20;

///// mesure et filtrage
// variables
int mesure_poids;
float mesure_poids_f; 
float poids_f;
// param
float aplha_mesure_poids = 0.001;

///// Gain et offset calib
float gain_poids = 0.369004;
float offset_poids = 0.0;

///// variables intermédiaires de calibration
float poids1,poids2,mesure_poids1,mesure_poids2;


/******Capteur Couple*************/

int pinCouple = 14;

///// mesure et filtrage
// variables
int mesure_couple;
float mesure_couple_f; 
float couple_f;
// param
float aplha_mesure_couple = 0.001;

///// Gain et offset calib
float gain_couple = 1.351114;
float offset_couple = 0.0;
///// variables intermédiaires de calibration
float couple1,couple2,mesure_couple1,mesure_couple2;

/******Général calibration*************/

// Variable d'état de la procédure de calibration
int8_t calibration_state = 0;
// Pour lecture serial
char caractere;

/******Affichage*************/
uint8_t enable_plot = 0;

/******Timers*************/

// temps d'acquisition
uint32_t dt_acquisition = 1000;
uint32_t tt_acquisition;

// temps de calibration
uint32_t dt_calib = 10000000;
uint32_t tt_calib;
uint32_t dt_calib_WIP = 1000000;
uint32_t tt_calib_WIP;

// temps d'affichage
uint32_t dt_plot = 10000;
uint32_t tt_plot;

void setup()
{
  // initialisation serial
  Serial.begin(115200);

  // résolution des pin analog
  analogReadResolution(12);
  
  // Initialisation des timers
  tt_acquisition = micros();

  // Initialisation des filtres
  mesure_poids_f = (float)analogRead(pinPoids);
  mesure_couple_f = (float)analogRead(pinCouple);

  // Initialisation de la procédure de calibration
  calibration_state = -1;

  plot_commands();
  
}


void loop()
{  

  // Boucle de lecture et filtrage des capteurs.
  if ( (micros() - tt_acquisition) > dt_acquisition )
  {   
    tt_acquisition += dt_acquisition;

    // Aquisition, filtrage, corraction de la mesure de poids
    mesure_poids = analogRead(pinPoids);
    mesure_poids_f = (1-aplha_mesure_poids)*mesure_poids_f+aplha_mesure_poids*(float)mesure_poids;
    poids_f = gain_poids*(mesure_poids_f - offset_poids);

    // Aquisition, filtrage, corraction  de la mesure de couple
    mesure_couple = analogRead(pinCouple);
    mesure_couple_f = (1-aplha_mesure_couple)*mesure_couple_f+aplha_mesure_couple*(float)mesure_couple;
    couple_f = gain_couple*(mesure_couple_f - offset_couple);

    // Affichage à la fréquence choisie
    if ( (micros() - tt_plot) > dt_plot )
    {   
      tt_plot += dt_plot;
      if(enable_plot)
      {
        Serial.print( poids_f );Serial.print("\t");
        Serial.print( couple_f );Serial.print("\n");
        
        Serial.print( mesure_couple );Serial.print("\t");
        Serial.print( mesure_couple_f );Serial.print("\n");
      }
    }
  }

  if( calibration_state == 0 )
  {
    Serial.print ("/////  Calibration du capteur de poids  /////");Serial.print("\n");Serial.print("\n");
    Serial.print ("Placer le premier poids sur le capteur (Typiquement capteur à vide)");Serial.print("\n");
    Serial.print ("Entrer le poids en gramme dans le Serial");Serial.print("\n");
    dt_acquisition = 1000;
    aplha_mesure_poids = 0.001;
    calibration_state = 1;
  } else if ( calibration_state == 1 )
  {
    if (Serial.available())
    {
      poids1 = Serial.parseFloat();
      Serial.print ("Premier poids :");Serial.print( poids1 );Serial.print ("g");Serial.print("\n");
      while(Serial.available())
      {
        Serial.read();
      }
      Serial.print ("Attendez 10s pour la prise de mesure");Serial.print("\n");
      tt_calib = micros();
      mesure_poids_f = (float)analogRead(pinPoids);
      calibration_state = 2;
    } 
  } else if ( calibration_state == 2 )
  {
    if ( (micros() - tt_calib) > dt_calib )
    {   
      mesure_poids1 = mesure_poids_f;
      Serial.print ("Première mesure_poids :");Serial.print( mesure_poids1 );Serial.print("\n");Serial.print("\n");
      Serial.print ("Placer le second poids sur le capteur");Serial.print("\n");
      Serial.print ("Entrer le poids en gramme dans le Serial");Serial.print("\n");
      calibration_state = 3;
    }
  } else if ( calibration_state == 3 )
  {
    if (Serial.available())
    {
      poids2 = Serial.parseFloat();
      Serial.print ("Second poids :");Serial.print( poids2 );Serial.print ("g");Serial.print("\n");
      while(Serial.available())
      {
        Serial.read();
      }
      Serial.print ("Attendez 10s pour la prise de mesure");Serial.print("\n");
      tt_calib = micros();
      mesure_poids_f = (float)analogRead(pinPoids);
      calibration_state = 4;
    }
  } else if ( calibration_state == 4 )
  {
    if ( (micros() - tt_calib) > dt_calib )
    {   
      mesure_poids2 = mesure_poids_f;
      Serial.print ("Seconde mesure_poids :");Serial.print( mesure_poids2 );Serial.print("\n");Serial.print("\n");
      Serial.print ("Calibration terminée");Serial.print("\n");
      gain_poids = (poids2-poids1)/(mesure_poids2-mesure_poids1);
      Serial.print ("Gain de calibration : ");Serial.print( gain_poids ,6);Serial.print("\n");Serial.print("\n");
      aplha_mesure_poids = 0.005;
      plot_commands();
      calibration_state = -1;
    }
  } else if( calibration_state == 6 )
  {
    Serial.print ("/////  Calibration du capteur de couple  /////");Serial.print("\n");Serial.print("\n");
    Serial.print ("Placer le premier poids sur le capteur de couple à 10cm (Typiquement capteur à vide)");Serial.print("\n");
    Serial.print ("Entrer le poids en gramme dans le Serial");Serial.print("\n");
    dt_acquisition = 1000;
    aplha_mesure_couple= 0.001;
    calibration_state = 7;
  } else if ( calibration_state == 7 )
  {
    if (Serial.available())
    {
      couple1 = Serial.parseFloat();
      Serial.print ("Premier poids :");Serial.print( couple1 );Serial.print ("g à 10cm");Serial.print("\n");
      while(Serial.available())
      {
        Serial.read();
      }
      Serial.print ("Attendez 10s pour la prise de mesure");Serial.print("\n");
      tt_calib = micros();
      mesure_couple_f = (float)analogRead(pinCouple);
      calibration_state = 8;
    } 
  } else if ( calibration_state == 8 )
  {
    if ( (micros() - tt_calib) > dt_calib )
    {   
      mesure_couple1 = mesure_couple_f;
      Serial.print ("Première mesure :");Serial.print( mesure_couple1 );Serial.print("\n");Serial.print("\n");
      Serial.print ("Placer le second poids sur le capteur");Serial.print("\n");
      Serial.print ("Entrer le poids en gramme dans le Serial");Serial.print("\n");
      calibration_state = 9;
    }
  } else if ( calibration_state == 9 )
  {
    if (Serial.available())
    {
      couple2 = Serial.parseFloat();
      Serial.print ("Second poids :");Serial.print( couple2 );Serial.print ("g à 10cm");Serial.print("\n");
      while(Serial.available())
      {
        Serial.read();
      }
      Serial.print ("Attendez 10s pour la prise de mesure");Serial.print("\n");
      tt_calib = micros();
      mesure_couple_f = (float)analogRead(pinCouple);
      calibration_state = 10;
    }
  } else if ( calibration_state == 10)
  {
    if ( (micros() - tt_calib) > dt_calib )
    {   
      mesure_couple2 = mesure_couple_f;
      Serial.print ("Seconde mesure :");Serial.print( mesure_couple2 );Serial.print("\n");Serial.print("\n");
      Serial.print ("Calibration terminée");Serial.print("\n");
      gain_couple = 10*(couple2-couple1)/(mesure_couple2-mesure_couple1);
      Serial.print ("Gain de calibration : ");Serial.print( gain_couple ,6);Serial.print("\n");Serial.print("\n");
      aplha_mesure_couple= 0.005;
      plot_commands();
      calibration_state = -1;
    }
  } 

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////
///// Gestion des commandes du Serial

  if (Serial.available())
  {
    caractere = Serial.read();
    if(caractere == 't' && calibration_state==-1 )
    {
      tare_all();
    }
    if(caractere == 'p' && calibration_state==-1 )
    {
      calibration_state = 0;
      enable_plot = 0;
    }
    if(caractere == 'c' && calibration_state==-1 )
    {
      calibration_state = 6;
      enable_plot = 0;
    }
    if(caractere == 'e' && calibration_state==-1 )
    {
      enable_plot = !enable_plot;
      if(enable_plot == 0)
      {
        plot_commands();
      }
    }
    while(Serial.available())
    {
      Serial.read();
    }
  }
  
}
void tare_all()
{
  int32_t dt_acquisition_PC_tmp;
  float aplha_mesure_poids_tmp;
  float aplha_mesure_couple_tmp;

  dt_acquisition_PC_tmp = dt_acquisition;
  aplha_mesure_poids_tmp = aplha_mesure_poids;
  aplha_mesure_couple_tmp = aplha_mesure_couple;
  
  dt_acquisition = 1000;
  aplha_mesure_poids = 0.001;
  aplha_mesure_couple = 0.001;
  
  mesure_poids_f = analogRead(pinPoids);
  tt_calib_WIP = micros();
  tt_calib = micros();
  tt_acquisition = micros();

  Serial.print("\n");
  Serial.print("Tare des capteur de couple et de poids en cours ");
    
  while ((micros() - tt_calib) < dt_calib )
  {
    if ( (micros() - tt_acquisition) > dt_acquisition )
    {   
      tt_acquisition += dt_acquisition;
      
      mesure_poids = analogRead(pinPoids);
      mesure_poids_f = (1-aplha_mesure_poids)*mesure_poids_f+aplha_mesure_poids*(float)mesure_poids;

      mesure_couple = analogRead(pinCouple);
      mesure_couple_f = (1-aplha_mesure_couple)*mesure_couple_f+aplha_mesure_couple*(float)mesure_couple;
    }
    if ( (micros() - tt_calib_WIP) > dt_calib_WIP )
    { 
      tt_calib_WIP += dt_calib_WIP;
      Serial.print(".");
    }
  }

  offset_poids = mesure_poids_f;
  offset_couple = mesure_couple_f;
  
  dt_acquisition = dt_acquisition_PC_tmp;
  aplha_mesure_poids = aplha_mesure_poids_tmp;
  aplha_mesure_couple = aplha_mesure_couple_tmp;

  Serial.print("\n");Serial.print("Tare OK");Serial.print("\n");
  
  return;

}

void plot_commands()
{
  Serial.print("\n");
  Serial.print( "///////////////////////////////////////////" );Serial.print("\n");
  Serial.print( "Commandes disponible dans le Serial (desactivé pendant les phases de calibration) :");Serial.print("\n");
  Serial.print( "'t' tare des capteurs de couple et de poids");Serial.print("\n");
  Serial.print( "'p' Calibration du capteur de poids");Serial.print("\n");
  Serial.print( "'c' Calibration du capteur de couple");Serial.print("\n");
  Serial.print( "'e' enable/disable plot");Serial.print("\n");Serial.print("\n");
}
