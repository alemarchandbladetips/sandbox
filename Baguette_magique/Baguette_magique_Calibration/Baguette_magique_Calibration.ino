
/******Capteur Poids*************/

int pinPoids = 16;

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

// temps d'acquisition
int32_t dt_micros = 1000;
// temps de calibration
int32_t dt_calib_micros = 10000000;
// temps d'affichage
int32_t dt_plot_micros = 10000;


/******Variables timers*************/
uint32_t tt;
uint32_t tt_calib;
uint32_t tt_plot;

void setup()
{
  Serial.begin(115200);
  analogReadResolution(12);
  // Timers
  tt = micros();

  mesure_poids_f = (float)analogRead(pinPoids);
  mesure_couple_f = (float)analogRead(pinCouple);
  calibration_state = 0;
  
}


void loop()
{  

  // Boucle de lecture et filtrage des capteurs.
  if ( (micros() - tt) > dt_micros )
  {   
    tt += dt_micros;

    // Aquisition, filtrage, corraction de la mesure de poids
    mesure_poids = analogRead(pinPoids);
    mesure_poids_f = (1-aplha_mesure_poids)*mesure_poids_f+aplha_mesure_poids*(float)mesure_poids;
    poids_f = gain_poids*(mesure_poids_f - offset_poids);

    // Aquisition, filtrage, corraction  de la mesure de couple
    mesure_couple = analogRead(pinCouple);
    mesure_couple_f = (1-aplha_mesure_couple)*mesure_couple_f+aplha_mesure_couple*(float)mesure_couple;
    couple_f = gain_couple*(mesure_couple_f - offset_couple);

    // Affichage à la fréquence choisie
    if ( (micros() - tt_plot) > dt_plot_micros )
    {   
      tt_plot += dt_plot_micros;
      if(calibration_state == 12 || calibration_state == -1)
      {
        Serial.print( poids_f );Serial.print("\t");
        Serial.print( couple_f );Serial.print("\n");
        
        //Serial.print( mesure_couple );Serial.print("\t");
        //Serial.print( mesure_couple_f );Serial.print("\n");
      }
    }
  }

  if( calibration_state == 0 )
  {
    Serial.print ("/////  Calibration du capteur de poids  /////");Serial.print("\n");Serial.print("\n");
    Serial.print ("Placer le premier poids sur le capteur (Typiquement capteur à vide)");Serial.print("\n");
    Serial.print ("Entrer le poids en gramme dans le Serial");Serial.print("\n");
    dt_micros = 1000;
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
    if ( (micros() - tt_calib) > dt_calib_micros )
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
    if ( (micros() - tt_calib) > dt_calib_micros )
    {   
      mesure_poids2 = mesure_poids_f;
      Serial.print ("Seconde mesure_poids :");Serial.print( mesure_poids2 );Serial.print("\n");Serial.print("\n");
      Serial.print ("Calibration terminée");Serial.print("\n");
      gain_poids = (poids2-poids1)/(mesure_poids2-mesure_poids1);
      Serial.print ("Gain de calibration : ");Serial.print( gain_poids ,6);Serial.print("\n");Serial.print("\n");
      calibration_state = 5;
    }
  } else if( calibration_state == 5 )
  {
    Serial.print ("/////  Calibration du capteur de couple  /////");Serial.print("\n");Serial.print("\n");
    Serial.print ("Placer le premier poids sur le capteur de couple à 10cm (Typiquement capteur à vide)");Serial.print("\n");
    Serial.print ("Entrer le poids en gramme dans le Serial");Serial.print("\n");
    dt_micros = 1000;
    aplha_mesure_couple= 0.001;
    calibration_state = 6;
  } else if ( calibration_state == 6 )
  {
    if (Serial.available())
    {
      couple1 = Serial.parseFloat();
      Serial.print ("Premier poids :");Serial.print( poids1 );Serial.print ("g à 10cm");Serial.print("\n");
      while(Serial.available())
      {
        Serial.read();
      }
      Serial.print ("Attendez 10s pour la prise de mesure");Serial.print("\n");
      tt_calib = micros();
      mesure_couple_f = (float)analogRead(pinCouple);
      calibration_state = 7;
    } 
  } else if ( calibration_state == 7 )
  {
    if ( (micros() - tt_calib) > dt_calib_micros )
    {   
      mesure_couple1 = mesure_couple_f;
      Serial.print ("Première mesure :");Serial.print( mesure_couple1 );Serial.print("\n");Serial.print("\n");
      Serial.print ("Placer le second poids sur le capteur");Serial.print("\n");
      Serial.print ("Entrer le poids en gramme dans le Serial");Serial.print("\n");
      calibration_state = 8;
    }
  } else if ( calibration_state == 8 )
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
      calibration_state = 9;
    }
  } else if ( calibration_state == 9)
  {
    if ( (micros() - tt_calib) > dt_calib_micros )
    {   
      mesure_couple2 = mesure_couple_f;
      Serial.print ("Seconde mesure :");Serial.print( mesure_couple2 );Serial.print("\n");Serial.print("\n");
      Serial.print ("Calibration terminée");Serial.print("\n");
      gain_couple = 10*(couple2-couple1)/(mesure_couple2-mesure_couple1);
      Serial.print ("Gain de calibration : ");Serial.print( gain_couple ,6);Serial.print("\n");Serial.print("\n");
      calibration_state = 10;
    }
  } else if ( calibration_state == 10 )
  {
    Serial.print ("Retirez les poids et entrez 't' pour tarer les capteurs");Serial.print("\n");Serial.print("\n");
    calibration_state = 11;
  } else if ( calibration_state == 11 )
  {
    if (Serial.available())
    {
      caractere = Serial.read();
      if(caractere == 't')
      {
        Serial.print ("Tare en cours, attendez");Serial.print("\n");
        while(Serial.available())
        {
          Serial.read();
        }
        tare_all();
        aplha_mesure_poids = 0.002;
        aplha_mesure_couple = 0.002;
        calibration_state = 12;
      }
    }
  }
  
}

void tare_all()
{
  int32_t dt_micros_tmp;
  int32_t aplha_mesure_poids_tmp;

  dt_micros_tmp = dt_micros;
  aplha_mesure_poids_tmp = aplha_mesure_poids;
  
  dt_micros = 1000;
  aplha_mesure_poids = 0.001;
  
  mesure_poids_f = analogRead(pinPoids);
  tt_calib = micros();
    
  while ((micros() - tt_calib) < dt_calib_micros )
  {
    if ( (micros() - tt) > dt_micros )
    {   
      tt += dt_micros;
      
      mesure_poids = analogRead(pinPoids);
      mesure_poids_f = (1-aplha_mesure_poids)*mesure_poids_f+aplha_mesure_poids*(float)mesure_poids;

      mesure_couple = analogRead(pinCouple);
      mesure_couple_f = (1-aplha_mesure_couple)*mesure_couple_f+aplha_mesure_couple*(float)mesure_couple;
    }
  }

  offset_poids = mesure_poids_f;
  offset_couple = mesure_couple_f;
  
  dt_micros = dt_micros_tmp;
  aplha_mesure_poids = aplha_mesure_poids_tmp;
  
  return;

}
