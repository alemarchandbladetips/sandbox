#include <SPI.h>
#include <SD.h>
#include <Wire.h>

#define PACKAGE_SIZE 50


/// from Josué
#define TEMPS_WAIT 4000  // correspond à max vitesse de 15000 rpm 

int pinInt = 15;
float compteur = 0;

float vitesse = 0;
uint32_t dt = 1, temps, dt_wait, temps_wait;
int NbPulse = 10;
int rpm = 0, last_rpm=0;

int val=0, compteur1 = 0, compteur2 = 0;

bool EtatA=LOW, last_EtatA=LOW, ok_Desc=LOW;

//end from Josué


const int chipSelect = 4;
String filename;
int state, log_state, state2;
long click_time,timer,log_start_time;
File dataFile;
int long_num = 1;

const int button_pin = 14;
const int led_pin = 13;

long t0,t1;

float buffer_float;
unsigned char *ptr_buffer = (unsigned char *)&buffer_float;
uint8_t package_buffer[PACKAGE_SIZE];

int16_t i,j;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial1.begin(115200);
  pinMode(led_pin,OUTPUT);
  pinMode(button_pin,INPUT);
  state = 0;
  state2 = 0;
  log_state = 0;

  //pinMode(pinInt,INPUT_PULLUP);

  Wire.begin();

  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
  
  digitalWrite(led_pin,1);
  delay(250);
  digitalWrite(led_pin,0);
  delay(250);
  digitalWrite(led_pin,1);
  delay(250);
  digitalWrite(led_pin,0);
  delay(250);
  digitalWrite(led_pin,1);
  delay(250);
  digitalWrite(led_pin,0);
  delay(250);
  digitalWrite(led_pin,1);
  delay(250);
  digitalWrite(led_pin,0);

  
  temps = micros();
  temps_wait = temps;
}

void loop() {
  // put your main code here, to run repeatedly:
  uint8_t x;

  if((state == 0) && (digitalRead(button_pin) == 0))
  {
    state = 1;
    click_time = millis();
  }

  if(digitalRead(button_pin) == 1)
  {
    state = 0;
    state2 = 0;
  }

  if((state == 1) && ((millis()-click_time) > 1000) && (state2 == 0))
  {
    state2 = 1;
    if(log_state == 0)
    {
      Serial.print("Opening file ... ");
      filename = "logtac";
      filename = filename + long_num;
      filename = filename + ".txt";
      Serial.println(filename);
      dataFile = SD.open(filename, FILE_WRITE);
      if(dataFile)
      {
        Serial.println("file openend");
        log_state = 1;
        long_num++;
        log_start_time = millis();
      }else
      {
        Serial.println("Opening failed");
      }
      
      timer = micros();

    } else
    {
      Serial.print("Closing file ... ");
      dataFile.close();
      Serial.println("file closed");
      log_state = 0;
    }
  }
  digitalWrite(led_pin,log_state);

  if((log_state == 1) && ((millis() - log_start_time) > 300000))
  {
    dataFile.close();
    Serial.print("Opening file ... ");
    filename = "logtac";
    filename = filename + long_num;
    filename = filename + ".txt";
    Serial.println(filename);
    dataFile = SD.open(filename, FILE_WRITE);
    if(dataFile)
    {
      Serial.println("file openend");
      log_state = 1;
      long_num++;
      log_start_time = millis();
    }else
    {
      Serial.println("Opening failed");
    }
    
    dataFile.println("Timestamp(us) GyroX(dps) GyroY(dps) GyroZ(dps)");
    timer = micros();

  }

  val = analogRead(pinInt);
    
  if (val <10) // en face du capteur Hall lecture donne 0
  {
    EtatA = LOW; // Moment quand le capteur passe en face de l'aimant
  }else
  {
    EtatA = HIGH;
  }

 ok_Desc = ( (EtatA == LOW) && (last_EtatA == HIGH) );
 last_EtatA = EtatA;
 
 if (ok_Desc)
 {  
    dt_wait = micros() - temps_wait;
    if ( (val < 10) && (dt_wait > TEMPS_WAIT) ) //état bas
    {
        compteur++;
        temps_wait += dt_wait;
    }
   
    if( compteur > (NbPulse-1) ) // si Nb de tour est NbPulse, calcul de vitesse
    { 
        dt = micros() - temps;
        temps += dt;
        
        if(dt > 0)
        {
            vitesse = (compteur / dt)*60000.0*1000.0;
        }
        else
        {
            vitesse = 0;
        }
        
        compteur = 0;
      
    }
  }

  // filtre pour la vitesse du capteur
  rpm = 0.9*vitesse + 0.1*last_rpm;
  //if( abs(rpm - last_rpm) > 50  ) { Serial.println(rpm);}
  if(rpm != last_rpm ) 
  { 
    if(log_state == 1)
    {
      dataFile.println(rpm);
      Serial.print("=> ");
    }
    Serial.println(rpm);
  }
  last_rpm = rpm;
  
 //Serial.println(analogRead(A5));
 //Serial.println(rpm);

  
}
