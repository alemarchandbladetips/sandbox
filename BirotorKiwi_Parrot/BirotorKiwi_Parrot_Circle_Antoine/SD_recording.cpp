#include "SD_recording.h"
#include <SD.h>

/******* Carte SD **************/
File dataFile;
bool Open = false, Closed = true;
String dataString;
char *filename;
uint32_t nb_file = 0, counter = 0;
bool  OK_SDCARD = false;
char Num[8];
uint32_t temps_log, dt_log, timer_mode, time_switch, temps2;
uint8_t last_sw_Val=2;

void checkExist(void)
{
  do
  {
    strcpy(filename, "log");
    sprintf(Num, "%lu", counter);
    strcat(filename, Num); strcat(filename, ".txt");
    counter++;
  }
  while ( SD.exists(filename) );
  nb_file = counter - 1;
}

void init_SD_record(void)
{
  OK_SDCARD = SD.begin(BUILTIN_SDCARD);
  if ( !OK_SDCARD )
  {
    Serial.println("Card failed, or not present");
    //while(1);
  }
  else
  {
    Serial.println("card initialized.");
    filename = (char*) malloc( strlen("log") + strlen(Num) + strlen(".txt") + 1) ;
    checkExist(); //vérifie existence du fichier et l'écriture commence à partir du numéro de fichier que n'existe pas
  }
}

void SD_record(float *DATA, int NbData, uint16_t secs, uint8_t sw_Val)
{

  if (OK_SDCARD)
  {
    if (!Open)
    {
      strcpy(filename, "log");
      sprintf(Num, "%lu", nb_file);
      strcat(filename, Num); strcat(filename, ".txt");
      
      dataFile = SD.open(filename, FILE_WRITE);
      
      nb_file++;
      Open = true;
      Closed = false;
      temps2 = millis();
    } else if (Open && !Closed)
    {
      temps_log = millis();
      dataFile.print(temps_log); dataFile.print(";");
      
      for( int i=0; i< NbData; i++)
      {
         
         //dataFile.print(DATA[i], 1); dataFile.print(";");
         dataFile.print((int16_t)(DATA[i]*10)); dataFile.print(";");
      }
      dataFile.println();

      if ( (millis() - temps2 > secs * 1000) ||  (last_sw_Val != sw_Val)  )
      {
        dataFile.close();
        Closed = true;
        Open = false;
      }
      last_sw_Val = sw_Val;

    }
  }
}
