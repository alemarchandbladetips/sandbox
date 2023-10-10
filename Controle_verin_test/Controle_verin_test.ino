#include <Adafruit_BNO055.h>
#include <SD.h>

#define PRINT_DEBUG 1

#define PIN_PLUS 7
#define PIN_MINUS 8

#define DT_US 20000

#define HYST_ANGLE 5

#define ALPHA_ANGLE 0.2
#define M_ANGLE 0.85

float k = 1/(1+2*M_ANGLE/ALPHA_ANGLE+(1/ALPHA_ANGLE)*(1/ALPHA_ANGLE));
float a1 = 2*M_ANGLE/ALPHA_ANGLE+2*(1/ALPHA_ANGLE)*(1/ALPHA_ANGLE);
float a2 = -(1/ALPHA_ANGLE)*(1/ALPHA_ANGLE);

int8_t motor_direction = 0;

Adafruit_BNO055 bno = Adafruit_BNO055();

float angle, angle_f, angle_f1, angle_f2 = 0;

int8_t SD_ok;
File dataFile;
bool Open = false, Closed = true;
String dataString;
char *filename;
uint32_t nb_file = 0, counter = 0;
char Num[8];

uint32_t timer,temps2,temps_log;


void setup() {
  // put your setup code here, to run once:
  pinMode(PIN_PLUS,OUTPUT);
  pinMode(PIN_MINUS,OUTPUT);

  digitalWrite(PIN_PLUS,0);
  digitalWrite(PIN_MINUS,0);

  if ( !bno.begin() )
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  SD_ok = SD.begin(BUILTIN_SDCARD);
  if ( !SD_ok )
  {
    Serial.println("Card failed, or not present");
  } else
  {
    digitalWrite(13,1);
    Serial.println("card initialized.");
    filename = (char*) malloc( strlen("log") + strlen(Num) + strlen(".txt") + 1) ;
    checkExist(); //vérifie existence du fichier et l'écriture commence à partir du numéro de fichier que n'existe pas
    pinMode(13,OUTPUT);
    digitalWrite(13,1);
  }

  timer = micros();

  
}

void loop() {
   if( micros()-timer > DT_US)
  {
    timer += DT_US;

    imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> gyr = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);


    angle = atan2(acc.y(),acc.z())*180/3.14;

    angle_f2 = angle_f1;
    angle_f1 = angle_f;

    angle_f = k*(angle + a1*angle_f1 + a2*angle_f2); 

    if(motor_direction == 0)
    {
      if(angle_f > HYST_ANGLE)
      {
        motor_direction = 1;
      }
      if(angle_f < -HYST_ANGLE )
      {
        motor_direction = -1;
      }
    } else if(motor_direction > 0)
    {
      if(angle_f < 0)
      {
        motor_direction = 0;
      }
      
    } else if(motor_direction < 0)
    {
      if(angle_f > 0)
      {
        motor_direction = 0;
      }
    }
    digitalWrite(PIN_PLUS,motor_direction > 0);
    digitalWrite(PIN_MINUS,motor_direction < 0);

    if(PRINT_DEBUG)
    {
      Serial.print(acc.x());Serial.print("\t");
      Serial.print(acc.y());Serial.print("\t");
      Serial.print(acc.z());Serial.print("\t");

      Serial.print(gyr.x());Serial.print("\t");
    
      Serial.print(angle);Serial.print("\t");
      Serial.print(angle_f);Serial.print("\t");

      Serial.print(motor_direction);Serial.print("\t");

      Serial.println("\t");
    }
    if (SD_ok) 
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

          dataFile.print(acc.x());dataFile.print(";");
          dataFile.print(acc.y());dataFile.print(";");
          dataFile.print(acc.z());dataFile.print(";");

          dataFile.print(gyr.x());dataFile.print(";");
          dataFile.print(gyr.y());dataFile.print(";");
          dataFile.print(gyr.z());dataFile.print(";");

          dataFile.println(" "); // gaffe à la dernière ligne
          
          if ((millis() - temps2 > 120.0 * 1000) )
          {
            dataFile.close();
            Closed = true;
            Open = false;
          }
       }
    }

  }
}

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
  Serial.print(filename);Serial.println(" created");
  nb_file = counter - 1;
}
