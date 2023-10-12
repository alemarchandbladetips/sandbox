#include <Adafruit_BNO055.h>
#include <SD.h>
#include "functions.h"
#include "GlobalVariables.h"

#define PRINT_DEBUG 0

#define PIN_PLUS 9
#define PIN_MINUS 10

#define RAD2DEG 57.2957795
#define DEG2RAD 0.01745329252

#define DT_US 20000
#define DT_S 0.02

#define HYST_ANGLE 3*DEG2RAD

#define ALPHA_ACC 0.05
#define M_ACC 1

#define ALPHA_GYR 0.5
#define M_GYR 1


#define OFF_NACELLE -7.25*DEG2RAD
#define OFF_CHASSIS 2.1*DEG2RAD

float alpha_Kalman = 0.01;

float angle_nacelle, angle_chassis = 0;
float angle_nacelle_f, angle_chassis_f = 0;
float angle_parallelogramme = 0;
float v_gps, courbure_est, acc_courbure_est, angle_courbure_est, angle_err;


float acc_nacelle[3], acc_nacelle_f[3], acc_nacelle_f1[3], acc_nacelle_f2[3] = {0,0,0};
float acc_chassis[3], acc_chassis_f[3], acc_chassis_f1[3], acc_chassis_f2[3] = {0,0,0};

float gyr_nacelle[3], gyr_nacelle_f[3], gyr_nacelle_f1[3], gyr_nacelle_f2[3] = {0,0,0};
float gyr_chassis[3], gyr_chassis_f[3], gyr_chassis_f1[3], gyr_chassis_f2[3] = {0,0,0};

float k_acc = 1/(1+2*M_ACC/ALPHA_ACC+(1/ALPHA_ACC)*(1/ALPHA_ACC));
float a1_acc = 2*M_ACC/ALPHA_ACC+2*(1/ALPHA_ACC)*(1/ALPHA_ACC);
float a2_acc = -(1/ALPHA_ACC)*(1/ALPHA_ACC);

float k_gyr = 1/(1+2*M_GYR/ALPHA_GYR+(1/ALPHA_GYR)*(1/ALPHA_GYR));
float a1_gyr = 2*M_GYR/ALPHA_GYR+2*(1/ALPHA_GYR)*(1/ALPHA_GYR);
float a2_gyr = -(1/ALPHA_GYR)*(1/ALPHA_GYR);

int8_t motor_direction = 0;

//// BN0s ////
Adafruit_BNO055 bno_nacelle = Adafruit_BNO055(-1,BNO055_ADDRESS_A,&Wire);
Adafruit_BNO055 bno_chassis = Adafruit_BNO055(-1,BNO055_ADDRESS_A,&Wire1);

//// SD ////
int8_t SD_ok;
File dataFile;
bool Open = false, Closed = true;
String dataString;
char *filename;
uint32_t nb_file = 0, counter = 0;
char Num[8];

//// GPS ////
const int NbXB_RX = 60;
uint8_t START1 = 137, START2 = 157, STOP = 173;

//GPS DATA from T40
RxTxSerial GPS_DATA(Serial1,true);
int16_t DATA_GPS_RX[NbXB_RX];
float vx_gps, vy_gps, vz_gps;
float x_gps, y_gps, z_gps;
int16_t  GPS_init_done = 0;

//// timers ////

uint32_t timer,temps2,temps_log,enlapsed_time, timer_step;
uint32_t counter_led = 0;
uint32_t led_period = 20;
uint8_t led_status = 0;


void setup() {
  // put your setup code here, to run once:
  pinMode(PIN_PLUS,OUTPUT);
  pinMode(PIN_MINUS,OUTPUT);

  digitalWrite(PIN_PLUS,0);
  digitalWrite(PIN_MINUS,0);

  pinMode(13,OUTPUT);
  digitalWrite(13,0);

  Serial1.begin(57600); //GPS

  if ( !bno_nacelle.begin(OPERATION_MODE_ACCGYRO) )
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("BNO nacelle failed to initialize");
    while (1);
  } else
  {
    Serial.println("BNO nacelle initialized");
  }

  digitalWrite(13,1);

  if ( !bno_chassis.begin(OPERATION_MODE_ACCGYRO) )
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("BNO chassis failed to initialize");
    while (1);
  }else
  {
    Serial.println("BNO chassis initialized");
  }
  
  digitalWrite(13,0);

  SD_ok = SD.begin(BUILTIN_SDCARD);
  if ( !SD_ok )
  {
    Serial.println("Card failed, or not present");
  } else
  {
    digitalWrite(13,1);
    Serial.println("card initialized.");
    filename = (char*) malloc( strlen("log_proto_remorque_") + strlen(Num) + strlen(".txt") + 1) ;
    checkExist(); //vérifie existence du fichier et l'écriture commence à partir du numéro de fichier que n'existe pas
  }

  digitalWrite(13,0); delay(0.5);digitalWrite(13,1); delay(0.5);
  digitalWrite(13,0); delay(0.5);digitalWrite(13,1); delay(0.5);
  digitalWrite(13,0); delay(0.5);

  timer = micros();
  
  timer_step = millis();
  
}

void loop() {
  scan_GPS_serial_port();
  enlapsed_time = micros()-timer;

   if( enlapsed_time > DT_US)
  {
    timer += DT_US;

    ///////////////////////////////////////////////////////////////////////////////////////////////////
    //////// Récupération des données IMU et changement de repère

    imu::Vector<3> acc_nacelle = bno_nacelle.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> gyr_nacelle = bno_nacelle.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

    imu::Vector<3> acc_chassis = bno_chassis.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> gyr_chassis = bno_chassis.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

    acc_nacelle[0] = -acc_nacelle.x(); acc_nacelle[1] = -acc_nacelle.y(); acc_nacelle[2] = acc_nacelle.z();
    acc_chassis[0] = acc_chassis.x(); acc_chassis[1] = acc_chassis.y(); acc_chassis[2] = acc_chassis.z();

    gyr_nacelle[0] = -gyr_nacelle.x(); gyr_nacelle[1] = -gyr_nacelle.y(); gyr_nacelle[2] = gyr_nacelle.z();
    gyr_chassis[0] = gyr_chassis.x(); gyr_chassis[1] = gyr_chassis.y(); gyr_chassis[2] = gyr_chassis.z();

    ///////////////////////////////////////////////////////////////////////////////////////////////////
    //////// Filytrage des données IMU et calcul des angles corespondants

    for(int i=0;i<3;i++)
    {
      acc_nacelle_f2[i] = acc_nacelle_f1[i];
      acc_nacelle_f1[i] = acc_nacelle_f[i];

      acc_nacelle_f[i] = k_acc*(acc_nacelle[i] + a1_acc*acc_nacelle_f1[i] + a2_acc*acc_nacelle_f2[i]); 

      acc_chassis_f2[i] = acc_chassis_f1[i];
      acc_chassis_f1[i] = acc_chassis_f[i];

      acc_chassis_f[i] = k_acc*(acc_chassis[i] + a1_acc*acc_chassis_f1[i] + a2_acc*acc_chassis_f2[i]);


      gyr_nacelle_f2[i] = gyr_nacelle_f1[i];
      gyr_nacelle_f1[i] = gyr_nacelle_f[i];

      gyr_nacelle_f[i] = k_gyr*(gyr_nacelle[i] + a1_gyr*gyr_nacelle_f1[i] + a2_gyr*gyr_nacelle_f2[i]); 

      gyr_chassis_f2[i] = gyr_chassis_f1[i];
      gyr_chassis_f1[i] = gyr_chassis_f[i];

      gyr_chassis_f[i] = k_gyr*(gyr_chassis[i] + a1_gyr*gyr_chassis_f1[i] + a2_gyr*gyr_chassis_f2[i]);
    }

    angle_nacelle_f = atan2(acc_nacelle_f[1],acc_nacelle_f[2])+OFF_NACELLE;
    angle_chassis_f = atan2(acc_chassis_f[1],acc_chassis_f[2])+OFF_CHASSIS;

    ///////////////////////////////////////////////////////////////////////////////////////////////////
    //////// Kalman sur l'angle parallélogramme

    angle_nacelle = atan2(acc_nacelle[1],acc_nacelle[2])+OFF_NACELLE;
    angle_chassis = atan2(acc_chassis[1],acc_chassis[2])+OFF_CHASSIS;

    angle_parallelogramme += (gyr_nacelle[0] - gyr_chassis[0]) * DT_S * DEG2RAD;
    angle_parallelogramme = (1-alpha_Kalman)*angle_parallelogramme + alpha_Kalman*(angle_nacelle-angle_chassis);

    ///////////////////////////////////////////////////////////////////////////////////////////////////
    //////// Estimation de la courbure et de l'angle de consigne

    v_gps = sqrtf(vx_gps*vx_gps+vy_gps*vy_gps);
    if(v_gps > 0.5)
    {
      courbure_est = gyr_chassis_f[2]*DEG2RAD/v_gps;
      acc_courbure_est = gyr_chassis_f[2]*DEG2RAD*v_gps;
    } else
    {
      courbure_est = 0;
      acc_courbure_est = 0;
    }

    angle_courbure_est = -atan2(acc_courbure_est,9.81);

    angle_err = (-1.435*angle_courbure_est+angle_parallelogramme);//+1.435*angle_nacelle_f);

    if(motor_direction == 0)
    {
      if(angle_err > HYST_ANGLE)
      {
        motor_direction = -1;
      }
      if(angle_err < -HYST_ANGLE )
      {
        motor_direction = 1;
      }
    } else if(motor_direction < 0)
    {
      if(angle_err < 0)
      {
        motor_direction = 0;
      }
      
    } else if(motor_direction > 0)
    {
      if(angle_err > -0)
      {
        motor_direction = 0;
      }
    }
    digitalWrite(PIN_PLUS,motor_direction > 0);
    digitalWrite(PIN_MINUS,motor_direction < 0);


    if(PRINT_DEBUG)
    {
      /*
      Serial.print(enlapsed_time/1000.0);Serial.print("\t");

      // IMU nacelle
      Serial.print(acc_nacelle[0],2);Serial.print("\t");
      Serial.print(acc_nacelle[1],2);Serial.print("\t");
      Serial.print(acc_nacelle[2],2);Serial.print("\t");

      Serial.print(gyr_nacelle.x(),2);Serial.print("\t");
      Serial.print(gyr_nacelle.y(),2);Serial.print("\t");
      Serial.print(gyr_nacelle.z(),2);Serial.print("\t");

      // IMU chassis
      Serial.print(acc_chassis[0],2);Serial.print("\t");
      Serial.print(acc_chassis[1],2);Serial.print("\t");
      Serial.print(acc_chassis[2],2);Serial.print("\t");

      Serial.print(gyr_chassis.x(),2);Serial.print("\t");
      Serial.print(gyr_chassis.y(),2);Serial.print("\t");
      Serial.print(gyr_chassis.z(),2);Serial.print("\t");
*/
      // GPS
      Serial.print(x_gps,2);Serial.print("\t");
      Serial.print(y_gps,2);Serial.print("\t");

      Serial.print(vx_gps,2);Serial.print("\t");
      Serial.print(vy_gps,2);Serial.print("\t");

      Serial.print(GPS_init_done,2);Serial.print("\t");

Serial.print(gyr_chassis.x(),2);Serial.print("\t");
Serial.print(gyr_nacelle.x(),2);Serial.print("\t");
      Serial.print(angle_nacelle_f*RAD2DEG,2);Serial.print("\t");
      Serial.print(angle_chassis_f*RAD2DEG,2);Serial.print("\t");
      Serial.print(angle_nacelle*RAD2DEG,2);Serial.print("\t");
      Serial.print(angle_chassis*RAD2DEG,2);Serial.print("\t");
      Serial.print(angle_parallelogramme*RAD2DEG,2);Serial.print("\t");
      Serial.print(acc_courbure_est,2);Serial.print("\t");
      Serial.print(angle_courbure_est,2);Serial.print("\t");
      Serial.print(angle_err*RAD2DEG,2);Serial.print("\t");
      Serial.print(motor_direction);Serial.print("\t");

      Serial.println("\t");
    }
    if (SD_ok) 
    {
      if(GPS_init_done)
      {
        led_period = 5;
      } else
      {
        led_period = 20;
        
      }
      if(counter_led>led_period)
      {
        counter_led = 0;
        led_status = !led_status;
      }
      counter_led ++;
      digitalWrite(13,led_status);
        if (!Open) 
        {
          strcpy(filename, "log_proto_remorque_");
          sprintf(Num, "%lu", nb_file);
          strcat(filename, Num); strcat(filename, ".txt");
          
          dataFile = SD.open(filename, FILE_WRITE);
          Serial.print(filename);Serial.println(" created");
          
          nb_file++;
          Open = true;
          Closed = false;
          temps2 = millis();
       } else if (Open && !Closed) 
       {
          temps_log = millis();
          dataFile.print(temps_log); dataFile.print(";");

          dataFile.print(acc_nacelle.x(),2);dataFile.print(";");
          dataFile.print(acc_nacelle.y(),2);dataFile.print(";");
          dataFile.print(acc_nacelle.z(),2);dataFile.print(";");

          dataFile.print(gyr_nacelle.x(),2);dataFile.print(";");
          dataFile.print(gyr_nacelle.y(),2);dataFile.print(";");
          dataFile.print(gyr_nacelle.z(),2);dataFile.print(";");

          dataFile.print(acc_chassis.x(),2);dataFile.print(";");
          dataFile.print(acc_chassis.y(),2);dataFile.print(";");
          dataFile.print(acc_chassis.z(),2);dataFile.print(";");

          dataFile.print(gyr_chassis.x(),2);dataFile.print(";");
          dataFile.print(gyr_chassis.y(),2);dataFile.print(";");
          dataFile.print(gyr_chassis.z(),2);dataFile.print(";");

          dataFile.print(x_gps,2);dataFile.print(";");
          dataFile.print(y_gps,2);dataFile.print(";");

          dataFile.print(vx_gps,2);dataFile.print(";");
          dataFile.print(vy_gps,2);dataFile.print(";");

          dataFile.print(motor_direction);dataFile.print(";");
          dataFile.print(angle_nacelle_f*RAD2DEG,2);dataFile.print(";");
          dataFile.print(angle_chassis_f*RAD2DEG,2);dataFile.print(";");
          dataFile.print(angle_parallelogramme*RAD2DEG);dataFile.print(";");
          dataFile.print(acc_courbure_est,2);dataFile.print(";");
          dataFile.print(angle_courbure_est*RAD2DEG,2);dataFile.print(";");
          dataFile.print(angle_err*RAD2DEG,2);dataFile.print(";");


          dataFile.println(" "); // gaffe à la dernière ligne
          
          if ((millis() - temps2) > 120000 )
          {
            dataFile.close();
            Serial.print(filename);Serial.println(" closed");
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
    strcpy(filename, "log_proto_remorque_");
    sprintf(Num, "%lu", counter);
    strcat(filename, Num); strcat(filename, ".txt");
    counter++;
  }
  while ( SD.exists(filename) );
  Serial.print(filename);Serial.println(" created");
  nb_file = counter - 1;
}

void scan_GPS_serial_port()
{
  //Serial.println(Serial1.available());
  if ( GPS_DATA.getData_int16_t( 7, START1, STOP, DATA_GPS_RX) > 0 )
  {
    //Serial.println("............................................");
    x_gps = DATA_GPS_RX[0]/10.0;
    y_gps = DATA_GPS_RX[1]/10.0;
    z_gps = DATA_GPS_RX[2]/10.0;
    vx_gps = DATA_GPS_RX[3]/100.0;
    vy_gps = DATA_GPS_RX[4]/100.0;
    vz_gps = DATA_GPS_RX[5]/100.0;
    GPS_init_done = DATA_GPS_RX[6];
  }
}
