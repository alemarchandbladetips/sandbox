#include <SPI.h>
#include <SD.h>
#include <MPU6050_tockn.h>
#include <Wire.h>
#include "wiring_private.h" // pinPeripheral() function

// WARNING !!!! 2-10K pullup resistors are required on SDA and SCL, both go to 3.3V! You can use your oscilloscope to see the data traces

///////////////// declaration of I2C on pin 11(SDA) and 13(SCL) /////////////////////////
TwoWire myWire(&sercom1, 11, 13);

#define MCP4725_CMD_WRITEDAC            (0x40)
#define MCP4725_ADDR                    (0x62)

//////////////////////////////////////////////////////////////////
////////////////// MPUs definitions //////////////////////////////

MPU6050 mpu6050(Wire);
MPU6050 mpu6050_sat(myWire);

//////////////////////////////////////////////////////////////////
///////////////////// log handeling //////////////////////////////

const int chipSelect = 4;
String filename;
int state, log_state, state2;
long blink_timer;
long blink_duration;
int8_t led_status;

long click_time,timer,log_start_time,timestamp;
File dataFile;
int long_num = 1;
int32_t log_line = 0;

const int button_pin = 14;
const int led_pin = 15;

//////////////////////////////////////////////////////////////////
////////////////// Variance computations ////////////////////////

float acc_filt[3] = {0,0,0};
float acc_var[3] = {0,0,0};
float acc[3];
float acc_norm, acc_norm_var, acc_norm_filt, acc_var_norm = 0;

// Variance forgetting factor.
const float var_forget_factor = 0.02;

// Mean forgetting factor.
const float mean_forget_factor = 0.1;

//////////////////////////////////////////////////////////////////
////////////////////////// Gyro analize //////////////////////////

float gyr_filt = 0;
float gyr_filtx = 0;

// Mean forgetting factor.
const float gyr_forget_factor = 0.1;

int8_t gyro_trigger;
const float gyro_trigger_speed = 1;
float angle = 0;

float loc_max, loc_max_valid = 0;
float loc_min, loc_min_valid = 0;
int8_t max_search_status = 0;
float amplitude = 0;
float frequency = 0;
int32_t gyro_count = 0;

//////////////////////////////////////////////////////////////////

long t0,t1;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(led_pin,OUTPUT);
  pinMode(button_pin,INPUT);
  state = 0;
  state2 = 0;
  log_state = 0;

  Wire.begin();
  myWire.begin();

   // Assign pins 13 & 11 to SERCOM functionality
  pinPeripheral(11, PIO_SERCOM);
  pinPeripheral(13, PIO_SERCOM);

  
//  while (!Serial) {
//    ; // wait for serial port to connect. Needed for native USB port only
//  }

  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");

  mpu6050.begin();
  mpu6050_sat.begin();
  
  delay(1500);
  
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  mpu6050_sat.begin();

  blink_timer = millis();
  digitalWrite(led_pin,0);
  led_status = 0;
  blink_duration = 250;
}

void loop() {
  // put your main code here, to run repeatedly:

  int8_t i;
 float var_tmp;



  if( (micros() - timer > 10000) && (log_state == 1)){

    float dt = (float)(micros() - timer);
    timer = micros();
    //Serial.println(dt);

    mpu6050.update();
    mpu6050_sat.update();
    timestamp = millis();
//
//    acc[0] = 9.81*mpu6050_sat.getAccX();
//    acc[1] = 9.81*mpu6050_sat.getAccY();
//    acc[2] = 9.81*mpu6050_sat.getAccZ();

    acc[0] = 9.81*mpu6050.getAccX();
    acc[1] = 9.81*mpu6050.getAccY();
    acc[2] = 9.81*mpu6050.getAccZ();

    
    acc_norm = sqrt(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);
    acc_norm_filt = (1-mean_forget_factor)*acc_norm_filt + mean_forget_factor*acc_norm;
    acc_norm_var = (1-var_forget_factor)*acc_norm_var + var_forget_factor*((acc_norm-acc_norm_filt)*(acc_norm-acc_norm_filt));

    for (i=0;i<3;i++)
    {
      acc_filt[i] = (1-mean_forget_factor)*acc_filt[i] + mean_forget_factor*acc[i];
      var_tmp = (acc[i]-acc_filt[i])*(acc[i]-acc_filt[i]);
      acc_var[i] = (1-var_forget_factor)*acc_var[i] + var_forget_factor*var_tmp;
    }

    gyr_filt = (1-gyr_forget_factor)*gyr_filt + gyr_forget_factor*mpu6050.getGyroY();
    gyr_filtx = (1-gyr_forget_factor)*gyr_filtx + gyr_forget_factor*mpu6050.getGyroX();

//    if(abs(gyr_filt)>gyro_trigger_speed)
//    {
//      gyro_trigger = 100;
//      //Serial.println("1");
//    } else if(gyro_trigger>0)
//    {
//      gyro_trigger --;
//      //Serial.println("2");
//    } else
//    {
//      angle = 0;
//      loc_max = 0;
//      loc_max_valid = 0;
//      loc_min = 0;
//      loc_min_valid = 0;
//      max_search_status = 0;
//      amplitude = 0;
//      frequency = 0;
//      //Serial.println("3");
//    }

    angle += dt*gyr_filt/1000000;
    angle = 0.975*angle - 0.025*atan2(acc_filt[0],acc_filt[2])*57.296;

//    if(gyro_trigger>0)
//    {
//      angle += dt*mpu6050.getGyroY()/1000000;
//      //Serial.println("4");
//      if(angle>loc_max && max_search_status!=1)
//      {
//        loc_max = angle;
//      } else if(angle<0.9*loc_max && max_search_status!=1)
//      {
//        loc_max_valid = loc_max;
//        loc_max = -1000;
//        if(max_search_status!=0)
//        {
//          frequency = 500.0/(millis()-gyro_count);
//          amplitude = loc_max_valid-loc_min_valid;
//        }
//        max_search_status = 1;
//        gyro_count = millis();
//      }
//
//      if(angle<loc_min && max_search_status!=-1)
//      {
//        loc_min = angle;
//      } else if(angle>0.9*loc_min && max_search_status!=-1)
//      {
//        loc_min_valid = loc_min;
//        loc_min = 1000;
//        if(max_search_status!=0)
//        {
//          frequency = 500.0/(millis()-gyro_count);
//          amplitude = loc_max_valid-loc_min_valid;
//        }
//        max_search_status = -1;
//        gyro_count = millis();
//      }
//    }

    log_line++;

//    Serial.print(-atan2(acc_filt[0],acc_filt[2])*57.296);Serial.print("\t");
//    Serial.print(angle);Serial.print("\t");

//    Serial.print(log_line);Serial.print("\t");
//
//    Serial.print(acc[0]);Serial.print("\t");
//    Serial.print(acc[1]);Serial.print("\t");
//    Serial.print(acc[2]);Serial.print("\t");
//    Serial.print(acc_norm);Serial.print("\t");
//    
//    Serial.print(acc_filt[0]);Serial.print("\t");
//    Serial.print(acc_filt[1]);Serial.print("\t");
//    Serial.print(acc_filt[2]);Serial.print("\t");
//    Serial.print(acc_norm_filt);Serial.print("\t");
//    Serial.print(mpu6050.getGyroX());Serial.print("\t");
//    Serial.print(mpu6050.getGyroY());Serial.print("\t");
//    Serial.print(mpu6050.getGyroZ());Serial.print("\t");
//
//    Serial.print(acc_norm_filt);Serial.print("\t");
//    Serial.print(acc_norm);Serial.print("\t");
//    Serial.print(sqrt(acc_norm_var)*10);Serial.print("\t");
//    Serial.print(10*sqrt(acc_var[1])/sqrt(acc_var[0]));Serial.print("\t");
//    
//    Serial.print(gyr_filt);Serial.print("\t");
//    Serial.print(angle);Serial.print("\t");
//    Serial.print(amplitude);Serial.print("\t");
//    Serial.print(frequency);Serial.print("\t");

    dataFile.print(log_line);dataFile.print("\t");

    dataFile.print(acc[0]);dataFile.print("\t");
    dataFile.print(acc[1]);dataFile.print("\t");
    dataFile.print(acc[2]);dataFile.print("\t");
    dataFile.print(acc_norm);dataFile.print("\t");
    
    dataFile.print(acc_filt[0]);dataFile.print("\t");
    dataFile.print(acc_filt[1]);dataFile.print("\t");
    dataFile.print(acc_filt[2]);dataFile.print("\t");
    dataFile.print(acc_norm_filt);dataFile.print("\t");
    
    dataFile.print(mpu6050.getGyroX());dataFile.print("\t");
    dataFile.print(mpu6050.getGyroY());dataFile.print("\t");
    dataFile.print(mpu6050.getGyroZ());dataFile.print("\t");
    dataFile.print(gyr_filtx);dataFile.print("\t");
    dataFile.print(gyr_filt);dataFile.print("\t");
    
    
    //dataFile.print(sqrt(acc_norm_var)*10);dataFile.print("\t");
    //dataFile.print(10*sqrt(acc_var[1])/sqrt(acc_var[2]));dataFile.print("\t");
    
    //dataFile.print(gyr_filt);dataFile.print("\t");
    //dataFile.print(gyr_filtx);dataFile.print("\t");
    dataFile.print(angle);dataFile.print("\t");
    //dataFile.print(amplitude);dataFile.print("\t");
    //dataFile.print(frequency);dataFile.print("\t");

    Serial.println(" "); 
    dataFile.println(" "); 
  }

  if ((millis()-blink_timer > blink_duration) && (blink_duration!=0))
  {
    blink_timer = millis();
    led_status = !led_status;
    digitalWrite(led_pin,led_status);
  }

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
    blink_duration = 50;
    if(log_state == 0)
    {
      Serial.print("Opening file ... ");
      filename = "logmpu";
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
        log_line = 0;
      }else
      {
        Serial.println("Opening failed");
      }
      
      //dataFile.println("Timestamp(us) GyroX(dps) GyroY(dps) GyroZ(dps)");
      timer = micros();

    } else
    {
      Serial.print("Closing file ... ");
      dataFile.close();
      Serial.println("file closed");
      log_state = 0;
      blink_duration = 250;
    }
  }
  //digitalWrite(led_pin,log_state);

  if((log_state == 1) && ((millis() - log_start_time) > 120000))
  {
    dataFile.close();
    Serial.print("Opening file ... ");
    filename = "logmpu";
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
      log_line = 0;
    }else
    {
      Serial.println("Opening failed");
    }
    
    dataFile.println("Timestamp(us) GyroX(dps) GyroY(dps) GyroZ(dps)");
    timer = micros();

  }

   
}
