#include <arduinoFFT.h>

#include <SPI.h>
#include <SD.h>
#include <math.h>
#include <MPU6050_tockn.h>
#include <Wire.h>
#include "wiring_private.h" // pinPeripheral() function


// WARNING !!!! 2-10K pullup resistors are required on SDA and SCL, both go to 3.3V! You can use your oscilloscope to see the data traces

///////////////// declaration of I2C on pin 11(SDA) and 13(SCL) /////////////////////////
TwoWire myWire(&sercom1, 11, 13);

#define MCP4725_CMD_WRITEDAC            (0x40)
#define MCP4725_ADDR                    (0x62)

//////////////////////////////////////////////////////////////////
///////////////// Parameters /////////////////////////////////////

// Variance forgetting factor.
const float var_forget_factor = 0.02;

// Mean forgetting factor.
const float mean_forget_factor = 0.05;


//////////////////////////////////////////////////////////////////
///////////////// FFT vars /////////////////////////////////////
//
//arduinoFFT FFT = arduinoFFT(); /* Create FFT object */
//
//const uint16_t samples = 64; //This value MUST ALWAYS be a power of 2
//const double samplingFrequency = 100;
//
///*
//These are the input and output vectors
//Input vectors receive computed results from FFT
//*/
//double vReal[samples];
//double vImag[samples];
//
//int16_t samples_in_buffer = 0;
//int16_t max1, max2;
//
//double max_frequency;

//////////////////////////////////////////////////////////////////

MPU6050 mpu6050(Wire);
MPU6050 mpu6050_sat(myWire);

const int chipSelect = 4;
String filename;
int state, log_state, state2;
long click_time,timer,log_start_time,timestamp;
File dataFile;
int long_num = 1;

const int button_pin = 14;
const int led_pin = 15;

long t0,t1;

float acc_filt[3] = {0,0,0};
float acc_var[3] = {0,0,0};
float acc[3];
float acc_norm, acc_norm_var, acc_norm_filt, acc_var_norm = 0;

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

  delay(2000);

  mpu6050_sat.begin();
  //mpu6050_sat.calcGyroOffsets(true);

  delay(2000);

  mpu6050_sat.begin();
  //mpu6050_sat.calcGyroOffsets(true);
  
  Serial.println("MPU_started");
  timer = micros();

}

void loop() {
  // put your main code here, to run repeatedly:

  int16_t i,j;
  float var_tmp;

   mpu6050_sat.update();

  if(micros() - timer > 10000){
    
    timer = micros();
    timestamp = millis();

    acc[0] = mpu6050_sat.getAccX();
    acc[1] = mpu6050_sat.getAccY();
    acc[2] = mpu6050_sat.getAccZ();
    acc_norm = sqrt(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);
    acc_norm_filt = (1-mean_forget_factor)*acc_norm_filt + mean_forget_factor*acc_norm;
    acc_norm_var = (1-var_forget_factor)*acc_norm_var + var_forget_factor*((acc_norm-acc_norm_filt)*(acc_norm-acc_norm_filt));

    //Serial.print(10*acc_norm*9.81);Serial.print("\t");
    //Serial.print(10*acc_norm_filt*9.81);Serial.print("\t");
    Serial.print(sqrt(acc_norm_var)*1000);Serial.print("\t");

    //Serial.print(timer);Serial.print("\t");

//    if (samples_in_buffer < samples)
//    {
//      vReal[samples_in_buffer] = (acc_norm-acc_norm_filt);
//      vImag[samples_in_buffer] = 0;
//      samples_in_buffer ++;
//    } else
//    {
//      samples_in_buffer = 0;
//      FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
//      FFT.Compute(vReal, vImag, samples, FFT_FORWARD);
//      FFT.ComplexToMagnitude(vReal, vImag, samples);
//      //max_frequency = FFT.MajorPeak(vReal, samples, samplingFrequency);
//
//      max1 = find_max(vReal,samples/2);
//      vReal[max1] = 0;
//      max2 = find_max(vReal,samples/2);
//
//      //Serial.print(max(max1,max2)); Serial.print(" ");
//      //Serial.print(min(max1,max2)); Serial.print(" ");
//
////      for(j=0;j<samples/2;j++)
////      {
////        Serial.print(vReal[j]); Serial.print(" ");
////      }
//      //Serial.println(" ");
//      
//    }
//    //Serial.println(max_frequency);

    for (i=0;i<3;i++)
    {
      acc_filt[i] = (1-mean_forget_factor)*acc_filt[i] + mean_forget_factor*acc[i];
      var_tmp = (acc[i]-acc_filt[i])*(acc[i]-acc_filt[i]);
      acc_var[i] = (1-var_forget_factor)*acc_var[i] + var_forget_factor*var_tmp;

//      Serial.print(10*acc[i]*9.81);Serial.print("\t");
//      Serial.print(10*acc_filt[i]*9.81);Serial.print("\t");
//      Serial.print(sqrt(acc_var[i])*1000);Serial.print("\t");
    }

    acc_var_norm = sqrt((sqrt(acc_var[0])*sqrt(acc_var[0])) + (sqrt(acc_var[1])*sqrt(acc_var[1])) + (sqrt(acc_var[2])*sqrt(acc_var[2])));

    Serial.print(500*sqrt(acc_var[0])/acc_var_norm);Serial.print("\t");
    Serial.print(500*sqrt(acc_var[1])/acc_var_norm);Serial.print("\t");
    Serial.print(500*sqrt(acc_var[2])/acc_var_norm);Serial.print("\t");
   
    Serial.println(" ");
  }
}

//int16_t find_max(double *input_vect, int16_t vector_size)
//{
//  int16_t i;
//  double current_max = input_vect[0];
//  int16_t idx_current_max = 0;
//  
//  
//  for (i=1;i<vector_size;i++)
//  {
//    if(input_vect[i] > current_max)
//    {
//      current_max = input_vect[i];
//      idx_current_max = i;
//    }
//  }
//
//  return idx_current_max;
//}

