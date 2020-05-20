
#include <inttypes.h>
#include <Wire.h>
#include "Arduino.h"
#include "SDP3x.h"

#define SDP_DEBUG

#ifdef SDP_DEBUG
#define DEBUG_PRINT(label, value) \
  Serial.print(" ["); \
  Serial.print(label); Serial.print(" = "); Serial.print(value); \
  Serial.print("] ");

#else
#define DEBUG_PRINT(label, value)
#endif
/******************************************************************************
 * Global Functions
 ******************************************************************************/


 /******************************************************************************
  * setI2CAddress
  *  Changes I2C address
  *
  * @param i2cAddress - the I2C address to use
  ******************************************************************************/
 void SDP3xClass::setI2CAddress(uint8_t i2cAddress)
 {
   mI2CAddress = i2cAddress;
 }

/**********************************************************
 * getPressureDiff
 *  Gets the current Pressure Differential from the sensor.
 *
 * @return float - The Pressure in Pascal
 ********************************************/
 void SDP3xClass::getInfo_PT(void)
 {
    uint8_t txData[COMMAND_DATA_LENGTH] =
    {SDP_MEASUREMENT_COMMAND_0, SDP_MEASUREMENT_COMMAND_1};

  if (!OK_Request)
  {
      Wire.beginTransmission(mI2CAddress);
      Wire.write(txData, COMMAND_DATA_LENGTH);
      Wire.endTransmission();
      OK_Request = true;
      temps_exec = millis();
     
  }
  
  dt_exec = millis() - temps_exec;
  if (dt_exec > 10)
  {    
    temps_exec += dt_exec; // decommenté si "OK_Request = false" est commenté
    int16_t dp_ticks;
    int16_t dp_scale;
    uint8_t readData[RESULT_DATA_LENGTH] = { 0 };
  
    readSensor(readData, RESULT_DATA_LENGTH);
    // merge chars to one int
    dp_ticks = (int16_t) BIU16(readData, 0);
    dp_scale = BIU16(readData, 6);
    scale_dP_SDP3x = dp_scale;
    dP_SDP3x = dp_ticks/(float)dp_scale;
    
    int16_t  temperature_ticks;
    temperature_ticks = BIU16(readData, 3);
    float t_scale = 200.0;
    temperature_SDP3x = temperature_ticks/t_scale;
  }
  
 }  

float SDP3xClass::getdP_SDP3X(void)
  {
      return dP_SDP3x;
  }

uint8_t SDP3xClass::readSensor(uint8_t* readData, uint8_t size)
{
  uint8_t rxByteCount=0;

  Wire.requestFrom((uint8_t)mI2CAddress, size);
  
  //wait a little
  uint32_t temps_wait = micros();
  while ( (micros() - temps_wait) < 100 ) {}
  
  if (Wire.available() == size)
  {
    for (rxByteCount=0; rxByteCount<size ;rxByteCount++)
    {
        readData[rxByteCount] = Wire.read();
    }
  }
  else
  {
      rxByteCount = Wire.available();
  }
  //OK_Request = false; //force to request data from Sensor SDP3x
  return rxByteCount;
}

SDP3xClass SDP3x;
