#include "Functions.h"
#include "stdint.h"


/***** Fonctions *******/

//Moyenne des valuers
 void Mean(float A[], float* prom, int NbData, int posf)
 {
      float sum=0;
      int vali=posf+1-NbData;
      int valf=posf+1;
      for(int k=vali;k<valf;k++){
        sum+=A[k];
        }
      *prom=sum/NbData;      
 }
 
// Function Historique des valeurs
void HistoriqueVal(float val, float A[])
{
      for(int k=0;k<(H-1);k++){
        A[k]=A[k+1];
        
      }
      A[H-1]=val;
}

void HistoriqueN(float val, float A[], uint8_t N)
{
      for(int k=0;k<(N-1);k++){
        A[k]=A[k+1];
        
      }
      A[N-1]=val;
}

float signo(float val)
{   
    if(val<0)
    {
        return -1;
    }
    else
    {
        return 1;
    }
}

float ang_180(float a1)
{
    float temp = fmodf(a1,360.0);
    if(temp > 180)
    { 
      temp -= 360.0;
    }
    else if (temp < -180)
    {
       temp += 360.0;
    }
    return temp;
}

float ang_pi(float a1)
{
    float temp = fmodf(a1,2*pi);
    if(temp > pi)
    { 
      temp -= 2*pi;
    }
    else if (temp < -pi)
    {
       temp += 2*pi;
    }
    return temp;
}

float lineVal(float x_i, float x_f, float y_i, float y_f, float x)
{
  float y;
  if (x > x_f)
  {
    y = y_f;
  }
  else if ( x < x_i)
  {
    y = y_i;
  }
  else
  {
    y = (y_f - y_i) / (x_f - x_i) * (x - x_i) + y_i;
  }

  return y;
}

// CheckSum function: provided by SBG
uint16_t calcCRC(const void *pBuffer, uint16_t bufferSize)
{
  const uint8_t *pBytesArray =  (const uint8_t*)pBuffer;
  uint16_t poly = 0x8408;
  uint16_t crc = 0;
  uint8_t carry;
  uint8_t i_bits;
  uint16_t j;
  for (j =0; j < bufferSize; j++)
  { 
    crc = crc ^ pBytesArray[j];
    for (i_bits = 0; i_bits < 8; i_bits++)
    {
      carry = crc & 1;
      crc = crc / 2;
      if (carry)
      {
        crc = crc^poly;
      } 
    }
  }
  return crc; 
}

void send_in16_t(Stream &myPort, int16_t *N)
{  
   myPort.write( *( (uint8_t *) N) ); 
   myPort.write( *( (uint8_t *) N + 1) );
}

void sendData_int16_t(Stream &myPort, uint8_t Ini, uint8_t Fin, int16_t *N, int NbData)
{
      myPort.write(Ini);
      for (int i =0; i< NbData; i++)
      {
          send_in16_t(myPort, N+i);
      }
      int16_t Checksum = calcCRC(N, 2*NbData);
      send_in16_t(myPort, &Checksum);
      myPort.write(Fin);
}
