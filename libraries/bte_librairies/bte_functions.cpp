#include "bte_functions.h"
#include "stdint.h"


/***** Fonctions *******/

//Moyenne des valuers
 void bte_Mean(float A[], float* prom, int NbData, int posf)
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
void bte_HistoriqueVal(float val, float A[], int NbData)
{
      for(int k=0;k<(NbData-1);k++){
        A[k]=A[k+1];
        
      }
      A[NbData-1]=val;
}

///Filtre median avec int16
uint16_t bte_medianFilter16(uint16_t val, uint16_t XX[], uint8_t Elem)
{
  uint8_t i, j;
  uint16_t S[Elem];
  //Historique des valeurs stocké dans X
  for (i = 0; i < (Elem - 1) ; i++) {
    XX[i] = XX[i + 1];
  }
  XX[Elem - 1] = val;

  for (i = 0; i < Elem ; i++) {
    S[i] = XX[i];
  }

  //Rangement des valeurs (min au max)
  uint16_t temp;
  for ( i = 0; i < (Elem-1); i++)
  {
    boolean change = false;

    for ( j = 1; j < (Elem - i); j++)
    {
      temp = S[j - 1];
      if ( temp > S[j] )
      {
        S[j - 1] = S[j];
        S[j] = temp;
        change = true;
      }
    }
    if (change == false) {
        i=Elem;
        //break;  //ceci implique que tous les vals sont ordonnées
    }
  }

  return S[(Elem - 1) / 2]; // valeur au millieu
}

///Filtre median avec float
float bte_medianFilter(float val, float XX[], uint8_t Elem)
{
  uint8_t i, j;
  float S[Elem];
  //Historique des valeurs stocké dans X
  for (i = 0; i < (Elem - 1) ; i++) {
    XX[i] = XX[i + 1];
  }
  XX[Elem - 1] = val;

  for (i = 0; i < Elem ; i++) {
    S[i] = XX[i];
  }

  //Rangement des valeurs (min au max)
  float temp;
  for ( i = 0; i < (Elem-1); i++)
  {
    boolean change = false;

    for ( j = 1; j < (Elem - i); j++)
    {
      temp = S[j - 1];
      if ( temp > S[j] )
      {
        S[j - 1] = S[j];
        S[j] = temp;
        change = true;
      }
    }
    if (change == false) {
        i=Elem;
        //break;  //ceci implique que tous les vals sont ordonnées
    }
  }

  return S[(Elem - 1) / 2]; // valeur au millieu
}

void bte_HistoriqueN(float val, float A[], uint8_t N)
{
      for(int k=0;k<(N-1);k++){
        A[k]=A[k+1];
        
      }
      A[N-1]=val;
}

float bte_signo(float val)
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

float bte_ang_180(float a1)
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

float bte_ang_360(float a1)
{
    float temp = fmodf(a1,360.0);
    if(temp > 360)
    { 
      temp -= 360.0;
    }
    else if (temp < 0)
    {
       temp += 360.0;
    }
    return temp;
}

float bte_ang_pi(float a1)
{
    float temp = fmodf(a1,2*PI);
    if(temp > PI)
    { 
      temp -= 2*PI;
    }
    else if (temp < -PI)
    {
       temp += 2*PI;
    }
    return temp;
}

float bte_lineVal(float x_i, float x_f, float y_i, float y_f, float x)
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
uint16_t bte_calcCRC(const void *pBuffer, uint16_t bufferSize)
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

void bte_send_in16_t(Stream &myPort, int16_t *N)
{  
   myPort.write( *( (uint8_t *) N) ); 
   myPort.write( *( (uint8_t *) N + 1) );
}

void bte_sendData_int16_t(Stream &myPort, uint8_t Ini, uint8_t Fin, int16_t *N, int NbData, bool verif)
{
      myPort.write(Ini);
      for (int i =0; i< NbData; i++)
      {
          bte_send_in16_t(myPort, N+i);
      }
      if (verif)
      {
        int16_t Checksum = bte_calcCRC(N, 2*NbData);
        bte_send_in16_t(myPort, &Checksum); 
      }
      
      myPort.write(Fin);
}

bool bte_recep_int16_t(Stream &myPort, int NbData, uint8_t Ini, uint8_t Fin, int16_t *DATA, bool verif)
{    
     int NbDataCRC = (verif)? 1 : 0;
     uint8_t data_raw[2*(NbData+NbDataCRC)+1];
     
     if ( myPort.available() > (2*(NbData+NbDataCRC)+1) )
      {  
          if (myPort.read() == Ini)
          {    
              myPort.readBytes(data_raw,2*(NbData+NbDataCRC)+1);
            
                  if(data_raw[2*(NbData+NbDataCRC)] == Fin ) 
                  {   
                      int16_t val_int16;
                      uint8_t *ptr_int16 = (uint8_t *)&val_int16;
                      
                      for (int i =0; i < NbData; i++) 
                      {
                          *ptr_int16 = data_raw[2*i];
                          *(ptr_int16+1) = data_raw[2*i+1];
                          DATA[i] =  val_int16;
                          
                       }
                      if (verif)
                      {
                         uint16_t verifySum = bte_calcCRC( DATA,2*NbData );
  
                         uint16_t val_uint16;
                         uint8_t *ptr_uint16 = (uint8_t *)&val_uint16;
                          
                         *ptr_uint16 = data_raw[2*NbData];
                         *(ptr_uint16 +1) =  data_raw[2*NbData+1];
                         if ( val_uint16 == verifySum )
                         {
                            return true;
                         }
                         else
                         {
                            return false;
                         }               
                      }
                      else
                      {
                        return true;
                      }
                   }
                   else
                   {
                      return false;
                   }
          }
          else
          {
              return false;
          }
      }
      else
      {
          return false;
      }
}

void bte_send_float(Stream &myPort, float *N)
{  
   myPort.write( *( (uint8_t *) N) ); 
   myPort.write( *( (uint8_t *) N + 1) );
   myPort.write( *( (uint8_t *) N + 2) );
   myPort.write( *( (uint8_t *) N + 3) );
}

void bte_sendData_float(Stream &myPort, uint8_t Ini, uint8_t Fin, float *N, int NbData, bool verif)
{
      myPort.write(Ini);
      for (int i =0; i< NbData; i++)
      {
          bte_send_float(myPort, N+i);
      }
      if (verif)
      {
        int16_t Checksum = bte_calcCRC(N, 4*NbData);
        bte_send_in16_t(myPort, &Checksum);
      }
      
      myPort.write(Fin);
}

bool bte_recep_float(Stream &myPort, int NbData, uint8_t Ini, uint8_t Fin, float *DATA, bool verif)
{    
     int NbDataCRC = (verif)? 1 : 0;
     uint8_t data_raw[4*(NbData)+2*(NbDataCRC)+1];
     
     if ( myPort.available() > ( 4*(NbData)+2*(NbDataCRC)+1) )
     
      {  
          uint8_t temp_val = myPort.read();
          if (temp_val == Ini)
          {   
              myPort.readBytes( data_raw, 4*(NbData)+2*(NbDataCRC)+1 );
                   
                  if( data_raw[4*(NbData)+2*(NbDataCRC)] == Fin ) 
                  {   
                      float val_float;
                      uint8_t *ptr_float = (uint8_t *)&val_float;
                      
                      for (int i =0; i < NbData ;i++) 
                      {
                          *ptr_float     = data_raw[4*i];
                          *(ptr_float+1) = data_raw[4*i+1];
                          *(ptr_float+2) = data_raw[4*i+2];
                          *(ptr_float+3) = data_raw[4*i+3];
                          DATA[i]        = val_float;  
                       }

                      if (verif)
                      { 
                         uint16_t verifySum = bte_calcCRC( DATA,4*NbData );
  
                         uint16_t val_uint16;
                         uint8_t *ptr_uint16 = (uint8_t *)&val_uint16;
                         
                         *ptr_uint16 = data_raw[4*NbData];
                         *(ptr_uint16 +1) =  data_raw[4*NbData+1];
                         
                         if ( val_uint16 == verifySum )
                         {
                            return true;
                         }
                         else
                         {
                            return false;
                         }               
                      }
                      else
                      {
                        return true;
                      }
                   }
                   else
                   {
                      return false;
                   }
          }
          else
          {
              return false;
          }
      }
      else
      {
          return false;
      }
}
