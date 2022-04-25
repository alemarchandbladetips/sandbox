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

float ValMean:: getMean(float  val)
{ 
  if ( pos == (nbElem-1) ) maxPos = true;
  
  sum = sum + val - A[pos];
  
  A[pos]= val;

  if (maxPos)
  { 
    mean = sum/nbElem;
  }
  else
  { 
    mean = sum/(pos+1);
  }
  
  pos++;
  if ( pos > (nbElem-1) ) pos = 0;
 
  return mean;
  
}

float FilterVal:: getVal(float val)
{   
    if ( filtertype == 1  || filtertype == 2)
    {
        A[1] = (2.0)*(int16_t)(val/2.0);
        //A[1] = val;
        filterval = ( A[0] + A[1] )/2.0;
        A[0] = filterval;
        
        if ( filtertype == 2)
        {   
            A[3] = 2.0*(int16_t)( A[0]/2.0);
            //A[3] =  A[0];
            filterval = (A[2] + A[3] )/2.0;
            A[2] = filterval;
        }
    }
    else
    {
        filterval = val;
    }
    return filterval;
}

float getMod(float val1, float val2)
{
    return sqrtf(val1*val1+val2*val2);
}

void sat(float &val, float Val1, float Val2)
{   
   
    if  ( Val1 < Val2 )
    {
        if ( val < Val1 ) val = Val1;
        else if ( val > Val2 ) val = Val2;
    }
    else
    {
        if ( val < Val2 ) val = Val2;
        else if ( val > Val1 ) val = Val1;
    }
    
}

// CheckSum function: provided by SBG
uint16_t RxTxSerial:: calcCRC(const void *pBuffer, uint16_t bufferSize)
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

void RxTxSerial:: send_in16_t(int16_t *N)
{  
   myPort.write( *( (uint8_t *) N) ); 
   myPort.write( *( (uint8_t *) N + 1) );
}

void RxTxSerial:: sendData_int16_t(uint8_t Ini, uint8_t Fin, int16_t *N, int NbData)
{
      myPort.write(Ini);
      for (int i =0; i< NbData; i++)
      {
          send_in16_t(N+i);
      }
      if (verif)
      {
        int16_t Checksum = calcCRC(N, 2*NbData);
        send_in16_t(&Checksum); 
      }
      
      myPort.write(Fin);
}


bool RxTxSerial:: getData_int16_t(int NbData, uint8_t Ini, uint8_t Fin, int16_t *DATA)
{    
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
                         uint16_t verifySum = calcCRC( DATA,2*NbData );
  
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

void RxTxSerial:: send_float(float *N)
{  
   myPort.write( *( (uint8_t *) N) ); 
   myPort.write( *( (uint8_t *) N + 1) );
   myPort.write( *( (uint8_t *) N + 2) );
   myPort.write( *( (uint8_t *) N + 3) );
   
}

void RxTxSerial:: sendData_float(uint8_t Ini, uint8_t Fin, float *N, int NbData)
{
      myPort.write(Ini);
      for (int i =0; i< NbData; i++)
      {
          send_float(N+i);
      }
      if (verif)
      { 
        int16_t Checksum = calcCRC(N, 4*NbData);
        send_in16_t( &Checksum);
      }
      myPort.write(Fin);
      
}

bool RxTxSerial:: getData_float(int NbData, uint8_t Ini, uint8_t Fin, float *DATA)
{    
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
                         uint16_t verifySum = calcCRC( DATA,4*NbData );
  
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


int8_t RxTxSerial:: getData_int16_2(uint8_t Ini1, uint8_t Ini2, uint8_t Fin, int16_t *DATA)
{    
     uint8_t tmp_return = 0; //Nb de données recues
     
     if ( !PACK_WAITING  )
     {  
        NbData = 0;
        tmp_return = 0;
        
        if (myPort.available() > 3-1)
        {
          if ( data_temp2 != Ini1 )
          {
            data_temp1 = myPort.read();
          }
          else
          {
            data_temp1 = data_temp2;
          }
          
          if ( data_temp1 == Ini1 )
          {
            data_temp2 = myPort.read();
            if (  data_temp2 == Ini2 )
            {  
               NbData =  myPort.read(); //Nb de données recues et à verifier avec la bit final
               PACK_WAITING = true;
            }
          }
        }
     } 
     
     if ( PACK_WAITING ) // waiting for data to decrypt
     {  
        tmp_return = 0;
        
        if ( myPort.available() > 2*(NbData+NbDataCRC) )
        {  
            PACK_WAITING =  false;
        
            uint8_t data_raw[2*(NbData+NbDataCRC)+1];
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
                   uint16_t verifySum = calcCRC( DATA,2*NbData );
    
                   uint16_t val_uint16;
                   uint8_t *ptr_uint16 = (uint8_t *)&val_uint16;
                   
                   *ptr_uint16 = data_raw[2*NbData];
                   *(ptr_uint16 +1) =  data_raw[2*NbData+1];
                   if ( val_uint16 == verifySum )
                   {
                      tmp_return = NbData;
                   }              
                }
                else
                { 
                  tmp_return = NbData;
                }
            }
        }
        
     }
     
     return tmp_return;

}

void RxTxSerial:: sendData_int16_2(uint8_t Ini1, uint8_t Ini2, uint8_t Fin, int16_t *N, int NbData)
{
      myPort.write(Ini1);
      myPort.write(Ini2);
      myPort.write(NbData);
      for (int i =0; i< NbData; i++)
      {
          send_in16_t(N+i);
      }
      if (verif)
      {
        int16_t Checksum = calcCRC(N, 2*NbData);
        send_in16_t(&Checksum); 
      }
      
      myPort.write(Fin);
}
