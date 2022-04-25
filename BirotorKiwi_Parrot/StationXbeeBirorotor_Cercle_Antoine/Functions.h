#include "Arduino.h"
#include "GlobalVariables.h"

void Mean(float A[], float* prom, int NbData, int posf);
void HistoriqueVal(float val, float A[]);

//void getPosSpeedAiles(void);

float signo(float val);

float ang_180(float a1);

float ang_pi(float a1);

float lineVal(float x_i, float x_f, float y_i, float y_f, float x);

void HistoriqueN(float val, float A[], uint8_t N);

void send_in16_t(Stream &myport, int16_t *N);

void sendData_int16_t(Stream &myPort, uint8_t Ini, uint8_t Fin, int16_t *N, int NbData, bool verif);


uint16_t calcCRC(const void *pBuffer, uint16_t bufferSize);

bool recep_int16_t(Stream &myPort, int NbData, uint8_t Ini, uint8_t Fin, int16_t *DATA, bool verif);

class ValMean {
  private :
    int pos = 0;
    bool maxPos = false;
    float sum = 0;
    int const nbElem;
    float mean = 0;
    float *A;
  
  public:
    ValMean(int val1): nbElem(val1) 
    {   
        if (val1 > 1 )
        {
            A = new float[val1];
            for (int i=0; i < val1 ;i++)
            { 
              A[i] = 0;
            } 
        }
        else
        {
          A = new float[1];
          A[0] = 0;
        }
        
    }
    
    float getMean(float val);
};

class FilterVal {
  private:
    float *A;
    float filterval = 0;
    int filtertype;
  public:
    FilterVal(int x): filtertype(x)
    {
      if (x == 1 || x == 2)
      {
        A = new float[2 * x];
        for (int i=0; i < 2*x ;i++)
        {
          A[i] = 0;
        }
      }
      else
      {
        A = new float[1];
        A[0] = 0;
      }
    }
    
    float getVal(float val);
};

float getMod(float val1, float val2);
void sat(float & val, float Val1, float Val2);

class RxTxSerial{
    private:
      uint8_t data_temp1 = 0;
      uint8_t data_temp2 = 0;
      int NbData = 0;
      int NbDataCRC = 0;
      bool PACK_WAITING = false;
      Stream &myPort;
      int8_t nb_data=0;
      bool verif;
      
    public:
      RxTxSerial(Stream &myport_ptr, bool verif) : myPort(myport_ptr), verif(verif)
      {
        if (verif) NbDataCRC = 1; else NbDataCRC = 0;
      }
      void send_in16_t(int16_t *N);
      
      void sendData_int16_t(uint8_t Ini, uint8_t Fin, int16_t *N, int NbData);
      
      bool getData_int16_t(int NbData, uint8_t Ini, uint8_t Fin, int16_t *DATA);
      
      void sendData_int16_2(uint8_t Ini1, uint8_t Ini2, uint8_t Fin, int16_t *N, int NbData);

      int8_t getData_int16_2( uint8_t Ini1, uint8_t Ini2, uint8_t Fin, int16_t *DATA);

      void send_float(float *N);

      bool getData_float(int NbData, uint8_t Ini, uint8_t Fin, float *DATA);
      
      void sendData_float(uint8_t Ini, uint8_t Fin, float *N, int NbData);
      
      uint16_t calcCRC(const void *pBuffer, uint16_t bufferSize);
        
};
