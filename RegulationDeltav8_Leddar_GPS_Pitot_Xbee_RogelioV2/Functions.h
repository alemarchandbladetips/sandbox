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

void sendData_int16_t(Stream &myPort, uint8_t Ini, uint8_t Fin, int16_t *N, int NbData);

uint16_t calcCRC(const void *pBuffer, uint16_t bufferSize);
