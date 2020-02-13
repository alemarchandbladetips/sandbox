#ifndef _BTE_FUNCTIONS_H
#define _BTE_FUNCTIONS_H

#include "Arduino.h"

#define RAD2DEG 57.29577951308
#define DEG2RAD 0.0174532925
#define PI 3.14159265359

void bte_Mean(float A[], float* prom, int NbData, int posf);

void bte_HistoriqueVal(float val, float A[], int NbData);

uint16_t bte_medianFilter16(uint16_t val, uint16_t XX[], uint8_t Elem);

//void getPosSpeedAiles(void);

float bte_signo(float val);

float bte_ang_180(float a1);

float bte_ang_pi(float a1);

float bte_lineVal(float x_i, float x_f, float y_i, float y_f, float x);

void bte_HistoriqueN(float val, float A[], uint8_t N);

void bte_send_in16_t(Stream &myport, int16_t *N);

void bte_sendData_int16_t(Stream &myPort, uint8_t Ini, uint8_t Fin, int16_t *N, int NbData);

uint16_t bte_calcCRC(const void *pBuffer, uint16_t bufferSize);

#endif