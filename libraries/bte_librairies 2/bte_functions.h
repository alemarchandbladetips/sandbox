#ifndef _BTE_FUNCTIONS_H
#define _BTE_FUNCTIONS_H

#include "Arduino.h"

#define RAD2DEG 57.29577951308
#define DEG2RAD 0.0174532925

#ifndef PI
#define PI 3.14159265359
#endif

// filtre moyenne 
void bte_Mean(float A[], float* prom, int NbData, int posf);

// buffer
void bte_HistoriqueVal(float val, float A[], int NbData);

// filtre median
uint16_t bte_medianFilter16(uint16_t val, uint16_t XX[], uint8_t Elem);

float bte_medianFilter(float val, float XX[], uint8_t Elem);

// fonction signe
float bte_signo(float val);

// modulo -180/180
float bte_ang_180(float a1);

//modulo 0/360
float bte_ang_360(float a1);

// modulo pi
float bte_ang_pi(float a1);

// interpolation linéaire
float bte_lineVal(float x_i, float x_f, float y_i, float y_f, float x);

// buffer
void bte_HistoriqueN(float val, float A[], uint8_t N);

void bte_send_in16_t(Stream &myport, int16_t *N);

void bte_sendData_int16_t(Stream &myPort, uint8_t Ini, uint8_t Fin, int16_t *N, int NbData, bool verif);

void bte_sendData_int16_t(Stream &myPort, uint8_t Ini, uint8_t Fin, int16_t *N, int NbData);

bool bte_recep_int16_t(Stream &myPort, int NbData, uint8_t Ini, uint8_t Fin, int16_t *DATA, bool verif);

void bte_sendData_int16_t(Stream &myPort, uint8_t Ini, uint8_t Fin, int16_t *N, int NbData);

void bte_send_float(Stream &myPort, float *N);

void bte_sendData_float(Stream &myPort, uint8_t Ini, uint8_t Fin, float *N, int NbData, bool verif);

bool bte_recep_float(Stream &myPort, int NbData, uint8_t Ini, uint8_t Fin, float *DATA, bool verif);

// fonction checksum SBG
uint16_t bte_calcCRC(const void *pBuffer, uint16_t bufferSize);

#endif