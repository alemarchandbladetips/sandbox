// max range of current to be sent by serial comm is 32 000 mAmps

#include "Wire.h"
#include "GlobalVariables.h"
#include "Functions.h"
#include "Adafruit_INA260.h"

/**********Capteur Tension courant *********/
int const NB_ina260 = 2;
Adafruit_INA260 ina260[NB_ina260] = {Adafruit_INA260()};
int addr_ina260[NB_ina260] = {0x40, 0x41};
int8_t ina260_init[NB_ina260] = {false};
TwoWire *wire_ina260[NB_ina260] = {&Wire, &Wire1};

// Variables mesures
float courant_ina[NB_ina260], tension_ina[NB_ina260], puissance_ina[NB_ina260];
// Variables filtrées
float courant_ina_f[NB_ina260], tension_ina_f[NB_ina260], puissance_ina_f[NB_ina260];
// param filtre
float alpha_ina = 0.02;

float total_current = 0;
float total_power = 0;

//speed
float rpm_mot;
/***********Reception RPM via port Serie*********/
RxTxSerial Serial_RPM(Serial5,0);
RxTxSerial Serial_FT(Serial2,0);

int const  NB_SERIAL_DATA = 6;// temps(uin32_t)+2*tension(int16_t)+2*courant(int16_t)
int16_t serial_data[NB_SERIAL_DATA];
int START_BYTE = 137;
int STOP_BYTE = 173;

/******Affichage*************/
#define DEBUG
uint32_t tt_plot;
uint32_t tt_ina;
uint32_t period_plot = 10000;
uint32_t period_ina = 2000; //ina query takes 1 ms roughly

void setup()
{

  //Serial begin
  Serial.begin(115200);

  //Serial begin
  Serial_RPM.begin(230400);
  Serial_FT.begin(230400);


  ///// Initialisation INA
  for (int i= 0; i<NB_ina260 ; i++ )
  {
    ina260_init[i] = ina260[i].begin(addr_ina260[i],wire_ina260[i]);
    if (!ina260_init[i])
    {
      print("Ina_260 number ", i ,"\t");
      print(" failed to initialized","\n");
      delay(1000);
    } 
  }
  
  ///// init filtres
  for (int i= 0; i<NB_ina260 ; i++ )
  { 
    if (ina260_init[i])
    {
      courant_ina_f[i] = ina260[i].readCurrent()/1000.0; // en Ampère
      tension_ina_f[i] = ina260[i].readBusVoltage()/1000.0; // en Volts
      print("Ina_260 number ", addr_ina260[i] ,"\t");
      print(" initialized","\n");
    }
  }

}



void loop()
{ 
  if ( elapsed_us(tt_ina,period_ina) )
  { 
    total_current = 0;
    total_power = 0;
    for (int i= 0; i<NB_ina260 ; i++ )
    {
      if (ina260_init[i])
      {
        courant_ina[i] = ina260[i].readCurrent()/1000.0; // en Ampère
        tension_ina[i] = ina260[i].readBusVoltage()/1000.0; // en Volts
        puissance_ina[i] = courant_ina[i]*tension_ina[i];

        courant_ina_f[i]   += alpha_ina * ( courant_ina[i] - courant_ina_f[i] );
        tension_ina_f[i]   += alpha_ina * ( tension_ina[i] - tension_ina_f[i] );
        puissance_ina_f[i] += alpha_ina * ( puissance_ina[i] - puissance_ina_f[i] );

        total_current += courant_ina[i];
        total_power   += puissance_ina[i];
      }
      
      //sending data through serial port to teeensy Force Torque
      serial_data[0] = *((int16_t*) (&tt_ina));
      serial_data[1] = *((int16_t*) (&tt_ina) + 1);
      serial_data[2] = courant_ina[0]*1000;
      serial_data[3] = tension_ina[0]*1000;
      serial_data[4] = courant_ina[1]*1000;
      serial_data[5] = tension_ina[1]*1000;
      
      Serial_FT.sendData_int16_t(START_BYTE, STOP_BYTE, serial_data, NB_SERIAL_DATA);

    }
  }


///// Affichage à la fréquence choisie
#ifdef DEBUG
  if ( elapsed_us(tt_plot,period_plot) )
  { 
    // println(tt_plot);
    // print("I1","V1",courant_ina[0],tension_ina[0],"\t");
    // print("I2","V2",courant_ina[1],tension_ina[1],"\t");
    // print("It","Pt", total_current, total_power,"\n");

    // serial_data[0] = (int16_t)(tt_ina >> 16);
    // serial_data[1] = (int16_t)((int16_t)(tt_ina << 16) >>16);

    
  
  }

#endif 

}
