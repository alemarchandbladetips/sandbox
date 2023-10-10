#include "functions.h"
#include "GlobalVariables.h"
#include "decodageGPS.h"

/**** XBEE DATA ****************/
uint8_t START1 = 137, START2 = 157, STOP = 173;

//GPS DATA to main T40
// RxTxSerial GPS_DATA_comm(Serial1, true);
RxTxSerial GPS_DATA_comm(Serial2, false);

//const int NbXB_TX = 27 + 3; //DATA + 3 params
const int NbXB_TX = 9; //DATA + 3 params
int16_t GPS_DATA_TX[NbXB_TX];

//Timers
uint32_t tt, dt;
uint32_t dt_gps_data, tt_gps_data;

void setup()
{
  Serial.begin(115200); //debug
  Serial1.begin(57600);
  Serial2.begin(57600);
//Serial4.begin(115200); //GPS, fixer le baudrate
 Serial.println("Init");
  //timers
  tt = micros();
  tt_gps_data = micros();

}


void loop()
{
  Serial.println("tic");
  decodageGPS(Serial1);
  Serial.println("tac");
  //Data from Xbees
  dt_gps_data = micros() - tt_gps_data;
  if (dt_gps_data > 10'000)
      {
      tt_gps_data += dt_gps_data;

      // x_gps = 10;
      // y_gps = 20;
      // z_gps = 30;
      // vx_gps = 11;
      // vy_gps = 12;
      // vz_gps = 13;

      /********XBEE Send Data to Station***************/
      GPS_DATA_TX[0] = x_gps*10;                       GPS_DATA_TX[1] = y_gps*10;                      GPS_DATA_TX[2] = z_gps*10;
      GPS_DATA_TX[3] = vx_gps*100;                     GPS_DATA_TX[4] = vy_gps*100;                    GPS_DATA_TX[5] = vz_gps*100;
      GPS_DATA_TX[6] = GPS_init_done;                  GPS_DATA_TX[7] = 0;                             GPS_DATA_TX[8] = 111;
//      GPS_DATA_TX[9] = 1;                            GPS_DATA_TX[10] = 2;                            GPS_DATA_TX[11] = 3;
//      GPS_DATA_TX[12] = 0;                           GPS_DATA_TX[13] = 0;                            GPS_DATA_TX[14] = 0;
//      GPS_DATA_TX[15] = 0;                           GPS_DATA_TX[16] = 0;                            GPS_DATA_TX[17] = 0;
//      GPS_DATA_TX[18] = 0;                           GPS_DATA_TX[19] = 0;                            GPS_DATA_TX[20] = 0;
//      GPS_DATA_TX[21] = 0;                           GPS_DATA_TX[22] =  0;                           GPS_DATA_TX[23] = 0;
//      GPS_DATA_TX[24] = 0;                           GPS_DATA_TX[25] = 0;                            GPS_DATA_TX[26] = 0;

      GPS_DATA_comm.sendData_int16_t(START1,STOP,GPS_DATA_TX,7);
      //GPS_DATA_comm.sendData_int16_2(START1, START2, STOP, GPS_DATA_TX, NbXB_TX);

     Serial.print("GPS: ");Serial.print("\t");
     Serial.print(vx_gps);Serial.print("\t");
     Serial.print(vy_gps);Serial.print("\t");
     Serial.print(vz_gps);Serial.print("\t");
     Serial.print(x_gps);Serial.print("\t");
     Serial.print(y_gps);Serial.print("\t");
     Serial.print(z_gps);Serial.print("\n");

    }


    }
