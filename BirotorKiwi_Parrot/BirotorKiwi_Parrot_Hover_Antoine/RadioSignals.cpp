#include "RadioSignals.h"
#include "stdint.h"

//#define SERIAL_RADIO Serial3

////////////////////////////////////////////////////////////
uint8_t const NB_DATA = 16;
uint8_t PACK_temp[NB_DATA];

uint16_t val_uint16;
uint8_t *ptr_uint16 = (uint8_t*) &val_uint16;

uint16_t _aileron;
uint16_t _elevator;
uint16_t _rudder;
uint16_t _thrust;

uint16_t _switch_C, _switch_D, _switch_F;
uint16_t _knob;

float aileron=0;
uint16_t min_aileron =35159, max_aileron= 36521;
float knob = 0;
uint16_t min_knob=10582, max_knob=11946;
float elevator=0;
uint16_t min_elevator=4439, max_elevator=5801;
float rudder = 0;
uint16_t  min_rudder=6487, max_rudder=7849;
float thrust = 0;
uint16_t min_thrust=342, max_thrust=1706;

uint16_t _aileron2;
uint16_t min_aileron2 =2391, max_aileron2= 3752;
uint16_t _knob2 = 0;
uint16_t min_knob2=10582, max_knob2=11946;
uint16_t _elevator2=0;
uint16_t min_elevator2=4439, max_elevator2=5801;
uint16_t _rudder2 = 0;
uint16_t  min_rudder2=6487, max_rudder2=7849;

uint8_t const switch_C_dflt = 2;
uint8_t const switch_D_dflt = 2;
uint8_t const switch_F_dflt = 2;

uint8_t switch_C = switch_C_dflt, switch_D = switch_D_dflt, switch_F = switch_F_dflt;
///////////////////////////////////////////////////////////

float aileron_dflt=0;
float knob_dflt = 0;
float elevator_dflt=0;
float rudder_dflt = 0;
float thrust_dflt = 0;

////////////////////////////////////////////////////////////
bool NODATA = false;
uint32_t dt_nodata, temps_nodata;
uint32_t dt_norecep;

bool FIRST_TIME_RADIO = true;
uint8_t perteRadio = 0;
//////////////////////////////

void lectureRadio(Stream &mySerial)
{  
   if (FIRST_TIME_RADIO)
   {
     temps_nodata = micros();
     FIRST_TIME_RADIO = false;
   }
   
   if( !NODATA )
   {
     if ( mySerial.available()  > 0 )
     {  
       clean_Serial(mySerial);
       dt_nodata = 0;
       temps_nodata = micros();
     }
     else
     {  
        dt_nodata = micros() - temps_nodata;
        if ( dt_nodata > 1*1000 )
        {
          NODATA = true;
          dt_nodata = 0;
        }
     }
   }

   if ( NODATA && mySerial.available()  > NB_DATA-1  )
   {
      mySerial.readBytes(PACK_temp,NB_DATA);
      NODATA = false;
      temps_nodata = micros();
      if ( identPack(PACK_temp) == 1 )
      {
          *(ptr_uint16+1) = PACK_temp[2];
          *(ptr_uint16) = PACK_temp[3];
          _aileron = val_uint16;
          
          aileron = normVal_pn(_aileron, min_aileron, max_aileron);

          *(ptr_uint16+1) = PACK_temp[4];
          *(ptr_uint16) = PACK_temp[5];
          _knob = val_uint16;

          knob = normVal(_knob, min_knob, max_knob); 
          
          *(ptr_uint16+1) = PACK_temp[6];
          *(ptr_uint16) = PACK_temp[7];
          _elevator = val_uint16;

          elevator = normVal_pn(_elevator, min_elevator, max_elevator);

          *(ptr_uint16+1) = PACK_temp[8];
          *(ptr_uint16) = PACK_temp[9];
          _rudder = val_uint16;

          rudder = normVal_pn(_rudder, min_rudder, max_rudder);

          *(ptr_uint16+1) = PACK_temp[10];
          *(ptr_uint16) = PACK_temp[11];
          _thrust = val_uint16;

          thrust = normVal(_thrust, min_thrust, max_thrust);
          
          *(ptr_uint16+1) = PACK_temp[12];
          *(ptr_uint16) = PACK_temp[13];
          _switch_C = val_uint16; //16042, 15360, 14678
           switch_C = switch_state(_switch_C, 16042, 15360, 14678);


          *(ptr_uint16+1) = PACK_temp[14];
          *(ptr_uint16) = PACK_temp[15];
          _switch_F = val_uint16; //13994, 13312, 12630
          //Serial.println(_switch_F);
           switch_F = switch_state(_switch_F, 13994, 13312, 12630);
//           Serial.print(_switch_F);Serial.print("\t");
//           Serial.print(switch_F);Serial.print("\n");

      }
      else
      {   
          *(ptr_uint16+1) = PACK_temp[2];
          *(ptr_uint16) = PACK_temp[3];
          _aileron2 = val_uint16;

          aileron = normVal_pn(_aileron2, min_aileron2, max_aileron2);

          *(ptr_uint16+1) = PACK_temp[4];
          *(ptr_uint16) = PACK_temp[5];
          _knob2 = val_uint16;

          knob = normVal(_knob2, min_knob2, max_knob2); 
                    
          *(ptr_uint16+1) = PACK_temp[6];
          *(ptr_uint16) = PACK_temp[7];
          _elevator2 = val_uint16;

          elevator = normVal_pn(_elevator2, min_elevator2, max_elevator2);
          
          *(ptr_uint16+1) = PACK_temp[8];
          *(ptr_uint16) = PACK_temp[9];
          _rudder2 = val_uint16;

          rudder = normVal_pn(_rudder2, min_rudder2, max_rudder2);
          
          *(ptr_uint16+1) = PACK_temp[10];
          *(ptr_uint16) = PACK_temp[11];
          _switch_D = val_uint16; //9898, 9216, 8534
          switch_D = switch_state(_switch_D, 10239, 9216, 8193);

      }

   }

   if ( micros() - temps_nodata > 2000*1000 )
   {    
        perteRadio = 1;
        // default values
        switch_C = switch_C_dflt;
        switch_D = switch_D_dflt;
        switch_F = switch_F_dflt;
        aileron = aileron_dflt;
        knob = knob_dflt;
        elevator = elevator_dflt;
        rudder = rudder_dflt;
        thrust = thrust_dflt;
   }
   else
   {
        perteRadio = 0;
   }

}


void print_Serial(uint8_t *X, int NbData, char *S)
{
   for (int i= 0; i<NbData ;i++)
   {
     Serial.print(X[i]);Serial.print(S);
   }
   Serial.println();
}


void clean_Serial(Stream &mySerial)
{
  while (mySerial.available()> 0) {mySerial.read();}
}

uint8_t identPack(uint8_t *P)
{   
    uint8_t result = 1;
    if ( P[NB_DATA-4] == 68 && P[NB_DATA-3] == 0 && P[NB_DATA-2] == 76 )
    {
      result = 2;
    }

    return result;
}

float normVal(float val, float minVal, float maxVal)
{
  float temp_val = 0;
  
  temp_val = (val-minVal)/(maxVal -minVal);
  
  if ( temp_val < 0) { temp_val = 0 ;}
  else if ( temp_val > 1.0 ) { temp_val = 1.0 ;}
  
  return temp_val;
}

float normVal_pn(float val, float minVal, float maxVal)
{
    float temp_val = 0;
    temp_val = 2.0*normVal(val, minVal, maxVal) -1.0;
    
    if ( temp_val < -1.0 ) { temp_val = -1.0 ;}
    else if ( temp_val > 1.0 ) { temp_val = 1.0 ;}
    return temp_val;
}

uint8_t switch_state(uint16_t sw_val, int val0, int  val1, int  val2)
{ 
  uint8_t sw_state = 1;
  if ( sw_val == val0 )
  {
      sw_state = 0;
  }
  else if (sw_val == val1)
  {
       sw_state = 1;
  }
  else if (sw_val == val2)
  {
       sw_state = 2;
  }

  return sw_state;
}
