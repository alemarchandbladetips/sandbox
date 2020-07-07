#include "bte_controller.h"
#include "stdint.h"


bte_controller::bte_controller(HardwareSerial *controller_serial)
{
  Trim = 0;
  Queue = 100;
  Pitch = 100;
  Roll = 100;
  Collectif = 200;
  Switch_C = 1;
  Switch_D = 1;
  Switch_F = 1;
  Switch = 0;
  IdSerial1 = 111;
  IdSerial2 = 110;
  
  auxTrim = 0;
  auxQueue = 100;
  auxCollectif = 200;
  auxPitch = 100;
  auxRoll = 100;
  auxSwitch = 0;
  auxSwitch_C = 1;
  auxSwitch_D = 1;
  auxSwitch_F = 1;
  
  lastTrim = 0;
  lastQueue = 100;
  lastCollectif = 200;
  lastPitch = 100;
  lastRoll = 100;
  lastSwitch = 0;
  lastSwitch_C = 1;
  lastSwitch_D = 1;
  lastSwitch_F = 1;
  
  _connection_lost_time_millis = 2000;

  _controller_serial = controller_serial;
  (*_controller_serial).begin(115200); // lecture radio télécommande
  
}

/**** switchState retourne letat du switch: 1, 2, 3. ***/
int bte_controller::switchState(int Sw, int a, int b, int c) {
  if ( Sw <= a )
  {
    return 2;
  } else if ( Sw == b )
  {
    return 1;
  } else if ( Sw >= c )
  {
    return 0;
  }
  return 0;
}

///Filtre median pour les valeur de la télécommande, elimine les bizarries
uint8_t bte_controller::medianFilter(int val, uint8_t X[], uint8_t Elem)
{
  uint8_t i, j;
  uint8_t S[Elem];
  //Historique des valeurs stocké dans X
  for (i = 0; i < (Elem - 1) ; i++) {
    X[i] = X[i + 1];
  }
  X[Elem - 1] = val;

  for (i = 0; i < Elem ; i++) {
    S[i] = X[i];
  }

  //Rangement des valeurs (min au max)
  uint8_t temp;
  for ( i = 0; i < Elem; i++)
  {
    bool change = false;

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
      break;  //ceci implique que tous les vals sont ordonnées
    }
  }

  return S[(Elem - 1) / 2]; // valeur au millieu
}

//Lecture Radio Telecommande
void bte_controller::update_controller(void)
{    
  if ( (*_controller_serial).available() > (BTE_CONTROLLER_NB_DATA_RADIO - 1) )
  {  
    if ((*_controller_serial).read() == 68)
    {
      if ((*_controller_serial).read() == 0)
      {
        if ((*_controller_serial).read() == 76)
        {
          (*_controller_serial).readBytes(DataRadio, BTE_CONTROLLER_NB_DATA_RADIO - 3);

          auxTrim = ( (DataRadio[5] - 41) * 256 + (DataRadio[6] - 86) ) * 100.0 / 1364; auxTrim = constrain(auxTrim, 0, 100);
          auxQueue = ( (DataRadio[9] - 27) * 256 + (DataRadio[10] - 252) ) * 100.0 / 677 + 100 ; auxQueue = constrain(auxQueue, 0, 200);
          if ( DataRadio[11] > 32)
          {
               if (DataRadio[27] < 32)
               {
                  auxCollectif = 200 - ( (DataRadio[27] - 1) * 256 + (DataRadio[28] - 86) ) * 200.0 / 1364; auxCollectif = constrain(auxCollectif, 0, 200);
               }
               
          }
          else
          {
               auxCollectif = 200 - ( (DataRadio[11] - 1) * 256 + (DataRadio[12] - 86) ) * 200.0 / 1364; auxCollectif = constrain(auxCollectif, 0, 200);
          }
          //auxCollectif=1000-( (DataRadio[11]-1)*256+(DataRadio[12]-86) )*1000.0/1620; auxCollectif=constrain(auxCollectif,0,1000);
          auxPitch = ( (DataRadio[7] - 19) * 256 + (DataRadio[8] - 248) ) * 100.0 / 681 + 100; auxPitch = constrain(auxPitch, 0, 200);
          if ( (DataRadio[19] - 9) > 6 )
          {
            auxRoll = ( (DataRadio[19] - 140) * 256 + (DataRadio[20]) ) * 100.0 / 681 + 100 ;
          } else
          {
            auxRoll = ( (DataRadio[19] - 12) * 256 + (DataRadio[20]) ) * 100.0 / 681 + 100 ;
          }

          auxRoll = constrain(auxRoll, 0, 200);

          if (DataRadio[13] < 68 )
          {
          //Serial.println(DataRadio[13]);
          auxSwitch_C = switchState(DataRadio[13], 57, 60, 62); //position de valeur du Switch et 3 possible valeurs
          }
          //auxSwitch_C = switchState(DataRadio[13], 57, 60, 62); //position de valeur du Switch et 3 possible valeurs

          if (DataRadio[27] >= 32)
          {
          //Serial.println(DataRadio[27]);
              auxSwitch_D = switchState(DataRadio[27], 33, 36, 38);
          }
          else if (DataRadio[27] < 32)
          {
              if(DataRadio[11] < 32)
              {
              //Serial.println(DataRadio[11]);
                  auxSwitch_D = switchState(DataRadio[11], 33, 36, 38);
              }
          }
          
          if (DataRadio[15] < 76)
          {
          		
              auxSwitch_F = switchState(DataRadio[15], 49, 52, 54);
          }

          Trim      =  medianFilter( auxTrim , TrimHisto, BTE_CONTROLLER_NF);
          Queue     =  medianFilter( auxQueue , QueueHisto, BTE_CONTROLLER_NF);
          Collectif =  medianFilter( auxCollectif , CollectifHisto, BTE_CONTROLLER_NF);
          Pitch     =  medianFilter( auxPitch , PitchHisto, BTE_CONTROLLER_NF);
          Roll      =  medianFilter( auxRoll , RollHisto, BTE_CONTROLLER_NF);
          Switch_C  =  medianFilter( auxSwitch_C , SwitchC_Histo, BTE_CONTROLLER_NF_SWITCH);
          Switch_D  =  medianFilter( auxSwitch_D , SwitchD_Histo, BTE_CONTROLLER_NF_SWITCH);
          Switch_F  =  medianFilter( auxSwitch_F , SwitchF_Histo, BTE_CONTROLLER_NF_SWITCH);

          
          //avoidGazMax(Collectif);

          last_millis = millis();
          temps_radio = 0;
          _perte_connection = 0;
        }
      }
    }
  }
  temps_radio = millis() - last_millis; //temps entre deux lecture des données de la télécommande
  if ( temps_radio > _connection_lost_time_millis ) //perte de Connection, on conserve les valeur d'avant sauf pour le collectif
  {
    Trim      = lastTrim;
    Queue     = lastQueue;
    Pitch     = lastPitch;
    Roll      = lastRoll;
    Switch_C  = lastSwitch_C;
    Switch_D  = lastSwitch_D;
    Switch_F  = lastSwitch_F;
    _perte_connection = 1;
    Collectif = 200;
  }
  else
  {
    lastTrim = Trim;
    lastQueue = Queue;
    lastPitch = Pitch;
    lastRoll = Roll;
    lastSwitch_C = Switch_C;
    lastSwitch_D = Switch_D;
    lastSwitch_F = Switch_F;
    lastCollectif = Collectif;

  }

  _thrust = constrain(-(Collectif-200.0)/200.0,0,1);
  _elevator = constrain(Pitch/100.0 - 1.0,-1,1);
  _rudder = constrain(Queue/100.0 - 1.0,-1,1);
  _aileron = constrain(Roll/100.0 - 1.0,-1,1);
  _knob = constrain(Trim/100.0,0,1);
  _switch_C = Switch_C;
  _switch_D = Switch_D;
  _switch_F = Switch_F;

}

void bte_controller::set_connection_lost_time_millis(uint32_t connection_lost_time_millis)
{
	_connection_lost_time_millis = connection_lost_time_millis;
}
