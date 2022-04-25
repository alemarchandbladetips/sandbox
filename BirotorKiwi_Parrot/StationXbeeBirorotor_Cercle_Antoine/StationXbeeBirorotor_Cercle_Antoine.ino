#include "CommandHandler.h"
#include "MegunoLink.h"
#include "Functions.h"

CommandHandler<10,55,20>  SerialCommandHandler(Serial);
TimePlot Data1plot("P1");
TimePlot Data2plot("P2");
TimePlot Data3plot("P3");
TimePlot Data4plot("P4");
TimePlot Data5plot("P5");
TimePlot Data6plot("P6");

InterfacePanel Panel;

// Xbee comm
RxTxSerial XBeeComm(Serial3,true);

// Xbee reception des données depuis le drône
const int NbXB_RX = 29+3;
int16_t DATA_XBEE_RX[NbXB_RX];

// Xbee transmission vers drône
const int NbXB_TX = 3;
int16_t DATA_XBEE_TX[NbXB_TX];

uint8_t START1 = 137, START2 = 157, STOP = 173;

bool Ok2sent = true;
uint32_t dt_sent;

// Gains
int GAIN[9] ={0};
uint8_t GAIN_CHNG = 0;
int Gain2Sent = 0;
int gain_val[9]={0};
int DT_envoi = 100; // 20 ms

//Données reçue depuis le drône
float roll, pitch, yaw;
float w_roll, w_pitch, w_yaw;  
float Courant, Voltage, Power;  
float Servo1, Servo2, Servo3, Servo4, Mot1,  Mot2;
float roll_ref, pitch_ref, yaw_ref; 
float regRoll, regPitch, regYaw; 
float e_angle_z;
float x_gps=0, y_gps=0, z_gps=0;
float vx_gps=0, vy_gps=0, vz_gps=0;
float pitch_rate = 0;
float thrust=0;

bool mode_auto = false;



void setup()
{   
    Serial.begin(500000);
    
    Serial3.begin(111111); // XBEE Station
    
    //Meguno Function Handler
    SerialCommandHandler.AddCommand(F("resendVal"), Cmd_resendVal); // Associe 'resendVal' a la function  Cmd_resend
    SerialCommandHandler.AddCommand(F("SendALL"), Cmd_SendALL);     // Associe 'SendALL' a la function  Cmd_resend

}

void loop()
{ 
   SerialCommandHandler.Process(); // lance le procès MegunoLink

   
   if ( (GAIN_CHNG > 0) && Ok2sent)  // si on a envoyé un gain dès meguno et c'est OK pour envoyer
   {  
      update_data(GAIN_CHNG);  //mise à jour des données DATA_XBEE_TX à envoyer
      XBeeComm.sendData_int16_2(START1, START2, STOP, DATA_XBEE_TX, NbXB_TX); // envoi des données
      Ok2sent = false;         //bloque la transmission jusqu'à la confirmation de reception ou jusqu'un certain temps
      dt_sent = micros();      // on commence à conter le temps pour un nouvel envoi
   }
   
   if ( XBeeComm.getData_int16_2(START1, START2, STOP, DATA_XBEE_RX) > 0  )   // si on a reçu des données depuis le drône
   {   
       if (  GAIN_CHNG> 0 && (DATA_XBEE_RX[NbXB_RX-3] == GAIN_CHNG) && (DATA_XBEE_RX[NbXB_RX-2] == GAIN[GAIN_CHNG-1]) )  // Verification des données reçues
       {  
          Ok2sent = true;                        // Ok pour renvoyer un gain si nécessaire 
          BackColor(GAIN_CHNG,"Lime");           // Change couleur du bouton 'send' en meguno à vert
          PutText(GAIN_CHNG,GAIN[GAIN_CHNG-1]);  //ecrit le gain envoyé en meguno en guise de confirmation
          updateGain2Sent();                     // mise à jour du prochain gain à envoyer en cas d'envoyer tout les gains
       }
       
       // Données reçues depuis le drône
        roll      = DATA_XBEE_RX[0]/10.0;   pitch      = DATA_XBEE_RX[1]/10.0;   yaw    = DATA_XBEE_RX[2]/10.0;
        w_roll    = DATA_XBEE_RX[3]/10.0;   w_pitch    = DATA_XBEE_RX[4]/10.0;   w_yaw  = DATA_XBEE_RX[5]/10.0;
        Courant   = DATA_XBEE_RX[6];        Voltage    = DATA_XBEE_RX[7]/1000.0; Power  = DATA_XBEE_RX[8];
        Servo1    = DATA_XBEE_RX[9];        pitch_rate = DATA_XBEE_RX[10]/10.0;  Servo2 = DATA_XBEE_RX[11];        
        e_angle_z = DATA_XBEE_RX[12]/10.0;  Mot1       = DATA_XBEE_RX[13]/10.0;  Mot2   = DATA_XBEE_RX[14]/10.0;
        roll_ref  = DATA_XBEE_RX[15]/10.0;  pitch_ref  = DATA_XBEE_RX[16]/10.0;  yaw_ref= DATA_XBEE_RX[17]/10.0;
        regRoll   = DATA_XBEE_RX[18];       regPitch   = DATA_XBEE_RX[19];       regYaw = DATA_XBEE_RX[20];
        mode_auto = DATA_XBEE_RX[21];                      x_gps      = DATA_XBEE_RX[22]/10.0;  y_gps  = DATA_XBEE_RX[23]/10.0;
        z_gps     = DATA_XBEE_RX[24]/10.0;  vx_gps     = DATA_XBEE_RX[25]/10.0;  vy_gps = DATA_XBEE_RX[26]/10.0;
        vz_gps    = DATA_XBEE_RX[27]/10.0;  thrust     = DATA_XBEE_RX[28]/10.0;

        // envoie des donnée à ploter en meguno
        plotData1(roll, roll_ref, w_roll);
        plotData2(pitch,pitch_ref,w_pitch);
        plotData3(e_angle_z , w_yaw, Power/10.0);
          
        float vxy = getMod(vx_gps,vy_gps);
        plotData4( Power/100.0, vxy, vz_gps,mode_auto*10);
        plotData5( Voltage, Power/100.0, Servo1/9.5,mode_auto*10);
        plotData6( x_gps,y_gps,z_gps);

        // envoi des donnée pour le case 'Pow' et 'pwm' en meguno
        Panel.SetNumber("pow", Power);
        Panel.SetNumber("pwm", Mot1*10);
   }
   
   if ( (micros()-dt_sent) > DT_envoi*1000 )  // compteur pour faire un nouvel envoi
   {
     Ok2sent = true;
   }

   
}

/******************************************************************************************************************************/
/****** Fonctions ************/

void plotData1(float val1, float val2, float val3)
{
      Data1plot.SendData("R",val1);
      Data1plot.SendData("R_r",val2);
      Data1plot.SendData("Wx", val3);
}

void plotData2(float val1, float val2, float val3)
{
    Data2plot.SendData("P",val1);
    Data2plot.SendData("P_r",val2);
    Data2plot.SendData("Wy", val3); 

}

void plotData3(float val1, float val2, float val3)
{
    Data3plot.SendData("Ey",val1);
    Data3plot.SendData("wz",val2);
    Data3plot.SendData("P", val3);

}

void plotData4(float val1, float val2, float val3, int mode)
{
    Data4plot.SendData("P",val1);
    Data4plot.SendData("vxy",val2);
    Data4plot.SendData("Vz", val3);
    Data4plot.SendData("Mode", mode);

}

void plotData5(float val1, float val2, float val3, int mode)
{
    Data5plot.SendData("V",val1);
    Data5plot.SendData("Pow",val2);
    Data5plot.SendData("Ang", val3);
    Data5plot.SendData("Mode", mode);

}

void plotData6(float val1, float val2, float val3)
{
    Data6plot.SendData("x",val1);
    Data6plot.SendData("y",val2);
    Data6plot.SendData("z", val3);

}

void update_data(uint8_t NbGain)
{ 
  if ( NbGain == 0 )
  {
    DATA_XBEE_TX[0] = NbGain;
    DATA_XBEE_TX[1] = 0;
    DATA_XBEE_TX[2] = 2;
  }
  else
  {
    DATA_XBEE_TX[0] = NbGain;
    DATA_XBEE_TX[1] = GAIN[NbGain-1];
    DATA_XBEE_TX[2] = 1;
    
  }
}

void BackColor(int K, char *C)
{ 
  char *s= (char*) malloc(strlen("T_K")+ sizeof(int)+1);
  char *k = (char*) malloc(sizeof(int)+1);
  
  strcpy(s,"T_K");
  sprintf(k, "%d", K);  
  strcat(s,k);
  
  Panel.SetBackColor(s,C);
  
  free (s);
  free (k);
  
}

void PutText(int id, int val)
{ 
  char *s= (char*) malloc(strlen("T_K")+ sizeof(int)+1);
  char *k = (char*) malloc(sizeof(int)+1);
  char *r = (char*) malloc(sizeof(int)+1);
  
  strcpy(s,"T_K");
  sprintf(k, "%d", id);  
  strcat(s,k);
  sprintf(r, "%d", val);
 
  Panel.SetText(s,r);
  
  free (s);
  free (k);
  free (r);
  
}

void updateGain2Sent(void)
{ 
  if(--Gain2Sent<0) Gain2Sent = 0;
  GAIN_CHNG = Gain2Sent;
  if ( Gain2Sent > 0 )
  {
    GAIN[GAIN_CHNG-1] =  gain_val[Gain2Sent-1];
    BackColor(GAIN_CHNG, "Yellow");
  }

}

void Cmd_SendALL(CommandParameter &q)
{  
   for (int i =0; i<9; i++)
   {
      gain_val[i]= q.NextParameterAsInteger(0);
   }
   Gain2Sent = 9;
   GAIN_CHNG = Gain2Sent;
   GAIN[GAIN_CHNG-1] = gain_val[Gain2Sent-1];
   BackColor(GAIN_CHNG, "Yellow");
   
}

void Cmd_resendVal(CommandParameter &p)
{   
    int cas=p.NextParameterAsInteger(0);
    int Val=p.NextParameterAsInteger(0);
    Gain2Sent = 0;
    
    if ( cas == 10 )
    {
      for (int i =1; i < 10 ;i++)
      {
        BackColor(i, "Yellow");
      }  
    }
    else if (cas < 10 )
    {
      GAIN_CHNG = cas;
      GAIN[GAIN_CHNG-1] = Val;
      BackColor(GAIN_CHNG, "Yellow");
      
      Gain2Sent = 0; //stop l'envoi de tous les gains
    }

}
