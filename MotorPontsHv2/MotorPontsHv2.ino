#include "GlobalVariables.h"
#include "Functions.h"
#include "RadioSignals.h"

 MyServo E[12]; 
 //          pin, pwmMin, pwmMax, pwmInit, pwm, control, rot
 motor EL[12]{ {1,  0, 2000, 0, 0, 0, -1},
               {2,  0, 2000, 0, 0, 0, -1},  
               {3,  0, 2000, 0, 0, 0, -1},  
               {4,  0, 2000, 0, 0, 0, -1},
               {5,  0, 2000, 0, 0, 0, -1}, 
               {6,  0, 2000, 0, 0, 0, -1}, 
               {7,  0, 2000, 0, 0, 0, -1},  
               {8,  0, 2000, 0, 0, 0, -1},  
               {9,  0, 2000, 0, 0, 0, -1},
               {10, 0, 2000, 0, 0, 0, -1}, 
               {11, 0, 2000, 0, 0, 0, -1},
               {12, 0, 2000, 0, 0, 0, -1}, 
           };
 float Control_E[6] ={0};
 float const ang_bob[6] = {0, PI, -d60, PI-d60, d60-PI, d60};
 
 const int WIDTH_PWM = 2000;

 //regulation motor
 float ang=0;
 float Amp = 1000;
 
 float const D_lat = 1000;
 vect2 pos_xy{0,0,D_lat}; //mod, ang, lim

 float d_x=0, d_y=0;
 
   /***********Timers*********************/
 uint32_t tt, dt;
 uint32_t tt_mot, dt_mot;


void setup()
{
  Serial.begin(250400);
  Serial.setTimeout(0);
  Serial1.begin(115200); //Radio

 
// //initializing electromagnets
 for (int  i =0;i<12;i++)
 {
    E[i].set_resol(12);
    E[i].attach_pin_us(EL[i].pin,EL[i].pwmMin,EL[i].pwmMax,WIDTH_PWM);
    E[i].write_us(EL[i].pwmMin);
 }
 
 tt= micros();
// tt_pwm = micros();
 
}


void loop()
{  
   //Lecture Radio
   lectureRadio(Serial1); 
   
    dt = micros() - tt;
   if (dt > WIDTH_PWM)
   {
     tt += dt;

     ang = rudder*PI;
     d_x = aileron*D_lat;
     d_y = elevator*D_lat;
     
     pos_xy.mod = getMod(-d_x,d_y);
     pos_xy.ang = atan2f(-d_x,d_y);
     sat(pos_xy.mod,0,pos_xy.lim);
      
     // Control pour chaque paire de bobines(chaque phase)
     for (int i=0; i<3; i++)
     {  
//        float aux_Amp_1 = Amp + pos_xy.mod*cos( -i*d60 - pos_xy.ang);
//        float aux_Amp_2 = Amp + pos_xy.mod*cos( -i*d60 + PI - pos_xy.ang);
        Amp = 1000*knob;

//// Controle avec variation d'amplitude
        
        //float aux_Amp_1 = Amp*( 1+(pos_xy.mod/1000.0)*cos( ang_bob[2*i] - pos_xy.ang) );
        //float aux_Amp_2 = Amp*( 1+(pos_xy.mod/1000.0)*cos( ang_bob[2*i+1] - pos_xy.ang) );

        //Control_E[2*i]   = aux_Amp_1*(cosf(ang+i*d120)); 
        //Control_E[2*i+1] = aux_Amp_2*(cosf(ang+i*d120));

//// Controle avec variation d'offset

        //Control_E[2*i]   = Amp*(cosf(ang+i*d120) + (pos_xy.mod/1000.0)*cos( -ang + ang_bob[2*i] - pos_xy.ang)); 
        //Control_E[2*i+1] = Amp*(cosf(ang+i*d120) + (pos_xy.mod/1000.0)*cos( -ang + ang_bob[2*i+1] - pos_xy.ang));

//// Test

        Control_E[2*i]   = constrain(Amp*(0.5*cosf(ang+i*d120) + 2*(pos_xy.mod/1000.0)*cos(-ang + ang_bob[2*i] - pos_xy.ang)),-2000,2000); 
        Control_E[2*i+1] = constrain(Amp*(0.5*cosf(ang+i*d120) + 2*(pos_xy.mod/1000.0)*cos(-ang + ang_bob[2*i+1] - pos_xy.ang)),-2000,2000);

        if(i <2)
        {
          Serial.print(Control_E[2*i]);Serial.print("\t");
          Serial.print(Control_E[2*i+1]);Serial.print("\t");
        }
        else
        {
          Serial.print(Control_E[2*i]);Serial.print("\t");
          Serial.print(Control_E[2*i+1] );Serial.print("\n");
        }
        
     }
     
     // control pwm pour chaque bobine
     for (int i=0; i<6; i++)
     {    
          if ( Control_E[i] > 0)
          {
            EL[2*i].pwm   = Control_E[i]; //A1_1
            EL[2*i+1].pwm = 0;            //A1_2
          }
          else
          {
            EL[2*i].pwm   =   0;             //A1_1
            EL[2*i+1].pwm = - Control_E[i];  //A1_2
          }

          E[2*i].write_us( EL[2*i].pwm );
          E[2*i+1].write_us( EL[2*i+1].pwm );
        
     }
        
   }
}
