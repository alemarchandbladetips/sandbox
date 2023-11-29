#include "Functions.h"
#include "stdint.h"


/***** Fonctions *******/

//Moyenne des valuers
 void Mean(float A[], float* prom, int NbData, int posf)
 {
      float sum=0;
      int vali=posf+1-NbData;
      int valf=posf+1;
      for(int k=vali;k<valf;k++){
        sum+=A[k];
        }
      *prom=sum/NbData;      
 }
 
// Function Historique des valeurs
void HistoriqueVal(float val, float A[])
{
      for(int k=0;k<(H-1);k++){
        A[k]=A[k+1];
        
      }
      A[H-1]=val;
}

void HistoriqueN(float val, float A[], uint8_t N)
{
      for(int k=0;k<(N-1);k++){
        A[k]=A[k+1];
        
      }
      A[N-1]=val;
}

float signo(float val)
{   
    if(val<0)
    {
        return -1;
    }
    else
    {
        return 1;
    }
}

float ang_180(float a1)
{
    float temp = fmodf(a1,360.0);
    if(temp > 180)
    { 
      temp -= 360.0;
    }
    else if (temp < -180)
    {
       temp += 360.0;
    }
    return temp;
}

float ang_pi(float a1)
{
    float temp = fmodf(a1,2*PI);
    if(temp > PI)
    { 
      temp -= 2*PI;
    }
    else if (temp < -PI)
    {
       temp += 2*PI;
    }
    return temp;
}

float lineVal(float x_i, float x_f, float y_i, float y_f, float x)
{
  float y;
  if (x > x_f)
  {
    y = y_f;
  }
  else if ( x < x_i)
  {
    y = y_i;
  }
  else
  {
    y = (y_f - y_i) / (x_f - x_i) * (x - x_i) + y_i;
  }

  return y;
}

float interpol(float* A, float* B, int nb, float val)
{ 
  float out_val = 0;
  
  if ( val < *A ) out_val = *B;
  else if ( val > *(A+nb-1) )  out_val = *(B+nb-1);
  else
  {
      for (int i= 1; i < nb; ++i)
      {
        if ( val < *(A+i) )
        {
            out_val = ( *(B+i) - *(B+i-1) )/( *(A+i) - *(A+i-1) ) * ( val - *(A+i-1) ) + *(B+i-1);
            i = nb-1;
        }
      }
  }

  return out_val;
}

float ValMean:: getMean(float  val)
{ 
  if ( pos == (nbElem-1) ) maxPos = true;
  
  sum = sum + val - A[pos];
  
  A[pos]= val;

  if (maxPos)
  { 
    mean = sum/nbElem;
  }
  else
  { 
    mean = sum/(pos+1);
  }
  
  pos++;
  if ( pos > (nbElem-1) ) pos = 0;
 
  return mean;
  
}

float FilterVal:: getVal(float val)
{   
    if ( filtertype == 1  || filtertype == 2)
    {
        A[1] = (2.0)*(int16_t)(val/2.0);
        //A[1] = val;
        filterval = ( A[0] + A[1] )/2.0;
        A[0] = filterval;
        
        if ( filtertype == 2)
        {   
            A[3] = 2.0*(int16_t)( A[0]/2.0);
            //A[3] =  A[0];
            filterval = (A[2] + A[3] )/2.0;
            A[2] = filterval;
        }
    }
    else
    {
        filterval = val;
    }
    return filterval;
}

float getMod(float val1, float val2)
{
    return sqrtf(val1*val1+val2*val2);
}

float sat(float &val, float Val1, float Val2)
{   
   
    if  ( Val1 < Val2 )
    {
        if ( val < Val1 ) val = Val1;
        else if ( val > Val2 ) val = Val2;
    }
    else
    {
        if ( val < Val2 ) val = Val2;
        else if ( val > Val1 ) val = Val1;
    }
    return val;
}

// CheckSum function: provided by SBG
uint16_t RxTxSerial:: calcCRC(const void *pBuffer, uint16_t bufferSize)
{
  const uint8_t *pBytesArray =  (const uint8_t*)pBuffer;
  uint16_t poly = 0x8408;
  uint16_t crc = 0;
  uint8_t carry;
  uint8_t i_bits;
  uint16_t j;
  for (j =0; j < bufferSize; j++)
  { 
    crc = crc ^ pBytesArray[j];
    for (i_bits = 0; i_bits < 8; i_bits++)
    {
      carry = crc & 1;
      crc = crc / 2;
      if (carry)
      {
        crc = crc^poly;
      } 
    }
  }
  return crc; 
}

void RxTxSerial:: send_in16_t(int16_t *N)
{  
   myPort.write( *( (uint8_t *) N) ); 
   myPort.write( *( (uint8_t *) N + 1) );
}

void RxTxSerial:: sendData_int16_t(uint8_t Ini, uint8_t Fin, int16_t *N, int NbData)
{
      myPort.write(Ini);
      for (int i =0; i< NbData; i++)
      {
          send_in16_t(N+i);
      }
      if (verif)
      {
        int16_t Checksum = calcCRC(N, 2*NbData);
        send_in16_t(&Checksum); 
      }
      
      myPort.write(Fin);
}


bool RxTxSerial:: getData_int16_t(int NbData, uint8_t Ini, uint8_t Fin, int16_t *DATA)
{    
     uint8_t data_raw[2*(NbData+NbDataCRC)+1];
     
     if ( myPort.available() > (2*(NbData+NbDataCRC)+1) )
      {  
          if (myPort.read() == Ini)
          {    
              myPort.readBytes(data_raw,2*(NbData+NbDataCRC)+1);
            
                  if(data_raw[2*(NbData+NbDataCRC)] == Fin ) 
                  {   
                      int16_t val_int16;
                      uint8_t *ptr_int16 = (uint8_t *)&val_int16;
                      
                      for (int i =0; i < NbData; i++) 
                      {
                          *ptr_int16 = data_raw[2*i];
                          *(ptr_int16+1) = data_raw[2*i+1];
                          DATA[i] =  val_int16;
                          
                       }
                      if (verif)
                      {
                         uint16_t verifySum = calcCRC( DATA,2*NbData );
  
                         uint16_t val_uint16;
                         uint8_t *ptr_uint16 = (uint8_t *)&val_uint16;
                          
                         *ptr_uint16 = data_raw[2*NbData];
                         *(ptr_uint16 +1) =  data_raw[2*NbData+1];
                         if ( val_uint16 == verifySum )
                         {
                            return true;
                         }
                         else
                         {
                            return false;
                         }               
                      }
                      else
                      {
                        return true;
                      }
                   }
                   else
                   {
                      return false;
                   }
          }
          else
          {
              return false;
          }
      }
      else
      {
          return false;
      }
}

void RxTxSerial:: send_float(float *N)
{  
   myPort.write( *( (uint8_t *) N) ); 
   myPort.write( *( (uint8_t *) N + 1) );
   myPort.write( *( (uint8_t *) N + 2) );
   myPort.write( *( (uint8_t *) N + 3) );
   
}

void RxTxSerial:: sendData_float(uint8_t Ini, uint8_t Fin, float *N, int NbData)
{
      myPort.write(Ini);
      for (int i =0; i< NbData; i++)
      {
          send_float(N+i);
      }
      if (verif)
      { 
        int16_t Checksum = calcCRC(N, 4*NbData);
        send_in16_t( &Checksum);
      }
      myPort.write(Fin);
      
}

bool RxTxSerial:: getData_float(int NbData, uint8_t Ini, uint8_t Fin, float *DATA)
{    
     uint8_t data_raw[4*(NbData)+2*(NbDataCRC)+1];
     
     if ( myPort.available() > ( 4*(NbData)+2*(NbDataCRC)+1) )
     
      {  
          uint8_t temp_val = myPort.read();
          if (temp_val == Ini)
          {   
              myPort.readBytes( data_raw, 4*(NbData)+2*(NbDataCRC)+1 );
                   
                  if( data_raw[4*(NbData)+2*(NbDataCRC)] == Fin ) 
                  {   
                      float val_float;
                      uint8_t *ptr_float = (uint8_t *)&val_float;
                      
                      for (int i =0; i < NbData ;i++) 
                      {
                          *ptr_float     = data_raw[4*i];
                          *(ptr_float+1) = data_raw[4*i+1];
                          *(ptr_float+2) = data_raw[4*i+2];
                          *(ptr_float+3) = data_raw[4*i+3];
                          DATA[i]        = val_float;  
                       }

                      if (verif)
                      { 
                         uint16_t verifySum = calcCRC( DATA,4*NbData );
  
                         uint16_t val_uint16;
                         uint8_t *ptr_uint16 = (uint8_t *)&val_uint16;
                         
                         *ptr_uint16 = data_raw[4*NbData];
                         *(ptr_uint16 +1) =  data_raw[4*NbData+1];
                         
                         if ( val_uint16 == verifySum )
                         {
                            return true;
                         }
                         else
                         {
                            return false;
                         }               
                      }
                      else
                      {
                        return true;
                      }
                   }
                   else
                   {
                      return false;
                   }
          }
          else
          {
              return false;
          }
      }
      else
      {
          return false;
      }
}


int8_t RxTxSerial:: getData_int16_2(uint8_t Ini1, uint8_t Ini2, uint8_t Fin, int16_t *DATA)
{    
     uint8_t tmp_return = 0; //Nb de données recues
     
     if ( !PACK_WAITING  )
     {  
        NbData = 0;
        tmp_return = 0;
        
        if (myPort.available() > 3-1)
        {
          if ( data_temp2 != Ini1 )
          {
            data_temp1 = myPort.read();
          }
          else
          {
            data_temp1 = data_temp2;
          }
          
          if ( data_temp1 == Ini1 )
          {
            data_temp2 = myPort.read();
            if (  data_temp2 == Ini2 )
            {  
               NbData =  myPort.read(); //Nb de données recues et à verifier avec la bit final
               PACK_WAITING = true;
            }
          }
        }
     } 
     
     if ( PACK_WAITING ) // waiting for data to decrypt
     {  
        tmp_return = 0;
        
        if ( myPort.available() > 2*(NbData+NbDataCRC) )
        {  
            PACK_WAITING =  false;
        
            uint8_t data_raw[2*(NbData+NbDataCRC)+1];
            myPort.readBytes(data_raw,2*(NbData+NbDataCRC)+1);
    
            if(data_raw[2*(NbData+NbDataCRC)] == Fin ) 
            {   
                int16_t val_int16;
                uint8_t *ptr_int16 = (uint8_t *)&val_int16;
                
                for (int i =0; i < NbData; i++) 
                {
                    *ptr_int16 = data_raw[2*i];
                    *(ptr_int16+1) = data_raw[2*i+1];
                    DATA[i] =  val_int16;
                }
                if (verif)
                {  
                   uint16_t verifySum = calcCRC( DATA,2*NbData );
    
                   uint16_t val_uint16;
                   uint8_t *ptr_uint16 = (uint8_t *)&val_uint16;
                   
                   *ptr_uint16 = data_raw[2*NbData];
                   *(ptr_uint16 +1) =  data_raw[2*NbData+1];
                   if ( val_uint16 == verifySum )
                   {
                      tmp_return = NbData;
                   }              
                }
                else
                { 
                  tmp_return = NbData;
                }
            }
        }
        
     }
     
     return tmp_return;

}

void RxTxSerial:: sendData_int16_2(uint8_t Ini1, uint8_t Ini2, uint8_t Fin, int16_t *N, int NbData)
{
      myPort.write(Ini1);
      myPort.write(Ini2);
      myPort.write(NbData);
      for (int i =0; i< NbData; i++)
      {
          send_in16_t(N+i);
      }
      if (verif)
      {
        int16_t Checksum = calcCRC(N, 2*NbData);
        send_in16_t(&Checksum); 
      }
      
      myPort.write(Fin);
}

bool Pid:: reset_int(void)
{
  int_error = 0;
  return true;
}

bool Pid:: set_kp(float val)
{
  kp = val;
  return true;
}

bool Pid:: set_kd(float val)
{
  kd = val;
  return true;
}

bool Pid::set_ki(float val)
{
  ki = val;
  return true;
}

bool Pid:: set_int_dt(float val)
{
  dt = val;
  return true;
}

bool Pid:: set_prop_lim(float val)
{
  prop_lim = val;
  return true;
}

bool Pid:: set_speed_lim(float val)
{
  speed_lim = val;
  return true;
}

bool Pid:: set_int_lim(float val)
{
  int_lim = val;
  return true;
}

bool Pid:: set_reg_lim(float val)
{ 
  reg_lim = val;
  return true;
}

bool Pid:: invert_reg(bool val)
{
  inv_reg = val;
  return true;
}

bool Pid:: feed_pid(float e, float e_s)
{ 
  error = e;
  e_speed = e_s;
  
  compute_prop();
  compute_der();
  compute_int();
  compute_reg();
  
  return true;
}

bool Pid:: update_pid(void)
{
  compute_prop();
  compute_der();
  compute_reg();
  return true;
}

void Pid:: compute_prop(void)
{
  P = kp*error;
  if (prop_lim >= 0 )
  {
    P = constrain(P, -prop_lim, prop_lim);
  }
}

void Pid:: compute_der(void)
{
  D = kd*e_speed;
  if ( speed_lim >= 0)
  {
    D = constrain(D, -speed_lim, speed_lim);
  }
}

void Pid:: compute_int(void)
{ 
  int_error += ki*error*dt;
  
  if ( int_lim >= 0 )
  {
    int_error = constrain(int_error, -int_lim, int_lim);
  }
}

void Pid::compute_reg(void)
{ 
  reg  = P + D + int_error;
  if (reg_lim >= 0 )
  {
    reg = constrain(reg, -reg_lim, reg_lim);
  }
  if (inv_reg)
  {
    reg = - reg;
  }
} 

float Pid:: get_reg(void)
{
  return reg;
}

float Pid:: get_P(void)
{
  return P;
}

float Pid:: get_D(void)
{
  return D;
}

float Pid:: get_I(void)
{
  return int_error;
}

void Deriv:: compute(float val)
{  
   est_x += (est_v - kx * (est_x - val) ) * dt;
   est_v += (-kv * (est_x - val) ) * dt;
   
   if ( lim >= 0 )
   {
    sat(est_v,-lim, lim);
   }

}

void Deriv:: compute_cyc(float val)
{  
   est_x += (est_v - kx * asin(sinf(est_x - val)) ) * dt;
   est_x = ang_pi(est_x);
   est_v += (-kv * asin(sinf(est_x - val)) ) * dt;
   
   if ( lim >= 0 )
   {
    sat(est_v,-lim, lim);
   }

}

void Deriv:: compute2(float val, float val2)
{  
   est_x += (est_v - kx * (est_x - val) ) * dt;
   est_v += (-kv*(est_v - val2) - kx*(est_x - val) ) * dt;
   
   if ( lim >= 0 )
   {
    sat(est_v,-lim, lim);
   }

}

void Deriv:: compute_cyc2(float val, float val2)
{  
   est_x += (est_v - kx * asin(sinf(est_x - val)) ) * dt;
   est_x = ang_pi(est_x);
   est_v += (-kv*(est_v - val2) -kx * asin(sinf(est_x - val)) ) * dt;
   
   if ( lim >= 0 )
   {
    sat(est_v,-lim, lim);
   }

}

void Deriv:: set_kx(float val)
{
   kx = val;
}

void Deriv:: set_kv(float val)
{
   kv = val;
}

float Deriv:: get_est_v(void)
{
  return est_v;
}

float Deriv:: get_est_x(void)
{
  return est_x;
}

void Deriv:: set_lim(float val)
{
   lim = val;
}

vect3 Quat2Euler(float q0,float q1,float q2,float q3,int flag)
{ 
  vect3  Euler_ang;
    if (flag == 1 )
    {   // rotations roll, pitch , yaw
        Euler_ang.x = atan2f( -2*(q2*q3-q0*q1), sq(q0)-sq(q1)-sq(q2)+sq(q3) );
        float aux_sin_pitch = 2*(q0*q2+q1*q3); 
        Euler_ang.y = asinf( sat(aux_sin_pitch,-1.0,1.0) );
        Euler_ang.z = atan2f( -2*(q1*q2-q0*q3), sq(q0)+sq(q1)-sq(q2)-sq(q3) );
    }
    else
    {  // rotations yaw, pitch, roll
        Euler_ang.x = atan2f( 2*(q0*q1+q2*q3), sq(q0)-sq(q1)-sq(q2)+sq(q3) );
        float aux_sin_pitch = 2*(q0*q2-q1*q3); 
        Euler_ang.y = asinf( sat(aux_sin_pitch,-1.0,1.0) );
        Euler_ang.z = atan2f( 2*(q0*q3+q1*q2), sq(q0)+sq(q1)-sq(q2)-sq(q3) );
        
    }

  return Euler_ang;
}

void rotDeg(float &x, float &y, float ang)
{   
    float tmp_x = x, tmp_y = y;
    x = tmp_x*cosf(ang) - tmp_y*sinf(ang);
    y = tmp_x*sinf(ang) + tmp_y*cosf(ang);

}

void mult_quat( quat &q1, quat &q2, quat &q3)
{
    q3.r = q1.r*q2.r - (q1.i*q2.i+q1.j*q2.j+q1.k*q2.k);
    q3.i = q1.r*q2.i + q2.r*q1.i + q1.j*q2.k - q1.k*q2.j;
    q3.j = q1.r*q2.j + q2.r*q1.j + q1.k*q2.i - q1.i*q2.k;
    q3.k = q1.r*q2.k + q2.r*q1.k + q1.i*q2.j - q1.j*q2.i;
}

vect3 Quat2Euler(quat q ,int flag)
{ 
  vect3  Euler_ang;
    if (flag == 1 )
    {   // rotations roll, pitch , yaw
        Euler_ang.x = atan2f( -2*(q.j*q.k-q.r*q.i), sq(q.r)-sq(q.i)-sq(q.j)+sq(q.k) );
        float aux_sin_pitch = 2*(q.r*q.j+q.i*q.k); 
        Euler_ang.y = asinf( sat(aux_sin_pitch,-1.0,1.0) );
        Euler_ang.z = atan2f( -2*(q.i*q.j-q.r*q.k), sq(q.r)+sq(q.i)-sq(q.j)-sq(q.k) );
    }
    else
    {  // rotations yaw, pitch, roll
        Euler_ang.x = atan2f( 2*(q.r*q.i+q.j*q.k), sq(q.r)-sq(q.i)-sq(q.j)+sq(q.k) );
        float aux_sin_pitch = 2*(q.r*q.j-q.i*q.k); 
        Euler_ang.y = asinf( sat(aux_sin_pitch,-1.0,1.0) );
        Euler_ang.z = atan2f( 2*(q.r*q.k+q.i*q.j), sq(q.r)+sq(q.i)-sq(q.j)-sq(q.k) );
        
    }
  return Euler_ang;
}

void MyServo::attach_pin(int pin, int min_val, int max_val, uint16_t period)
{
  pin_pwm = pin;
  min_pwm = min_val;
  max_pwm = max_val;
  PERIOD_PWM = period;
  
  pinMode(pin_pwm,OUTPUT);
  analogWriteResolution(resol);
  
  freq = (PERIOD_PWM > 0) ? 1E6/PERIOD_PWM : 1E6/2000;
  analogWriteFrequency(pin_pwm,freq);
}


void MyServo::attach_pin_us(int pin, int min_us, int max_us, uint16_t period)
{
    pin_pwm = pin;
    PERIOD_PWM = period;
    min_pwm = ((float)min_us /PERIOD_PWM)*(1 << resol);
    max_pwm = ((float)max_us /PERIOD_PWM)*(1 << resol);
    
    pinMode(pin_pwm,OUTPUT);
    analogWriteResolution(resol);
    
    freq = (PERIOD_PWM > 0) ? 1E6/PERIOD_PWM : 1E6/2000;
    analogWriteFrequency(pin_pwm,freq);

}

void MyServo::set_period(int period_PWM)
{
  PERIOD_PWM = period_PWM;
}

void MyServo:: set_init_pwm(uint16_t val)
{
  init_pwm = val;
}

void MyServo::write_pwm( uint16_t pwm_val )
{ 
  if ( pwm_val > max_pwm)
  {
     pwm_val = max_pwm;
  }
  else if (pwm_val < min_pwm )
  {
    pwm_val = min_pwm;
  }
  
  analogWrite(pin_pwm,pwm_val);
}

void MyServo:: set_resol(uint16_t val )
{
  if (val > 12 ) val = 12;
  else if (val <9) val = 9;
  resol = val;
//  analogWriteResolution(val);
  
}

void MyServo:: write_us( uint16_t val )
{ 
  
  uint16_t pwm_val = ((float)val /PERIOD_PWM)*(1 << resol);
  write_pwm(pwm_val);

}

float MyServo:: get_info(int id)
{ 
  float tmp_out = 0;
  if ( id == 1 ) tmp_out = pin_pwm;
  else if ( id == 2 ) tmp_out = min_pwm;
  else if ( id == 3 ) tmp_out = max_pwm;
  else if ( id == 4 ) tmp_out = PERIOD_PWM;
  else if ( id == 5 ) tmp_out = freq;
  else if ( id == 6 ) tmp_out = resol;

  return tmp_out;
   
}

//void MyServo:: change_pwm(void)
//{
//    if( (++count_cycles > NB_CYC_PWM) && PWM_SENT)
//    {
//      tt_high_pwm = micros();
//      count_cycles = 0;
//      PWM_SENT = false;
//    }
//}

//bool MyServo:: begin_monitor(int pinMonit)
//{
//  pinMonitor = pinMonit;
//  pinMode(pinMonitor, INPUT);
//  attachInterrupt(pinMonitor, &change_pwm,RISING);
//
//  return true;
//}
