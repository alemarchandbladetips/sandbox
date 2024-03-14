#include "Arduino.h"
#include "GlobalVariables.h"

#ifndef FUNCTIONS_H
  #define FUNCTIONS_H

  void Mean(float A[], float* prom, int NbData, int posf);
  void HistoriqueVal(float val, float A[]);
  

  // print function
  void print(int nb, String str[], float val[], char *term );
  void print(int nb, float val[], char *term );
  void println(char* var);
  void print(char *var, char *term);
  void print(char *str1, float val1, char *term, char *sep ="\t");
  void print(char *str1, char *str2, float val1, float val2, char *term, char *sep = "\t");
  void print(char *str1, char *str2, char *str3, float val1, float val2, float val3, char *term , char *sep = "\t");
 
  void println(float val);
  void print(float val, char *term);
  void print(float val1, float val2, char *term, char *sep = "\t");
  void print(float val1, float val2, float val3, char *term , char *sep = "\t");

  //elapsed functions
  bool elapsed_us(uint32_t &time, uint32_t period);
  bool elapsed_ms(uint32_t &time, uint32_t period);

  //void getPosSpeedAiles(void);
  
  float signo(float val);
  
  float ang_180(float a1);
  
  float ang_pi(float a1);
  
  float lineVal(float x_i, float x_f, float y_i, float y_f, float x);
  
  float interpol(float* A, float* B, int nb, float val);
  
  void HistoriqueN(float val, float A[], uint8_t N);
  
  // void send_in16_t(Stream &myport, int16_t *N);
  
  // void sendData_int16_t(Stream &myPort, uint8_t Ini, uint8_t Fin, int16_t *N, int NbData, bool verif);
  
  
  uint16_t calcCRC(const void *pBuffer, uint16_t bufferSize);
  
  // bool recep_int16_t(Stream &myPort, int NbData, uint8_t Ini, uint8_t Fin, int16_t *DATA, bool verif);
  
  class ValMean {
    private :
      int pos = 0;
      bool maxPos = false;
      float sum = 0;
      int const nbElem;
      float mean = 0;
      float *A;
    
    public:
      ValMean(int val1): nbElem(val1) 
      {   
          if (val1 > 1 )
          {
              A = new float[val1];
              for (int i=0; i < val1 ;i++)
              { 
                A[i] = 0;
              } 
          }
          else
          {
            A = new float[1];
            A[0] = 0;
          }
          
      }
      
      float getMean(float val);
  };
  
  class FilterVal {
    private:
      float *A;
      float filterval = 0;
      int filtertype;
    public:
      FilterVal(int x): filtertype(x)
      {
        if (x == 1 || x == 2)
        {
          A = new float[2 * x];
          for (int i=0; i < 2*x ;i++)
          {
            A[i] = 0;
          }
        }
        else
        {
          A = new float[1];
          A[0] = 0;
        }
      }
      
      float getVal(float val);
  };
  
  float getMod(float val1, float val2);
  float sat(float & val, float Val1, float Val2);
  
  class RxTxSerial{
      private:
        uint8_t data_temp1 = 0;
        uint8_t data_temp2 = 0;
        int NbData = 0;
        int NbDataCRC = 0;
        bool PACK_WAITING = false;
        HardwareSerial &myPort;
        // Stream &myPort;
        int8_t nb_data=0;
        bool verif;
        
      public:
        // RxTxSerial(Stream &myport_ptr, bool verif) : myPort(myport_ptr), verif(verif)
        RxTxSerial(HardwareSerial &myport_ptr, bool verif) : myPort(myport_ptr), verif(verif)
        {
          if (verif) NbDataCRC = 1; else NbDataCRC = 0;
        }
        void begin(int baudrate = 115200);
        void send_in16_t(int16_t *N);
        
        void sendData_int16_t(uint8_t Ini, uint8_t Fin, int16_t *N, int NbData);
        
        bool getData_int16_t(int NbData, uint8_t Ini, uint8_t Fin, int16_t *DATA);
        
        void sendData_int16_2(uint8_t Ini1, uint8_t Ini2, uint8_t Fin, int16_t *N, int NbData);
  
        int8_t getData_int16_2( uint8_t Ini1, uint8_t Ini2, uint8_t Fin, int16_t *DATA);
  
        void send_float(float *N);
  
        bool getData_float(int NbData, uint8_t Ini, uint8_t Fin, float *DATA);
        
        void sendData_float(uint8_t Ini, uint8_t Fin, float *N, int NbData);
        
        uint16_t calcCRC(const void *pBuffer, uint16_t bufferSize);
          
  };
  
  class Pid{
    private:
      float kp = 0;
      float kd =0;
      float ki = 0;
      float error = 0;
      float reg_lim = -1;
      float prop_lim = -1;
      float speed_lim = -1;
      float int_lim = -1;
      float int_error = 0;
      float e_speed = 0;
      float P = 0;
      float D = 0;
      float reg = 0;
      float dt = 0.010;
      bool inv_reg = false;
      void compute_int(void);
      void compute_der(void);
      void compute_prop(void);
      void compute_reg(void);
      
    public:
      Pid(float _kp, float _kd, float _ki, float int_e):kp(_kp), kd(_kd), ki(_ki), int_error(int_e) {}
      bool reset_int(void); //reset val of integral and 
      bool set_kp(float val);
      bool set_kd(float val);
      bool set_ki(float val);
      bool set_int_dt(float val);
      bool feed_pid(float error, float e_speed);
      bool set_reg_lim(float val);
      bool set_int_lim(float val);
      bool set_speed_lim(float val);
      bool set_prop_lim(float val);
      float get_reg(void);
      float get_P(void);
      float get_D(void);
      float get_I(void);
      bool update_pid(void);
      bool invert_reg(bool val);
  };

class Deriv{
  private:
    float kx = 1;
    float kv = 1;
    float lim = -1;
    float est_x = 0;
    float est_v = 0;
    float dt = 0.010;
    
  public:
    Deriv( float k1, float k2):kx(k1), kv(k2) {}
    void compute(float val);
    void compute_cyc(float val);
    void compute2(float val,float val2);
    void compute_cyc2(float val,float val2);
    float get_est_v(void);
    float get_est_x(void);
    void set_kx(float val);
    void set_kv(float val);
    void set_lim(float val);
    void set_dt(float val);
  
};

vect3 Quat2Euler(float q0,float q1,float q2,float q3,int flag);
void rotDeg(float &x, float &y, float ang);
void mult_quat( quat &q1, quat &q2, quat &q3);
vect3 Quat2Euler(quat q ,int flag);

class MyServo
{
  private:
    int pin_pwm;
    uint16_t max_pwm;
    uint16_t min_pwm;
    uint16_t init_pwm = 1023;
    uint16_t PERIOD_PWM;
    float freq;
    uint16_t resol = 11;
//    int pinMonitor;
//    volatile uint32_t tt_high_pwm;
//    volatile int count_cycles;
//    int NB_CYC_PWM = 3;
//    volatile bool PWM_SENT = true;
    
    
  public:
    MyServo() {}
    MyServo(int _pin_pwm, int _min_pwm, int _max_pwm, uint16_t period): pin_pwm(_pin_pwm), min_pwm(_min_pwm), max_pwm(_max_pwm), PERIOD_PWM(period) 
    {
      if (PERIOD_PWM>0) freq = 1.0/PERIOD_PWM;
      //attach_pin(pin_pwm,min_pwm,max_pwm,PERIOD_PWM);
    }
    void attach_pin(int pin, int min_val, int max_val, uint16_t period);
    void attach_pin_us(int pin, int min_us, int max_us, uint16_t period);
    void set_period(int period_PWM);
    void write_pwm( uint16_t pwm_val );
//    bool begin_monitor(int pinMonit);
//    void change_pwm(void);
    void set_init_pwm(uint16_t val);
    void set_resol(uint16_t val);
    void write_us( uint16_t  val);
    float get_info(int id);
//    void setNbCycle2Count(int val);
};

#endif
