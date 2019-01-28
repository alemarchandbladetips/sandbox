#include <NXPMotionSense.h>
#include <NXPMotionSense1.h>
#include <Wire.h>
#include <EEPROM.h>
#include <util/crc16.h>

NXPMotionSense imu;
NXPMotionSense1 imu1;

int loopcount = 0;

float alti_filt;
float alti_filt1;
float alti_offset = 0;
float a_filt = 0.002;
uint8_t init_status,init_status1 = 0;
uint32_t t_init;

void setup() {
  Serial.begin(115200);
  while (!Serial) ; // wait for serial port open
  Serial.println("GO");
  while (!imu.begin()) {
    Serial.println("config error IMU0");
    delay(1000);
  }
  Serial.println("config IMU0 OK");
  while (!imu1.begin()) {
    Serial.println("config error IMU1");
    delay(1000);
  }
  Serial.println("config IMU1 OK");
  alti_filt = 0;
  alti_filt1 = 0;
  t_init = millis();
}

void loop() {
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
  float alti,alti1;
  int16_t temperature;

  // get and print uncalibrated data
  if (imu.available()) {
    imu.readMotionSensor(ax, ay, az, gx, gy, gz, mx, my, mz, alti);
    alti_filt = a_filt*alti+(1-a_filt)*alti_filt;
    if(alti!=0 && init_status==0)
    {
      init_status = 1;
      alti_filt = alti;
      t_init = millis();
    }
    Serial.print(alti);
    Serial.print(' ');
    Serial.print(alti_filt);
    Serial.println(' ');
    //Serial.println();
  }
  if (imu1.available()) {
    imu1.readMotionSensor(ax, ay, az, gx, gy, gz, mx, my, mz, alti1);
    alti_filt1 = a_filt*alti1+(1-a_filt)*alti_filt1;
    if(alti1!=0 && init_status1==0)
    {
      init_status1 = 1;
      alti_filt1 = alti1;
      t_init = millis();
    }
    /*Serial.print(alti1);
    Serial.print(' ');
    Serial.print(alti_filt1);
    Serial.println();*/
    //Serial.println(alti_filt1-alti_filt-alti_offset);
  }

  if(init_status==1 && init_status1==1 && (millis()-t_init) > 30000)
  {
    alti_offset = alti_filt1-alti_filt;
    init_status = 2;
    init_status = 2;
    a_filt = 0.025;
  }
  
}
