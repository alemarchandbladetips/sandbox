/*
GolfMobile_lecture_Joystick.ino
GolfMobile project
Created by Antoine Lemarchand, 2023

This code is embeded in the GolfMobile remote controller.

This code is used to read the joystick inputs and send them to the GolfMobile.
It handles the wifi connection between the ESP32 and the GolfMobile.
It also handles the battery level of the remote and show it with the leds.
The leds are also used to show the state of the wifi pairing with the GolfMobile.

*/

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>


// Usefull when using the nunchuck
#include <WiiChuck.h>
Accessory nunchuck;


// Defining input pins
//#define GM_JOYX_PIN 2
//#define GM_JOYY_PIN 4
#define GM_JOYX_PIN 35
#define GM_JOYY_PIN 34
#define GM_CRUISE_PIN 15
#define GM_BRAKE_PIN 12
#define GM_LINK_PIN 16
//#define GM_BATT_PIN 13
#define GM_BATT_PIN 33
#define GM_JOY_FULL_SCALE 2000.0

// defining led pins
#define GM_LED0_PIN 25
#define GM_LED1_PIN 26
#define GM_LED2_PIN 32
uint8_t led_connexion = 0;

//// defining wifi protocole 
// packet start
#define WIFI_START1 0x42
#define WIFI_START2 0x24

// defining ssid and password
const char* ssid = "Golfmobile";             
const char* password = "BTE&Vergnet2023";

IPAddress adresse = IPAddress(192,168,4,1);

// declaring udp socket
WiFiUDP udp_socket;
const uint16_t port = 60000;

uint8_t incomingPacket[256];
uint8_t accept_packet[] = "OK";
uint8_t deny_packet[] = "NO";
uint8_t HiPacket[] = "HI";
uint8_t data_packet[6];

//// led management
uint16_t led_counter;

//// battery management
#define GM_BATT_THRESHOLD_0 3.8
#define GM_BATT_THRESHOLD_1 4.0
#define GM_BATT_THRESHOLD_EPS 0.05

uint16_t batt_level_raw;
float batt_voltage = 4;
uint8_t batt_level = 0;

//// Input data treatment
uint8_t cruise_button, brake_button, link_button = 0;

uint16_t joyX_raw, joyY_raw;
uint8_t joyX_U8, joyY_U8;

float joyX_scale, joyY_scale, joyX0, joyY0 = 0;

#define GM_JOY_DEAD_ZONE 50

//// flags
uint8_t count, connected2AP_flag, connected2UDP_flag;
uint32_t connexion_counter = 0;


//// Global time management
uint32_t timer;
uint32_t dt_us = 20000;


void input_read(void);
void input_scale(void);
void led_blink(uint16_t period_sample, float duty_cycle);
void led_switch_all(uint8_t status);
void led_switch_one(uint8_t led_num, uint8_t status);
void led_show_batt_level(void);
void print_inputs(void);
int8_t wait_Golfmobile(void);
int8_t connect_to_Golfmobile(void);


void setup() {
  Serial.begin(115200);

  
  // Usefull when using the nunchuck
  nunchuck.begin();
	if (nunchuck.type == Unknown) {
		nunchuck.type = NUNCHUCK;
	}

  
  //// Defining PINS

  pinMode(GM_JOYX_PIN,INPUT);
  pinMode(GM_JOYY_PIN,INPUT);
  pinMode(GM_BATT_PIN,INPUT);

  pinMode(GM_CRUISE_PIN,INPUT_PULLUP);
  pinMode(GM_BRAKE_PIN,INPUT_PULLUP);
  pinMode(GM_LINK_PIN,INPUT_PULLUP);

  pinMode(GM_LED0_PIN,OUTPUT);
  pinMode(GM_LED1_PIN,OUTPUT);
  pinMode(GM_LED2_PIN,OUTPUT);

  //// Wifi setup
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  //// Joystick zero setting
  led_switch_all(1);
  delay(100);
  led_switch_all(0);
  for(int j=0;j<10;j++)
  {
    for(int i=0;i<5;i++)
    {
      joyX0 += analogRead(GM_JOYX_PIN)/50.0;
      joyY0 += analogRead(GM_JOYY_PIN)/50.0;
      delay(10);
    }
    led_switch_all(led_connexion);
    led_connexion = !led_connexion;
  }
  led_switch_all(1);

  joyX_scale = min((float)(4096.0-joyX0-GM_JOY_DEAD_ZONE), joyX0-GM_JOY_DEAD_ZONE);
  joyY_scale = min((float)(4096.0-joyY0-GM_JOY_DEAD_ZONE), joyY0-GM_JOY_DEAD_ZONE);

  Serial.print("joystick :");Serial.print("\t");
  Serial.print(joyX0);Serial.print("\t");
  Serial.print(joyY0);Serial.print("\t");
  Serial.print(joyX_scale);Serial.print("\t");
  Serial.print(joyY_scale);Serial.print("\t");


  timer = micros();

}

void loop() {

  if( micros()-timer > dt_us)
  {
    timer = micros();
    led_counter ++;

    input_read();
    //input_scale();
    print_inputs();
    
    if(!connected2AP_flag)// && link_button == 1)
    {
      led_switch_one(0,0);
      led_switch_one(1,1);
      led_switch_one(2,0);
      connected2AP_flag = connect_to_Golfmobile();
    }
    
    if(!WiFi.isConnected())
    {
      connected2AP_flag = 0;
    }

    data_packet[0] = WIFI_START1;
    data_packet[1] = WIFI_START2;
    data_packet[2] = joyX_U8;
    data_packet[3] = joyY_U8;
    data_packet[4] = cruise_button;
    data_packet[5] = brake_button;

    if(connected2AP_flag)
    {
      
      udp_socket.beginPacket(adresse,port);
      udp_socket.write(data_packet,6);
      udp_socket.endPacket();
      if(wait_Golfmobile()!=1)
      {
        Serial.println("Not recieved");
        connexion_counter++;
      } else
      {
        connexion_counter = 0;
      }
      if (connexion_counter>10)
      {
        led_blink(10, 0.5);
      } else
      {
        led_show_batt_level();
      }
    } 
  }
}


void input_read(void)
{

  
    // Usefull when using the nunchuck
    nunchuck.readData();
    joyX_U8 = nunchuck.getJoyX();
    joyY_U8 = nunchuck.getJoyY();
    cruise_button = nunchuck.getButtonC();
    brake_button = nunchuck.getButtonZ();
    
/*
  joyX_raw = analogRead(GM_JOYX_PIN);
  joyY_raw = analogRead(GM_JOYY_PIN);
  batt_level_raw = analogRead(GM_BATT_PIN);
  cruise_button = !digitalRead(GM_CRUISE_PIN);
  brake_button = !digitalRead(GM_BRAKE_PIN);
  link_button = !digitalRead(GM_LINK_PIN);
  */
}

void input_scale(void)
{
  //// removing dead zone
  // X axis
  if( abs(joyX_raw - joyX0) < GM_JOY_DEAD_ZONE)
  {
    joyX_raw = joyX0;
  } else if (joyX_raw>joyX0)
  {
    joyX_raw -= GM_JOY_DEAD_ZONE;
  } else
  {
    joyX_raw += GM_JOY_DEAD_ZONE;
  }

  // Y axis
  if( abs(joyY_raw - joyY0) < GM_JOY_DEAD_ZONE)
  {
    joyY_raw = joyY0;
  } else if (joyY_raw>joyY0)
  {
    joyY_raw -= GM_JOY_DEAD_ZONE;
  } else
  {
    joyY_raw += GM_JOY_DEAD_ZONE;
  }

  // Converting to uint8_t
  joyX_U8 = (uint8_t)(127.0*constrain((joyX_raw - joyX0)/(joyX_scale),-1,1)+127.0);
  joyY_U8 = (uint8_t)(-127.0*constrain((joyY_raw - joyY0)/(joyY_scale),-1,1)+127.0);

  batt_voltage = 0.99*batt_voltage + 0.01*(batt_level_raw/2048.0*3.6 + 0.1);

}

void led_blink(uint16_t period_sample, float duty_cycle)
{
  
  if(led_counter<=(duty_cycle*period_sample))
  {
    digitalWrite(GM_LED0_PIN,1);
    digitalWrite(GM_LED1_PIN,1);
    digitalWrite(GM_LED2_PIN,1);
  } else
  {
    digitalWrite(GM_LED0_PIN,0);
    digitalWrite(GM_LED1_PIN,0);
    digitalWrite(GM_LED2_PIN,0);
  }

  if(led_counter>period_sample)
  {
    led_counter = 0;
  }
}

void led_switch_all(uint8_t status)
{
  digitalWrite(GM_LED0_PIN,status);
  digitalWrite(GM_LED1_PIN,status);
  digitalWrite(GM_LED2_PIN,status);
}

void led_switch_one(uint8_t led_num, uint8_t status)
{
  if(led_num==0)
  {
    digitalWrite(GM_LED0_PIN,status);
  }
  if(led_num==1)
  {
    digitalWrite(GM_LED1_PIN,status);
  }
  if(led_num==2)
  {
    digitalWrite(GM_LED2_PIN,status);
  }
}

void led_show_batt_level(void)
{
  if(batt_level == 0)
  {
    if(batt_voltage > (GM_BATT_THRESHOLD_0+GM_BATT_THRESHOLD_EPS) )
    {
      batt_level = 1;
    }
  }

  if(batt_level == 1)
  {
    if(batt_voltage > (GM_BATT_THRESHOLD_1+GM_BATT_THRESHOLD_EPS) )
    {
      batt_level = 2;
    }
    if(batt_voltage < (GM_BATT_THRESHOLD_0-GM_BATT_THRESHOLD_EPS) )
    {
      batt_level = 0;
    }
  }

  if(batt_level == 2)
  {
    if(batt_voltage < (GM_BATT_THRESHOLD_1-GM_BATT_THRESHOLD_EPS) )
    {
      batt_level = 1;
    }
  }

  digitalWrite(GM_LED0_PIN,1);
  digitalWrite(GM_LED1_PIN,0);
  digitalWrite(GM_LED2_PIN,0);

  if( batt_level > 0 )
  {
    digitalWrite(GM_LED1_PIN,1);
  } 
  if( batt_level > 1 )
  {
    digitalWrite(GM_LED2_PIN,1);
  }
}

void print_inputs(void)
{
  Serial.print("joystick :");Serial.print("\t");
  Serial.print(joyX_raw);Serial.print("\t");
  Serial.print(joyY_raw);Serial.print("\t");
  Serial.print(joyX_scale);Serial.print("\t");
  Serial.print(joyY_scale);Serial.print("\t");
  Serial.print(joyX_U8);Serial.print("\t");
  Serial.print(joyY_U8);Serial.print("\t");
  
  Serial.print("batt :");Serial.print("\t");
  Serial.print(batt_level_raw);Serial.print("\t");
  Serial.print(batt_voltage);Serial.print("\t");
  Serial.print(batt_level);Serial.print("\t");
  Serial.print("buttons :");Serial.print("\t");
  Serial.print(cruise_button);Serial.print("\t");
  Serial.print(brake_button);Serial.print("\t");
  Serial.print(link_button);Serial.print("\t");
  Serial.print("flags:");Serial.print("\t");

  Serial.println("\t");
}

int8_t wait_Golfmobile(void)
{
  uint32_t timer = micros();  
  int packetSize = 0;
  packetSize = udp_socket.parsePacket();

  Serial.println( packetSize);

  while( packetSize!=2 )
  {
    packetSize = udp_socket.parsePacket();
    if(micros()-timer > 100000)
    {
      Serial.println("Timeout");
      return -1;
    }
  }
  //Serial.println(micros()-timer);
  // receive incoming UDP packets
  Serial.printf("Received %d bytes from %s, port %d\n", packetSize, udp_socket.remoteIP().toString().c_str(), udp_socket.remotePort());
  udp_socket.read(incomingPacket, 255);
  //Serial.printf("UDP packet contents: %s\n", incomingPacket);
  if( incomingPacket[0]== accept_packet[0] && incomingPacket[1]== accept_packet[1])
  {
    Serial.println("OK");
    return 1;
  }
  if( incomingPacket[0]== deny_packet[0] && incomingPacket[1]== deny_packet[1])
  {
    Serial.println("NO");
    return 0;
  }
  return -1;
}

int8_t connect_to_Golfmobile(void)
{
  int n = WiFi.scanNetworks();
  Serial.println("scan done");
  Serial.print(n);
  Serial.println(" networks found");
  for (int i = 0; i < n; ++i)
  {
    //Serial.println("tic");
    if(WiFi.SSID(i) == ssid)
    {
      WiFi.begin(ssid, password);
      led_switch_one(0,0);
      led_switch_one(1,0);
      led_switch_one(2,1);
      while (WiFi.status() != WL_CONNECTED) 
      {
        delay(500);
        Serial.print(".");
      }
      Serial.println("WiFi connected to Golfmobile");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());

      int j = 0;

      while(j<5)
      {
        udp_socket.begin(port);
        udp_socket.beginPacket(adresse,port);
        udp_socket.write(HiPacket,2);
        udp_socket.endPacket();
        if(wait_Golfmobile()==1)
        {
          while(Serial.available())
          {
            Serial.read();
          }
          led_switch_all(1);
          delay(100);
          led_switch_all(0);
          delay(100);
          led_switch_all(1);
          delay(100);
          led_switch_all(0);
          delay(100);
          led_switch_all(1);
          delay(100);
          led_switch_all(0);
          return 1;
        }
        j++;
      }
      WiFi.disconnect();
    }
  }
  return 0;
}