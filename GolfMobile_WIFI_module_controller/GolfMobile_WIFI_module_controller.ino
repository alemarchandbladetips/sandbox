
#include <WiFi.h>

#define WIFI_START1 0x42
#define WIFI_START2 0x24

const char* ssid = "Golfmobile";             
const char* password = "BTE&Vergnet2023";

const char* host = "192.168.4.1";
const uint16_t port = 60000;

IPAddress adresse = IPAddress(192,168,4,1);

WiFiClient client;
WiFiUDP Udp;

char incomingPacket[256];
char accept_packet[] = "OK";
char deny_packet[] = "NO";
char HiPacket[] = "HI";
char data_packet[6];

uint8_t joyX, joyY, button_cruise, button_brake, button_link, serial_data;

uint8_t count, connected2AP_flag, connected2UDP_flag, link_button = 0;
uint32_t timer;

void setup(void) {

  pinMode(4, INPUT_PULLUP);  // initialize onboard LED as output
  Serial.begin(115200);


  uint32_t realSize = ESP.getFlashChipRealSize();
  uint32_t ideSize = ESP.getFlashChipSize();
  FlashMode_t ideMode = ESP.getFlashChipMode();

  Serial.printf("Flash real id:   %08X\n", ESP.getFlashChipId());
  Serial.printf("Flash real size: %u\n\n", realSize);

  Serial.printf("Flash ide  size: %u\n", ideSize);
  Serial.printf("Flash ide speed: %u\n", ESP.getFlashChipSpeed());
  Serial.printf("Flash ide mode:  %s\n", (ideMode == FM_QIO ? "QIO" : ideMode == FM_QOUT ? "QOUT" : ideMode == FM_DIO ? "DIO" : ideMode == FM_DOUT ? "DOUT" : "UNKNOWN"));

  if (ideSize != realSize) {
    Serial.println("Flash Chip configuration wrong!\n");
  } else {
    Serial.println("Flash Chip configuration ok.\n");
  }

  // Set WiFi to station mode and disconnect from an AP if it was previously connected
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  button_link = 1;
}

int8_t waitOK()
{
  uint32_t timer = micros();  
  int packetSize = 0;
  Udp.parsePacket();

  while( packetSize!=2 )
  {
    packetSize = Udp.parsePacket();
    if(micros()-timer > 10000)
    {
      Serial.println("Timeout");
      return -1;
    }
  }
  //Serial.println(micros()-timer);
  // receive incoming UDP packets
  //Serial.printf("Received %d bytes from %s, port %d\n", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());
  Udp.read(incomingPacket, 255);
  //Serial.printf("UDP packet contents: %s\n", incomingPacket);
  if( incomingPacket[0]== accept_packet[0] && incomingPacket[1]== accept_packet[1])
  {
    //Serial.println("OK");
    return 1;
  }
  if( incomingPacket[0]== deny_packet[0] && incomingPacket[1]== deny_packet[1])
  {
    //Serial.println("NO");
    return 0;
  }
  return -1;
}

int8_t connect_to_Golfmobile()
{
  int n = WiFi.scanNetworks();
  Serial.println("scan done");
  Serial.print(n);
  Serial.println(" networks found");
  for (int i = 0; i < n; ++i)
  {
    if(WiFi.SSID(i) == ssid)
    {
      WiFi.begin(ssid, password);
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
        Udp.begin(port);
        Udp.beginPacket(adresse,port);
        Udp.write(HiPacket);
        Udp.endPacket();
        if(waitOK()==1)
        {
          while(Serial.available())
          {
            Serial.read();
          }
          return 1;
        }
        j++;
      }
      WiFi.disconnect();
    }
  }
  return 0;
}


void loop() {

    // WiFi.scanNetworks will return the number of networks found
  
  if(!connected2AP_flag && button_link == 0)
  {
    connected2AP_flag = connect_to_Golfmobile();
  }
  

  if(!WiFi.isConnected())
  {
    connected2AP_flag = 0;
  }
  
  if(Serial.available()>6)
  {
    //Serial.println("hfhf");
    serial_data = Serial.read();
    //Serial.println(serial_data);
    if(serial_data == WIFI_START1)
    {
      serial_data = Serial.read();
      if(serial_data == WIFI_START2)
      {
        joyX = Serial.read();
        joyY = Serial.read();
        button_cruise = Serial.read();
        button_brake = Serial.read();
        button_link = Serial.read();

        Serial.print(micros()-timer);Serial.print("\t");
        Serial.print(WIFI_START1);Serial.print("\t");
        Serial.print(WIFI_START2);Serial.print("\t");
        Serial.print(joyX);Serial.print("\t");
        Serial.print(joyY);Serial.print("\t");
        Serial.print(button_cruise);Serial.print("\t");
        Serial.print(button_brake);Serial.print("\t");
        Serial.print(button_link);Serial.print("\t");
        Serial.println("\t");
        timer = micros();
        
        data_packet[0] = WIFI_START1;
        data_packet[1] = WIFI_START2;
        data_packet[2] = joyX;
        data_packet[3] = joyY;
        data_packet[4] = button_cruise;
        data_packet[5] = button_brake;
        if(connected2AP_flag)
        {
          Udp.beginPacket(adresse,port);
          Udp.write(data_packet,6);
          Udp.endPacket();
          if(waitOK()!=1)
          {
            Serial.println("Not recieved");
          } 
        }
      }
    }
  }
}

