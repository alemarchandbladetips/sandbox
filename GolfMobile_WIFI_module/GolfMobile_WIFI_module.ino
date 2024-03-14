
#include <ESP8266WiFi.h>
#include <WiFiUDP.h>

#define WIFI_START1 0x42
#define WIFI_START2 0x24

const char* ssid = "Golfmobile";             //!!!!!!!!!!!!!!!!!!!!! modify this
const char* password = "BTE&Vergnet2023";                //!!!!!!!!!!!!!!!!!!!!!modify this

IPAddress adresse = IPAddress(192,168,4,1);
IPAddress controller_adresse;
//WiFiServer server(adresse,60000);
const int port=60000;
WiFiClient client;
WiFiUDP Udp;

uint8_t joyX, joyY, button_cruise, button_brake, serial_data;
uint32_t timer;

char incomingPacket[256];
char accept_packet[] = "OK";
char deny_packet[] = "NO";
char HiPacket[] = "HI";

const int enable_print=1;

/* Just a little test message.  Go to http://192.168.4.1 in a web browser
   connected to this access point to see it.
*/

uint8_t client_connected_flag = 0;

void setup() {
  delay(1000);
  Serial.begin(115200);
  Serial.println();

  WiFi.softAP(ssid, password);

  IPAddress myIP = WiFi.softAPIP();

  Serial.print("AP IP address: ");
  Serial.println(myIP);
  Udp.begin(port);
  //server.begin();
  Serial.println("HTTP server started");
  timer = micros();
}

void loop() {

  if(!client_connected_flag)
  {
    
    int packetSize = Udp.parsePacket();
    if (packetSize)
    {
      controller_adresse = Udp.remoteIP();
      // receive incoming UDP packets
      Serial.printf("Received %d bytes from %s, port %d\n", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());
      int len = Udp.read(incomingPacket, 255);
      if (len > 0)
      {
        incomingPacket[len] = 0;
      }
      Serial.printf("UDP packet contents: %s\n", incomingPacket);
      if( incomingPacket[0]== HiPacket[0] && incomingPacket[1]== HiPacket[1])
      {
        client_connected_flag = 1;
        Serial.printf("client ask connection");
        // send back a reply, to the IP address and port we got the packet from
        Udp.beginPacket(controller_adresse, port);
        Udp.write(accept_packet,2);
        Udp.endPacket();
      }
    }
  }
  if(client_connected_flag)
  {
    int packetSize = Udp.parsePacket();
    if (packetSize)
    {
      // receive incoming UDP packets
      //Serial.printf("Received %d bytes from %s, port %d\n", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());
      int len = Udp.read(incomingPacket, 255);
      if (len > 0)
      {
        incomingPacket[len] = 0;
      }
      if(packetSize==2)
      {
        if( incomingPacket[0]== HiPacket[0] && incomingPacket[1]== HiPacket[1])
        {
          Serial.printf("client ask connection");
          if(Udp.remoteIP() == controller_adresse)
          {
            // send back a reply, to the IP address and port we got the packet from
            Udp.beginPacket(controller_adresse, port);
            Udp.write(accept_packet,2);
            Udp.endPacket();
          } else
          {
            Udp.beginPacket(controller_adresse, port);
            Udp.write(deny_packet,2);
            Udp.endPacket();
          }
        }
      }
      if(packetSize==6 && incomingPacket[0] == WIFI_START1 && incomingPacket[1] == WIFI_START2)
      {
        Udp.beginPacket(controller_adresse, port);
        Udp.write(accept_packet,2);
        Udp.endPacket();

        joyX = incomingPacket[2];
        joyY = incomingPacket[3];
        button_cruise = incomingPacket[4];
        button_brake = incomingPacket[5];


        Serial.write(WIFI_START1);
        Serial.write(WIFI_START2);
        Serial.write(joyX);
        Serial.write(joyY);
        Serial.write(button_cruise);
        Serial.write(button_brake);
        
/*
        Serial.print(WIFI_START1);Serial.print("\t");
        Serial.print(WIFI_START2);Serial.print("\t");
        Serial.print(joyX);Serial.print("\t");
        Serial.print(joyY);Serial.print("\t");
        Serial.print(button_cruise);Serial.print("\t");
        Serial.print(button_brake);Serial.print("\t");
        Serial.println("\t");
*/

        timer = micros();

        
      }
    }
  }
}