/*
  Serial to WiFi adapter for EQAstro Arduino Telescope Motor Board - built on Serial to WiFi adapter for Skywatcher telescopes code by Vladimir Atehortua
  Copyright (c) 2020 Michael Norton. All rights reserved.
  This program is free software: you can redistribute it and/or modify
  it under the terms of the version 3 GNU General Public License as
  published by the Free Software Foundation.
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License along with this program.
  If not, see <http://www.gnu.org/licenses/>
*/



/**
    Hardware used:
    NodeMCU ESP8266 development board (version 12E) I used the HiLetgo model https://www.amazon.com/gp/product/B010O1G1ES 
    100 Ohm resistor
    50 Ohm resistor (or two 100 Ohm resistors in parallel)
    RJ11 Pinout:
        --- No Connection
        --- GND
        --- "TX" to a 50 Ohm resistor and then to NodeMCU's pin D8 (GPIO_15)
        --- VCC 12V (to a 5V drop-down volate regulator to power the NodeMCU thrugh it's Vin pin)
        --- "RX" to a 100 Ohm resistor and then to NodeMCU's pin D7 (GPIO_13)
        --- No Connection
    
    ESP8266 pinout:
     GND:  To the GND pin of the RJ11 connector (and to GND of your chosen power source)
     D7:   to a 50 Ohm resstor and then to the "TX" pin on the RJ11 connector
     D8:   to a 100 Ohm resistor and then to the "RX" pin on the RJ11 connector

     When not connected to a computer via USB:
     Vin:  5V~9V from any power source (you can use a step down from 12V to ~5V to power the NodeMCU from most skywatcher mounts)
*/

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <SoftwareSerial.h>  // used to debug the project, sending log messages to the Arduino IDE serial monitor

#define mountBaudRate 9600
#define loggerBaudRate 9600

#define bufferSize 8192
#define SerialBufferSize 64
SoftwareSerial* logger;

#define WiFi_Access_Point_Name "SynScan_WiFi_1234"   // Name of the WiFi access point this device will create for your tablet/phone to connect to.
#define udpPort 11880 // UDP udpPort expected by SynScan
WiFiUDP udp;
IPAddress remoteIp;
int UDPremoteudpPort;

uint8_t udpBuffer[bufferSize];
uint8_t udpIndex = 0;

uint8_t serialBuffer[SerialBufferSize];
uint8_t serialIndex = 0;

byte data = 0;

boolean ignore = true;  // Because the mount connection seems to share the wire for RX and TX, commands sent to the mount are recieved back, as an "echo", and must be ignored.

void setup() {

  delay(1000);
  Serial.begin(mountBaudRate);
  Serial.swap();  // This will set the hardware UART RX to pin D7 (GPIO_13) and TX to pin D8 (GPIO_15).
  logger = new SoftwareSerial(3, 1); // The original pins (RX/GPIO_3 and TX/_GPIO1) are now used by softwareserial for logging debug data to a computer
  logger->begin(loggerBaudRate);
  logger->println("\n\nSkywatcher WiFi adatper powered on");


  WiFi.mode(WIFI_AP);
  IPAddress ip(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.softAPConfig(ip, ip, subnet);
  WiFi.softAP(WiFi_Access_Point_Name); // configure ssid and password for softAP
  udp.begin(udpPort);
  logger->print("Exposing UDP port ");
  logger->print(udpPort);
  logger->print(" on IP " );
  logger->println(WiFi.softAPIP());
}

void loop()
{
  int packetSize = udp.parsePacket();
  int SerialSize = 0;


  
  if (packetSize > 0)                     // when data arrives via WiFi
      {
        // take note of address and port to send our response to:
        remoteIp = udp.remoteIP();
        UDPremoteudpPort = udp.remotePort();
    
        udp.read(udpBuffer, bufferSize); //read the incoming data
        for (int j = 0; j < packetSize; j++)  // write it to the log for debugging purposes
              {
                logger->print("From IP: ");
                logger->print(remoteIp);
                logger->print(" / Port: ");
                logger->print(UDPremoteudpPort);
                logger->print(" - UDP Data: ");
                logger->println(udpBuffer[j]);
              }
        Serial.write(udpBuffer, packetSize);  // forward the recieved data straight to the serial connection to the telescope mount
        ignore = true;    // we need to ignore the first characters that we get from the telescope mount (an echo of our command / garbage) until we get the "=" character that signals the beginning of the actual response
        delay (20);
      }
      
  SerialSize = Serial.available();  // Test for Serial Data Received

  if (SerialSize > 0) 
  {                                                     // when data arrives from the mount via the serial port
      logger ->print("Serial Bytes Received: ");
      logger ->println(SerialSize);
      serialIndex=0;
      
      for (int i = 0; i<SerialSize; i++)
            { 
                byte data=Serial.read();
                logger ->print("From MCB: ");
                logger ->println(data);
                serialBuffer[serialIndex] = data;
                serialIndex++;
             }
                                                     // Now we send the message recieved from the telescope mount, as an UDP packet to the client app (via WiFi):
      udp.beginPacket(remoteIp, UDPremoteudpPort); 
      for (int j = 0; j < SerialSize; j++)
        {
          logger->print("UDP Sending: ");
          logger->println(serialBuffer[j]);
        }
      udp.write(serialBuffer, serialIndex);
      udp.endPacket();
      yield();
      serialIndex = 0;
  }
}