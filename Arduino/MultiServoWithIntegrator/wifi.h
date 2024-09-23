#include <WiFiEspAT.h>
#include <IPAddress.h>

#ifndef WIFI_H
#define WIFI_H

// WiFi information
char         ssid[] = "TRENDnet";
char         pass[] = "";
int          status = WL_IDLE_STATUS; 
IPAddress    server(10, 42, 0, 197);
WiFiClient   client;

// Declare other global variables if needed

class WiFiHardware {

  public:
  WiFiHardware() {};

  void init() {
    client.connect(server, 11411);
    while (!client.connect(server, 11411)) {
      Serial.println("Attempting to connect to ROS");
      delay(1000);
    }
    Serial.println("Connected to ROS");
  }

  // reading byte from the serial port
  // -1 = failure (ex: if the TCP connection is not open)
  int read() {  
    return client.read();
  }

  // write data to the connection to ROS
  void write(uint8_t* data, int length) {
    for (int i = 0; i < length; i++) {
      client.write(data[i]);
    }
  }

  // returns milliseconds since start of program
  unsigned long time() {
    return millis();
  }
};

#endif