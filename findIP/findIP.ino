#include <WiFi.h>
#include <WiFiAP.h>
#include <WiFiClient.h>
#include <WiFiGeneric.h>
#include <WiFiMulti.h>
#include <WiFiSTA.h>
#include <WiFiScan.h>
#include <WiFiServer.h>
#include <WiFiType.h>
#include <WiFiUdp.h>


const char* ssid = "Engineering Club";
const char* pwd = "polyuef005";
unsigned int localUdpPort = 8088;

WiFiUDP udp;

void setup() {

  Serial.begin(115200);
  Serial.print("Connecting to ");
  Serial.println(ssid);


  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(ssid, pwd);

  while (WiFi.status() != WL_CONNECTED) { 
    delay(500);
    Serial.println("...");
  }
  udp.begin(localUdpPort);

  Serial.print("Wi-Fi Connected & IP address: ");
  Serial.println(WiFi.localIP());

}
