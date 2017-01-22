
#include <WiFi101.h>
#include <ArduinoCloud.h>

/////// Wifi Settings ///////
char ssid[] = "";
char pass[] = "";


// Arduino Cloud settings and credentials
const char userName[]   = "aravind_arduino11";
const char thingName[] = "led";
const char thingId[]   = "665685ad-639d-4d53-b300-1dfc19193a3c";
const char thingPsw[]  = "7b646c15-bf63-493f-aef3-5d820881e3cf";


WiFiSSLClient sslClient;


// build a new object "led"
ArduinoCloudThing led;


void setup() {
  Serial.begin (9600);

  // attempt to connect to WiFi network:
  Serial.print("Attempting to connect to WPA SSID: ");
  Serial.println(ssid);

  while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
    // unsuccessful, retry in 4 seconds
    Serial.print("failed ... ");
    delay(4000);
    Serial.print("retrying ... ");
  }


  led.begin(thingName, userName, thingId, thingPsw, sslClient);
  led.enableDebug();
  // define the properties
  led.addProperty("cloudLedStatus", INT, R);
  
}

void loop() {
  led.poll();

  led.writeProperty("cloudLedStatus", "oh...");
  
  delay(1000);
  led.writeProperty("cloudLedStatus", "yeah!");
  
  delay(1000);
}
