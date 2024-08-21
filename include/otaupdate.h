// Library for web page definition for OTA code update
// Scorgan - Ulrich Meine

#ifndef _OTAUPDATE_H
#define _OTAUPDATE_H

#include <ESPmDNS.h>
#include <Update.h>
#include <WebServer.h>
#include <WiFi.h>
#include <WiFiAP.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

#include "WiFiSecrets.h" // Passwords saved in this file to be hidden from version control and sharing
#include "debugoutput.h"

const IPAddress AP_IPADDRESS(192, 168, 21, 1); // Wifi access point server ip address
const IPAddress AP_GATEWAY(192, 168, 21, 1); // Wifi access point server gateway
const IPAddress AP_SUBNET(255, 255, 255, 0); // Wifi access point server subnet
static const char* WIFI_HOST = "batteryesp32"; // hostname "batteryesp32.local"
static const int WEBSERVER_PORT = 80;

// Setup WiFi access point with SSID and password
int otaStartWifi();
// Define OTA web server with code update function
//int otaDefineOTAWebServer(WebServer* server);
int otaDefineOTAWebServer(AsyncWebServer* server);
// Define OTA login page
void otaDefineOTAWebPages(String& pg_login, String& pg_update);

#endif