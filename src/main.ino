/*
  This code is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  This code is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

// Reads battery RS485 messages from Junctek KH1x0F Battery Monitor and forwards them to N2k bus
// Creates N2k messages: 127508 - battery status, 127506 - DC detailed status, (126996 - product info)
// Version 0.6, 05/03/2024, Scorgan - Ulrich Meine

// Battery monitor message reading and parsing takes ideas from: PlJakobs https://github.com/pljakobs/JuncTek_Batterymonitor/tree/main
// Uses the great work of Timo Lappalainen and his NMEA 2000 libraries: https://github.com/ttlappalainen
// All N2k handling uses code by AK-Homberger: https://github.com/AK-Homberger
// Webserver logging by OER Informatik: https://gitlab.com/oer-informatik/mcu/arduino-esp/-/blob/main/src/loggingWLANWebpage/loggingWLANWebpage.ino

#define DEBUG // Flag to activate logging to serial console (i.e. serial monitor in arduino ide)
String LOG_LEVEL_NAMES[] = { "OFF", "FATAL", "ERROR", "WARN", "INFO", "DEBUG", "TRACE", "ALL" };
const int MIN_LOG_LEVEL = 6;
// #define TEST // no device connected -> provide test data sentence
// #define WEBLOG  // configure logging via web server

// M5 Atom Lite GPIO settings
#define ESP32_CAN_TX_PIN GPIO_NUM_22 // set CAN TX port to 22
#define ESP32_CAN_RX_PIN GPIO_NUM_19 // set CAN RX port to 19
#define RS485_RX_PIN GPIO_NUM_32     // define the RS485 RX port
#define RS485_TX_PIN GPIO_NUM_26     // define the RS485 TX port
#define USE_N2K_CAN 7                // #define for NMEA2000_CAN library for use with ESP32

#include <Arduino.h>
// #include <M5stack.h>
#include <NMEA2000_CAN.h> // This will automatically choose right CAN library and create suitable NMEA2000 object
#include <N2kMessages.h>
#include <Preferences.h>
#include <WiFi.h>
#include <string.h>
#include <string>
// #include <Time.h>
#include <sys/time.h>

#define BM_ADDRESS 1 // battery monitor device address
const char BM_RBASE_CMD[] = "R00"; // battery monitor read state command
const char BM_RMSG_CMD[] = "R50"; // battery monitor read measures command
const char BM_RSETT_CMD[] = "R51"; // battery monitor read settings command
#define BM_MAX_MSG_NO 27 // battery monitor max no. of message fields, incl. checksum
#define BM_MAX_MSG_SIZE 125 // battery monitor max message size (current max R51 message size is 113 chars)

#define TempSendOffset 0 // variable name to be changed
#define SlowDataUpdatePeriod 1000 // Time between CAN Messages sent
#define N2K_LOAD_LEVEL 1 // Device power load on N2k bus in multiples of 50mA

HardwareSerial SerRS485(2);

String setupLogText = "";
String loopLogText = "";

typedef struct {
    int
        deviceAddress,
        checksum,
        temperature,
        reserved,
        outputState,
        currentDir,
        timeAdjust,
        stateOfCharge,
        stateOfHealth;
    long
        operationRecVal,
        date,
        lastReadTime,
        batteryTimeLeft;
    float
        voltage,
        current,
        remainingCapacity,
        totalCapacity,
        dischargedKWh,
        chargedKWh;
    char
        msgType[5];
    unsigned char
        instance;
    tN2kDCType
        batteryType;
} batteryData_t;

batteryData_t batteryData; // current battery data
String BMDataSentence; // raw string of BM measured values

int NodeAddress; // To store last N2k device node address
Preferences preferences; // Nonvolatile storage on ESP32 - to store LastDeviceAddress
const unsigned long TransmitMessages[] PROGMEM = { 127506UL, // DC detailed status
    127508UL, // battery status
    0 };

bool IsTimeToUpdate(unsigned long NextUpdate);
unsigned long InitNextUpdate(unsigned long Period, unsigned long Offset);
void SetNextUpdate(unsigned long& NextUpdate, unsigned long Period);

#ifdef WEBLOG
#include "secrets.h" // Passwords saved in this file to be hidden from versioncontrol and sharing
#include <WiFiMulti.h>

// WiFi-Settings (if not defined in secrets.h replace your SSID/PW here)
const char* WIFI_SSID = SECRET_WIFI_SSID; // Wifi network name (SSID)
const char* WIFI_PASSWORD = SECRET_WIFI_PASSWORD; // Wifi network password
const uint32_t CONNECTION_TIMEOUT_MS = 10000; // WiFi connect timeout per AP.
const uint32_t MAX_CONNECTION_RETRY = 20; // Reboot ESP after __ times connection errors
WiFiMulti wifiMulti;

//-------------------------------------------------------------------------------------
// Configuration of the NTP-Server
//-------------------------------------------------------------------------------------

#include "time.h"
const char* NTP_SERVER = "pool.ntp.org";
const long GMT_OFFSET_SEC = 3600;
const int DAYLIGHT_OFFSET_SEC = 3600;

// Set Route and Port for the Logpage-Webserver
#include <WebServer.h>
const int WEBSERVER_PORT = 8080;
const char* WEBSERVER_ROUTE_TO_DEBUG_OUTPUT = "/log";

WebServer server(WEBSERVER_PORT);
#endif

//
// -------------------------------------   setup   -----------------------------------------
//
void setup()
{
    uint8_t chipid[6];
    uint32_t id = 0;
    int i = 0;

    // battery static data -> adjust
    batteryData.instance = 0x02; // battery instance no. for service battery
    batteryData.batteryType = N2kDCt_Battery; // type of service battery
//    batteryData.totalCapacity = 2.2; // capacity of service battery at Ah
    batteryData.stateOfHealth = 100; // static definition for SoH as 100%

    debugOutput("Starting Programm...", 6, true);

    if (BMGetBatterySettings(BM_ADDRESS)) {
        debugOutput("Total battery capacity: " + String(batteryData.totalCapacity), 5, true);
     } else {
        debugOutput("Error retreiving total battery capacity", 2, true);
    }

#ifdef DEBUG
    Serial.begin(115200); // Activate debugging via serial monitor
    delay(100);
#endif

//  Disable WiFi and bluetooth since we don't need it
#ifndef WEBLOG
    WiFi.mode(WIFI_OFF);
#endif
    btStop();

    SerRS485.begin(115200, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN); // Initialize RS485 communication
    delay(100);

    esp_efuse_mac_get_default(chipid);
    for (i = 0; i < 6; i++)
        id += (chipid[i] << (7 * i));

    // Reserve enough buffer for sending all messages. This does not work on small memory devices like Uno or Mega
    NMEA2000.SetN2kCANMsgBufSize(8);
    NMEA2000.SetN2kCANReceiveFrameBufSize(250);
    NMEA2000.SetN2kCANSendFrameBufSize(250);

    // Set Product information
    NMEA2000.SetProductInformation("A9529AFB16", // Manufacturer's Model serial code; here, M5Stack Atom Lite
        6001, // Manufacturer's product code
        "BM Monitor", // Manufacturer's Model ID
        "0.8.0.0", // Manufacturer's Software version code
        "1.0.0.0", // Manufacturer's Model version
        N2K_LOAD_LEVEL, // Device power load on N2k bus
        0xffff, 0x01
    );
    // Set device information
    NMEA2000.SetDeviceInformation(id, // Unique number. Use e.g. Serial number.
        170, // Device function=Battery:reports battery status. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
        35, // Device class=Electrical generation. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
        6702 // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
    );

    // Uncomment 3 rows below to see, what device will send to bus
    // NMEA2000.SetForwardStream(&Serial);  // PC output on due programming port
    // NMEA2000.SetForwardType(tNMEA2000::fwdt_Text); // Show in clear text. Leave uncommented for default Actisense format.
    // NMEA2000.SetForwardOwnMessages();

    preferences.begin("nvs", false); // Open nonvolatile storage (nvs)
    NodeAddress = preferences.getInt("LastNodeAddress", 30); // Read stored last NodeAddress, default 30
    preferences.end();
    debugOutput("N2k node address = " + String(NodeAddress), 4);

    // If you also want to see all traffic on the bus use N2km_ListenAndNode instead of N2km_NodeOnly below
    NMEA2000.SetMode(tNMEA2000::N2km_NodeOnly, NodeAddress);
    NMEA2000.ExtendTransmitMessages(TransmitMessages);

    debugOutput("Opening N2k stream.", 5);
    NMEA2000.Open();
    delay(200);

#ifdef WEBLOG
    debugOutput("WiFi will be established", 6, true);
    WiFi.mode(WIFI_STA); // Connectmode Station: as client on accesspoint
    wifiMulti.addAP(WIFI_SSID, WIFI_PASSWORD); // multpile networks possible
    debugOutput("Connecting to a WiFi Accesspoint", 5, true);
    ensureWIFIConnection(); // Call connection-function for the first

    // Init and get the time
    debugOutput("Connection to NTP-Timeserver", 6, true);
    configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER);

    debugOutput("Starting Webserver...", 6, true);
    server.on(WEBSERVER_ROUTE_TO_DEBUG_OUTPUT, respondRequestWithLogEntries);
    String log_address = "http://" + WiFi.localIP().toString() + ":" + String(WEBSERVER_PORT) + WEBSERVER_ROUTE_TO_DEBUG_OUTPUT;
    debugOutput("Logging will be published on: " + log_address, 5, true);
    server.begin();
    debugOutput("Finished startup.", 6, true);
#endif

    //  Reduce CPU frequency to save power
    //  debugOutput("CPU Freq: " + String(getCpuFrequencyMhz()), 7);
    //  setCpuFrequencyMhz(80);
    //  debugOutput("New CPU Freq: " + String(getCpuFrequencyMhz()), 7);
}

//
// -------------------------------------   Main loop   -----------------------------------------
//
void loop()
{
    int SourceAddress;

#ifdef WEBLOG
    //  ensureWIFIConnection();
    //  server.handleClient();
#endif

    debugOutput("\n\n_____________ Next loop _____________\n", 5);

    if (BMGetBatteryState(BM_ADDRESS)) {
        SendN2kBatteryState();
    } else {
        debugOutput("Error retreiving battery state", 2);
    }

    NMEA2000.ParseMessages();
    SourceAddress = NMEA2000.GetN2kSource();
    if (SourceAddress != NodeAddress) { // Save potentially changed Source Address to NVS memory
        NodeAddress = SourceAddress; // Set new Node Address (to save only once)
        preferences.begin("nvs", false);
        preferences.putInt("LastNodeAddress", SourceAddress);
        preferences.end();
        debugOutput("Address change. New address = " + String(SourceAddress), 1);
    }

    delay(1000);
}

bool BMGetBatteryState(const int BMaddress)
{
    BMSendCmd(BM_RMSG_CMD, BMaddress);
    BMDataSentence = BMreadMsg();

#ifdef TEST
    BMDataSentence = ":r50=1,36,5050,510,1174,8325,8207,7922,122,0,99,1,44,100,101,0,\r\n"; // test data -> no device needs to be connected
#endif

    debugOutput("Message: " + BMDataSentence, 5);

    if (BMparseData(BMDataSentence, BM_RMSG_CMD)) {
        return true;
    } else {
        debugOutput("No data received, wrong data type or wrong checksum!", 2);
        return false;
    }
}

bool BMGetBatterySettings(const int BMaddress)
{
    BMSendCmd(BM_RSETT_CMD, BMaddress);
    BMDataSentence = BMreadMsg();

#ifdef TEST
    BMDataSentence = ":r50=1,36,5050,510,1174,8325,8207,7922,122,0,99,1,44,100,101,0,\r\n"; // test data -> no device needs to be connected
#endif

    debugOutput("Message: " + BMDataSentence, 5);

    if (BMparseData(BMDataSentence, BM_RSETT_CMD)) {
        return true;
    } else {
        debugOutput("No data received, wrong data type or wrong checksum!", 2);
        return false;
    }
}

bool BMSendCmd(const char *c_MsgCmd, const int BMaddress)
{
    const char cmdHead[] = ":";
    const char Equals[] = "=";
    const char cmdTail[] = ",2,1,\n";
    char cmd[22] = ""; // command string; max. command size is command such as ":W12=71,0,20,15,59,<CR>,<LF>" => 21 char's + "\0"
    char c_BMaddress[3] = ""; // char array of battery monitor adddress

    strlcpy(cmd, cmdHead, sizeof(cmd));
    strlcat(cmd, c_MsgCmd, sizeof(cmd));
    strlcat(cmd, Equals, sizeof(cmd));
    itoa(BMaddress, c_BMaddress, 10);
    strlcat(cmd, c_BMaddress, sizeof(cmd));
    strlcat(cmd, cmdTail, sizeof(cmd));
    debugOutput("Battery monitor command: " + String(cmd), 6);

    SerRS485.write(cmd);
    SerRS485.flush(true); // make sure that all buffer data is written to the bus
    delay(50);

    return true;
}

String BMreadMsg()
{
    char c;
    String receivedData;

    debugOutput("No. of bytes available on RS485: " + String(SerRS485.available()), 6);

    while (SerRS485.available() > 0) { // Check if data is available to read
        c = SerRS485.read(); // Read the next incoming char

        if (c != '\r') { // Check for carriage return
            receivedData += c; // Add the incoming byte to the buffer
        } else {
            break;
        }
    }

    // SerRS485.flush(); // drop everything in buffer after <CR>
    while (SerRS485.available()) { // drop everything in buffer after <CR> (<Serial.flush()> does not seem to work properly)
        SerRS485.read();
    }

    debugOutput("Data: " + receivedData, 7);
    return receivedData;
}

bool BMparseData(const String data, const char *MsgCmd)
{
    // result syntax: :r50=<addr>,<checksum>,<voltage>,<current_amps>,<remaining_batt_cap>,<discharged_kWh>,<charged_kWh>,<op_record_val>,
    // <temp>,<reserved>,<output_state>,<current_direction>,<remaining_time>,<time_adj>,<date>,<time>

    char c_MsgNr[3] = ""; // pure message command number w/o leading character
    int i, dataSetNo;
    long dataSet[BM_MAX_MSG_NO];
    int startIdx = 0;
    int nextIdx = 0;

    sprintf(c_MsgNr, "%*s", 2, MsgCmd+1);
//    c_MsgCmd[0] = toupper((unsigned char) MsgCmd[1]);
    debugOutput("Expected Cmd: <" + String(c_MsgNr) + ">", 6);
    debugOutput("CompareTo: <" + data.substring(2, 4) + ">", 6);

    if (data.substring(2, 4) == c_MsgNr) { // data sentence is of correct type
        i = -1;
        startIdx = data.indexOf(','); // search for 1st data field
        nextIdx = data.indexOf(',', startIdx + 1); // search for end of current data field
        while (nextIdx != -1 && i < BM_MAX_MSG_NO) {
            i++;
            debugOutput("Start: " + String(startIdx) + " End: " + String(nextIdx), 7);
            dataSet[i] = long(data.substring(startIdx + 1, nextIdx).toInt());
            debugOutput("Field " + String(i) + ": <" + String(dataSet[i]) + ">", 6);
            startIdx = nextIdx;
            nextIdx = data.indexOf(',', startIdx + 1); // search for end of current data field
        }
        dataSetNo = i; // store no. of fields in parsed data

    } else { // data sentence is empty or of wrong type
        return false;
    }

    if (BMchkChecksum(dataSet, dataSetNo)) { // if checksum of data sentence is correct
        batteryData.checksum = int(dataSet[0]); // conversion of double to int, since "String.toInt()" returns "long"
        batteryData.voltage = float(dataSet[1]) / 100;
        batteryData.current = float(dataSet[2]) / 100;
        batteryData.remainingCapacity = float(dataSet[3]) / 1000; // decimals are delivering 3 digits of precision; any output is limited to 2 digits of precision, though
        batteryData.dischargedKWh = float(dataSet[4]) / 1000;
        batteryData.chargedKWh = float(dataSet[5]) / 1000;
        batteryData.operationRecVal = int(dataSet[6]);
        batteryData.temperature = int(dataSet[7] - 100);
        batteryData.reserved = int(dataSet[8]);
        batteryData.outputState = int(dataSet[9]);
        batteryData.currentDir = int(dataSet[10]);
        batteryData.batteryTimeLeft = long(dataSet[11]);
        batteryData.timeAdjust = int(dataSet[12]);
        batteryData.date = dataSet[13];
        batteryData.lastReadTime = dataSet[14];

        batteryData.stateOfCharge = int(batteryData.remainingCapacity / batteryData.totalCapacity * 100);

        debugOutput("Checksum: " + String(batteryData.checksum), 5);
        debugOutput("Voltage: " + String(batteryData.voltage) + " V", 5);
        debugOutput("Current: " + String(batteryData.current) + " A", 5);
        debugOutput("Remaining capacity: " + String(batteryData.remainingCapacity) + " kWh", 5);
        debugOutput("Total capacity: " + String(batteryData.totalCapacity) + " kWh", 5);
        debugOutput("State of charge SOC: " + String(batteryData.stateOfCharge) + " %", 5);
        debugOutput("State of health SOH: " + String(batteryData.stateOfHealth) + " %", 5);
        debugOutput("Discharged kWh: " + String(batteryData.dischargedKWh) + " kWh", 5);
        debugOutput("Charged kWh: " + String(batteryData.chargedKWh) + " kWh", 5);
        debugOutput("Oper value: " + String(batteryData.operationRecVal), 5);
        debugOutput("Temperature: " + String(batteryData.temperature) + " °C", 5);
        debugOutput("Reserved: " + String(batteryData.reserved), 5);
        debugOutput("Output state: " + String(batteryData.outputState), 5);
        debugOutput("Current direction: " + String(batteryData.currentDir), 5);
        debugOutput("Remaining batt time: " + String(batteryData.batteryTimeLeft) + " min.", 5);
        debugOutput("Time adjust: " + String(batteryData.timeAdjust), 5);
        debugOutput("Date: " + String(batteryData.date), 5);
        debugOutput("Time: " + String(batteryData.lastReadTime), 5);

    } else { // checksum of data sentence is wrong
        return false;
    }

    return true;
}

// Check checksum of last battery monitor message read
// data sentence consists of 1 to 27 parameters
bool BMchkChecksum(long param[], int noOfParam)
{
    //  int noOfParam = sizeof(param) / sizeof(param[0]);
    long chkSum = 0;
    long checkSumRead = param[0]; // 1st param contains checksum

    debugOutput("Param 0:" + String(param[0]), 6);
    debugOutput("noOfParam:" + String(noOfParam), 6);
    for (int i = 1; i <= noOfParam; i++) { // start with 2nd param, since 1st param contains checksum read
        chkSum += param[i];
    }
    chkSum = (chkSum % 255) + 1;

    debugOutput("Checksum read: " + String(checkSumRead), 5);
    debugOutput("Checksum calculated: " + String(chkSum), 5);
    return checkSumRead == chkSum;
}

// Calculate checksum for battery monitor setting command
// setting command consists of 1 to 3 parameters
int BMcalcChecksum(const int param1, const int param2, const int param3)
{
    int chkSum = 0;

    chkSum = (param1 + param2 + param3) % 255 + 1;
    debugOutput("Checksum: " + chkSum, 6);

    return chkSum;
}

void SendN2kBatteryState(void)
{
    static unsigned long SlowDataUpdated = InitNextUpdate(SlowDataUpdatePeriod, TempSendOffset);
    tN2kMsg N2kMsg;

    if (IsTimeToUpdate(SlowDataUpdated)) {
        SetNextUpdate(SlowDataUpdated, SlowDataUpdatePeriod);

        unsigned char battInstance = batteryData.instance;
        double battVoltage = double(batteryData.voltage);
        double battCurrent = double(batteryData.current);
        double battTemperature = CToKelvin(double(batteryData.temperature));
        tN2kDCType battType = batteryData.batteryType;
        unsigned char battSOC = (unsigned char)(batteryData.stateOfCharge);
        unsigned char battSOH = batteryData.stateOfHealth;
        double battTimeLeft = double(batteryData.batteryTimeLeft * 60); // remaining battery time is expected to be in seconds for N2k PGN 127506
        debugOutput("Battery time left long: " + String(batteryData.batteryTimeLeft), 6);
        debugOutput("Battery time left double: " + String(battTimeLeft), 6);
        double battTotalCap = AhToCoulomb(double(batteryData.totalCapacity));

        // bool ParseN2kPGN127506(const tN2kMsg &N2kMsg, unsigned char &SID, unsigned char &DCInstance, tN2kDCType &DCType,
        //                        unsigned char &StateOfCharge, unsigned char &StateOfHealth, double &TimeRemaining, double &RippleVoltage, double &Capacity);
        // <TimeRemaining> is in seconds
        SetN2kPGN127506(N2kMsg, 0xff, battInstance, battType, battSOC, battSOH, battTimeLeft, 0, battTotalCap);
        NMEA2000.SendMsg(N2kMsg);

        //  SetN2kPGN127508(tN2kMsg &N2kMsg, unsigned char BatteryInstance, double BatteryVoltage, double BatteryCurrent=N2kDoubleNA,
        //                  double BatteryTemperature=N2kDoubleNA, unsigned char SID=0xff);
        SetN2kPGN127508(N2kMsg, battInstance, battVoltage, battCurrent, battTemperature, 0xff);
        NMEA2000.SendMsg(N2kMsg);
    }
}

bool IsTimeToUpdate(unsigned long NextUpdate)
{
    return (NextUpdate < millis());
}

unsigned long InitNextUpdate(unsigned long Period, unsigned long Offset = 0)
{
    return millis() + Period + Offset;
}

void SetNextUpdate(unsigned long& NextUpdate, unsigned long Period)
{
    while (NextUpdate < millis())
        NextUpdate += Period;
}

//-------------------------------------------------------------------------------------
//
// Webserver logging routines
//
//-------------------------------------------------------------------------------------
void debugOutput(String text, int logLevel, bool setupLog)
{
    if (MIN_LOG_LEVEL >= logLevel) {
        String timeAsString = "";
        /*    struct tm timeinfo;
            if (!getLocalTime(&timeinfo)) {
              timeAsString = "[no NTP]";
            } else {
              char timeAsChar[20];
              strftime(timeAsChar, 20, "%Y-%m-%d_%H:%M:%S", &timeinfo);
              timeAsString = String(timeAsChar);
            } */
        if (setupLog) {
            setupLogText = setupLogText + "[" + timeAsString + "] " + " [" + LOG_LEVEL_NAMES[logLevel] + "] " + text + "<br/>\n";
        } else {
            loopLogText = loopLogText + "[" + timeAsString + "] " + " [" + LOG_LEVEL_NAMES[logLevel] + "] " + text + "<br/>\n";
        }
//    DEBUG_PRINTLN("[" + timeAsString + "] [" + LOG_LEVEL_NAMES[logLevel] + "] " + text);
#ifdef DEBUG
        Serial.println("[" + timeAsString + "] [" + LOG_LEVEL_NAMES[logLevel] + "] " + text);
#endif
    }
}

void debugOutput(String text, int logLevel)
{
    debugOutput(text, logLevel, false); // log to loopLogText is default
}

void debugOutput(String text)
{
    debugOutput(text, 4); // no loglevel present? use "INFO"
}

/*
  void ensureWIFIConnection() {
  if (WiFi.status() != WL_CONNECTED) {
    debugOutput("No WIFI Connection found. Re-establishing...", 3, true);
    int connectionRetry = 0;
    while ((wifiMulti.run(CONNECTION_TIMEOUT_MS) != WL_CONNECTED)) {
      delay(1000);
      connectionRetry++;
      debugOutput("WLAN Connection attempt number " + String(connectionRetry), 4, true);
      if (connectionRetry > MAX_CONNECTION_RETRY) {
        debugOutput("Connection Failed! Rebooting...", 4, true);
        delay(5000);
        ESP.restart();
      }
    }
    debugOutput("WiFi is connected", 4, true);
    debugOutput("IP address: " + (WiFi.localIP().toString()), 4, true);
    debugOutput("Connected to (SSID): " + String(WiFi.SSID()), 5, true);
    debugOutput("Signal strength (RSSI): " + String(WiFi.RSSI()) + "(-50 = perfect / -100 no signal)", 5, true);
  }
  }

  String renderHtml(String header, String body) {
  // HTML & CSS contents which display on web server
  String html = "";
  html = html + "<!DOCTYPE html>\n<html>\n" + "<html lang='de'>\n<head>\n<meta charset='utf-8'>\n<title>" + header + "</title>\n</head><body>\n<h1>";
  html = html + header + "</h1>\n";
  html = html + body + "\n</body>\n</html>\n";
  return html;
  }

  void respondRequestWithLogEntries() {
  String header = "Debugging-Log-Entries";
  String body = "";
  body = "<h2>Logging on Startup / during configuration (Setup-Log)</h2>\n";
  body = body + setupLogText;
  body = body + "<h2>Logging during operation (Loop-Log)</h2>\n";
  body = body + loopLogText;
  server.send(200, "text/html; charset=utf-8", renderHtml(header, body));
  }
*/