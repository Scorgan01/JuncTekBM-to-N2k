/* This code is free software; you can redistribute it and/or
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

/* Reads battery RS485 messages from Junctek KH1x0F battery monitor and forwards them to N2k bus
   Creates N2k messages: 127508 - battery status, 127506 - DC detailed status, (126996 - product info)
   Version 0.9, 09/06/2024, Scorgan - Ulrich Meine

   Battery monitor message reading and parsing takes ideas from: PlJakobs https://github.com/pljakobs/JuncTek_Batterymonitor/tree/main
   Uses the great work of Timo Lappalainen and his NMEA 2000 libraries: https://github.com/ttlappalainen
   All N2k handling uses code by AK-Homberger: https://github.com/AK-Homberger
   Webserver logging by OER Informatik: https://gitlab.com/oer-informatik/mcu/arduino-esp/-/blob/main/src/loggingWLANWebpage/loggingWLANWebpage.ino
   OTAWebUpdater.ino Example from ArduinoOTA Library: * Rui Santos http://randomnerdtutorials.com
   Design OTA web page: https://lastminuteengineers.com/esp32-ota-web-updater-arduino-ide/
*/

// need some code clean-up (e.g. #includes WiFiMulti, ...) and debug messages (e.g. OTA update wrong status)

#define DEBUG // Flag to activate logging to serial console (i.e. serial monitor in arduino ide)
#define TEST // provide test data sentence if no device connected

// M5 Atom Lite GPIO settings
#define ESP32_CAN_TX_PIN GPIO_NUM_22 // set CAN TX port to 22
#define ESP32_CAN_RX_PIN GPIO_NUM_19 // set CAN RX port to 19
#define RS485_RX_PIN GPIO_NUM_32 // define the RS485 RX port
#define RS485_TX_PIN GPIO_NUM_26 // define the RS485 TX port
#define USE_N2K_CAN 7 // #define for NMEA2000_CAN library for use with ESP32

#include <Arduino.h>
#include <ESPmDNS.h>
#include <N2kMessages.h>
#include <NMEA2000_CAN.h> // This will automatically choose right CAN library and create suitable NMEA2000 object
#include <Preferences.h>
#include <Update.h>
#include <WebServer.h>

#include "debugoutput.h" // standardized debug messages to Serial
#include "otaupdate.h" // all parameter and functions for WiFi access point and OTA update

#define BM_ADDRESS 1 // battery monitor device address
#define BM_MAX_MSG_NO 27 // battery monitor max no. of message fields, incl. checksum
#define BM_MAX_MSG_SIZE 125 // battery monitor max message size (current max R51 message size is 113 chars)
const char BM_RBASE_CMD[] = "R00"; // battery monitor read state command
const char BM_RMSR_CMD[] = "R50"; // battery monitor read measures command
const char BM_RSETT_CMD[] = "R51"; // battery monitor read settings command
#define BM_TYPE_KLF 2 // battery monitor BM type is KL-F series
#define BM_TYPE_KHF 4 // battery monitor BM type is KH-F series

#define TempSendOffset 0 // variable name to be changed
#define SlowDataUpdatePeriod 1000 // Time between CAN Messages sent
#define N2K_LOAD_LEVEL 1 // Device power load on N2k bus in multiples of 50mA

HardwareSerial SerRS485(2);

typedef struct {
    int
        bmType,
        deviceAddress,
        checksum,
        outputState,
        currentDir,
        timeAdjust,
        stateOfCharge,
        stateOfHealth;
    long
        operationRecVal,
        temperature,
        reserved,
        date,
        lastReadTime,
        batteryTimeLeft,
        battRuntime;
    float
        voltage,
        current,
        remainingCapacity,
        totalCapacity,
        dischargedKWh,
        dischargedAh,
        chargedKWh,
        intResist;
    char
        msgType[5];
    unsigned char
        instance;
    tN2kDCType
        batteryType;
} batteryData_t;

// battery data store and static battery data initialization -> adjust
batteryData_t batteryData = { .stateOfHealth = 1, .instance = 0x02, .batteryType = N2kDCt_Battery };
String BMDataSentence; // raw string of BM measured values

int NodeAddress; // To store last N2k device node address
Preferences preferences; // Nonvolatile storage on ESP32 - to store LastDeviceAddress
const unsigned long TransmitMessages[] PROGMEM = { 127506UL, // DC detailed status
    127508UL, // battery status
    0 };

unsigned long startTime, loopTime, timeout; // timer variables for timeout of BM type retrieval and OTA WiFi AP

bool IsTimeToUpdate(unsigned long NextUpdate);
unsigned long InitNextUpdate(unsigned long Period, unsigned long Offset);
void SetNextUpdate(unsigned long& NextUpdate, unsigned long Period);

// Set webserver object and Port for the OTA webserver
WebServer otaServer(WEBSERVER_PORT);
bool OtaWifiAPUP; // Indicator for running OTA WiFi access point

// const char* WEBSERVER_ROUTE_TO_DEBUG_OUTPUT = "/log";
// const uint32_t CONNECTION_TIMEOUT_MS = 10000; // WiFi connect timeout per AP.
// const uint32_t MAX_CONNECTION_RETRY = 20; // Reboot ESP after xx times connection errors
// WiFiMulti wifiMulti;

/*//-------------------------------------------------------------------------------------
// Configuration of the NTP-Server
//-------------------------------------------------------------------------------------

#include "time.h"
const char* NTP_SERVER = "pool.ntp.org";
const long GMT_OFFSET_SEC = 3600;
const int DAYLIGHT_OFFSET_SEC = 3600;
*/

//-------------------------------------------------------------------------------------
//   setup
//-------------------------------------------------------------------------------------
void setup()
{
    uint8_t chipid[6];
    uint32_t id = 0;
    int i = 0;

#ifdef DEBUG
    Serial.begin(115200); // Activate debugging via serial monitor
    delay(100);
#endif

    debugOutput("Starting Programm...", 6, true);

    //  Disable bluetooth since we don't need it
    btStop();

    //      // Reduce CPU frequency to save power
    //        debugOutput("CPU Freq: " + String(getCpuFrequencyMhz()), 6, true);
    //        setCpuFrequencyMhz(80);
    //        debugOutput("New CPU Freq: " + String(getCpuFrequencyMhz()), 6, true);

    SerRS485.begin(115200, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN); // Initialize RS485 communication
    delay(100);

    // Identify battery monitor type and battery capacity
    timeout = 30; // time limit in seconds
    startTime = millis();
    while (!BMGetBatteryMonitortype(BM_ADDRESS)) {
        loopTime = (millis() - startTime) / 1000;
        debugOutput("Error retreiving battery monitor device type for seconds: " + String(loopTime), 2, true);
        if (loopTime < timeout) {
            delay(2000);
        } else {
            debugOutput("Unable to retreive battery monitor device type. Restarting.", 1, true);
            ESP.restart();
        }
    }
    debugOutput("Battery monitor device type is: " + String(batteryData.bmType), 4, true);

    // Identify battery capacity
    if (BMGetBatterySettings(BM_ADDRESS)) {
        debugOutput("Total battery capacity: " + String(batteryData.totalCapacity) + " kWh", 4, true);
    } else {
        debugOutput("Error retreiving total battery capacity", 2, true);
    }

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
        "1.0.0.0", // Manufacturer's Software version code
        "1.0.0.0", // Manufacturer's Model version
        N2K_LOAD_LEVEL, // Device power load on N2k bus
        0xffff, 0x01);
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
    debugOutput("N2k node address = " + String(NodeAddress), 4, true);

    // If you also want to see all traffic on the bus use N2km_ListenAndNode instead of N2km_NodeOnly below
    NMEA2000.SetMode(tNMEA2000::N2km_NodeOnly, NodeAddress);
    NMEA2000.ExtendTransmitMessages(TransmitMessages);

    if (NMEA2000.Open()) {
        debugOutput("Opened N2k stream.", 5, true);
    } else {
        debugOutput("Error opening N2k stream.", 5, true);
    };
    delay(200);

    // Setup WiFi access point with SSID and password
    OtaWifiAPUP = false;
    debugOutput("Setting WiFi access point ...", 6, true);
    switch (otaStartWifi()) {
    case 0:
        debugOutput("Established WiFi access point", 4, true);
        debugOutput("mDNS responder started. Hostname: " + String(WIFI_HOST), 4, true);
        OtaWifiAPUP = true;
        break;
    case 1:
        debugOutput("Error setting up WiFi access point", 2, true);
        break;
    case 2:
        debugOutput("Error setting up mDNS responder.", 2, true);
    }

    // wrong error message handling
    switch (otaDefineOTAWebServer(&otaServer)) {
    case 0:
        debugOutput("Update success. Rebooting ...", 4, true);
        break;
    case 1:
        debugOutput("Error updating firmware.", 2, true);
    }

    otaServer.begin();

    // set timeout and start time for OTA update time limit
    timeout = 300; // time limit for WiFi AP shutdown in seconds
    startTime = millis();
}

//-------------------------------------------------------------------------------------
//    Main loop
//-------------------------------------------------------------------------------------
void loop()
{
    int SourceAddress;

    debugOutput("_____________ Next loop _____________\n\n", 5);

    // Check for OTA web access
    if (OtaWifiAPUP) {
        loopTime = (millis() - startTime) / 1000;
        if (loopTime < timeout) {
            otaServer.handleClient();
        } else {
            // stop OTA Wifi access point 5 minutes after system start; we don't want to run the AP forever
            otaServer.stop();
            if (WiFi.mode(WIFI_OFF)) {
                debugOutput("Shutdown OTA WiFi access point. No firmware update occured.", 4);
                OtaWifiAPUP = false;
            } else {
                debugOutput("Shutdown OTA WiFi access point failed.", 3);
            }
        };
    }

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

bool BMGetBatteryMonitortype(const int BMaddress)
{
    BMSendCmd(BM_RBASE_CMD, BMaddress);
    BMDataSentence = BMreadMsg();

#ifdef TEST
    BMDataSentence = ":r00=1,206,2110,132,3,\r\n"; // test data -> no device needs to be connected
#endif

    debugOutput("Message: " + BMDataSentence, 5);

    if (BMparseData(BMDataSentence, BM_RBASE_CMD)) {
        return true;
    } else {
        debugOutput("Error parsing data sentence!", 2);
        return false;
    }
}

bool BMGetBatterySettings(const int BMaddress)
{
    BMSendCmd(BM_RSETT_CMD, BMaddress);
    BMDataSentence = BMreadMsg();

#ifdef TEST
    BMDataSentence = ":r51=1,36,5050,510,1174,8325,8207,7922,122,0,99,1,44,100,101,0,\r\n"; // test data (currently wrong)-> no device needs to be connected
#endif

    debugOutput("Message: " + BMDataSentence, 5);

    if (BMparseData(BMDataSentence, BM_RSETT_CMD)) {
        return true;
    } else {
        debugOutput("Error parsing data sentence!", 2);
        return false;
    }
}

bool BMGetBatteryState(const int BMaddress)
{
    BMSendCmd(BM_RMSR_CMD, BMaddress);
    BMDataSentence = BMreadMsg();

#ifdef TEST
    BMDataSentence = ":r50=1,36,5050,510,1174,8325,8207,7922,122,0,99,1,44,100,101,0,\r\n"; // test data when no device needs is connected
#endif

    debugOutput("Message: " + BMDataSentence, 5);

    if (BMparseData(BMDataSentence, BM_RMSR_CMD)) {
        return true;
    } else {
        debugOutput("Error parsing data sentence!", 2);
        return false;
    }
}

bool BMSendCmd(const char* c_MsgCmd, const int BMaddress)
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
    String receivedData = "";

    debugOutput("No. of bytes available on RS485: " + String(SerRS485.available()), 6);
    while (SerRS485.available() > 0) { // Check if data is available to read
        c = SerRS485.read(); // Read the next incoming char

        if (c != '\r') { // Check for carriage return
            receivedData += c; // Add the incoming byte to the buffer
        } else {
            break;
        }
    }

    if (sizeof(receivedData) / sizeof(receivedData[0]) == 0) {
        debugOutput("No data received", 2);
    } else {
        debugOutput("Dataset size:" + String(sizeof(receivedData) / sizeof(receivedData[0])), 7);
        debugOutput("Data: " + receivedData, 7);
    }

    // SerRS485.flush(); // drop everything in buffer after <CR>
    while (SerRS485.available()) { // drop everything in buffer after <CR> (<Serial.flush()> does not seem to work properly)
        SerRS485.read();
    }

    return receivedData;
}

bool BMparseData(const String data, const char* MsgCmd)
{
    char c_MsgNr[3] = ""; // pure message command number w/o leading ":" colon character
    String c_MsgCmd;
    int i, dataSetNo;
    long dataSet[BM_MAX_MSG_NO];
    int startIdx = 0;
    int nextIdx = 0;

    //    sprintf(c_MsgNr, "%*s", 2, MsgCmd + 1);
    c_MsgCmd = data.substring(1, 4);
    c_MsgCmd[0] = toupper((unsigned char)c_MsgCmd[0]);
    debugOutput("Expected Cmd: <" + String(MsgCmd) + ">", 6);
    debugOutput("CompareTo: <" + String(c_MsgCmd) + ">", 6);

    if (c_MsgCmd == MsgCmd) { // data sentence is of correct type
        // split data sentence into individual data fields
        i = -1;
        startIdx = data.indexOf(','); // search for 1st data field after command + device address -> checksum
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

    } else { // data sentence is of wrong type
        debugOutput("Data sentence is of wrong message type.", 2);
        return false;
    }

    // verify checksum of data sentence
    if (!BMchkChecksum(dataSet, dataSetNo)) {
        debugOutput("Data sentence has wrong check sum.", 2);
        return false;
    }

    // determine battery monitor device type - only once on startup
    if (c_MsgCmd == BM_RBASE_CMD) { // data sentence contains BM type basic information
        i = int(dataSet[1] / 1000); // read left digit from data
        switch (i) {
        case 2:
            batteryData.bmType = BM_TYPE_KLF;
            break;
        case 4:
            batteryData.bmType = BM_TYPE_KHF;
            break;
        default:
            batteryData.bmType = 0; // no supported BM type could be determined
            debugOutput("Supported BM device type could not be determined. Reported type is: " + String(batteryData.bmType), 7);
        }
        debugOutput("BM device type: " + String(batteryData.bmType), 7);
    }

    switch (batteryData.bmType) {
    case BM_TYPE_KLF:
        BMassignDataKLF(c_MsgCmd, dataSet);
        break;
    case BM_TYPE_KHF:
        BMassignDataKHF(c_MsgCmd, dataSet);
        break;
    default:
        return false; // should never get here
    }

    return true;
}

void BMassignDataKLF(String c_MsgCmd, long* dataSet)
{
    // KL-F data format:
    // * Base Info - :r00=<addr>,<checksum>,<model>,<sw_version>,<ser_no>
    // * Measures -  :r50=<addr>,<checksum>,<voltage>,<current_amps>,<remaining_batt_cap>,<consumed_amph>,<charged_kWh>,<runtime>,<temp>,<reserved>,<output_state>,
    //                    <current_direction>,<remaining_time>,<int_resistance>
    // * State -     :r51=<addr>,<checksum>,<overvolt_prot>,<undervolt_prot>,<overdischrg_prot>,<overchrg_prot>,<overpwr_prot>,<overtmp_prot>,<prot_rcvr_time>,
    //                    <prot_delay_time>,<batt_cap>,<volt_clbr>,<curr_clbr>,<temp_clbr>,<undef_func>,<relay_type>,<curr_mltplr>,<volt_crv_scale>,<amp_crv_scale>

    if (c_MsgCmd == BM_RMSR_CMD) { // data sentence contains battery measure values

        batteryData.checksum = int(dataSet[0]); // conversion of long to int, since "String.toInt()" returns "long"
        batteryData.voltage = float(dataSet[1]) / 100;
        batteryData.current = float(dataSet[2]) / 100;
        batteryData.remainingCapacity = float(dataSet[3]) / 1000; // decimals are delivering 3 digits of precision; any output is limited to 2 digits of precision, though
        batteryData.dischargedAh = float(dataSet[4]) / 1000;
        batteryData.chargedKWh = float(dataSet[5]) / 1000;
        batteryData.battRuntime = long(dataSet[6]);
        batteryData.temperature = long(dataSet[7] - 100);
        batteryData.reserved = long(dataSet[8]);
        batteryData.outputState = int(dataSet[9]);
        batteryData.currentDir = int(dataSet[10]);
        batteryData.batteryTimeLeft = long(dataSet[11]);
        batteryData.intResist = float(dataSet[12]) / 100;
        batteryData.stateOfCharge = int(batteryData.remainingCapacity / batteryData.totalCapacity * 100);

        debugOutput("Checksum: " + String(batteryData.checksum), 5);
        debugOutput("Voltage: " + String(batteryData.voltage) + " V", 5);
        debugOutput("Current: " + String(batteryData.current) + " A", 5);
        debugOutput("Remaining capacity: " + String(batteryData.remainingCapacity) + " kWh", 5);
        debugOutput("Discharged Ah: " + String(batteryData.dischargedAh) + " Ah", 5);
        debugOutput("Charged kWh: " + String(batteryData.chargedKWh) + " kWh", 5);
        debugOutput("Battery runtime: " + String(batteryData.battRuntime), 5);
        debugOutput("Temperature: " + String(batteryData.temperature) + " °C", 5);
        debugOutput("Reserved: " + String(batteryData.reserved), 5);
        debugOutput("Output state: " + String(batteryData.outputState), 5);
        debugOutput("Current direction: " + String(batteryData.currentDir), 5);
        debugOutput("Remaining batt time: " + String(batteryData.batteryTimeLeft) + " min.", 5);
        debugOutput("Battery internal resistance: " + String(batteryData.intResist) + " mOhm", 5);
        debugOutput("State of charge SOC: " + String(batteryData.stateOfCharge) + " %" + "\n", 5);

    } else if (c_MsgCmd == BM_RSETT_CMD) { // data sentence contains battery settings values

        batteryData.totalCapacity = float(dataSet[9]) / 10;
        debugOutput("Total capacity: " + String(batteryData.totalCapacity) + " kWh", 5);
    }
}

void BMassignDataKHF(String c_MsgCmd, long* dataSet)
{
    // KH-F data format:
    // * Base Info - :r00=<addr>,<checksum>,<model>,<sw_version>,<ser_no>
    // * Measures -  :r50=<addr>,<checksum>,<voltage>,<current_amps>,<remaining_batt_cap>,<discharged_kWh>,<charged_kWh>,<op_record_val>,
    //                    <temp>,<reserved>,<output_state>,<current_direction>,<remaining_time>,<time_adj>,<date>,<time>
    // * State -     :r51=<addr>,<checksum>,<overvolt_prot>,<undervolt_prot>,<overdischrg_prot>,<overchrg_prot>,<overpwr_prot>,<overtmp_prot>,<prot_rcvr_time>,
    //                    <prot_delay_time>,<batt_cap>,<volt_clbr>,<curr_clbr>,<temp_clbr>,<undef_func>,<relay_type>,<curr_mltplr>,<time_clbr>,<logging>,
    //                    <full_chrg_volt>,<low_batt_volt>,<full_chrg_curr>,<monitor_time>,<low_temp_prot>,<temp_unit>,<bluet_pwd>,datalog_interval>

    if (c_MsgCmd == BM_RMSR_CMD) { // data sentence contains battery measure values

        batteryData.checksum = int(dataSet[0]); // conversion of double to int, since "String.toInt()" returns "long"
        batteryData.voltage = float(dataSet[1]) / 100;
        batteryData.current = float(dataSet[2]) / 100;
        batteryData.remainingCapacity = float(dataSet[3]) / 1000; // decimals are delivering 3 digits of precision; any output is limited to 2 digits of precision, though
        batteryData.dischargedKWh = float(dataSet[4]) / 1000;
        batteryData.chargedKWh = float(dataSet[5]) / 1000;
        batteryData.operationRecVal = long(dataSet[6]);
        batteryData.temperature = long(dataSet[7] - 100);
        batteryData.reserved = long(dataSet[8]);
        batteryData.outputState = int(dataSet[9]);
        batteryData.currentDir = int(dataSet[10]);
        batteryData.batteryTimeLeft = long(dataSet[11]);
        batteryData.timeAdjust = int(dataSet[12]);
        batteryData.date = long(dataSet[13]);
        batteryData.lastReadTime = long(dataSet[14]);
        batteryData.stateOfCharge = int(batteryData.remainingCapacity / batteryData.totalCapacity * 100);

        debugOutput("Checksum: " + String(batteryData.checksum), 5);
        debugOutput("Voltage: " + String(batteryData.voltage) + " V", 5);
        debugOutput("Current: " + String(batteryData.current) + " A", 5);
        debugOutput("Remaining capacity: " + String(batteryData.remainingCapacity) + " kWh", 5);
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
        debugOutput("State of charge SOC: " + String(batteryData.stateOfCharge) + " %" + "\n", 5);

    } else if (c_MsgCmd == BM_RSETT_CMD) { // data sentence contains battery settings values

        batteryData.totalCapacity = float(dataSet[9]) / 10;
        debugOutput("Total capacity: " + String(batteryData.totalCapacity) + " kWh", 5);
    }
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
        unsigned char battSOH = (unsigned char)(batteryData.stateOfHealth);
        double battTimeLeft = double(batteryData.batteryTimeLeft * 60); // remaining battery time is expected to be in seconds for N2k PGN 127506
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
