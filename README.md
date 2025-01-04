# JuncTekBM-to-N2k
Reads JuncTek KH-F and KL-F series battery monitor data from its RS485 interface and converts it to NMEA2000.<br>
Designed for M5Stack Atom Lite ESP32 processor + RS485-to-TTL and CAN modules.

Creates NMEA 2000 PGNs
- 127506 (DC detailed status)
- 127508 (Battery status)

Battery data delivered with these messages are
- NMEA2000 PGN 127508:
  - Voltage (V)
  - Current (A)
  - Battery temperature (K)
- NMEA2000 PGN 127506:
  - State-of-Charge (percent)
  - State-of-Health (percent - static)
  - Remaining battery time (seconds)
  - Battery capacity (Coulomb)

Required libraries are
- ttlappalainen/NMEA2000-library@ v4.21.3
- ttlappalainen/NMEA2000_esp32@ v1.0.3
-	khoih-prog/ESP_WiFiManager_Lite@ v1.10.5
-	me-no-dev/ESP Async WebServer@ v1.2.4

The code has been implemented with the platformIO IDE. You can find the full set of project parameters at the <platformio.ini> file.

<b>Important hint:</b><br>
Initially, I used the "Tail485" interface for the M5Stack Atom lite. Unfortunately, I recognized some loose contact between the Atom module and two different interfaces. The cable connection to a "RS485-to-TTL" device was much more reliable.<br>
One "Tail485" interface required a terminator resistor between the A and B data lines to read the data of two JuncTek battery monitor devices correctly. So, you can try to put a small 1/10 watt resistor of 120 Ohm in parallel to the two wires, if you experience weird data reading. Anything between 100-150 Ohms should work.
My current setup with a "RS485-to-TTL" interface is working without any terminator resistor.<br>

You need to adjust the code, if you want to attach a display, which comes with the KH-F version, to the battery monitor in parallel to the ESP N2K interface. The JuncTek display generates its own status query messages on the RS485 bus, which the code doesn't check in the current version. I don't see any need for a separate cable-connected display, since you have the option to display the data with a bluetooth-connected smartphone, and, of course, with any device that can display N2k battery messages.

The code checks the availability of a battery monitor of the KL-F or KH-F family. The Atom will restart after 30 seconds, if no such device is detected. I am not sure, whether the JuncTek KG or KM family has a similar RS485 data communication structure. You will probably need to adjust the code, if you want to use these device types.

<b>OTA firmware update</b><br>
You can update the device with a new firmware via WiFi.
The device creates an own WiFi access point after startup for 5 minutes. You can call the web update page with these parameters:<br>
SSID: BatteryWiFi<br>
Password: 12345678<br>
IP address: 192.168.21.1<br>
Device name: batteryesp32.local<br>
URL: http://batteryesp32.local<br>
Default login credentials: admin/esp32<br>
You can change the WiFi parameters in <wifisecrets.h> and IP parameters in <otaupdate.h>.<br>
The device will shutdown the WiFi AP after 5 minutes in order not to run the WiFi unnecessarily. Restart the device, if you missed to update the device within that timeframe.<br>
Be careful: There is no validation of the firmware file. You need to know what you are doing.

Kudos:<br>
The implementation bases on the great work of other projects, first and foremost on the NMEA2000 libraries of Timo Lappalainen: https://github.com/ttlappalainen<br>
All N2k handling uses code by AK-Homberger: https://github.com/AK-Homberger<br>
Battery monitor message reading and parsing takes ideas from PlJakobs: https://github.com/pljakobs/JuncTek_Batterymonitor/tree/main<br>
OTA web update leverages the example from ArduinoOTA Library and work from Rui Santos for adjustments to work with WiFi AP: http://randomnerdtutorials.com<br>
OTA web page design was inspired by: https://lastminuteengineers.com/esp32-ota-web-updater-arduino-ide/
