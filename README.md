# JuncTekBM-to-N2k
Reads JuncTek KH-F and KL-F series battery monitor data from its RS485 interface and converts it to NMEA2000.<br>
Designed for M5Stack Atom Lite ESP32 processor + Tail485 and CAN modules.

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

The code has been implemented with the platformIO IDE. You can find the full set of project parameters at the <platformio.ini> file.

<b>Important hint:</b><br>
The RS485 connection of my two JuncTek battery monitor devices with the "Tail485" interface for M5Stack Atom required a terminator resistor between the A and B data lines.
So, I put a small 1/10 watt resistor of 120 Ohm in parallel to the two wires. Anything between 100-150 Ohms should work.
Without that resistor, the M5Stack Atom was not able to read clear data from the battery monitors.

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
Battery monitor message reading and parsing takes ideas from PlJakobs https://github.com/pljakobs/JuncTek_Batterymonitor/tree/main<br>
OTA web update leverages the example from ArduinoOTA Library and work from Rui Santos for adjustments to work with WiFi AP: http://randomnerdtutorials.com<br>
OTA web page design was inspired by: https://lastminuteengineers.com/esp32-ota-web-updater-arduino-ide/
