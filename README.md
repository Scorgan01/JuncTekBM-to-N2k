# JuncTekBM-to-N2k
Reads JuncTek KH-F and KL-F series battery monitor data from its RS485 interface and converts it to NMEA2000.<br>
Designed for M5Stack Atom Lite ESP32 processor + Tail485 and CAN modules.

Creates NMEA 2000 PGNs
- 127506 (DC detailed status)
- 127508 (Battery status)

Battery data deliverd with these messages are
- NMEA2000 PGN 127508:
  - Voltage (V)
  - Current (A)
  - Battery temperature (K)
- NMEA2000 PGN 127506:
  - State-of-Charge (percent)
  - State-of-Health (percent - static)
  - Remaining battery time (seconds)
  - Battery capacity (Coulomb)

Important hint:<br>
The RS485 connection of my two JuncTek battery monitor devices with the "Tail485" interface for M5Stack Atom required a terminator resistor between the A and B data lines.
So, I put a small 1/10 watt resistor of 120 Ohm in parallel to the two wires. Anything between 100-150 Ohms should work.
Without that resistor, the M5Stack Atom was not able to read clear data from the battery monitors.

Required libraries are
- ttlappalainen/NMEA2000-library@ v4.21.3
- ttlappalainen/NMEA2000_esp32@ v1.0.3
-	khoih-prog/ESP_WiFiManager_Lite@ v1.10.5

The code has been implemented with the platformIO IDE. You can find the full set of project parameters at the <platformio.ini> file.

Kudos:<br>
The implementation bases on the great work of other projects, first and foremost on the NMEA2000 libraries of Timo Lappalainen: https://github.com/ttlappalainen<br>
All N2k handling uses code by AK-Homberger: https://github.com/AK-Homberger<br>
Battery monitor message reading and parsing takes ideas from PlJakobs https://github.com/pljakobs/JuncTek_Batterymonitor/tree/main<br>
OTA web update leverages the example from ArduinoOTA Library and work from Rui Santos for adjustments to work with WiFi AP: http://randomnerdtutorials.com<br>
OTA web page design was inspired by: https://lastminuteengineers.com/esp32-ota-web-updater-arduino-ide/
