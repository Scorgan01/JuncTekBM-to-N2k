# JuncTekBM-to-N2k
Reads JuncTek KH-F series battery monitor data from its RS485 interface and converts it to NMEA2000.
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
  - Battery capacity (Coulomb)
  - State-of-Charge (percent)
  - Remaining battery time (seconds)
  - State-of-Health (percen)

Important hint:
The RS485 connection of my two JuncTek battery monitor devices with the "Tail485" interface for M5Stack Atom required a terminator resistor between the A and B data lines.
So, I put a small 1/10 watt resistor of 120 Ohm in parallel to the two wires. Anything between 100-150 Ohms should work.
Without that resistor, the M5Stack Atom was not able to read clear data from the battery monitors.
