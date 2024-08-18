// Library for debug output to Serial
// Scorgan - Ulrich Meine

#ifndef _DEBUGOUTPUT_H
#define _DEBUGOUTPUT_H

#include <Arduino.h>
#include <WString.h>

const int MIN_LOG_LEVEL = 5; // specify minimum log level
const String LOG_LEVEL_NAMES[] = { "OFF", "FATAL", "ERROR", "WARN", "INFO", "DEBUG", "TRACE", "ALL" };
extern HardwareSerial Serial;

void debugOutput(String text, int logLevel, bool setupLog);
void debugOutput(String text, int logLevel);
void debugOutput(String text);

#endif