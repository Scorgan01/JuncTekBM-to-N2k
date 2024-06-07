// Library for debug output to Serial
// Scorgan - Ulrich Meine

#include "debugoutput.h"

void debugOutput(String text, int logLevel, bool setupLog)
{
    String codeSegment;

    if (setupLog) {
        codeSegment = "setup";
    } else {
        codeSegment = "loop";
    }

    if (MIN_LOG_LEVEL >= logLevel) {
        Serial.println("[" + codeSegment + "] [" + LOG_LEVEL_NAMES[logLevel] + "] " + text);
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
