// Library for debug output to Serial
// Scorgan - Ulrich Meine

#include "debugoutput.h"
#include "otaupdate.h"

void debugOutput(String text, int logLevel, bool setupCode)
{
    String codeSegment;
    String logMessage;

    if (setupCode) {
        codeSegment = "setup";
    } else {
        codeSegment = "loop";
    }

    logMessage = "[" + codeSegment + "] [" + LOG_LEVEL_NAMES[logLevel] + "] " + text;

    if (MIN_LOG_LEVEL >= logLevel && Serial) {
        Serial.println(logMessage);
        myLogWebSocket.textAll(logMessage);
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
