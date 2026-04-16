#pragma once

#include <Arduino.h>
//===================================================================
// Arduino console input class - allows users to type a line of
// input characters terminated by CR (Return) chr(13) key.
//
// Dean Gienger, Claude, March 19, 2026
//
// Declare: ConsoleInput console = ConsoleInput(&Serial,128,true);
//
// Use in loop()
//
//  char cmd[132];
//
//  for (;;)
//  {
//    console.poll();
//    const char *cp = console.getLine();
//    if (cp != NULL)
//    {
//      strcpy(cmd,cp);
//      Serial.print("\nGot: "); 
//      Serial.println(cmd);
//      Serial.println("");
//    }
//  }
//
//===================================================================

class ConsoleInput {
public:
    // serial: pointer to any Stream (HardwareSerial, SoftwareSerial, etc.)
    // maxLen: maximum characters to accept before forcing a line break
    // ignoreNonPrintable: if true, characters outside 0x20-0x7E are silently dropped
    ConsoleInput(Stream* serial, int maxLen, bool ignoreNonPrintable = true);
    ~ConsoleInput();

    // Call this regularly from loop(). Reads available bytes, echoes, and accumulates.
    void poll();

    // Returns a pointer to the completed line (null-terminated), or NULL if no line
    // is ready yet. The pointer is valid until the next call to poll() that starts
    // a new line.
    const char* getLine();
    void setIgnoreNonPrintable(bool flag) { _ignoreNonPrintable = flag; }
private:
    Stream*  _serial;
    char*    _buf;
    int      _maxLen;
    int      _pos;
    bool     _lineReady;
    bool     _ignoreNonPrintable;
};
