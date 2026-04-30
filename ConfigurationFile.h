#pragma once
#include <Arduino.h>
#include <FS.h>
#include <SPIFFS.h>

//===================================================================
// ConfigurationFile - reads key=value pairs from a SPIFFS text file
//
// File format (one entry per line, # lines are ignored):
//   SSID=mywifi
//   PASSWORD=mypassword
//   TimeZone=-7 PDT
//   Name=W6DG
//   Location=N46 W156
//
// Usage:
//   ConfigurationFile config("/config.ini");
//
//   char ssid[128];
//   if (config.get("SSID", ssid, sizeof(ssid)))
//       Serial.println(ssid);
//
//   String tz = config.get("TimeZone");   // empty String if not found
//===================================================================
class ConfigurationFile {
public:
    // filename: full SPIFFS path, must start with '/'
    explicit ConfigurationFile(const char* filename);

    // Copy the value for key into outbuf (null-terminated).
    // Returns true if the key was found, false otherwise.
    // outbuf is set to an empty string on failure.
    bool get(const char* key, char* outbuf, int maxlen) const;

    // Convenience overload — returns the value as a String,
    // or an empty String if the key is not found.
    String get(const char* key) const;

private:
    const char* _filename;

    // Read one line from finp into buf (null-terminated).
    // Strips CR; stops at LF or buffer full.
    // Returns true while lines remain, false at EOF.
    static bool readln(File& finp, char* buf, int maxlen);
};
