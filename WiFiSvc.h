#pragma once
#include <Arduino.h>
#include <WiFi.h>

//===================================================================
// WiFiSvc - state-machine driven WiFi station connect / reconnect
//
// Replaces the header-only WiFiService.h.  Each instance owns its
// own state and remembers the SSID/password so the internal retry
// path (timeout -> reconnect, disconnect -> reconnect) works without
// the caller re-supplying credentials.
//
// Usage:
//   WiFiSvc wifi(ssid, pwd);
//
//   In setup() after config is loaded:
//     wifi.connect();
//
//   From a 1-second scheduler tick:
//     wifi.poll();
//
//   Anywhere:
//     if (wifi.isConnected()) { ... }
//===================================================================
class WiFiSvc {
public:
    enum State {
        DISCONNECTED  = 0,
        CONNECTING    = 1,
        CONNECTED     = 2,
        DISCOWAIT     = 3,
        ERRORTIMEOUT  = 4
    };

    WiFiSvc(const char* ssid, const char* pwd);

    // Initiate a connection to the AP using the stored credentials.
    // Also used internally for automatic reconnect attempts.
    void connect();

    // Disconnect from the current AP and stop reconnect attempts
    // until connect() is called again.
    void disconnect();

    // Call once per second.  Drives the connect / timeout / reconnect
    // state machine.
    void poll();

    bool  isConnected() const { return _state == CONNECTED; }
    State state()       const { return _state; }

private:
    State   _state;
    int     _waitCounter;   // seconds spent in current wait phase
    String  _ssid;
    String  _pwd;
};
