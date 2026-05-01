#include "WiFiSvc.h"

// Seconds to wait for an in-progress connect before declaring timeout.
// NOTE: this sketch copy diverges from Components/WiFiSvc.cpp.  The canonical
// component uses 30 / 60.  This sketch needs 60 / 3600 because WiFi.begin()
// corrupts the ILI9341 panel on this older single-USB CYD; the lifecycle
// driven from onSecondTick() recovers the panel on every attempt outcome,
// and the user wants attempts spaced an hour apart so the panel isn't
// blown away every minute.
static const int WIFIWAIT_MAX      =   60;   // 1 minute connect timeout
// Seconds to wait after a connect timeout before retrying.
static const int WIFIRECONNECT_MAX = 3600;   // 1 hour backoff between attempts
// Seconds to wait after discovering a dropped link before reconnecting.
static const int WIFIDISCOWAIT_MAX = 10;

#define WIFILOG Serial.println

WiFiSvc::WiFiSvc(const char* ssid, const char* pwd)
    : _state(DISCONNECTED), _waitCounter(0), _ssid(ssid), _pwd(pwd)
{
}

void WiFiSvc::connect()
{
    _state        = CONNECTING;
    _waitCounter  = 0;
    WiFi.mode(WIFI_STA);
    WiFi.begin(_ssid.c_str(), _pwd.c_str());
    WIFILOG("Wifi connection initiated");
}

void WiFiSvc::disconnect()
{
    _state        = DISCONNECTED;
    _waitCounter  = 0;
    WiFi.disconnect();
    WIFILOG("Wifi disconnect");
}

void WiFiSvc::poll()
{
    if (_state == CONNECTING)
    {
        if (WiFi.status() == WL_CONNECTED)
        {
            _state       = CONNECTED;
            _waitCounter = 0;
            WIFILOG("Wifi connected as ");
            WIFILOG(WiFi.localIP());
        }
        else if (++_waitCounter >= WIFIWAIT_MAX)
        {
            // connect attempt took too long — back off then retry
            _state       = ERRORTIMEOUT;
            _waitCounter = 0;
            WiFi.disconnect();
            WIFILOG("WiFi connection timed-out");
        }
    }

    if (_state == DISCOWAIT)
    {
        // settle period after a dropped link before retrying
        if (++_waitCounter >= WIFIDISCOWAIT_MAX)
        {
            connect();
        }
    }

    if (_state == CONNECTED && WiFi.status() != WL_CONNECTED)
    {
        WIFILOG("WiFi disconnect discovered");
        _state       = DISCOWAIT;
        _waitCounter = 0;
    }

    if (_state == ERRORTIMEOUT)
    {
        if (++_waitCounter >= WIFIRECONNECT_MAX)
        {
            connect();
        }
    }
}
