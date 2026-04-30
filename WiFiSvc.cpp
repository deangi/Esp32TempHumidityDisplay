#include "WiFiSvc.h"

// Seconds to wait for an in-progress connect before declaring timeout.
static const int WIFIWAIT_MAX      = 30;
// Seconds to wait after a connect timeout before retrying.
static const int WIFIRECONNECT_MAX = 60;
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
