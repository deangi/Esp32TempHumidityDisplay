#include "NtpSync.h"

NtpSync::NtpSync(ESP32Time& rtc, const char* server)
    : _rtc(rtc), _client(_udp, server), _synced(false)
{
}

void NtpSync::begin()
{
    _client.begin();
    _client.setTimeOffset(0);  // always UTC — never let the library add an offset
}

void NtpSync::poll()
{
    if (_synced) return;  // initial sync done; further resyncs via sync()

    if (!_client.update()) {
        // No cached valid time yet — send a request and wait for a response
        _client.forceUpdate();
    } else {
        applySync();
        _synced = true;
    }
}

bool NtpSync::sync()
{
    Serial.println("NTP: syncing...");
    _client.setTimeOffset(0);        // guard against any accumulated drift
    if (_client.forceUpdate()) {
        applySync();
        _synced = true;
        return true;
    }
    Serial.println("NTP: sync failed");
    return false;
}

void NtpSync::applySync()
{
    // Force offset=0 immediately before reading epoch.
    // NTPClient::getEpochTime() returns (_timeOffset + epoch), so any
    // non-zero offset would silently corrupt the stored UTC time.
    _client.setTimeOffset(0);
    unsigned long epoch = _client.getEpochTime();
    _rtc.setTime(epoch);
    Serial.printf("NTP GMT: %s  epoch=%lu\n",
                  _client.getFormattedTime().c_str(), epoch);
}
