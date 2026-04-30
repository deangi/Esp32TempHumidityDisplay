#pragma once
#include <Arduino.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <ESP32Time.h>

//===================================================================
// NtpSync - wraps NTPClient to provide clean UTC sync of ESP32Time
//
// Usage:
//   NtpSync ntpSync(rtc);           // or NtpSync ntpSync(rtc, "pool.ntp.org")
//
//   In setup() after WiFi connects:
//     ntpSync.begin();
//
//   In loop():
//     ntpSync.poll();               // retries until first sync succeeds
//
//   For a scheduled resync (e.g. daily):
//     ntpSync.sync();               // blocks briefly for one UDP round-trip
//
// Notes:
//   - The RTC is always set to raw UTC.  Timezone display is handled
//     separately by tzoffsetSeconds / applyTimezone().
//   - setTimeOffset(0) is forced immediately before every getEpochTime()
//     call to guard against NTPClient library state drift.
//===================================================================
class NtpSync {
public:
    NtpSync(ESP32Time& rtc, const char* server = "pool.ntp.org");

    // Call once from setup() after WiFi is up.
    void begin();

    // Call every loop() iteration.  Drives the initial sync retry until
    // the first successful sync; returns immediately after that.
    void poll();

    // Force a synchronous NTP request right now and update the RTC.
    // Returns true on success.  Safe to call from a scheduler callback.
    bool sync();

    // True after at least one successful sync.
    bool isSynced() const { return _synced; }

private:
    ESP32Time& _rtc;
    WiFiUDP    _udp;
    NTPClient  _client;
    bool       _synced;

    // Pull UTC epoch from client and apply to RTC + log.
    void applySync();
};
