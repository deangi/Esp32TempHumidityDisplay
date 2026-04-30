#pragma once
#include <Arduino.h>
#include <ESP32Time.h>

typedef void (*SchedulerCallback)();

//===================================================================
// SimpleScheduler - RTC-based periodic event dispatcher
//
// Detects transitions in second, minute, hour, and day, then calls
// registered callbacks.  The day callback fires once per day at
// 02:00:00 (avoids midnight when DST / NTP corrections can occur).
//
// Usage:
//   SimpleScheduler scheduler(rtc);
//
//   In setup():
//     scheduler.onSecond(mySecondHandler);
//     scheduler.onMinute(myMinuteHandler);
//     scheduler.onHour  (myHourHandler);
//     scheduler.onDay   (myDayHandler);
//
//   In loop():
//     scheduler.poll();
//
// Pass nullptr to any setter to unregister that callback.
// Clean. Here's a summary:

//   New files: SimpleScheduler.h / SimpleScheduler.cpp
//   - Owns all four last* state variables internally
//   - poll() checks second → minute → hour → day in order, fires any registered callback on a transition
//   - Day fires at 02:00:00 on a new calendar day (preserving original behavior)
//   - Callbacks are plain void (*)() function pointers; pass nullptr to unregister

//   Changes to WorldClock.ino
//   - Removed lastHr, lastSec, lastDay globals and all three *Detector() functions (~50 lines gone)
//   - Added SimpleScheduler scheduler(rtc) global
//   - Added four named callbacks (onSecondTick, onMinuteTick, onHourTick, onDailyReboot) and setupScheduler()
//   - loop() three if (detector()) blocks → single scheduler.poll()

//   To add per-minute work later, just fill in onMinuteTick() — no other changes needed.//===================================================================
class SimpleScheduler {
public:
    // rtc: reference to the application's ESP32Time instance
    explicit SimpleScheduler(ESP32Time& rtc);

    // Register callbacks — called once per detected period transition.
    void onSecond(SchedulerCallback cb);  // fires each time the second changes
    void onMinute(SchedulerCallback cb);  // fires each time the minute changes
    void onHour  (SchedulerCallback cb);  // fires each time the hour changes
    void onDay   (SchedulerCallback cb);  // fires once per day at 02:00:00

    // Call from loop() — checks for schedule events and fires registered callbacks.
    void poll();

private:
    ESP32Time&        _rtc;
    SchedulerCallback _onSecondCb;
    SchedulerCallback _onMinuteCb;
    SchedulerCallback _onHourCb;
    SchedulerCallback _onDayCb;

    int _lastSec;
    int _lastMin;
    int _lastHr;
    int _lastDay;
};
