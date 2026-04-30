#include "SimpleScheduler.h"

SimpleScheduler::SimpleScheduler(ESP32Time& rtc)
    : _rtc(rtc),
      _onSecondCb(nullptr), _onMinuteCb(nullptr),
      _onHourCb(nullptr),   _onDayCb(nullptr),
      _lastSec(-1), _lastMin(-1), _lastHr(-1), _lastDay(-1)
{
}

void SimpleScheduler::onSecond(SchedulerCallback cb) { _onSecondCb = cb; }
void SimpleScheduler::onMinute(SchedulerCallback cb) { _onMinuteCb = cb; }
void SimpleScheduler::onHour  (SchedulerCallback cb) { _onHourCb   = cb; }
void SimpleScheduler::onDay   (SchedulerCallback cb) { _onDayCb    = cb; }

void SimpleScheduler::poll()
{
    int sec = _rtc.getSecond();
    int min = _rtc.getMinute();
    int hr  = _rtc.getHour(true);
    int day = _rtc.getDay();

    // --- Second ---
    if (sec != _lastSec) {
        _lastSec = sec;
        if (_onSecondCb) _onSecondCb();
    }

    // --- Minute ---
    if (min != _lastMin) {
        _lastMin = min;
        if (_onMinuteCb) _onMinuteCb();
    }

    // --- Hour ---
    if (hr != _lastHr) {
        _lastHr = hr;
        if (_onHourCb) _onHourCb();
    }

    // --- Day --- fires at 02:00:00 on a new calendar day.
    // First poll initialises silently so we don't fire spuriously at boot.
    if (_lastDay == -1) {
        _lastDay = day;
    } else if (day != _lastDay) {
        if (hr == 2 && min == 0 && sec == 0) {
            _lastDay = day;
            if (_onDayCb) _onDayCb();
        }
        // If day changed but it's not yet 2am, leave _lastDay stale so
        // we keep checking on subsequent polls until 02:00:00 arrives.
    }
}
