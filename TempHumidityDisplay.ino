/*============================================================================
 * TempHumidityDisplay.ino
 *
 * Reads temperature (°F) and humidity (%) from a DHT11 on GPIO 22.
 * Displays live readings on the top ¼ of the CYD 2.8" TFT (320×240),
 * and a rolling 7-day graph on the bottom ¾.
 *
 * Hardware
 *   ESP32 CYD (Cheap Yellow Display) — single-USB version ESP32-WROOM board
 *   LovyanGFX driver at 40MHz SPI for 2.8" CYD board (Cheap Yellow Display)
 *   (uses older model CYD with single micro-usb port)
 *   (newer models have micro-usb and usb-c, but have different display drivers)
 *   DHT11 sensor → GPIO 22
 *
 * Libraries  (install via Arduino Library Manager)
 *   LovyanGFX             – display driver (configured in ESP32_SPI_9341.h)
 *   DHT sensor library    – Adafruit DHT sensor library 1.4.7
 *   Adafruit Unified Sensor
 *
 * Arduino IDE
 *   Tools → Partition Scheme → Default (must include a SPIFFS partition)
 *
 * SPIFFS log format: /temphumid.log
 *   Binary, 4 bytes per record: [int16 temp×10][int16 humidity×10]
 *   DHT11 is read every 10 s; readings are averaged over 6 minutes and
 *   stored as int16 (value × 10) for one decimal digit of precision.
 *   One record appended per 6-minute averaged sample.
 *
 *   Capacity math (6-min intervals):
 *     1 week  = 7 × 24 × 10 = 1680 records =  6720 bytes
 *     1 month = 4 weeks     = 6720 records = 26880 bytes (× 4 = 53760 w/ int16)
 *
 *   On boot:  load last 1680 records (1 week) into the plot buffer.
 *   Hourly:   append current averaged readings to log.
 *   Trim:     if log exceeds 400000 bytes (~13.5 months), remove the oldest
 *             month (53760 bytes) at a time until under threshold.
 *             trimLog() heap-allocates a buffer equal to the retained data;
 *             call only from setup() or the weekly timer.
 * ============================================================================
 * V4.0 - 28-Apr-2026 - ideas - 
 *  x1) add WiFi - NTP to get time/date, esp32time.h
 *  x2) log time/date with temp/humidity - change log file format
 *  x3) display time/date
 *  x4) update log file format with date/time, change purge limits
 *  x5) add some ftp server to allow wifi to pull data - actually added web service
 *      curl http://192.168.x.y/log >temphumid.csv
 *      curl http://192.168.x.y/config >config.ini
 *      curl -T config.ini http://192.168.x.y/config
 *      curl -T temphumid.csv http://192.168.x.y/log
 *      about 150kb max for put
 *  x6) going to need a config file with wifi ssid, pwd, ftp user/pwd
 *  7) Already have a command interpreter, perhaps add telnet as well?
 *  x8) components to use - NtpSync,  WiFi service,  telnetserver, rgb, simplescheduler, configurationfile,delimitedstringparser
 *  x9) reboot every day at midnight or 2am
 */

#include "ESP32_SPI_9341.h"   // LovyanGFX + CYD pin/panel config
#include <DHT.h>
#include <SPIFFS.h>
#include <ESP32Time.h>
#include <WebServer.h>
// DelimitedStringParser, ConsoleInput, CommandHandler, ConfigurationFile,
// SimpleScheduler, WiFiSvc, and NtpSync are copied from the shared Components
// library (../../Components) into this sketch folder.  The Arduino build
// system compiles all .cpp files in the sketch folder automatically, so only
// the headers need to be included here.
#include "DelimitedStringParser.h"
#include "ConsoleInput.h"
#include "CommandHandler.h"
#include "ConfigurationFile.h"
#include "SimpleScheduler.h"
#include "WiFiSvc.h"
#include "NtpSync.h"
#include "Smoother.h"

// ─── Version ──────────────────────────────────────────────────────────────────
#define VERSION       "TempHumidityDisplay v4.3  01-May-2026"
#define VERSION_SHORT "v4.3"

// Diagnostic: set to 0 to disable all NTP traffic (begin/poll/sync).
// WiFi still associates so we can isolate whether the display corruption is
// caused by NTP UDP packets or by the WiFi association itself.
#define ENABLE_NTP   1

// Diagnostic: set to 0 to skip WiFi.begin() entirely.  The WiFiSvc instance
// is also not constructed, so onSecondTick's wifi->poll() turns into a
// no-op (the pointer stays null).  Use this to confirm whether the display
// corruption is caused by anything WiFi-related at all.
#define ENABLE_WIFI  1

// ─── Hardware ─────────────────────────────────────────────────────────────────
#define DHT_PIN   22
#define DHT_TYPE  DHT11

// ─── Display geometry ─────────────────────────────────────────────────────────
static const int SCR_W = 320;
static const int SCR_H = 240;

static const int HDR_H = 60;             // top ¼  (header)
static const int GRF_Y = HDR_H;         // graph origin Y
static const int GRF_H = SCR_H - HDR_H; // 180 px (bottom ¾)

// Plot area inside the graph – margins leave room for axis labels
static const int LM = 38;               // left   (temp labels)
static const int RM = 32;               // right  (humidity labels)
static const int TM =  5;               // top
// BM bumped from 18 → 26 to free 8 px at the bottom for the status line
// (WiFi state on the left, version on the right).  Time tick labels still
// draw at PLT_Y + PLT_H + 2 and now sit cleanly above the status row.
static const int BM = 26;               // bottom (time labels + status line)

static const int PLT_X = LM;
static const int PLT_Y = GRF_Y + TM;
static const int PLT_W = SCR_W - LM - RM;   // 250 px
static const int PLT_H = GRF_H - TM - BM;   // 149 px

// Status line — bottom 8 px of the screen.  WiFi status (left) and version
// (right) drawn in Font0 (~8 px tall).  Refreshed once per second from
// onSecondTick and after every panel re-init.
static const int STAT_H = 8;
static const int STAT_Y = SCR_H - STAT_H;

// Clock strip — small date/time line tucked into the gap at the bottom of
// the header, between the big number row and the white separator at y=HDR_H-1.
// Font0 is ~8 px tall; the 10 px band leaves 1 px of padding above and below.
static const int CLK_Y = 48;
static const int CLK_H = 10;

// ─── Sampling ─────────────────────────────────────────────────────────────────
// Scheduling is RTC-driven (see SimpleScheduler callbacks).  The constants
// below are kept only as scaling factors — not as elapsed-time deadlines.
#define FAST_PERIOD_SEC  10                                 // raw DHT11 read every N sec
#define PLOT_PERIOD_MIN  6                                  // averaged plot point every N min
#define SAMPLE_MS        (PLOT_PERIOD_MIN * 60UL * 1000UL)  // 6-minute averaged plot interval
#define SAMPLES_PER_HOUR (3600000UL / SAMPLE_MS)            // 10 plot points per hour
#define MAX_SMPL         (int)(7UL * 24UL * SAMPLES_PER_HOUR) // 1680 — 7 days of in-memory history

// ─── SPIFFS / Logging ─────────────────────────────────────────────────────────
// Log format is ASCII, one line per averaged 6-min reading:
//   YYYYMMDD-HHMM,temp,humidity\n
// Example: 20260429-1430,72.5,45.3
// Path moved to .csv from the old binary .log so the formats don't mix; the
// old /temphumid.log file (if any) is orphaned and can be removed via `rm`.
#define LOG_PATH           "/temphumid.csv"

// 1 week of records loaded into the plot buffer on startup.
static const int LOG_WEEK_RECS  = 7 * 24 * (int)SAMPLES_PER_HOUR;

// Width of the plot's time window — also the cutoff used by loadFromSpiffs()
// to drop stale log entries (see "Stale-entry filter" below).
static const time_t PLOT_WINDOW_SEC = 7L * 86400L;

// Trim policy: byte-based since records are now variable-length ASCII.
// At ~30 bytes/record this is roughly: threshold ≈ 14 weeks, chunk ≈ 1 week.
static const size_t LOG_TRIM_THRESHOLD = 700000UL;
static const size_t LOG_TRIM_BYTES     =  50000UL;

// ─── Colors (RGB565) ──────────────────────────────────────────────────────────
#define C_BG      TFT_BLACK
#define C_HDR_BG  0x0841    // very dark blue
#define C_DIVIDER 0x4208    // dim grey
#define C_TEMP    0xFD00    // warm red-orange
#define C_HUMID   0x07FF    // cyan
#define C_GRID    0x2104    // dim grey
#define C_AXIS    0x8410    // medium grey
#define C_WHITE   TFT_WHITE

// ─── Globals ──────────────────────────────────────────────────────────────────
LGFX tft;
DHT  dht(DHT_PIN, DHT_TYPE);

// Real-time clock.  Holds raw UTC; timezone offset is applied at display time.
// Seeded to 2026-01-01 00:00:01 in setup() so timestamps are sane before the
// first NTP sync; NtpSync will overwrite this once WiFi is up.
ESP32Time rtc(0);

// All periodic work is dispatched from the RTC, not millis().  The callbacks
// are registered in setup() and fired from scheduler.poll() in loop().
SimpleScheduler scheduler(rtc);

// WiFi station service.  Heap-allocated in setup() once the config file has
// been read, because the WiFiSvc constructor copies ssid/password at
// construction time and the credentials are not known until runtime.
WiFiSvc* wifi = nullptr;

// NTP time-sync service.  Bound to the global rtc; uses the default NTP pool.
// begin() must be called only after WiFi is up — see onSecondTick() for the
// rising-edge logic that drives that.  poll() retries until the first sync
// succeeds, then becomes a no-op; sync() is invoked once per day to resync.
NtpSync ntp(rtc);

// HTTP server for remote GET/PUT of the log file and config.ini.  Started
// only when WiFi is up (see lifecycle in onSecondTick); stopped on link loss.
// handleClient() is pumped from loop() so HTTP latency isn't gated on the
// 1-second scheduler tick.
WebServer webServer(80);
bool      httpServerActive = false;

float  tBuf[MAX_SMPL];     // temperature history °F
float  hBuf[MAX_SMPL];     // humidity history %
time_t timeBuf[MAX_SMPL];  // capture time of each sample (Unix epoch, UTC)
int    nSmpl    = 0;       // total samples currently in buffer
int    nLogSmpl = 0;       // samples loaded from log at boot (hourly spacing)

// Fast-sample accumulator — sums 10-second reads over one 6-minute interval.
float         accumT     = 0.0f;
float         accumH     = 0.0f;
int           accumCount = 0;

// Pending log buffer — averaged plot points awaiting the hourly SPIFFS flush.
// Temperature/humidity stored as int16 (value × 10) for compactness; timestamp
// captured at sample time (NOT flush time) so each on-disk line carries the
// minute the reading was actually taken.
int16_t pendingT   [SAMPLES_PER_HOUR];
int16_t pendingH   [SAMPLES_PER_HOUR];
time_t  pendingTime[SAMPLES_PER_HOUR];
int     pendingCount = 0;

float curT = NAN, curH = NAN;

bool spiffsOk = false;

// ─── Configuration (loaded from /config.ini on the SPIFFS partition) ──────────
// The data/config.ini file is uploaded to SPIFFS via the Arduino "ESP32 Sketch
// Data Upload" tool (or equivalent) and re-read on every boot.
#define CONFIG_PATH    "/config.ini"
#define CFG_VAL_LEN    128
char cfgSsid[CFG_VAL_LEN] = "";
char cfgPwd [CFG_VAL_LEN] = "";

// ─── Serial command interface ─────────────────────────────────────────────────
ConsoleInput  console(&Serial, 128, true);
CommandHandler cmd(' ');

// ─── SPIFFS filesystem utilities (used by shell command handlers) ─────────────

void listDir(fs::FS& fs, const char* dirname, uint8_t levels) {
  Serial.printf("ls %s\n", dirname);
  File root = fs.open(dirname);
  if (!root || !root.isDirectory()) { Serial.println("  (not a directory)"); return; }
  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.printf("  DIR : %s\n", file.name());
      if (levels > 0) listDir(fs, file.name(), levels - 1);
    } else {
      Serial.printf("  %-32s  %6u bytes\n", file.name(), (unsigned)file.size());
    }
    file = root.openNextFile();
  }
}

void readFile(fs::FS& fs, const char* path) {
  File f = fs.open(path, FILE_READ);
  if (!f) { Serial.printf("cat: cannot open %s\n", path); return; }
  while (f.available()) Serial.write(f.read());
  f.close();
}

void copyFile(fs::FS& fs, const char* src, const char* dest) {
  File fsrc = fs.open(src, FILE_READ);
  if (!fsrc) { Serial.printf("cp: cannot open %s\n", src); return; }
  File fdst = fs.open(dest, FILE_WRITE);
  if (!fdst) { fsrc.close(); Serial.printf("cp: cannot create %s\n", dest); return; }
  uint8_t buf[256];
  size_t  n;
  while ((n = fsrc.read(buf, sizeof(buf))) > 0) fdst.write(buf, n);
  fdst.close();
  fsrc.close();
  Serial.printf("cp: %s -> %s\n", src, dest);
}

void deleteFile(fs::FS& fs, const char* path) {
  if (fs.remove(path)) Serial.printf("rm: deleted %s\n", path);
  else                  Serial.printf("rm: failed to delete %s\n", path);
}

void appendFile(fs::FS& fs, const char* path, const char* message) {
  File f = fs.open(path, FILE_APPEND);
  if (!f) { Serial.printf("ap: cannot open %s\n", path); return; }
  f.println(message);
  f.close();
}

// ─── Serial command handlers ──────────────────────────────────────────────────

// Parse a log line's leading "YYYYMMDD-HHMM" timestamp into a Unix epoch
// (UTC).  Returns 0 if the prefix isn't a valid timestamp, in which case the
// caller should skip the line.  Done with manual digit math (not mktime) so
// we don't depend on the system TZ being UTC.
time_t parseLogTimestamp(const char* s) {
  if (!s || strlen(s) < 13 || s[8] != '-') return 0;
  for (int i = 0; i < 13; i++) {
    if (i == 8) continue;
    if (s[i] < '0' || s[i] > '9') return 0;
  }
  int y  = (s[0]-'0')*1000 + (s[1]-'0')*100 + (s[2]-'0')*10 + (s[3]-'0');
  int mo = (s[4]-'0')*10 + (s[5]-'0');
  int d  = (s[6]-'0')*10 + (s[7]-'0');
  int hh = (s[9]-'0')*10 + (s[10]-'0');
  int mm = (s[11]-'0')*10 + (s[12]-'0');
  if (y < 2020 || y > 2100 || mo < 1 || mo > 12 || d < 1 || d > 31
      || hh > 23 || mm > 59) return 0;

  // Howard Hinnant's days_from_civil — exact, no leap-table lookup.
  int yy = y - (mo <= 2 ? 1 : 0);
  int era = (yy >= 0 ? yy : yy - 399) / 400;
  unsigned yoe = (unsigned)(yy - era * 400);
  unsigned doy = (153 * (mo > 2 ? mo - 3 : mo + 9) + 2) / 5 + d - 1;
  unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  long days = (long)era * 146097L + (long)doe - 719468L;
  return (time_t)(days * 86400L + (long)hh * 3600L + (long)mm * 60L);
}

// showlog — dump the log file (ASCII CSV: timestamp,temp,humidity per line).
int cmdShowLog(const DelimitedStringParser& args, String& errMsg) {
  if (!spiffsOk) { errMsg = "SPIFFS not available"; return 1; }
  if (!SPIFFS.exists(LOG_PATH)) { errMsg = "log file does not exist"; return 1; }

  File f = SPIFFS.open(LOG_PATH, FILE_READ);
  if (!f) { errMsg = "failed to open log file"; return 1; }

  size_t fileBytes = f.size();
  if (fileBytes == 0) { f.close(); Serial.println("(log file is empty)"); return CMD_OK; }

  Serial.printf("# %s  %u bytes\n", LOG_PATH, (unsigned)fileBytes);
  Serial.println("timestamp,temp,humidity");
  while (f.available()) Serial.write(f.read());
  f.close();
  return CMD_OK;
}

int lsHandler(const DelimitedStringParser& args, String& errMsg) {
  listDir(SPIFFS, "/", 9);
  return CMD_OK;
}

int catHandler(const DelimitedStringParser& args, String& errMsg) {
  if (args.count() < 2) { errMsg = "usage: cat <file>"; return 1; }
  readFile(SPIFFS, args[1].c_str());
  return CMD_OK;
}

int cpHandler(const DelimitedStringParser& args, String& errMsg) {
  if (args.count() < 3) { errMsg = "usage: cp <src> <dest>"; return 1; }
  copyFile(SPIFFS, args[1].c_str(), args[2].c_str());
  return CMD_OK;
}

int rmHandler(const DelimitedStringParser& args, String& errMsg) {
  if (args.count() < 2) { errMsg = "usage: rm <file>"; return 1; }
  deleteFile(SPIFFS, args[1].c_str());
  return CMD_OK;
}

int apHandler(const DelimitedStringParser& args, String& errMsg) {
  if (args.count() < 3) { errMsg = "usage: ap <file> <text>"; return 1; }
  String msg = args[2];
  for (int i = 3; i < args.count(); i++) { msg += " "; msg += args[i]; }
  appendFile(SPIFFS, args[1].c_str(), msg.c_str());
  return CMD_OK;
}

int helpHandler(const DelimitedStringParser& args, String& errMsg) {
  for (int i = 0; i < cmd.commandCount(); i++)
    Serial.printf("  %-8s %s\n",
                  cmd.getCommandName(i).c_str(),
                  cmd.getDescription(cmd.getCommandName(i)).c_str());
  return CMD_OK;
}

// ─── SPIFFS helpers ───────────────────────────────────────────────────────────

// Scan the log file for the newest timestamp present and return it as a UTC
// epoch.  Falls back to 2026-01-01 00:00:01 UTC (the same value setup() seeds
// the RTC with) if SPIFFS is unavailable, the file is missing or empty, or no
// line parses.  Used in setup() after the config file has set rtc.offset to
// advance the RTC past the boot seed so timestamps stay monotonic across the
// gap between power-up and the first NTP sync.  NTP overrides this later if
// it succeeds.
//
// Log lines are written in local time by appendToLog(), so the parsed value
// is converted back to UTC by subtracting rtc.offset — matches what
// loadFromSpiffs() does.
time_t findNewestLogTimestamp() {
  // 2026-01-01 00:00:01 UTC; same as the rtc.setTime() seed in setup().
  const time_t defaultEpoch = parseLogTimestamp("20260101-0000") + 1;

  if (!spiffsOk || !SPIFFS.exists(LOG_PATH)) return defaultEpoch;
  File f = SPIFFS.open(LOG_PATH, FILE_READ);
  if (!f) return defaultEpoch;

  time_t newest = 0;
  String line;
  line.reserve(40);
  while (f.available()) {
    char c = (char)f.read();
    if (c == '\r') continue;
    if (c == '\n') {
      time_t ts = parseLogTimestamp(line.c_str());
      if (ts > newest) newest = ts;
      line = "";
    } else {
      line += c;
    }
  }
  // Trailing line without a terminating newline (rare but possible).
  if (line.length() > 0) {
    time_t ts = parseLogTimestamp(line.c_str());
    if (ts > newest) newest = ts;
  }
  f.close();

  if (newest == 0) return defaultEpoch;
  return newest - rtc.offset;   // log is local time; return UTC
}

// On boot: read the last LOG_WEEK_RECS lines from the ASCII log into the
// sample buffer so the graph is populated immediately.  Two-pass approach:
// pass 1 counts newlines; pass 2 skips the leading lines we don't need and
// parses the remainder.  Avoids needing record-offset metadata or holding
// the whole file in RAM, at the cost of reading the file twice.
//
// Stale-entry filter: an entry is dropped unless its timestamp is within
// [now − PLOT_WINDOW_SEC, now] where now is `rtc.getEpoch()`.  At the boot
// call, setup() has already advanced the RTC to the newest log timestamp
// (findNewestLogTimestamp), so "stale" means "older than 7 days before the
// newest entry on disk" — that's how we drop legacy records like the
// pre-NTP 2025-12-31 lines a previous failed reboot wrote.  The same filter
// runs again on the post-NTP reload (reloadAfterNtp), this time against the
// real wall-clock time, which catches the case where the bootEpoch itself
// was a stale value.
void loadFromSpiffs() {
  if (!SPIFFS.exists(LOG_PATH)) {
    Serial.println("SPIFFS: no log file found – starting fresh");
    return;
  }

  File f = SPIFFS.open(LOG_PATH, FILE_READ);
  if (!f) {
    Serial.println("SPIFFS: failed to open log for reading");
    return;
  }
  if (f.size() == 0) {
    f.close();
    Serial.println("SPIFFS: log file is empty");
    return;
  }

  // Pass 1: count newline-terminated lines.
  int totalLines = 0;
  while (f.available()) {
    if (f.read() == '\n') totalLines++;
  }
  if (totalLines == 0) { f.close(); return; }

  int toSkip = (totalLines > LOG_WEEK_RECS) ? (totalLines - LOG_WEEK_RECS) : 0;

  // Pass 2: rewind, skip leading lines, parse the rest.
  f.seek(0);
  while (toSkip > 0 && f.available()) {
    if (f.read() == '\n') toSkip--;
  }

  // Stale-entry filter window: [minEpoch, maxEpoch].
  time_t maxEpoch = rtc.getEpoch();
  time_t minEpoch = maxEpoch - PLOT_WINDOW_SEC;

  String line;
  line.reserve(40);
  int    dropped = 0;
  while (f.available() && nSmpl < MAX_SMPL) {
    char c = (char)f.read();
    if (c == '\r') continue;
    if (c == '\n') {
      // Expect: YYYYMMDD-HHMM,temp,humidity
      int c1 = line.indexOf(',');
      int c2 = line.indexOf(',', c1 + 1);
      if (c1 > 0 && c2 > c1) {
        time_t ts = parseLogTimestamp(line.c_str());
        if (ts > 0) {
          // The on-disk timestamp is local wall-clock time (see appendToLog).
          // Convert back to UTC for in-memory storage so timeBuf[] stays in
          // a single canonical timezone regardless of future TZ changes.
          time_t ep = ts - rtc.offset;
          if (ep < minEpoch || ep > maxEpoch) {
            dropped++;
          } else {
            timeBuf[nSmpl] = ep;
            tBuf   [nSmpl] = line.substring(c1 + 1, c2).toFloat();
            hBuf   [nSmpl] = line.substring(c2 + 1).toFloat();
            nSmpl++;
          }
        }
      }
      line = "";
    } else {
      line += c;
    }
  }
  f.close();

  nLogSmpl = nSmpl;
  Serial.printf("SPIFFS: loaded %d of %d records (dropped %d stale, week capacity %d)\n",
                nSmpl, totalLines, dropped, LOG_WEEK_RECS);
}

// Re-load the plot buffer from disk now that the wall-clock is trustworthy.
// Called once on the first ntp.isSynced() rising edge.  The initial
// loadFromSpiffs() at boot uses bootEpoch (newest log entry) as the filter
// reference, which is the right answer when at least one good entry exists
// on disk but the wrong one when the whole log is stale (e.g. the device
// rebooted with no network and only wrote junk timestamps for hours).
// After NTP we know real time, so re-running the load with the same filter
// against rtc.getEpoch() drops anything older than 7 days from now and
// catches that case.  appendToLog() is called first so any pendingTime
// readings from the boot-to-NTP gap are written out and then read back in
// — without that flush the reset to nSmpl=0 would drop them.
void reloadAfterNtp() {
  if (!spiffsOk) return;
  Serial.println("Reload: post-NTP — flushing pending and re-reading log");
  appendToLog();
  nSmpl    = 0;
  nLogSmpl = 0;
  loadFromSpiffs();
  drawGraph();
}

// Flush the pending buffer to the log file (called every hour).
// Writes all readings accumulated since the last flush as ASCII lines, then
// clears the buffer.  Format: YYYYMMDD-HHMM,temp,humidity\n
void appendToLog() {
  if (!spiffsOk || pendingCount == 0) return;

  File f = SPIFFS.open(LOG_PATH, FILE_APPEND);
  if (!f) { Serial.println("SPIFFS: failed to open log for append"); return; }

  for (int i = 0; i < pendingCount; i++) {
    struct tm tm;
    // pendingTime[i] is UTC; add rtc.offset so the on-disk timestamp is
    // local wall-clock time, which is what someone reviewing the log expects.
    time_t t = pendingTime[i] + rtc.offset;
    gmtime_r(&t, &tm);
    f.printf("%04d%02d%02d-%02d%02d,%.1f,%.1f\n",
             tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
             tm.tm_hour, tm.tm_min,
             pendingT[i] / 10.0f, pendingH[i] / 10.0f);
  }
  f.close();
  Serial.printf("SPIFFS: flushed %d records to log\n", pendingCount);
  pendingCount = 0;
}

// If the log exceeds LOG_TRIM_THRESHOLD bytes, drop chunks of LOG_TRIM_BYTES
// from the front until under threshold.  Records are variable-length ASCII
// lines, so the drop point is rounded UP to the next newline so the kept
// portion always starts at the beginning of a record.  The rewrite uses a
// heap buffer sized to the retained data; call only from setup() or the
// daily timer to avoid starving other allocations.
void trimLog() {
  if (!spiffsOk || !SPIFFS.exists(LOG_PATH)) return;

  File f = SPIFFS.open(LOG_PATH, FILE_READ);
  if (!f) return;
  size_t fileBytes = f.size();
  f.close();

  if (fileBytes <= LOG_TRIM_THRESHOLD) return;  // nothing to do

  // Pick a tentative drop size that puts us under threshold.
  size_t dropBytes = 0;
  while (fileBytes - dropBytes > LOG_TRIM_THRESHOLD) {
    dropBytes += LOG_TRIM_BYTES;
  }
  if (dropBytes >= fileBytes) return;   // guard against an empty result

  // Align dropBytes to the next newline so we don't split a record.
  f = SPIFFS.open(LOG_PATH, FILE_READ);
  if (!f) return;
  f.seek(dropBytes);
  while (f.available()) {
    char c = (char)f.read();
    dropBytes++;
    if (c == '\n') break;
  }
  if (dropBytes >= fileBytes) { f.close(); return; }

  size_t keepBytes = fileBytes - dropBytes;
  Serial.printf("SPIFFS: trimming log (%u bytes, dropping %u, keeping %u)\n",
                (unsigned)fileBytes, (unsigned)dropBytes, (unsigned)keepBytes);

  uint8_t* buf = new uint8_t[keepBytes];
  if (!buf) { f.close(); return; }
  size_t got = f.read(buf, keepBytes);
  f.close();

  f = SPIFFS.open(LOG_PATH, FILE_WRITE);
  if (f) {
    f.write(buf, got);
    f.close();
    Serial.printf("SPIFFS: log trimmed to %u bytes\n", (unsigned)got);
  }
  delete[] buf;
}

// ─── Data management ──────────────────────────────────────────────────────────

// Append a sample with its capture timestamp; when full, shift the oldest
// out (O(n) every 6 min – fine).  All three parallel arrays move together.
// Decrement nLogSmpl when a log-loaded sample is evicted.
void pushSample(time_t ts, float t, float h) {
  if (nSmpl < MAX_SMPL) {
    timeBuf[nSmpl] = ts;
    tBuf   [nSmpl] = t;
    hBuf   [nSmpl] = h;
    ++nSmpl;
  } else {
    memmove(timeBuf, timeBuf + 1, (MAX_SMPL - 1) * sizeof(timeBuf[0]));
    memmove(tBuf,    tBuf    + 1, (MAX_SMPL - 1) * sizeof(tBuf[0]));
    memmove(hBuf,    hBuf    + 1, (MAX_SMPL - 1) * sizeof(hBuf[0]));
    timeBuf[MAX_SMPL - 1] = ts;
    tBuf   [MAX_SMPL - 1] = t;
    hBuf   [MAX_SMPL - 1] = h;
    if (nLogSmpl > 0) --nLogSmpl;
  }
}

// ─── Coordinate helpers ───────────────────────────────────────────────────────

// Value → Y pixel (vMax at top, vMin at bottom of plot area).
inline int valToY(float v, float vMin, float vMax) {
  float n = (vMax > vMin) ? (v - vMin) / (vMax - vMin) : 0.5f;
  n = constrain(n, 0.0f, 1.0f);
  return PLT_Y + PLT_H - (int)(n * PLT_H);
}

// ─── Header ───────────────────────────────────────────────────────────────────

void drawHeader() {
  // Hold the SPI bus for the entire function so WiFi-driven interrupts on
  // Core 0 can't land between the dozens of small LovyanGFX transactions
  // and desync the panel's CS/DC framing.
  tft.startWrite();
  tft.fillRect(0, 0, SCR_W, HDR_H - 1, C_HDR_BG);

  // ── Temperature panel (left half) ──
  tft.setTextDatum(lgfx::top_center);
  tft.setTextColor(C_TEMP, C_HDR_BG);
  tft.setFont(&fonts::Font2);
  tft.drawString("TEMP (F)", SCR_W / 4, 3);

  if (isnan(curT)) {
    tft.setFont(&fonts::Font4);
    tft.drawString("---", SCR_W / 4, 22);
  } else {
    String numStr = String((int)round(curT));
    tft.setFont(&fonts::Font4);
    int nw  = tft.textWidth(numStr);              // px width of number
    tft.setFont(&fonts::Font2);
    int sfx = tft.textWidth("F");                 // px width of "F"
    int cx  = SCR_W / 4;
    int tx  = cx - (nw + 6 + sfx) / 2;           // 6 px: gap + drawn °

    tft.setTextDatum(lgfx::top_left);
    tft.setFont(&fonts::Font4);
    tft.drawString(numStr, tx, 22);

    // Draw a small ° circle (r=3) just above the baseline of font 4
    int ox = tx + nw + 2;
    int oy = 24;
    tft.drawCircle(ox, oy, 3, C_TEMP);

    tft.setTextColor(C_TEMP, C_HDR_BG);
    tft.setTextDatum(lgfx::top_left);
    tft.setFont(&fonts::Font2);
    tft.drawString("F", ox + 6, 30);
  }

  // ── Centre divider ──
  tft.drawFastVLine(SCR_W / 2, 0, HDR_H - 1, C_DIVIDER);

  // ── Humidity panel (right half) ──
  tft.setTextDatum(lgfx::top_center);
  tft.setTextColor(C_HUMID, C_HDR_BG);
  tft.setFont(&fonts::Font2);
  tft.drawString("HUMIDITY", 3 * SCR_W / 4, 3);

  if (isnan(curH)) {
    tft.setFont(&fonts::Font4);
    tft.drawString("---", 3 * SCR_W / 4, 22);
  } else {
    String hStr = String((int)round(curH)) + "%";
    tft.setFont(&fonts::Font4);
    tft.drawString(hStr, 3 * SCR_W / 4, 22);
  }

  // ── Bottom border ──
  tft.drawFastHLine(0, HDR_H - 1, SCR_W, C_WHITE);
  tft.endWrite();
}

// ─── Clock ────────────────────────────────────────────────────────────────────
// Refreshes only the small date/time strip in the header.  Called once per
// second from onSecondTick(); cheap because it overwrites only ~10 px tall.
// Uses 24-hour format (%H, not %I).
void drawClock() {
  String dt = rtc.getTime("%m/%d/%Y %H:%M:%S");

  // Bracket fillRect + drawString as one atomic SPI session so WiFi
  // interrupts can't slip between them and desync the panel.
  tft.startWrite();
  tft.fillRect(0, CLK_Y, SCR_W, CLK_H, C_HDR_BG);
  tft.setTextColor(C_WHITE, C_HDR_BG);
  tft.setTextDatum(lgfx::middle_center);
  tft.setFont(&fonts::Font0);
  tft.drawString(dt, SCR_W / 2, CLK_Y + CLK_H / 2);
  tft.endWrite();
}

// ─── Status line ──────────────────────────────────────────────────────────────
// WiFi status on the left edge, version on the right.  Repaints the entire
// 8 px row every call so transitions (N/C → IP) are clean even though the
// new string is shorter than the old.
void drawStatus() {
  String w = (wifi && wifi->isConnected())
               ? (String("WiFi ") + WiFi.localIP().toString())
               : String("WiFi N/C");

  tft.startWrite();
  tft.fillRect(0, STAT_Y, SCR_W, STAT_H, C_BG);
  tft.setTextColor(TFT_DARKGREY, C_BG);
  tft.setFont(&fonts::Font0);
  tft.setTextDatum(lgfx::middle_left);
  tft.drawString(w, 2, STAT_Y + STAT_H / 2);
  tft.setTextDatum(lgfx::middle_right);
  tft.drawString(VERSION_SHORT, SCR_W - 2, STAT_Y + STAT_H / 2);
  tft.endWrite();
}

// ─── Graph ────────────────────────────────────────────────────────────────────

void drawGraph() {
  // Hold the SPI bus across all the small draw calls below.  drawGraph
  // issues hundreds of fillRect/drawString/drawLine ops; each one is its
  // own SPI transaction by default, and any of those inter-call gaps can
  // be hit by a WiFi-driven interrupt that desyncs the panel.
  tft.startWrite();
  tft.fillRect(0, GRF_Y, SCR_W, GRF_H, C_BG);

  // Build a sorted index of plottable samples.  Drops anything with a zero
  // timestamp and (only when the RTC is trustworthy) anything dated in the
  // future, then sorts ascending by time.  Protects the plot from
  // out-of-order log entries and from samples captured before NTP fixed
  // the clock.  The "trustworthy" gate matters at boot: drawGraph() runs at
  // the end of setup() before NTP has synced, so the RTC is still at its
  // seed (2026-01-01 00:00:01) and a strict timeBuf[i] <= now would discard
  // every real log point.  Once NTP syncs, recoverPanel() redraws the graph
  // and the future-filter then activates.
  static int idx[MAX_SMPL];
  time_t now           = rtc.getEpoch();
  bool   filterFuture  = ntp.isSynced();
  int n = 0;
  for (int i = 0; i < nSmpl; i++) {
    if (timeBuf[i] <= 0) continue;
    if (filterFuture && timeBuf[i] > now) continue;
    idx[n++] = i;
  }
  // Insertion sort by timestamp; the buffer is mostly already in order.
  for (int i = 1; i < n; i++) {
    int    k  = idx[i];
    time_t kt = timeBuf[k];
    int    j  = i;
    while (j > 0 && timeBuf[idx[j - 1]] > kt) { idx[j] = idx[j - 1]; j--; }
    idx[j] = k;
  }

  if (n < 2) {
    tft.setTextColor(TFT_DARKGREY, C_BG);
    tft.setTextDatum(lgfx::middle_center);
    tft.setFont(&fonts::Font2);
    tft.drawString("Collecting data...", SCR_W / 2, GRF_Y + GRF_H / 2);
    tft.endWrite();
    return;
  }

  // ── Auto-scale temperature Y axis ──
  float tMin = tBuf[idx[0]], tMax = tBuf[idx[0]];
  for (int i = 1; i < n; i++) {
    float v = tBuf[idx[i]];
    if (v < tMin) tMin = v;
    if (v > tMax) tMax = v;
  }
  // Guarantee at least a 10 °F span; pad 10 %; snap to 5 °F grid.
  float tSpan = max(tMax - tMin, 10.0f);
  tMin = floor((tMin - tSpan * 0.1f) / 5.0f) * 5.0f;
  tMax =  ceil((tMax + tSpan * 0.1f) / 5.0f) * 5.0f;

  const float hMin = 0.0f, hMax = 100.0f;

  // ── Horizontal grid (5 lines → 4 intervals) ──
  for (int g = 0; g <= 4; g++) {
    float frac = g / 4.0f;
    int   gy   = PLT_Y + (int)(frac * PLT_H);

    tft.drawFastHLine(PLT_X, gy, PLT_W, C_GRID);

    // Temp label – left axis
    int tLbl = (int)round(tMax - (tMax - tMin) * frac);
    tft.setTextColor(C_TEMP, C_BG);
    tft.setTextDatum(lgfx::middle_right);
    tft.setFont(&fonts::Font0);
    tft.drawString(String(tLbl), PLT_X - 2, gy);

    // Humidity label – right axis
    int hLbl = (int)round(hMax - (hMax - hMin) * frac);
    tft.setTextColor(C_HUMID, C_BG);
    tft.setTextDatum(lgfx::middle_left);
    tft.drawString(String(hLbl) + "%", PLT_X + PLT_W + 3, gy);
  }

  // ── Axis borders ──
  tft.drawFastVLine(PLT_X,         PLT_Y, PLT_H + 1, C_AXIS);
  tft.drawFastVLine(PLT_X + PLT_W, PLT_Y, PLT_H + 1, C_AXIS);
  tft.drawFastHLine(PLT_X, PLT_Y + PLT_H, PLT_W + 1, C_AXIS);

  // ── Time range: rolling 7-day window ending at the newest valid sample ──
  // Defense in depth: loadFromSpiffs already drops stale entries against the
  // RTC, but if any slip past (or pushSample picked up a garbage timestamp
  // before NTP synced), the plot domain would otherwise stretch to span
  // months and compress real data into a 1-pixel sliver at the left axis.
  time_t tEnd     = timeBuf[idx[n - 1]];
  time_t plotMin  = tEnd - PLOT_WINDOW_SEC;
  int    m = 0;
  for (int i = 0; i < n; i++) {
    if (timeBuf[idx[i]] >= plotMin) idx[m++] = idx[i];
  }
  n = m;
  if (n < 2) {
    tft.setTextColor(TFT_DARKGREY, C_BG);
    tft.setTextDatum(lgfx::middle_center);
    tft.setFont(&fonts::Font2);
    tft.drawString("Collecting data...", SCR_W / 2, GRF_Y + GRF_H / 2);
    tft.endWrite();
    return;
  }
  time_t tStart   = timeBuf[idx[0]];
  long   timeSpan = (long)(tEnd - tStart);
  if (timeSpan <= 0) timeSpan = 1;   // guard against degenerate single-time data

  // ── Time axis labels: hour-of-day in 12-hr A/P form (e.g. 8A, 1P, 12P) ──
  // Tick time is rounded to the nearest hour so labels are clean.
  tft.setTextColor(TFT_DARKGREY, C_BG);
  tft.setTextDatum(lgfx::top_center);
  tft.setFont(&fonts::Font0);
  for (int t = 0; t <= 4; t++) {
    int    xPix     = PLT_X + (int)((long)t * PLT_W / 4);
    // tickTime is UTC; add rtc.offset so the displayed hour is local.
    time_t tickTime = tStart + (long)t * timeSpan / 4 + rtc.offset;
    struct tm tm;
    gmtime_r(&tickTime, &tm);
    int h = tm.tm_hour + (tm.tm_min >= 30 ? 1 : 0);  // round to nearest hour
    if (h >= 24) h = 0;
    int  h12  = h % 12; if (h12 == 0) h12 = 12;
    char ampm = (h < 12) ? 'A' : 'P';
    char lbl[6];
    snprintf(lbl, sizeof(lbl), "%d%c", h12, ampm);
    tft.drawString(lbl, xPix, PLT_Y + PLT_H + 2);
  }

  // ── Plot lines — connect adjacent samples at their actual time positions ──
  for (int i = 0; i < n - 1; i++) {
    int a  = idx[i];
    int b  = idx[i + 1];
    int x1 = PLT_X + (int)((long)(timeBuf[a] - tStart) * (PLT_W - 1) / timeSpan);
    int x2 = PLT_X + (int)((long)(timeBuf[b] - tStart) * (PLT_W - 1) / timeSpan);
    tft.drawLine(x1, valToY(tBuf[a], tMin, tMax),
                 x2, valToY(tBuf[b], tMin, tMax), C_TEMP);
    tft.drawLine(x1, valToY(hBuf[a], hMin, hMax),
                 x2, valToY(hBuf[b], hMin, hMax), C_HUMID);
  }

  // ── Legend (top-right corner of plot) ──
  int lx = PLT_X + PLT_W - 52;
  int ly = PLT_Y + 4;
  tft.fillRect(lx - 2, ly - 2, 54, 24, C_BG);
  tft.drawFastHLine(lx,      ly + 4,  10, C_TEMP);
  tft.setTextColor(C_TEMP,  C_BG);
  tft.setTextDatum(lgfx::middle_left);
  tft.setFont(&fonts::Font0);
  tft.drawString("Temp", lx + 13, ly + 4);
  tft.drawFastHLine(lx,      ly + 16, 10, C_HUMID);
  tft.setTextColor(C_HUMID, C_BG);
  tft.drawString("Humid", lx + 13, ly + 16);
  tft.endWrite();
}

// ─── Panel recovery ───────────────────────────────────────────────────────────
// Re-runs the LovyanGFX init sequence (SWRESET + full panel init) and redraws
// everything currently on screen.  Required after any WiFi.begin() attempt on
// this older single-USB CYD: WiFi association reliably leaves the ILI9341 in
// a desynced state (white screen / random stripes) regardless of bus_shared,
// DMA, or freq_write settings.  Called whether the attempt succeeded or
// failed — the corruption happens at begin() time, not at success/failure.
void recoverPanel() {
  Serial.println("Panel: re-init");
  tft.init();
  tft.setRotation(1);
  tft.setBrightness(200);
  tft.fillScreen(C_BG);
  drawHeader();
  drawClock();
  drawGraph();
  drawStatus();
}

// ─── HTTP server ──────────────────────────────────────────────────────────────
// GET  /         → simple HTML index
// GET  /log      → stream LOG_PATH as text/csv
// PUT  /log      → replace LOG_PATH with the request body
// GET  /config   → stream CONFIG_PATH as text/plain
// PUT  /config   → replace CONFIG_PATH with the request body
//
// PUT bodies are buffered in heap via webServer.arg("plain") — fine for
// config.ini (~1 KB) but risky for full log replacement (~700 KB vs ~150 KB
// free heap).  If a large log restore is needed, do it incrementally or rm
// the file first via the serial command.

void httpHandleRoot() {
  // curT / curH are NaN until the first 6-minute averaging window completes.
  // Show "--" in that case rather than the printf default of "nan".
  char tStr[12], hStr[12];
  if (isnan(curT)) strcpy(tStr, "--"); else snprintf(tStr, sizeof(tStr), "%.1f", curT);
  if (isnan(curH)) strcpy(hStr, "--"); else snprintf(hStr, sizeof(hStr), "%.1f", curH);

  uint32_t up = millis() / 1000;
  uint32_t d  = up / 86400;
  uint32_t h  = (up % 86400) / 3600;
  uint32_t m  = (up % 3600)  / 60;
  uint32_t s  = up % 60;
  char upStr[32];
  snprintf(upStr, sizeof(upStr), "%lud %luh %lum %lus",
           (unsigned long)d, (unsigned long)h, (unsigned long)m, (unsigned long)s);

  String html = F("<html><body><h2>");
  html += VERSION;
  html += F("</h2><p>");
  html += rtc.getTime("%m/%d/%Y %H:%M:%S");
  html += F("</p><p>Uptime: ");
  html += upStr;
  html += F("</p><p>Temp: ");
  html += tStr;
  html += F(" &deg;F &nbsp; Humidity: ");
  html += hStr;
  html += F(" %</p><ul>"
           "<li><a href=\"/log\">/log</a> &mdash; sample log (CSV)</li>"
           "<li><a href=\"/config\">/config</a> &mdash; config.ini</li>"
           "</ul><p>PUT to /log or /config to replace.</p></body></html>");
  webServer.send(200, "text/html", html);
}

void httpServeFile(const char* path, const char* mime) {
  if (!spiffsOk || !SPIFFS.exists(path)) {
    webServer.send(404, "text/plain", "not found");
    return;
  }
  File f = SPIFFS.open(path, FILE_READ);
  if (!f) { webServer.send(500, "text/plain", "open failed"); return; }
  webServer.streamFile(f, mime);
  f.close();
}

void httpReplaceFile(const char* path) {
  if (!spiffsOk) { webServer.send(503, "text/plain", "SPIFFS unavailable"); return; }
  if (!webServer.hasArg("plain")) {
    webServer.send(400, "text/plain", "missing body");
    return;
  }
  const String& body = webServer.arg("plain");
  File f = SPIFFS.open(path, FILE_WRITE);
  if (!f) { webServer.send(500, "text/plain", "create failed"); return; }
  size_t n = f.print(body);
  f.close();
  String resp = String("wrote ") + n + " bytes\n";
  webServer.send(200, "text/plain", resp);
  Serial.printf("HTTP: replaced %s (%u bytes)\n", path, (unsigned)n);
}

void setupWebServer() {
  webServer.on("/",       HTTP_GET, httpHandleRoot);
  webServer.on("/log",    HTTP_GET, [](){ httpServeFile(LOG_PATH,    "text/csv");   });
  webServer.on("/log",    HTTP_PUT, [](){ httpReplaceFile(LOG_PATH);                });
  webServer.on("/config", HTTP_GET, [](){ httpServeFile(CONFIG_PATH, "text/plain"); });
  webServer.on("/config", HTTP_PUT, [](){ httpReplaceFile(CONFIG_PATH);             });
  webServer.onNotFound([](){ webServer.send(404, "text/plain", "not found"); });
}

void startWebServer() {
  if (httpServerActive) return;
  webServer.begin();
  httpServerActive = true;
  Serial.println("HTTP: server started on port 80");
}

void stopWebServer() {
  if (!httpServerActive) return;
  webServer.stop();
  httpServerActive = false;
  Serial.println("HTTP: server stopped");
}

// ─── Reboot ───────────────────────────────────────────────────────────────────
// Gracefully shut down the network stack before restarting.  Closes any open
// HTTP sockets, disconnects the WiFi station so the AP gets a deauth instead
// of a stale association timeout, then powers the radio off and restarts.
// Used by the daily 01:00 timer in onHourTick and by the boot-button test
// path in loop().  Does not return.
void rebootDevice(const char* reason) {
  Serial.printf("Reboot: %s\n", reason);
  stopWebServer();
  if (wifi) wifi->disconnect();
  WiFi.disconnect(true);   // wifioff=true: drop association and power down radio
  WiFi.mode(WIFI_OFF);
  Serial.flush();
  delay(500);
  ESP.restart();
}

// ─── Scheduler callbacks ──────────────────────────────────────────────────────
// All periodic work is driven off the RTC via SimpleScheduler.  Each callback
// fires on a clean wall-clock boundary (e.g. seconds divisible by 10) rather
// than millis() elapsed time.

// Every 10 seconds: read the DHT11 and accumulate into the 6-minute averager.
// Also drives the WiFi state machine on every tick (it expects 1 Hz polling).
void onSecondTick() {
  // ── WiFi lifecycle: drive state machine, watch for transitions ──
  // The local WiFiSvc.cpp uses 60 s connect timeout and 3600 s backoff, so
  // each CONNECTING phase ends in either CONNECTED or ERRORTIMEOUT and the
  // next attempt happens an hour later.  We recover the panel on BOTH
  // outcomes — WiFi.begin() corrupts the ILI9341 whether or not the
  // association ultimately succeeds.
  static WiFiSvc::State prevWifiState = WiFiSvc::DISCONNECTED;
  if (wifi) {
    wifi->poll();
    WiFiSvc::State currState = wifi->state();

    if (prevWifiState == WiFiSvc::CONNECTING && currState == WiFiSvc::CONNECTED) {
      Serial.println("WiFi: connected — recovering panel");
      recoverPanel();
      startWebServer();
    } else if (prevWifiState == WiFiSvc::CONNECTING && currState == WiFiSvc::ERRORTIMEOUT) {
      Serial.println("WiFi: 60s connect timeout — recovering panel; 1h backoff");
      recoverPanel();
    } else if (prevWifiState == WiFiSvc::CONNECTED && currState != WiFiSvc::CONNECTED) {
      Serial.println("WiFi: link dropped — stopping HTTP server");
      stopWebServer();
    }
    prevWifiState = currState;
  }

#if ENABLE_NTP
  // Initial NTP sync: once, the first time WiFi comes up.  begin() runs once
  // for the lifetime of the program; poll() keeps retrying only until the
  // first sync succeeds, then becomes a no-op.  Thereafter the daily resync
  // from onDayTick is the only NTP traffic.
  static bool ntpInitialized = false;
  static bool ntpEverSynced  = false;
  if (wifi && wifi->isConnected()) {
    if (!ntpInitialized) {
      Serial.println("NTP: starting");
      ntp.begin();
      ntpInitialized = true;
    }
    if (!ntp.isSynced()) ntp.poll();
  }
  // First-sync rising edge: the wall clock just became trustworthy, so
  // re-load the plot buffer from disk to drop any stale entries the
  // boot-time load couldn't recognise (e.g. the whole log was junk because
  // the previous boot never got NTP).  Runs exactly once per power cycle.
  if (!ntpEverSynced && ntp.isSynced()) {
    ntpEverSynced = true;
    reloadAfterNtp();
  }
#endif

  // Refresh the on-screen clock and status row every second.  Both are
  // small (8–10 px tall) so the cost is trivial and the WiFi line is
  // automatically up to date even without a transition hook.
  drawClock();
  drawStatus();

  if (rtc.getSecond() % FAST_PERIOD_SEC != 0) return;

  float h = dht.readHumidity();
  float t = dht.readTemperature(true);    // true → Fahrenheit

  if (!isnan(t) && !isnan(h)) {
    accumT += t;
    accumH += h;
    accumCount++;
  }
}

// Every 6 minutes: average the accumulator, redraw, stage for hourly flush.
void onMinuteTick() {
  if (rtc.getMinute() % PLOT_PERIOD_MIN != 0) return;

  if (accumCount == 0) return;

  // Average and round to 1 decimal digit.
  curT = roundf((accumT / accumCount) * 10.0f) / 10.0f;
  curH = roundf((accumH / accumCount) * 10.0f) / 10.0f;
  Serial.printf("Temp %.1f, Humidity %.1f  (avg of %d reads)\n",
                curT, curH, accumCount);
  accumT = 0.0f;  accumH = 0.0f;  accumCount = 0;

  drawHeader();
  pushSample(rtc.getEpoch(), curT, curH);
  drawGraph();

  // Stage for next hourly log flush.  Capture the timestamp NOW so the
  // on-disk record reflects when the reading was averaged, not when the
  // hourly flush happens to fire.
  if (pendingCount < (int)SAMPLES_PER_HOUR) {
    pendingTime[pendingCount] = rtc.getEpoch();
    pendingT   [pendingCount] = (int16_t)roundf(curT * 10.0f);
    pendingH   [pendingCount] = (int16_t)roundf(curH * 10.0f);
    pendingCount++;
  }
}

// Every hour: flush the pending averaged readings to the SPIFFS log.
// Also reboots the device once a day at 01:00 — appendToLog() runs first
// so the last hour of data is on disk before restart.  Reboot is gated by
// the local-hour transition into 1 (onHourTick fires once per hour change),
// so a single ESP.restart() is issued per day.
void onHourTick() {
  appendToLog();

  if (rtc.getHour() == 1) {
    rebootDevice("daily 01:00");
  }
}

// Daily at 02:00: trim the log if it has grown past the threshold, and
// force a fresh NTP resync to correct any RTC drift accumulated over 24 h.
// trimLog() is a no-op below threshold, so daily firing is safe and cheaper
// than the previous weekly polling block.
void onDayTick() {
  trimLog();
#if ENABLE_NTP
  if (wifi && wifi->isConnected()) ntp.sync();
#endif
}

// ─── Setup ────────────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  Serial.println(VERSION);

  // Seed the RTC to a known sane epoch (2026-01-01 00:00:01) so any timestamp
  // taken before NTP sync is at least monotonic and after the build date.
  // ESP32Time::setTime args are: sec, min, hour, day, month, year.
  rtc.setTime(1, 0, 0, 1, 1, 2026);
  Serial.printf("RTC seeded: %s\n", rtc.getDateTime().c_str());

  tft.init();
  tft.setRotation(1);       // landscape; USB port on left
  tft.setBrightness(200);   // LovyanGFX controls backlight via PWM on pin 21

  // Boot color sweep: RED → GREEN → BLUE → BLACK so panel health is
  // visible before any other code runs.
  tft.fillScreen(TFT_RED);   delay(500);
  tft.fillScreen(TFT_GREEN); delay(500);
  tft.fillScreen(TFT_BLUE);  delay(500);
  tft.fillScreen(C_BG);

  dht.begin();

  // Mount SPIFFS; format partition on first use.
  spiffsOk = SPIFFS.begin(true);
  if (spiffsOk) {
    Serial.println("SPIFFS mounted");

    // Read /config.ini.  Missing keys leave the buffers as empty strings so
    // downstream code can detect "not configured" without an extra flag.
    ConfigurationFile config(CONFIG_PATH);
    config.get("SSID",     cfgSsid, sizeof(cfgSsid));
    config.get("PASSWORD", cfgPwd,  sizeof(cfgPwd));

    // Time zone offset in hours (signed float, e.g. -8.0 for PST, 5.5 for IST).
    // Stored on rtc.offset (seconds).  rtc keeps UTC epochs internally — only
    // its display-format calls (getTime, getHour, …) and the explicit log /
    // plot formatters apply the offset.  Default 0 if missing → UTC.
    char tzBuf[16] = "";
    if (config.get("TimeZoneOffset", tzBuf, sizeof(tzBuf)) && tzBuf[0]) {
      float hours = atof(tzBuf);
      rtc.offset  = (long)(hours * 3600.0f);
      Serial.printf("TimeZoneOffset: %.2f h (%ld s)\n", hours, rtc.offset);
    } else {
      Serial.println("TimeZoneOffset: not set, using UTC");
    }

    // Advance the RTC from the 2026-01-01 seed to the newest timestamp in
    // the log.  Keeps plot timestamps monotonic across the boot-to-NTP gap;
    // NTP will overwrite this with real time once WiFi associates.
    {
      time_t bootEpoch = findNewestLogTimestamp();
      struct tm tm;
      gmtime_r(&bootEpoch, &tm);
      rtc.setTime(tm.tm_sec, tm.tm_min, tm.tm_hour,
                  tm.tm_mday, tm.tm_mon + 1, tm.tm_year + 1900);
      Serial.printf("RTC advanced from log: %s\n", rtc.getDateTime().c_str());
    }

    loadFromSpiffs();   // pre-fill plot buffer with last week of history
    trimLog();          // trim at boot if log is already oversized
  } else {
    Serial.println("SPIFFS mount failed – logging disabled");
  }

#if ENABLE_WIFI
  // Bring up WiFi using the credentials read from /config.ini.  WiFiSvc owns
  // a state machine that retries on timeout and reconnects on link drop; it
  // is driven by the scheduler's onSecondTick once registered below.
  if (cfgSsid[0] != '\0') {
    wifi = new WiFiSvc(cfgSsid, cfgPwd);
    // Lower the radio TX power BEFORE WiFi.begin() inside wifi->connect()
    // so the very first probe/auth packets don't peak at default +19.5 dBm.
    WiFi.mode(WIFI_STA);
    WiFi.setTxPower(WIFI_POWER_8_5dBm);
    wifi->connect();
  } else {
    Serial.println("WiFi: no SSID configured – skipping connect");
  }
#else
  Serial.println("WiFi: ENABLE_WIFI=0, skipping all WiFi init");
#endif

  // Wire up RTC-driven scheduling.  Each callback decides for itself whether
  // the current tick is a hit (e.g. seconds % 10 == 0) — see callback bodies.
  scheduler.onSecond(onSecondTick);
  scheduler.onMinute(onMinuteTick);
  scheduler.onHour  (onHourTick);
  scheduler.onDay   (onDayTick);

  // Register HTTP routes (server is begun later, only when WiFi is up).
  setupWebServer();

  // Register serial commands.
  cmd.addCommand("showlog", cmdShowLog, "dump log file as CSV (temp,humidity)");
  cmd.addCommand("ls",      lsHandler,  "list SPIFFS files");
  cmd.addCommand("cat",     catHandler, "print file contents");
  cmd.addCommand("cp",      cpHandler,  "copy file: cp <src> <dest>");
  cmd.addCommand("rm",      rmHandler,  "delete file: rm <file>");
  cmd.addCommand("ap",      apHandler,  "append text to file: ap <file> <text>");
  cmd.addCommand("help",    helpHandler,"list available commands");

  drawHeader();
  drawClock();
  drawGraph();
  drawStatus();
}

// ─── Loop ─────────────────────────────────────────────────────────────────────

void loop() {
  // Poll serial input; dispatch completed lines to the command handler.
  console.poll();
  const char* line = console.getLine();
  if (line) {
    int rc = cmd.execute(String(line));
    if (rc != CMD_OK) Serial.println(cmd.lastErrorMsg());
  }

  // Pump HTTP every loop iteration so client latency isn't gated on the
  // 1 Hz scheduler tick.  Only active when WiFi is up — see WiFi lifecycle
  // in onSecondTick(), which calls startWebServer/stopWebServer.
  if (httpServerActive) webServer.handleClient();

  // Fire any due RTC-driven events (fast read, plot, log flush, trim).
  scheduler.poll();
}
