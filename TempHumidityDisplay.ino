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
 *  2) log time/date with temp/humidity - change log file format
 *  x3) display time/date
 *  w4) update log file format with date/time, change purge limits
 *  5) add some ftp server to allow wifi to pull data
 *  x6) going to need a config file with wifi ssid, pwd, ftp user/pwd
 *  7) Already have a command interpreter, perhaps add telnet as well?
 *  x8) components to use - NtpSync,  WiFi service,  telnetserver, rgb, simplescheduler, configurationfile,delimitedstringparser
 *  9) reboot every day at midnight or 2am
 */

#include "ESP32_SPI_9341.h"   // LovyanGFX + CYD pin/panel config
#include <DHT.h>
#include <SPIFFS.h>
#include <ESP32Time.h>
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
#define VERSION "TempHumidityDisplay v4.0  30-Apr-2026"

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
static const int BM = 18;               // bottom (time labels)

static const int PLT_X = LM;
static const int PLT_Y = GRF_Y + TM;
static const int PLT_W = SCR_W - LM - RM;   // 250 px
static const int PLT_H = GRF_H - TM - BM;   // 157 px

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

float tBuf[MAX_SMPL];    // temperature history °F
float hBuf[MAX_SMPL];    // humidity history %
int   nSmpl    = 0;      // total samples currently in buffer
int   nLogSmpl = 0;      // samples loaded from log at boot (hourly spacing)

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

// On boot: read the last LOG_WEEK_RECS lines from the ASCII log into the
// sample buffer so the graph is populated immediately.  Two-pass approach:
// pass 1 counts newlines; pass 2 skips the leading lines we don't need and
// parses the remainder.  Avoids needing record-offset metadata or holding
// the whole file in RAM, at the cost of reading the file twice.
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

  String line;
  line.reserve(40);
  while (f.available() && nSmpl < MAX_SMPL) {
    char c = (char)f.read();
    if (c == '\r') continue;
    if (c == '\n') {
      // Expect: YYYYMMDD-HHMM,temp,humidity  (timestamp ignored on load)
      int c1 = line.indexOf(',');
      int c2 = line.indexOf(',', c1 + 1);
      if (c1 > 0 && c2 > c1) {
        tBuf[nSmpl] = line.substring(c1 + 1, c2).toFloat();
        hBuf[nSmpl] = line.substring(c2 + 1).toFloat();
        nSmpl++;
      }
      line = "";
    } else {
      line += c;
    }
  }
  f.close();

  nLogSmpl = nSmpl;
  Serial.printf("SPIFFS: loaded %d of %d available records (week capacity %d)\n",
                nSmpl, totalLines, LOG_WEEK_RECS);
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
    time_t t = pendingTime[i];
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

// Append a sample; when full, shift the oldest out (O(n) every 6 min – fine).
// Decrement nLogSmpl when a log-loaded sample is evicted so the time span
// calculation in drawGraph() stays accurate.
void pushSample(float t, float h) {
  if (nSmpl < MAX_SMPL) {
    tBuf[nSmpl] = t;
    hBuf[nSmpl] = h;
    ++nSmpl;
  } else {
    memmove(tBuf, tBuf + 1, (MAX_SMPL - 1) * sizeof(float));
    memmove(hBuf, hBuf + 1, (MAX_SMPL - 1) * sizeof(float));
    tBuf[MAX_SMPL - 1] = t;
    hBuf[MAX_SMPL - 1] = h;
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

// ─── Graph ────────────────────────────────────────────────────────────────────

void drawGraph() {
  // Hold the SPI bus across all the small draw calls below.  drawGraph
  // issues hundreds of fillRect/drawString/drawLine ops; each one is its
  // own SPI transaction by default, and any of those inter-call gaps can
  // be hit by a WiFi-driven interrupt that desyncs the panel.
  tft.startWrite();
  tft.fillRect(0, GRF_Y, SCR_W, GRF_H, C_BG);

  if (nSmpl < 2) {
    tft.setTextColor(TFT_DARKGREY, C_BG);
    tft.setTextDatum(lgfx::middle_center);
    tft.setFont(&fonts::Font2);
    tft.drawString("Collecting data...", SCR_W / 2, GRF_Y + GRF_H / 2);
    tft.endWrite();
    return;
  }

  // ── Auto-scale temperature Y axis ──
  float tMin = tBuf[0], tMax = tBuf[0];
  for (int i = 1; i < nSmpl; i++) {
    if (tBuf[i] < tMin) tMin = tBuf[i];
    if (tBuf[i] > tMax) tMax = tBuf[i];
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

  // ── Time axis labels ──
  // The buffer mixes log-loaded samples (6-min spacing, same as live) so the
  // total span is simply nSmpl × SAMPLE_MS.  nLogSmpl tracks eviction of the
  // startup-loaded records but spacing is uniform throughout.
  long spanMin = (long)nSmpl * (long)(SAMPLE_MS / 60000UL);

  tft.setTextColor(TFT_DARKGREY, C_BG);
  tft.setTextDatum(lgfx::top_center);
  tft.setFont(&fonts::Font0);
  for (int t = 0; t <= 4; t++) {
    int  xPix   = PLT_X + (int)((long)t * PLT_W / 4);
    long agoMin = spanMin - (long)t * spanMin / 4;
    String lbl;
    if (agoMin == 0) {
      lbl = "Now";
    } else if (agoMin < 60) {
      lbl = "-" + String((int)agoMin) + "m";
    } else if (agoMin < 1440) {
      lbl = "-" + String((int)(agoMin / 60)) + "h";
    } else {
      lbl = "-" + String((int)(agoMin / 1440)) + "d";
    }
    tft.drawString(lbl, xPix, PLT_Y + PLT_H + 2);
  }

  // ── Plot lines — raw float samples ──
  float scale = (float)(nSmpl - 1) / (float)(PLT_W - 1);
  for (int px = 0; px < PLT_W - 1; px++) {
    int i1 = constrain((int)roundf( px      * scale), 0, nSmpl - 1);
    int i2 = constrain((int)roundf((px + 1) * scale), 0, nSmpl - 1);
    tft.drawLine(PLT_X + px,     valToY(tBuf[i1], tMin, tMax),
                 PLT_X + px + 1, valToY(tBuf[i2], tMin, tMax), C_TEMP);
    tft.drawLine(PLT_X + px,     valToY(hBuf[i1], hMin, hMax),
                 PLT_X + px + 1, valToY(hBuf[i2], hMin, hMax), C_HUMID);
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

// ─── Scheduler callbacks ──────────────────────────────────────────────────────
// All periodic work is driven off the RTC via SimpleScheduler.  Each callback
// fires on a clean wall-clock boundary (e.g. seconds divisible by 10) rather
// than millis() elapsed time.

// Every 10 seconds: read the DHT11 and accumulate into the 6-minute averager.
// Also drives the WiFi state machine on every tick (it expects 1 Hz polling).
void onSecondTick() {
  if (wifi) wifi->poll();

#if ENABLE_NTP
  // Initial NTP sync: once, the first time WiFi comes up.  begin() runs once
  // for the lifetime of the program; poll() keeps retrying only until the
  // first sync succeeds, then becomes a no-op.  Thereafter the daily resync
  // from onDayTick is the only NTP traffic.  Run BEFORE the panel-recovery
  // block so the long tft.init() inside recovery can't push the first NTP
  // packet past lwIP's ready window.
  static bool ntpInitialized = false;
  if (wifi && wifi->isConnected()) {
    if (!ntpInitialized) {
      Serial.println("NTP: starting");
      ntp.begin();
      ntpInitialized = true;
    }
    if (!ntp.isSynced()) ntp.poll();
  }
#endif

  // Recover the panel after WiFi associates.  WiFi association on the older
  // single-USB CYD reliably leaves the ILI9341 in a desynced state (white
  // screen / random stripes), even when our own SPI accesses are atomic.
  // Re-running tft.init() once on the rising edge of WiFi connection
  // issues SWRESET + the full init sequence, restoring the panel; we then
  // redraw everything that should be on screen.  Sticky: only fires once.
  static bool panelRecovered = false;
  if (!panelRecovered && wifi && wifi->isConnected()) {
    tft.init();
    tft.setRotation(1);
    tft.setBrightness(200);
    tft.fillScreen(C_BG);
    drawHeader();
    drawClock();
    drawGraph();
    panelRecovered = true;
  }

  // Refresh the on-screen clock every second.
  drawClock();

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
  pushSample(curT, curH);
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
void onHourTick() {
  appendToLog();
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

  // Fire any due RTC-driven events (fast read, plot, log flush, trim).
  scheduler.poll();
}
