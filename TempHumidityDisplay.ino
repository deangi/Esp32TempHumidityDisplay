/*
 * TempHumidityDisplay.ino
 *
 * Reads temperature (°F) and humidity (%) from a DHT11 on GPIO 22.
 * Displays live readings on the top ¼ of the CYD 2.8" TFT (320×240),
 * and a rolling 7-day graph on the bottom ¾.
 *
 * Hardware
 *   ESP32 CYD (Cheap Yellow Display) — single-USB version ESP32-WROOM board
 *   LovyanGFX driver at 40MHz SPI for 2.8" CYD board (Cheap Yellow Display)
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
 *   Binary, 2 bytes per record: [uint8 temp °F][uint8 humidity %]
 *   One record appended per 6-minute sample.
 *
 *   Capacity math (6-min intervals):
 *     1 week  = 7 × 24 × 60 / 6 = 1680 records = 3360 bytes
 *     1 month = 4 weeks          = 6720 records = 13440 bytes
 *     1 year  = 365 × 24 × 10   = 87600 records ≈ 175200 bytes
 *
 *   On boot:  load last 1680 records (1 week) into the plot buffer.
 *   Hourly:   append current reading to log.
 *   Weekly:   if log exceeds 200000 bytes, remove the oldest month
 *             (13440 bytes) at a time until under threshold.
 *             trimLog() heap-allocates a buffer equal to the retained data
 *             (~187 KB worst-case); ESP32 free heap (~300 KB+) handles this,
 *             but call only from setup() or the weekly timer.
 */

#include "ESP32_SPI_9341.h"   // LovyanGFX + CYD pin/panel config
#include <DHT.h>
#include <SPIFFS.h>
// DelimitedStringParser, ConsoleInput, and CommandHandler are copied from
// the shared Components library into this sketch folder.  The Arduino build
// system compiles all .cpp files in the sketch folder automatically, so only
// the headers need to be included here.
#include "DelimitedStringParser.h"
#include "ConsoleInput.h"
#include "CommandHandler.h"

// ─── Version ──────────────────────────────────────────────────────────────────
#define VERSION "TempHumidityDisplay v3.0  16-Apr-2026"

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

// ─── Sampling ─────────────────────────────────────────────────────────────────
// One sample every 6 min → 1680 samples = 7 days of in-memory history
static const unsigned long SAMPLE_MS = 6UL * 60UL * 1000UL;
static const int           MAX_SMPL  = 1680;

// ─── SPIFFS / Logging ─────────────────────────────────────────────────────────
#define LOG_PATH           "/temphumid.log"
#define LOG_REC_BYTES      2              // [uint8 temp °F][uint8 humidity %]

// Append to log every hour (10 samples per hour × 6-min interval).
static const unsigned long LOG_INTERVAL_MS  = 60UL * 60UL * 1000UL;

// Check trim once per week.
static const unsigned long TRIM_INTERVAL_MS = 7UL * 24UL * 60UL * 60UL * 1000UL;

// 1 week of records loaded into the plot buffer on startup.
static const int LOG_WEEK_RECS  = 1680;                          // 7×24×10

// Trim policy: if the log exceeds this size, remove one month at a time.
static const size_t LOG_TRIM_THRESHOLD = 200000UL;               // bytes
static const size_t LOG_TRIM_BYTES     = 13440UL;                // 1 month = 4×1680×2

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

float tBuf[MAX_SMPL];    // temperature history °F
float hBuf[MAX_SMPL];    // humidity history %
int   nSmpl    = 0;      // total samples currently in buffer
int   nLogSmpl = 0;      // samples loaded from log at boot (hourly spacing)

float curT = NAN, curH = NAN;

bool          spiffsOk    = false;
unsigned long lastReadMs  = 0;
unsigned long lastSampleMs = 0;
unsigned long lastLogMs   = 0;
unsigned long lastTrimMs  = 0;

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

// showlog — dump /temphumid.log to Serial as CSV (temp,humidity per line).
int cmdShowLog(const DelimitedStringParser& args, String& errMsg) {
  if (!spiffsOk) { errMsg = "SPIFFS not available"; return 1; }
  if (!SPIFFS.exists(LOG_PATH)) { errMsg = "log file does not exist"; return 1; }

  File f = SPIFFS.open(LOG_PATH, FILE_READ);
  if (!f) { errMsg = "failed to open log file"; return 1; }

  size_t fileBytes = f.size();
  if (fileBytes == 0) { f.close(); Serial.println("(log file is empty)"); return CMD_OK; }

  int totalRecs = (int)(fileBytes / LOG_REC_BYTES);
  Serial.printf("# temphumid.log  %d records  %u bytes\n",
                totalRecs, (unsigned)fileBytes);
  Serial.println("temp,humidity");

  for (int i = 0; i < totalRecs; i++) {
    uint8_t tb, hb;
    if (f.read(&tb, 1) != 1 || f.read(&hb, 1) != 1) {
      Serial.printf("# unexpected EOF at record %d\n", i);
      break;
    }
    Serial.printf("%d,%d\n", (int)tb, (int)hb);
  }
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

// On boot: read the last 1680 records (1 week) from the log into the sample
// buffer so the graph is populated immediately.
//
// Truncated log: if the file holds fewer than LOG_WEEK_RECS records (e.g. the
// device is new, or the log was just trimmed), all available records are loaded
// starting from the beginning of the file — toLoad is capped at totalRecs.
// Integer division on fileBytes discards any trailing partial record that may
// have been left by a power-loss mid-write, so the read loop always sees only
// complete 2-byte records.  The per-read return-value check provides a final
// safety net in case of unexpected EOF.
void loadFromSpiffs() {
  // File won't exist on first boot or after a SPIFFS format — that is normal.
  if (!SPIFFS.exists(LOG_PATH)) {
    Serial.println("SPIFFS: no log file found – starting fresh");
    return;
  }

  File f = SPIFFS.open(LOG_PATH, FILE_READ);
  if (!f) {                           // filesystem error despite file existing
    Serial.println("SPIFFS: failed to open log for reading");
    return;
  }

  size_t fileBytes = f.size();
  if (fileBytes == 0) {               // file exists but is empty
    f.close();
    Serial.println("SPIFFS: log file is empty");
    return;
  }

  int    totalRecs = (int)(fileBytes / LOG_REC_BYTES); // integer division drops any partial trailing record
  int    toLoad    = min(totalRecs, LOG_WEEK_RECS);    // cap at 1 week; handles truncated log naturally
  if (toLoad == 0) { f.close(); return; }

  // When toLoad == totalRecs (truncated log) this seek resolves to 0,
  // i.e. we read from the start of the file.
  f.seek((size_t)(totalRecs - toLoad) * LOG_REC_BYTES);

  for (int i = 0; i < toLoad; i++) {
    uint8_t tb, hb;
    // Guard against unexpected EOF (e.g. file corruption).
    if (f.read(&tb, 1) != 1 || f.read(&hb, 1) != 1) {
      Serial.printf("SPIFFS: unexpected EOF at record %d – stopping load\n", i);
      break;
    }
    tBuf[nSmpl] = (float)tb;
    hBuf[nSmpl] = (float)hb;
    nSmpl++;
  }
  f.close();

  nLogSmpl = nSmpl;
  Serial.printf("SPIFFS: loaded %d of %d available records (week capacity %d)\n",
                nSmpl, totalRecs, LOG_WEEK_RECS);
}

// Append the current reading to the log file (called every hour).
void appendToLog() {
  if (!spiffsOk || isnan(curT) || isnan(curH)) return;

  File f = SPIFFS.open(LOG_PATH, FILE_APPEND);
  if (!f) { Serial.println("SPIFFS: failed to open log for append"); return; }

  uint8_t tb = (uint8_t)constrain((int)round(curT), 0, 255);
  uint8_t hb = (uint8_t)constrain((int)round(curH), 0, 100);
  f.write(tb);
  f.write(hb);
  f.close();
  Serial.printf("SPIFFS: logged T=%d H=%d\n", (int)tb, (int)hb);
}

// If the log exceeds LOG_TRIM_THRESHOLD bytes, remove LOG_TRIM_BYTES (one
// month) from the front, repeating until under the threshold.  The rewrite
// uses a heap buffer sized to the retained data (~187 KB worst-case at max
// log size).  ESP32 has ~300 KB+ free heap so this is fine, but trimLog()
// should only be called from setup() or the weekly timer — never in a tight
// loop — to avoid starving other allocations.
void trimLog() {
  if (!spiffsOk || !SPIFFS.exists(LOG_PATH)) return;

  File f = SPIFFS.open(LOG_PATH, FILE_READ);
  if (!f) return;
  size_t fileBytes = f.size();
  f.close();

  if (fileBytes <= LOG_TRIM_THRESHOLD) return;  // nothing to do

  // Calculate how many months to drop so we land under the threshold.
  size_t dropBytes = 0;
  while (fileBytes - dropBytes > LOG_TRIM_THRESHOLD) {
    dropBytes += LOG_TRIM_BYTES;
  }
  // Align to record boundary (LOG_REC_BYTES = 2, so already aligned, but be safe).
  dropBytes = (dropBytes / LOG_REC_BYTES) * LOG_REC_BYTES;
  if (dropBytes >= fileBytes) return;   // shouldn't happen; guard anyway

  size_t keepBytes = fileBytes - dropBytes;

  Serial.printf("SPIFFS: trimming log (%u bytes, dropping %u, keeping %u)\n",
                (unsigned)fileBytes, (unsigned)dropBytes, (unsigned)keepBytes);

  f = SPIFFS.open(LOG_PATH, FILE_READ);
  if (!f) return;
  f.seek(dropBytes);

  uint8_t* buf = new uint8_t[keepBytes];
  size_t   got = f.read(buf, keepBytes);
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

// Sample index → X pixel across the plot area.
inline int idxToX(int i, int total) {
  if (total <= 1) return PLT_X;
  return PLT_X + (int)((long)i * PLT_W / (total - 1));
}

// ─── Header ───────────────────────────────────────────────────────────────────

void drawHeader() {
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
}

// ─── Graph ────────────────────────────────────────────────────────────────────

void drawGraph() {
  tft.fillRect(0, GRF_Y, SCR_W, GRF_H, C_BG);

  if (nSmpl < 2) {
    tft.setTextColor(TFT_DARKGREY, C_BG);
    tft.setTextDatum(lgfx::middle_center);
    tft.setFont(&fonts::Font2);
    tft.drawString("Collecting data...", SCR_W / 2, GRF_Y + GRF_H / 2);
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

  // ── Plot lines ──
  for (int i = 0; i < nSmpl - 1; i++) {
    int x1 = idxToX(i,     nSmpl);
    int x2 = idxToX(i + 1, nSmpl);

    tft.drawLine(x1, valToY(tBuf[i],     tMin, tMax),
                 x2, valToY(tBuf[i + 1], tMin, tMax), C_TEMP);

    tft.drawLine(x1, valToY(hBuf[i],     hMin, hMax),
                 x2, valToY(hBuf[i + 1], hMin, hMax), C_HUMID);
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
}

// ─── Setup ────────────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  Serial.println(VERSION);

  tft.init();
  tft.setRotation(1);       // landscape; USB port on left
  tft.setBrightness(200);   // LovyanGFX controls backlight via PWM on pin 21
  tft.fillScreen(C_BG);

  dht.begin();

  // Mount SPIFFS; format partition on first use.
  spiffsOk = SPIFFS.begin(true);
  if (spiffsOk) {
    Serial.println("SPIFFS mounted");
    loadFromSpiffs();   // pre-fill plot buffer with last week of history
    trimLog();          // trim at boot if log is already oversized
  } else {
    Serial.println("SPIFFS mount failed – logging disabled");
  }

  // Start timers from now: first log write in 1 hour, first trim check in 1 week.
  lastLogMs  = millis();
  lastTrimMs = millis();

  // Register serial commands.
  cmd.addCommand("showlog", cmdShowLog, "dump log file as CSV (temp,humidity)");
  cmd.addCommand("ls",      lsHandler,  "list SPIFFS files");
  cmd.addCommand("cat",     catHandler, "print file contents");
  cmd.addCommand("cp",      cpHandler,  "copy file: cp <src> <dest>");
  cmd.addCommand("rm",      rmHandler,  "delete file: rm <file>");
  cmd.addCommand("ap",      apHandler,  "append text to file: ap <file> <text>");
  cmd.addCommand("help",    helpHandler,"list available commands");

  drawHeader();
  drawGraph();
}

// ─── Loop ─────────────────────────────────────────────────────────────────────

void loop() {
  unsigned long now = millis();

  // Poll serial input; dispatch completed lines to the command handler.
  console.poll();
  const char* line = console.getLine();
  if (line) {
    int rc = cmd.execute(String(line));
    if (rc != CMD_OK) Serial.println(cmd.lastErrorMsg());
  }

  // Read sensor every 3 s (DHT11 max ~0.5 Hz; allow extra margin).
  if (now - lastReadMs >= 3000UL) {
    lastReadMs = now;

    float h = dht.readHumidity();
    float t = dht.readTemperature(true);    // true → Fahrenheit

    Serial.printf("Temp %f, Humidity %f\n", t, h);
    if (!isnan(t) && !isnan(h)) {
      curT = t;
      curH = h;
      drawHeader();

      // Take the very first sample as soon as we get a valid reading.
      if (nSmpl == 0) {
        pushSample(curT, curH);
        lastSampleMs = now;
        drawGraph();
      }
    }
  }

  // Archive a plot sample every 6 minutes.
  if (nSmpl > 0 && (now - lastSampleMs >= SAMPLE_MS)) {
    lastSampleMs = now;
    if (!isnan(curT) && !isnan(curH)) {
      pushSample(curT, curH);
      drawGraph();
    }
  }

  // Append one record to SPIFFS log every hour.
  if (now - lastLogMs >= LOG_INTERVAL_MS) {
    lastLogMs = now;
    appendToLog();
  }

  // Once a week check whether the log needs trimming.
  if (now - lastTrimMs >= TRIM_INTERVAL_MS) {
    lastTrimMs = now;
    trimLog();
  }
}
