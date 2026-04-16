/*
 * TempHumidityDisplay.ino
 *
 * Reads temperature (°F) and humidity (%) from a DHT11 on GPIO 22.
 * Displays live readings on the top ¼ of the CYD 2.8" TFT (320×240),
 * and a rolling 24-hour graph on the bottom ¾.
 *
 * Hardware
 *   ESP32 CYD (Cheap Yellow Display) — single-USB version ESP32-WROOM board
 *    LovyanGFX driver at 40MHz SPI for 2.8" CYD board (Cheep Yellow Display)
 *   DHT11 sensor → GPIO 22
 *
 * Libraries  (install via Arduino Library Manager)
 *   LovyanGFX             – display driver (configured in ESP32_SPI_9341.h)
 *   DHT sensor library    – Adafruit DHT sensor library 1.4.7
 *   Adafruit Unified Sensor
 */

#include "ESP32_SPI_9341.h"   // LovyanGFX + CYD pin/panel config
#include <DHT.h>

// ─── Version ──────────────────────────────────────────────────────────────────
#define VERSION "TempHumidityDisplay v1.0  15-Apr-2026"

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
// One sample every 5 min → 288 samples = 24 hours of history
static const unsigned long SAMPLE_MS = 5UL * 60UL * 1000UL;
static const int MAX_SMPL = 288;

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
int   nSmpl = 0;

float curT = NAN, curH = NAN;

unsigned long lastReadMs   = 0;
unsigned long lastSampleMs = 0;

// ─── Data management ──────────────────────────────────────────────────────────

// Append a sample; when full, shift the oldest out (O(n) every 5 min – fine).
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
  // Total span in minutes represented by the stored samples.
  long spanMin = (long)nSmpl * (SAMPLE_MS / 60000UL);

  tft.setTextColor(TFT_DARKGREY, C_BG);
  tft.setTextDatum(lgfx::top_center);
  tft.setFont(&fonts::Font0);
  for (int t = 0; t <= 4; t++) {
    int  xPix  = PLT_X + (int)((long)t * PLT_W / 4);
    long agoMin = spanMin - (long)t * spanMin / 4;
    String lbl;
    if (agoMin == 0) {
      lbl = "Now";
    } else if (agoMin < 60) {
      lbl = "-" + String((int)agoMin) + "m";
    } else {
      lbl = "-" + String((int)(agoMin / 60)) + "h";
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

  drawHeader();
  drawGraph();
}

// ─── Loop ─────────────────────────────────────────────────────────────────────

void loop() {
  unsigned long now = millis();

  // Read sensor every 3 s (DHT11 max ~0.5 Hz; allow extra margin).
  if (now - lastReadMs >= 3000UL) {
    lastReadMs = now;

    float h = dht.readHumidity();
    float t = dht.readTemperature(true);    // true → Fahrenheit

    Serial.printf("Temp %f, Humidity %f\n", h, t);
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

  // Archive a sample every 5 minutes.
  if (nSmpl > 0 && (now - lastSampleMs >= SAMPLE_MS)) {
    lastSampleMs = now;
    if (!isnan(curT) && !isnan(curH)) {
      pushSample(curT, curH);
      drawGraph();
    }
  }
}
