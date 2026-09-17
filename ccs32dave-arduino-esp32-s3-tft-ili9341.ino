// ESP32-S3 + ILI9341 SPI TFT (240x320) + QCA7005 HomePlug modem
// Shows the modem status, the network join status, the modems in the network
// with their software versions, a log of the received frames, and the uptime
// in seconds (10 ms resolution, refreshed every 50 ms).
//
// Libraries: Adafruit ILI9341, Adafruit GFX Library (+ Adafruit BusIO)
// Board:     ESP32S3 Dev Module (esp32 core 3.x)
// Wiring:    see readme.md

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <esp_timer.h>

#include "qca7000.h"
#include "homeplug.h"
#include "modem_list.h"
#include "diag.h"
#include "v2gtp.h"
#include "v2g_exi.h"

// ---- TFT pins (ESP32-S3 default FSPI pins) ----------------------------------
#define TFT_MOSI 11
#define TFT_SCLK 12
#define TFT_MISO -1   // not connected (display is write-only)
#define TFT_CS   -1   // module has no CS pin (controller is always selected)
#define TFT_DC    9
#define TFT_RST   8
#define TFT_BL   -1   // backlight GPIO, or -1 if BLK is tied to 5V/3V3

#define SPI_FREQUENCY   40000000UL
#define UPDATE_INTERVAL_MS 50       // uptime refresh period (<= 250 ms)
#define LOG_DRAW_INTERVAL_MS 200    // frame log refresh period (at most)

// ---- QCA7005 pins (second SPI bus; the TFT has no CS and can't share) -------
#define QCA_SCLK  4
#define QCA_MOSI  5   // QCA SPI_SI
#define QCA_MISO  6   // QCA SPI_SO
#define QCA_CS   15

#define QCA_SPI_FREQUENCY       4000000UL
#define QCA_POLL_INTERVAL_MS    10     // fetch received frames
#define QCA_CHECK_INTERVAL_MS   1000   // signature check
#define QCA_REQUEST_INTERVAL_MS 5000   // GET_SW.REQ broadcast (modem list)
#define QCA_MAX_AGE_MS          (3 * QCA_REQUEST_INTERVAL_MS + 500)
// Join status of the local modem: VS_NW_INFO.REQ, sent only to the local modem (not onto the
// powerline), every second. The sniffer's desired state is re-sent in the same cycle, so a lost
// VS_SNIFFER.REQ or a modem reset heals within a second.
#define QCA_STATUS_INTERVAL_MS  1000
#define QCA_STATUS_MAX_AGE_MS   3500   // join status counts as unknown after this

// Sniffer rule (backlog_0006, revised 2026-09-17): on while the local modem is not joined, off
// while joined. `sniff on` (diag) keeps it on regardless - a test aid to produce the .IND load of a
// running session.
// "beacons only" stays on until this long after the last received beacon. Owner requirement
// 2026-09-15: at most 200 ms from beacon indication to TFT, both on and off. The AR7420 beacons
// every 40 ms (largest gap seen 71 ms), so 150 ms bridges one lost beacon and leaves ~50 ms for
// the loop.
#define SNIFFER_BEACON_HOLD_MS 150
// Beacon meter (owner request 2026-09-15): beacons received in the last ~200 ms, one segment each.
// A CCo beacons every 40 ms, so 5 = every beacon arrived. Exactly 200 ms sits on the 4/5 edge and
// flipped ~20x/s from our ~25 ms arrival jitter (bench); the extra 30 ms keeps an unbroken beacon
// train at a steady 5, so a 4 means one really went missing.
#define SNIFFER_METER_WINDOW_MS 230
#define SNIFFER_METER_SEGMENTS 5
#define PANEL_REFRESH_INTERVAL_MS 500  // re-evaluate time-based panel state (the value fade)

// ---- Page button (backlog_0008) -----------------------------------------------
// Push button to GND, internal pull-up; a press toggles main page / page 2. GPIO 16 is free on
// every ESP32-S3 module (it is only the 32 kHz crystal pin if such a crystal is fitted - not on
// the DevKitC-1) and is not a strapping, flash, PSRAM or USB pin.
#define PAGE_BUTTON_PIN 16
#define PAGE_BUTTON_DEBOUNCE_MS 30
// Page 2 shows one session's setup, so it does not age per value: it stays coloured while V2G
// messages (or SDP) keep arriving and grays as a whole once the session has been quiet this long
// (owner decision 2026-09-17). A new session clears it on the SLAC match, so nothing from the
// previous one can turn coloured again.
#define PAGE2_QUIET_MS 3000

#define SERIAL_BAUD 921600  // high rate, so logging every frame doesn't slow down the loop

// Locally administered MAC used as source of our requests
static const uint8_t MY_MAC[6] = {0xFE, 0xED, 0xBE, 0xEF, 0xAF, 0xFE};

Adafruit_ILI9341 tft(TFT_CS, TFT_DC, TFT_RST);

SPIClass qcaSpi(HSPI);
Qca7000 qca(qcaSpi, QCA_CS, QCA_SPI_FREQUENCY);
Diag diag(qca, Serial, MY_MAC);

struct ModemStatus {
  bool checked = false;  // signature read at least once
  uint16_t signature = 0;
  bool present = false;
  bool requestPending = false;  // send requests at the next loop run
};
ModemStatus modem;

ModemList modems;

struct NetworkStatus {
  bool valid = false;  // CM_NW_INFO.CNF received from the local modem recently
  uint32_t receivedMs = 0;
  homeplug::NetworkInfo info;
};
NetworkStatus network;

struct TrafficStats {
  uint32_t mme = 0;    // HomePlug frames other than the answers to our requests
  uint32_t ipv6 = 0;
  uint32_t other = 0;
};
TrafficStats traffic;

// backlog-0056: VS_SNIFFER auto-toggle state + the idle timer that drives it.
struct SnifferStatus {
  bool enabled = false;
  uint32_t indCount = 0;      // all VS_SNIFFER.IND (every delimiter, own transmissions included)
  uint32_t beaconCount = 0;   // only received beacons (homeplug::isReceivedBeaconInd)
  uint32_t lastBeaconMs = 0;  // 0 = none yet
  bool beaconActive = false;  // a received beacon within SNIFFER_BEACON_HOLD_MS
  uint32_t beaconTimes[8] = {};  // ring of the latest beacon arrival times, for the meter
  uint8_t beaconTimesNext = 0;
  uint8_t beaconLevel = 0;       // beacons in the last SNIFFER_METER_WINDOW_MS, capped
  uint8_t levelMin = 255, levelMax = 0, levelChanges = 0;  // for the SNIF summary line
};
SnifferStatus sniffer;

bool panelDirty = true;

// V2G values decoded from the reassembled DIN EXI traffic (backlog-0050/0051; struct and
// applyV2gValues() are defined further down, next to drawPanel() where they're used - kept
// out of this section because a struct with default member initializers here confused
// Arduino's automatic function-prototype generator into mis-placing the TextLine-using
// prototypes below ahead of TextLine's own definition).
V2gtpReassembler v2gtp;

// ---- Layout (landscape 320x240) ---------------------------------------------
static const uint16_t COLOR_DARKGREY = 0x7BEF;
static const int16_t TEXT_X = 10;
static const int16_t HEADER_LINE_Y = 21;

// Uptime box right of the headline, text size 1
static const int16_t UPTIME_W = 12 * 6;  // "12345678.90s"
static const int16_t UPTIME_H = 8;
static const int16_t UPTIME_X = 320 - 4 - 4 - UPTIME_W;
static const int16_t UPTIME_Y = 7;
GFXcanvas16 uptimeCanvas(UPTIME_W, UPTIME_H);

// Below the modem panel, the screen splits into two columns: the frame log (left) and a
// prominent V2G value panel (right) - owner request 2026-09-15, the log doesn't need the
// screen's full width once it no longer carries the numeric V2G detail itself.
static const int16_t COLUMNS_TOP_Y = 76;     // divider below the modem panel
static const int16_t COLUMNS_BOTTOM_Y = 223;  // divider above the counter line
static const int16_t PANEL_DIVIDER_X = 207;   // vertical divider between log and V2G panel
static const int16_t PANEL_X = 213;

// Frame log (left column), drawn off-screen and sent as one bitmap
static const uint8_t LOG_LINES = 14;
static const int16_t LOG_PITCH = 10;
static const int16_t LOG_X = TEXT_X;
static const int16_t LOG_Y = 80;
static const int16_t LOG_W = PANEL_DIVIDER_X - LOG_X - 4;
static const int16_t LOG_H = LOG_LINES * LOG_PITCH;
GFXcanvas16 logCanvas(LOG_W, LOG_H);

// ---- Text lines that are redrawn only when their content changes ------------
struct TextLine {
  int16_t x;
  int16_t y;
  uint8_t size;
  uint8_t cols;  // 0 = up to the right screen edge
  bool drawn;
  uint16_t color;
  char text[65];
};

TextLine statusLine  = {TEXT_X, 27, 2, 9};
TextLine networkLine = {130, 27, 2};
TextLine modemLines[ModemList::MAX_MODEMS] = {
    {TEXT_X, 47, 1}, {TEXT_X, 57, 1}, {TEXT_X, 67, 1}};

// V2G value panel (right column) - "big stacked numbers" per owner request 2026-09-15.
// cols=16 for label/message rows (16*6=96px), cols=7 for the big value rows (7*12=84px);
// both comfortably inside the panel's ~100px width, PANEL_X to the screen edge.
TextLine targetLabelLine  = {PANEL_X, 80, 1, 16};   // "TARGET"
TextLine targetVoltLine   = {PANEL_X, 89, 2, 7};    // "230.0V"
TextLine targetCurrLine   = {PANEL_X, 106, 2, 7};   // "10.0A"
TextLine presentLabelLine = {PANEL_X, 125, 1, 16};  // "PRESENT"
TextLine presentVoltLine  = {PANEL_X, 134, 2, 7};
TextLine presentCurrLine  = {PANEL_X, 151, 2, 7};
TextLine socLabelLine     = {PANEL_X, 170, 1, 16};  // "SoC"
TextLine socValueLine     = {PANEL_X, 179, 2, 7};   // "42%"
TextLine msgLine          = {PANEL_X, 199, 1, 16};  // last decoded message name
TextLine rcLine           = {PANEL_X, 209, 1, 16};  // response code, only shown if not OK

TextLine counterLine = {TEXT_X, 229, 1};

// Delimiter metadata, not logged per frame (tens per second once enabled). Only RECEIVED beacons
// count as "a CCo is in sight": the stream also reports the local modem's own transmissions, and
// counting those kept the beacon display on after the AR7420 was switched off (owner report
// 2026-09-15).
void handleSnifferInd(const uint8_t *frame, uint16_t len) {
  sniffer.indCount++;
  if (homeplug::isReceivedBeaconInd(frame, len)) {
    if (sniffer.beaconCount == 0) {
      Serial.println("Sniffer: first received beacon");
    }
    sniffer.beaconCount++;
    sniffer.lastBeaconMs = millis();
    sniffer.beaconTimes[sniffer.beaconTimesNext] = sniffer.lastBeaconMs;
    sniffer.beaconTimesNext = (sniffer.beaconTimesNext + 1) % 8;
  }
}

// Beacon meter right of the "beacons" label on the network line (text size 2 -> 16 px high).
static const int16_t METER_X = 222;
static const int16_t METER_Y = 28;
static const int16_t METER_SEG_W = 14;
static const int16_t METER_SEG_H = 14;
static const int16_t METER_GAP = 3;
static const uint16_t COLOR_METER_EMPTY = 0x39E7;
int8_t meterShown = -1;  // level currently on screen, -1 = meter not drawn

void drawBeaconMeter(uint8_t level) {
  if (meterShown < 0) {
    // First draw after full-width text: clear what the short label's padding didn't cover
    tft.fillRect(130 + 7 * 12, 27, tft.width() - (130 + 7 * 12), 16, ILI9341_BLACK);
  }
  for (uint8_t i = 0; i < SNIFFER_METER_SEGMENTS; i++) {
    bool on = i < level;
    if (meterShown >= 0 && on == (i < meterShown)) {
      continue;  // only segments that changed
    }
    tft.fillRect(METER_X + i * (METER_SEG_W + METER_GAP), METER_Y, METER_SEG_W, METER_SEG_H,
                 on ? ILI9341_ORANGE : COLOR_METER_EMPTY);
  }
  meterShown = level;
}

uint8_t lineColumns(const TextLine &line) {
  return line.cols ? line.cols : (tft.width() - line.x) / (6 * line.size);
}

void drawLine(TextLine &line, uint16_t color, const char *text) {
  if (line.drawn && line.color == color && strcmp(line.text, text) == 0) {
    return;
  }
  // Pad to the full line width, so the background color erases old text
  char buf[65];
  int cols = lineColumns(line);
  snprintf(buf, sizeof(buf), "%-*.*s", cols, cols, text);

  tft.setTextSize(line.size);
  tft.setTextColor(color, ILI9341_BLACK);
  tft.setCursor(line.x, line.y);
  tft.print(buf);

  strlcpy(line.text, text, sizeof(line.text));
  line.color = color;
  line.drawn = true;
}

void formatMac(char *buf, size_t len, const uint8_t *m) {
  snprintf(buf, len, "%02X:%02X:%02X:%02X:%02X:%02X",
           m[0], m[1], m[2], m[3], m[4], m[5]);
}

uint64_t uptimeCentiseconds() {
  // esp_timer is 64-bit microseconds -> no 49-day millis() wrap-around
  return esp_timer_get_time() / 10000ULL;
}

// ---- Frame log ----------------------------------------------------------------

struct LogEntry {
  uint64_t centiseconds;  // uptime of the last occurrence
  char text[24];
  uint8_t mac[3];         // last 3 bytes of the source MAC
  bool hasMac;
  uint16_t count;         // consecutive identical entries are combined
  uint16_t color;
};

LogEntry logEntries[LOG_LINES];
uint8_t logCount = 0;
uint8_t logNewest = LOG_LINES - 1;  // ring buffer index
bool logDirty = true;

void addLog(const char *text, uint16_t color, const uint8_t *srcMac = nullptr) {
  LogEntry *last = logCount ? &logEntries[logNewest] : nullptr;
  if (last && strcmp(last->text, text) == 0 && last->hasMac == (srcMac != nullptr) &&
      (!srcMac || memcmp(last->mac, &srcMac[3], 3) == 0)) {
    last->count++;
    last->centiseconds = uptimeCentiseconds();
    logDirty = true;
    return;
  }

  logNewest = (logNewest + 1) % LOG_LINES;
  if (logCount < LOG_LINES) {
    logCount++;
  }
  LogEntry &e = logEntries[logNewest];
  e.centiseconds = uptimeCentiseconds();
  strlcpy(e.text, text, sizeof(e.text));
  e.hasMac = srcMac != nullptr;
  if (srcMac) {
    memcpy(e.mac, &srcMac[3], 3);
  }
  e.count = 1;
  e.color = color;
  logDirty = true;
}

void drawLog() {
  logCanvas.fillScreen(ILI9341_BLACK);
  logCanvas.setTextWrap(false);
  logCanvas.setTextSize(1);

  // Oldest entry at the top, newest at the bottom
  uint8_t oldest = (logNewest + LOG_LINES + 1 - logCount) % LOG_LINES;
  for (uint8_t i = 0; i < logCount; i++) {
    const LogEntry &e = logEntries[(oldest + i) % LOG_LINES];
    // Narrower column (backlog-0055: the V2G panel took the space) - time drops to whole
    // seconds and the MAC suffix to its last 2 bytes, so the message name field (the part
    // worth keeping full-width) stays the same 20 characters it always was.
    char mac[6] = "";
    if (e.hasMac) {
      snprintf(mac, sizeof(mac), "%02X:%02X", e.mac[1], e.mac[2]);
    }
    char count[8] = "";
    if (e.count > 1) {
      snprintf(count, sizeof(count), " x%u", e.count);
    }
    char buf[64];
    snprintf(buf, sizeof(buf), "%5lu %-20.20s %-5s%s",
             (unsigned long)(e.centiseconds / 100), e.text, mac, count);
    logCanvas.setTextColor(e.color);
    logCanvas.setCursor(0, i * LOG_PITCH);
    logCanvas.print(buf);
  }

  tft.drawRGBBitmap(LOG_X, LOG_Y, logCanvas.getBuffer(), LOG_W, LOG_H);
}

// ---- Splash screen ------------------------------------------------------------
// Shown once at boot, for SPLASH_DURATION_MS, while the QCA comes up: project name, GitHub
// link, build date/time, and - if it answers in time - the local modem's own software
// version. Purely cosmetic (blocks setup(), nothing else needs to run during it).
static const uint32_t SPLASH_DURATION_MS = 5000;
static const char *const SPLASH_TITLE = "ccs32dave";
static const char *const SPLASH_SUBTITLE = "CCS Sniffer for ISO 15118 / DIN 70121";
static const char *const SPLASH_URL = "github.com/uhi22/ccs32dave";

static int16_t splashCenterX(const char *text, uint8_t size) {
  return (tft.width() - (int16_t)strlen(text) * 6 * size) / 2;
}

static void splashPrintCentered(int16_t y, uint8_t size, uint16_t color, const char *text) {
  tft.setTextSize(size);
  tft.setTextColor(color, ILI9341_BLACK);
  tft.setCursor(splashCenterX(text, size), y);
  tft.print(text);
}

// A small lightning bolt, toggled fully on/off to pulse (a bit of "charging" flavor).
static void splashDrawBolt(int16_t x, int16_t y, bool on) {
  uint16_t c = on ? ILI9341_YELLOW : ILI9341_BLACK;
  tft.drawLine(x + 8, y, x, y + 14, c);
  tft.drawLine(x + 1, y, x + 9, y, c);
  tft.drawLine(x, y + 14, x + 7, y + 14, c);
  tft.drawLine(x + 6, y + 15, x + 8, y + 15, c);
  tft.drawLine(x + 7, y + 14, x, y + 30, c);
}

static char splashLocalVersion[homeplug::VERSION_MAX_LEN + 1] = "";

// Used only during the splash: parses a GET_SW.CNF from the local modem directly, bypassing
// the normal onHomeplugFrame()/addLog() pipeline (the frame-log canvas isn't meant to be
// touched yet). Also seeds the modem table, so the main screen shows it immediately.
static void splashHandleFrame(const uint8_t *frame, uint16_t len) {
  if (homeplug::etherType(frame) != homeplug::ETHERTYPE_HOMEPLUG) {
    return;
  }
  if (homeplug::mmtype(frame) == homeplug::MMTYPE_VS_SNIFFER_IND) {
    handleSnifferInd(frame, len);  // beacons already count while the splash is shown
    return;
  }
  if (splashLocalVersion[0]) {
    return;
  }
  homeplug::SoftwareVersion sw;
  if (homeplug::parseGetSwCnf(frame, len, sw) && ModemList::isLocal(sw.mac)) {
    strlcpy(splashLocalVersion, sw.version, sizeof(splashLocalVersion));
    modems.update(sw, millis());
    Serial.printf("Local modem (splash): %s\n", sw.version);
  }
}

void showSplashScreen() {
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextWrap(false);
  tft.drawRoundRect(6, 6, tft.width() - 12, tft.height() - 12, 8, COLOR_DARKGREY);

  // Typewriter reveal of the title
  char partial[16] = "";
  size_t titleLen = strlen(SPLASH_TITLE);
  for (size_t i = 0; i <= titleLen; i++) {
    memcpy(partial, SPLASH_TITLE, i);
    partial[i] = '\0';
    tft.fillRect(0, 55, tft.width(), 27, ILI9341_BLACK);
    splashPrintCentered(55, 3, ILI9341_YELLOW, partial);
    delay(60);
  }

  splashPrintCentered(95, 1, ILI9341_GREEN, SPLASH_SUBTITLE);
  splashPrintCentered(108, 1, ILI9341_CYAN, SPLASH_URL);

  char buildInfo[40];
  snprintf(buildInfo, sizeof(buildInfo), "Built %s %s", __DATE__, __TIME__);
  splashPrintCentered(122, 1, COLOR_DARKGREY, buildInfo);

  static const int16_t MODEM_LINE_Y = 150;
  splashPrintCentered(MODEM_LINE_Y, 1, ILI9341_WHITE, "Local modem: detecting...");

  static const int16_t BAR_X = 40, BAR_Y = 190, BAR_W = 240, BAR_H = 12;
  tft.drawRect(BAR_X, BAR_Y, BAR_W, BAR_H, COLOR_DARKGREY);

  qca.begin(QCA_SCLK, QCA_MISO, QCA_MOSI);  // start the modem link now, so it can answer in time

  uint32_t t0 = millis();
  bool requested = false;
  bool shown = false;
  uint8_t frame[homeplug::MIN_ETH_FRAME_LEN];
  uint32_t lastPulse = 0;
  bool boltOn = false;
  int16_t lastFill = -1;

  while (millis() - t0 < SPLASH_DURATION_MS) {
    uint32_t elapsed = millis() - t0;

    if (!requested && qca.readSignature() == Qca7000::SIGNATURE) {
      qca.sendEthFrame(frame, homeplug::composeGetSwReq(frame, MY_MAC));
      // Sniffer on right away (backlog-0056): otherwise beacon detection only starts with the
      // first sendRequests() after the splash. No traffic has been seen yet, so "idle" holds.
      qca.sendEthFrame(frame, homeplug::composeVsSnifferReq(frame, MY_MAC, true));
      sniffer.enabled = true;
      requested = true;
    }
    if (requested) {
      qca.poll(splashHandleFrame);  // keep reading during the whole splash, the .INDs keep coming
    }
    if (requested && !shown) {
      if (splashLocalVersion[0]) {
        char line[40];
        const char *v = splashLocalVersion;
        if (strncmp(v, "MAC-", 4) == 0) {
          v += 4;  // saves space; all QCA versions start with it
        }
        snprintf(line, sizeof(line), "Local modem: %s", v);
        tft.fillRect(0, MODEM_LINE_Y, tft.width(), 8, ILI9341_BLACK);
        splashPrintCentered(MODEM_LINE_Y, 1, ILI9341_WHITE, line);
        shown = true;
      }
    }

    if (millis() - lastPulse >= 300) {
      lastPulse = millis();
      boltOn = !boltOn;
      splashDrawBolt(20, 50, boltOn);
      splashDrawBolt(tft.width() - 36, 50, boltOn);
    }

    int16_t fill = (int16_t)((uint32_t)(BAR_W - 2) * elapsed / SPLASH_DURATION_MS);
    if (fill != lastFill) {
      tft.fillRect(BAR_X + 1, BAR_Y + 1, fill, BAR_H - 2, ILI9341_GREEN);
      lastFill = fill;
    }

    delay(20);
  }
  if (requested && !shown) {
    qca.poll(splashHandleFrame);  // one last chance, in case the answer is just arriving
  }
}

// ---- Screen -----------------------------------------------------------------

uint8_t page = 1;  // 1 = main page, 2 = session setup (backlog_0008)

// Static parts of the main page, below the header
void drawMainStatic() {
  tft.drawFastHLine(0, COLUMNS_TOP_Y, tft.width(), COLOR_DARKGREY);
  tft.drawFastHLine(0, COLUMNS_BOTTOM_Y, tft.width(), COLOR_DARKGREY);
  tft.drawFastVLine(PANEL_DIVIDER_X, COLUMNS_TOP_Y, COLUMNS_BOTTOM_Y - COLUMNS_TOP_Y, COLOR_DARKGREY);
}

void drawStaticScreen() {
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextWrap(false);

  tft.setCursor(6, 3);
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_YELLOW);
  tft.print("ccs32dave");

  tft.drawRect(UPTIME_X - 4, UPTIME_Y - 4, UPTIME_W + 8, UPTIME_H + 8, COLOR_DARKGREY);

  tft.drawFastHLine(0, HEADER_LINE_Y, tft.width(), COLOR_DARKGREY);
  drawMainStatic();
}

// V2G values decoded from the reassembled DIN EXI traffic (backlog-0050/0051). Persist across
// messages - e.g. CurrentDemandRes doesn't repeat the target values, so the display keeps
// showing the last one seen until a newer message updates it (or the modem disappears).
struct V2gDisplay {
  // One "has" flag and one timestamp per value: during pre-charge only the voltages are sent,
  // so the currents must age out on their own.
  bool hasTargetVoltage = false, hasTargetCurrent = false;
  float targetVoltage = 0, targetCurrent = 0;      // PreChargeReq (V only) / CurrentDemandReq
  uint32_t targetVoltageMs = 0, targetCurrentMs = 0;
  bool hasPresentVoltage = false, hasPresentCurrent = false;
  float presentVoltage = 0, presentCurrent = 0;    // PreChargeRes (V only) / CurrentDemandRes
  uint32_t presentVoltageMs = 0, presentCurrentMs = 0;
  bool hasSoc = false;
  int8_t soc = 0;                                  // DC_EVStatus.EVRESSSOC, from the PEV
  uint32_t socUpdatedMs = 0;
  bool hasResponseCode = false;
  uint8_t responseCode = 0;
  char lastMsg[28] = "";
  bool lastOk = false;
  uint32_t decoded = 0, failed = 0;                // counters for the status line
};
V2gDisplay v2gDisplay;

// Session setup shown on page 2 (backlog_0009/0010). Kept until overwritten by the next session;
// page 2 grays as a whole after PAGE2_QUIET_MS without V2G traffic, and is cleared on a SLAC match.
struct SessionSetup {
  bool hasSdpReq = false;
  uint8_t reqSecurity = 0, reqTransport = 0;
  bool hasSdpRes = false;
  uint8_t resSecurity = 0, resTransport = 0;
  uint8_t evseIp[16] = {};
  uint16_t evsePort = 0;
  uint8_t appProtocolCount = 0;
  V2gValues::AppProtocol appProtocols[V2gValues::MAX_APP_PROTOCOLS];
  bool hasHandshakeResult = false;
  uint8_t handshakeResponseCode = 0;
  bool hasSelectedSchema = false;
  uint8_t selectedSchemaId = 0;
  uint32_t lastActivityMs = 0;  // last SDP or V2G message of this session
};
SessionSetup sessionSetup;

// SDP values (DIN 70121 / ISO 15118-2)
static const uint8_t SDP_SECURITY_TLS = 0x00;
static const uint8_t SDP_SECURITY_NONE = 0x10;
static const uint8_t SDP_TRANSPORT_TCP = 0x00;
static const uint8_t SDP_TRANSPORT_UDP = 0x10;

const char *sdpSecurityName(uint8_t s) {
  return s == SDP_SECURITY_TLS ? "TLS" : s == SDP_SECURITY_NONE ? "no TLS" : "sec?";
}

const char *sdpTransportName(uint8_t t) {
  return t == SDP_TRANSPORT_TCP ? "TCP" : t == SDP_TRANSPORT_UDP ? "UDP" : "?";
}

const char *handshakeResultName(uint8_t rc) {
  static const char *const NAMES[] = {"OK", "OK, minor deviation", "failed, no negotiation"};
  return rc < 3 ? NAMES[rc] : "?";
}

void storeHandshake(const V2gValues &v) {
  uint32_t t = millis();
  if (v.appProtocolCount > 0) {
    sessionSetup.appProtocolCount = v.appProtocolCount;
    memcpy(sessionSetup.appProtocols, v.appProtocols, sizeof(sessionSetup.appProtocols));
    Serial.printf("AppHandshake req: %u protocol(s)\n", v.appProtocolCount);
    for (uint8_t i = 0; i < v.appProtocolCount; i++) {
      const V2gValues::AppProtocol &a = v.appProtocols[i];
      Serial.printf("  ID %u prio %u v%u.%u %s\n", a.schemaId, a.priority, a.versionMajor,
                    a.versionMinor, a.ns);
    }
  }
  if (v.hasHandshakeResult) {
    sessionSetup.hasHandshakeResult = true;
    sessionSetup.handshakeResponseCode = v.handshakeResponseCode;
    sessionSetup.hasSelectedSchema = v.hasSelectedSchema;
    sessionSetup.selectedSchemaId = v.selectedSchemaId;
    if (v.hasSelectedSchema) {
      Serial.printf("AppHandshake res: %s, schema ID %u\n",
                    handshakeResultName(v.handshakeResponseCode), v.selectedSchemaId);
    } else {
      Serial.printf("AppHandshake res: %s, no schema\n", handshakeResultName(v.handshakeResponseCode));
    }
  }
}

// Applies one decode result (ok=false: v.msgName carries a short error tag instead) to the
// persistent display state above. Prints to serial only when the shown VALUES actually
// change (not on every message) - same convention as printModemTable() / the "Network:"
// line elsewhere in this file, so the TFT's content can be checked without seeing the screen.
//
// Each VALUE has its own "last updated" timestamp, refreshed whenever a message carries it at all
// (even if the value repeated). Not per group: PreChargeReq carries only the target voltage and
// PreChargeRes only the present voltage, so during pre-charge the currents are stale while the
// voltages keep updating - with a shared timestamp the currents kept showing the previous
// session's 10 A in full color (owner report 2026-09-17).
void applyV2gValues(const V2gValues &v, bool ok) {
  sessionSetup.lastActivityMs = millis();  // the session is alive - keeps page 2 coloured; also on
                                           // a failed decode, which is still V2G traffic
  if (ok) {
    storeHandshake(v);
    v2gDisplay.decoded++;
    bool changed = false;
    uint32_t now = millis();
    if (v.hasTargetVoltage) {
      if (v.targetVoltage != v2gDisplay.targetVoltage) { v2gDisplay.targetVoltage = v.targetVoltage; changed = true; }
      v2gDisplay.hasTargetVoltage = true;
      v2gDisplay.targetVoltageMs = now;
    }
    if (v.hasTargetCurrent) {
      if (v.targetCurrent != v2gDisplay.targetCurrent) { v2gDisplay.targetCurrent = v.targetCurrent; changed = true; }
      v2gDisplay.hasTargetCurrent = true;
      v2gDisplay.targetCurrentMs = now;
    }
    if (v.hasPresentVoltage) {
      if (v.presentVoltage != v2gDisplay.presentVoltage) { v2gDisplay.presentVoltage = v.presentVoltage; changed = true; }
      v2gDisplay.hasPresentVoltage = true;
      v2gDisplay.presentVoltageMs = now;
    }
    if (v.hasPresentCurrent) {
      if (v.presentCurrent != v2gDisplay.presentCurrent) { v2gDisplay.presentCurrent = v.presentCurrent; changed = true; }
      v2gDisplay.hasPresentCurrent = true;
      v2gDisplay.presentCurrentMs = now;
    }
    if (v.hasSoc) {
      if (v.soc != v2gDisplay.soc) { v2gDisplay.soc = v.soc; changed = true; }
      v2gDisplay.hasSoc = true;
      v2gDisplay.socUpdatedMs = now;
    }
    if (v.hasResponseCode) { v2gDisplay.hasResponseCode = true; v2gDisplay.responseCode = v.responseCode; }
    if (changed) {
      Serial.printf("V2G: %s  Tgt %.1fV %.1fA  Pres %.1fV %.1fA  SoC %d%%\n", v.msgName,
                    v2gDisplay.targetVoltage, v2gDisplay.targetCurrent, v2gDisplay.presentVoltage,
                    v2gDisplay.presentCurrent, v2gDisplay.soc);
    }
  } else {
    v2gDisplay.failed++;
  }
  strlcpy(v2gDisplay.lastMsg, v.msgName, sizeof(v2gDisplay.lastMsg));
  v2gDisplay.lastOk = ok;
  panelDirty = true;
}

// backlog-0055: fades a V2G value's color as it goes stale - mid-gray after 2 s without a fresh
// message for its field group, dark gray after 4 s. Only kicks in once a value has been seen at
// all; the "-" placeholder shown before that keeps its normal (fresh) color.
uint16_t staleColor(uint16_t freshColor, bool has, uint32_t updatedMs) {
  if (!has) {
    return freshColor;
  }
  uint32_t age = millis() - updatedMs;
  static const uint32_t STALE_MID_MS = 2000;
  static const uint32_t STALE_DARK_MS = 4000;
  static const uint16_t COLOR_STALE_DARK = 0x39E7;  // darker than COLOR_DARKGREY (0x7BEF)
  if (age >= STALE_DARK_MS) return COLOR_STALE_DARK;
  if (age >= STALE_MID_MS) return COLOR_DARKGREY;
  return freshColor;
}

void drawPanel() {
  char buf[65];

  drawLine(statusLine, modem.present ? ILI9341_GREEN : ILI9341_RED,
           modem.present ? "Modem OK" : "No modem");

  // Network join status from CM_NW_INFO.CNF of the local modem
  static const char *const ROLE[] = {"STA", "PCo", "CCo"};
  // backlog-0056: "beacons" + meter shares the network line. The short label leaves room for the
  // meter; every other text uses the full width again, and its padding erases the meter.
  bool showMeter = modem.present && network.valid && network.info.numNetworks == 0 &&
                   sniffer.beaconActive;
  uint8_t networkCols = showMeter ? 7 : 0;
  if (networkLine.cols != networkCols) {
    networkLine.cols = networkCols;
    networkLine.drawn = false;
    meterShown = -1;
  }
  if (!modem.present || !network.valid) {
    drawLine(networkLine, COLOR_DARKGREY, "Net ?");
  } else if (network.info.numNetworks == 0) {
    // "beacons" = a CCo is RF-reachable (received beacons) but never talks to us. beaconActive and
    // beaconLevel are updated on every loop() pass (updateBeaconActive()), not on the panel tick.
    if (showMeter) {
      drawLine(networkLine, ILI9341_ORANGE, "beacons");
      drawBeaconMeter(sniffer.beaconLevel);
    } else {
      drawLine(networkLine, ILI9341_YELLOW, "not joined");
    }
  } else {
    // No TEI here: the ping-pong firmware flips the modem's own TEI per frame, so it says nothing
    // and would only make the line flicker.
    snprintf(buf, sizeof(buf), "joined %s", network.info.role < 3 ? ROLE[network.info.role] : "?");
    drawLine(networkLine, ILI9341_GREEN, buf);
  }

  // One line per modem: "*MAC version", * = local modem
  for (uint8_t i = 0; i < ModemList::MAX_MODEMS; i++) {
    if (i >= modems.count()) {
      drawLine(modemLines[i], ILI9341_CYAN, "");
      continue;
    }
    const ModemList::Entry &e = modems[i];
    char mac[18];
    formatMac(mac, sizeof(mac), e.mac);
    const char *version = e.version;
    if (strncmp(version, "MAC-", 4) == 0) {
      version += 4;  // saves space; all QCA versions start with it
    }
    bool local = ModemList::isLocal(e.mac);
    snprintf(buf, sizeof(buf), "%c%s %s", local ? '*' : ' ', mac, version);
    drawLine(modemLines[i], local ? ILI9341_CYAN : ILI9341_WHITE, buf);
  }

  // V2G value panel (backlog-0051; layout backlog-0055): big stacked numbers, right column.
  drawLine(targetLabelLine, COLOR_DARKGREY, "TARGET");
  if (v2gDisplay.hasTargetVoltage) {
    snprintf(buf, sizeof(buf), "%.1fV", v2gDisplay.targetVoltage);
  } else {
    strlcpy(buf, "-", sizeof(buf));
  }
  drawLine(targetVoltLine, staleColor(ILI9341_YELLOW, v2gDisplay.hasTargetVoltage, v2gDisplay.targetVoltageMs), buf);
  if (v2gDisplay.hasTargetCurrent) {
    snprintf(buf, sizeof(buf), "%.1fA", v2gDisplay.targetCurrent);
  } else {
    strlcpy(buf, "-", sizeof(buf));
  }
  drawLine(targetCurrLine, staleColor(ILI9341_YELLOW, v2gDisplay.hasTargetCurrent, v2gDisplay.targetCurrentMs), buf);

  drawLine(presentLabelLine, COLOR_DARKGREY, "PRESENT");
  if (v2gDisplay.hasPresentVoltage) {
    snprintf(buf, sizeof(buf), "%.1fV", v2gDisplay.presentVoltage);
  } else {
    strlcpy(buf, "-", sizeof(buf));
  }
  drawLine(presentVoltLine, staleColor(ILI9341_GREEN, v2gDisplay.hasPresentVoltage, v2gDisplay.presentVoltageMs), buf);
  if (v2gDisplay.hasPresentCurrent) {
    snprintf(buf, sizeof(buf), "%.1fA", v2gDisplay.presentCurrent);
  } else {
    strlcpy(buf, "-", sizeof(buf));
  }
  drawLine(presentCurrLine, staleColor(ILI9341_GREEN, v2gDisplay.hasPresentCurrent, v2gDisplay.presentCurrentMs), buf);

  drawLine(socLabelLine, COLOR_DARKGREY, "SoC");
  if (v2gDisplay.hasSoc) {
    snprintf(buf, sizeof(buf), "%d%%", v2gDisplay.soc);
  } else {
    strlcpy(buf, "-", sizeof(buf));
  }
  drawLine(socValueLine, staleColor(ILI9341_CYAN, v2gDisplay.hasSoc, v2gDisplay.socUpdatedMs), buf);

  drawLine(msgLine, v2gDisplay.lastOk ? ILI9341_WHITE : ILI9341_RED, v2gDisplay.lastMsg);

  // Response code: blank until known, "OK" once a 0, "RC <n>" (red) for any fault code -
  // see dinresponseCodeType in src/exi/dinEXIDatatypes.h for the full list.
  if (!v2gDisplay.hasResponseCode) {
    drawLine(rcLine, COLOR_DARKGREY, "");
  } else if (v2gDisplay.responseCode == 0) {
    drawLine(rcLine, ILI9341_GREEN, "OK");
  } else {
    snprintf(buf, sizeof(buf), "RC %u", v2gDisplay.responseCode);
    drawLine(rcLine, ILI9341_RED, buf);
  }

  snprintf(buf, sizeof(buf), "MME %lu IPv6 %lu  SPI TX %lu RX %lu err %lu %04X",
           (unsigned long)traffic.mme, (unsigned long)traffic.ipv6,
           (unsigned long)qca.txFrames(), (unsigned long)qca.rxFrames(),
           (unsigned long)qca.errors(), modem.signature);
  drawLine(counterLine, COLOR_DARKGREY, buf);
}

// ---- Page 2: session setup (backlog_0008..0010) -------------------------------

TextLine headerHintLine = {126, 3, 2, 4};  // "TLS" in the header, both pages

TextLine p2SdpHead = {TEXT_X, 26, 1};
TextLine p2SdpCar = {TEXT_X, 37, 2};
TextLine p2SdpEvse = {TEXT_X, 55, 2};
TextLine p2SdpAddr = {TEXT_X, 73, 1};
static const int16_t PAGE2_DIVIDER_Y = 84;  // between the SDP block above and the schemas below
TextLine p2AphHead = {TEXT_X, 89, 1};
TextLine p2AphLines[V2gValues::MAX_APP_PROTOCOLS] = {
    {TEXT_X, 100, 1}, {TEXT_X, 110, 1}, {TEXT_X, 120, 1}, {TEXT_X, 130, 1}, {TEXT_X, 140, 1}};
TextLine p2AphResult = {TEXT_X, 154, 1};
TextLine p2AphSelected = {TEXT_X, 166, 2};
TextLine p2Footer = {TEXT_X, 229, 1};

// TLS negotiated -> the DIN messages can't be decoded, so say it on both pages. "TLS?" if only
// the car's request is known (the charger's response may not reach us - backlog_0009).
void drawHeaderHint() {
  if (sessionSetup.hasSdpRes) {
    drawLine(headerHintLine, ILI9341_RED,
             sessionSetup.resSecurity == SDP_SECURITY_TLS ? "TLS" : "");
  } else if (sessionSetup.hasSdpReq && sessionSetup.reqSecurity == SDP_SECURITY_TLS) {
    drawLine(headerHintLine, ILI9341_ORANGE, "TLS?");
  } else {
    drawLine(headerHintLine, ILI9341_RED, "");
  }
}

// Coloured while the session is alive, gray once it has been quiet - one decision for the whole
// page, so its values never disagree about which session they belong to.
bool page2Quiet() {
  return sessionSetup.lastActivityMs == 0 ||
         millis() - sessionSetup.lastActivityMs >= PAGE2_QUIET_MS;
}

uint16_t page2Color(uint16_t freshColor) {
  return page2Quiet() ? COLOR_DARKGREY : freshColor;
}

const char *protocolShortName(const char *ns) {
  if (strstr(ns, "din:70121")) return "DIN 70121";
  if (strstr(ns, "15118:2:2010")) return "ISO 15118-2:2010";
  if (strstr(ns, "15118:2:2013")) return "ISO 15118-2:2013";
  if (strstr(ns, "15118:-20")) return "ISO 15118-20";
  return ns;
}

void drawPage2() {
  const SessionSetup &s = sessionSetup;
  char buf[65];

  drawLine(p2SdpHead, COLOR_DARKGREY, "SDP - transport security");
  if (s.hasSdpReq) {
    snprintf(buf, sizeof(buf), "Car:  %s %s", sdpSecurityName(s.reqSecurity),
             sdpTransportName(s.reqTransport));
    drawLine(p2SdpCar, page2Color(s.reqSecurity == SDP_SECURITY_NONE ? ILI9341_GREEN : ILI9341_RED),
             buf);
  } else {
    drawLine(p2SdpCar, COLOR_DARKGREY, "Car:  -");
  }
  if (s.hasSdpRes) {
    snprintf(buf, sizeof(buf), "EVSE: %s %s", sdpSecurityName(s.resSecurity),
             sdpTransportName(s.resTransport));
    drawLine(p2SdpEvse, page2Color(s.resSecurity == SDP_SECURITY_NONE ? ILI9341_GREEN : ILI9341_RED),
             buf);
    const uint8_t *a = s.evseIp;
    snprintf(buf, sizeof(buf), "EVSE %x:%x:%x:%x:%x:%x:%x:%x port %u",
             (a[0] << 8) | a[1], (a[2] << 8) | a[3], (a[4] << 8) | a[5], (a[6] << 8) | a[7],
             (a[8] << 8) | a[9], (a[10] << 8) | a[11], (a[12] << 8) | a[13], (a[14] << 8) | a[15],
             s.evsePort);
    drawLine(p2SdpAddr, page2Color(ILI9341_WHITE), buf);
  } else {
    drawLine(p2SdpEvse, COLOR_DARKGREY, "EVSE: -");
    drawLine(p2SdpAddr, COLOR_DARKGREY, "");
  }

  drawLine(p2AphHead, COLOR_DARKGREY, "Schemas offered by the car (> = selected)");
  const V2gValues::AppProtocol *selected = nullptr;
  for (uint8_t i = 0; i < V2gValues::MAX_APP_PROTOCOLS; i++) {
    if (i >= s.appProtocolCount) {
      drawLine(p2AphLines[i], COLOR_DARKGREY, i == 0 ? "-" : "");
      continue;
    }
    const V2gValues::AppProtocol &a = s.appProtocols[i];
    bool isSelected = s.hasHandshakeResult && s.hasSelectedSchema &&
                      s.selectedSchemaId == a.schemaId;
    if (isSelected) selected = &a;
    snprintf(buf, sizeof(buf), "%cID%u P%u v%u.%u %s", isSelected ? '>' : ' ', a.schemaId,
             a.priority, a.versionMajor, a.versionMinor, a.ns);
    drawLine(p2AphLines[i], page2Color(isSelected ? ILI9341_GREEN : ILI9341_WHITE), buf);
  }

  if (s.hasHandshakeResult) {
    bool okResult = s.handshakeResponseCode < 2;
    if (s.hasSelectedSchema) {
      snprintf(buf, sizeof(buf), "Charger: %s, schema ID %u",
               handshakeResultName(s.handshakeResponseCode), s.selectedSchemaId);
    } else {
      snprintf(buf, sizeof(buf), "Charger: %s", handshakeResultName(s.handshakeResponseCode));
    }
    uint16_t color = page2Color(okResult ? ILI9341_GREEN : ILI9341_RED);
    drawLine(p2AphResult, color, buf);
    if (selected) {
      snprintf(buf, sizeof(buf), "-> %s", protocolShortName(selected->ns));
    } else if (s.hasSelectedSchema) {
      snprintf(buf, sizeof(buf), "-> ID %u", s.selectedSchemaId);
    } else {
      strlcpy(buf, "-> none", sizeof(buf));
    }
    drawLine(p2AphSelected, color, buf);
  } else {
    drawLine(p2AphResult, COLOR_DARKGREY, "Charger: -");
    drawLine(p2AphSelected, COLOR_DARKGREY, "");
  }

  drawLine(p2Footer, COLOR_DARKGREY, "page 2 - press the button for the main page");
}

// Everything below the header is redrawn from scratch after a page switch.
void showPage(uint8_t p) {
  static TextLine *const LINES[] = {
      &statusLine, &networkLine, &modemLines[0], &modemLines[1], &modemLines[2],
      &targetLabelLine, &targetVoltLine, &targetCurrLine, &presentLabelLine, &presentVoltLine,
      &presentCurrLine, &socLabelLine, &socValueLine, &msgLine, &rcLine, &counterLine,
      &p2SdpHead, &p2SdpCar, &p2SdpEvse, &p2SdpAddr, &p2AphHead, &p2AphLines[0], &p2AphLines[1],
      &p2AphLines[2], &p2AphLines[3], &p2AphLines[4], &p2AphResult, &p2AphSelected, &p2Footer};
  page = p;
  tft.fillRect(0, HEADER_LINE_Y + 1, tft.width(), tft.height() - HEADER_LINE_Y - 1, ILI9341_BLACK);
  for (TextLine *line : LINES) {
    line->drawn = false;
  }
  meterShown = -1;
  if (page == 1) {
    drawMainStatic();
    logDirty = true;
  } else {
    // divider between the SDP block and the schema block
    tft.drawFastHLine(0, PAGE2_DIVIDER_Y, tft.width(), COLOR_DARKGREY);
  }
  panelDirty = true;
  Serial.printf("Page: %u\n", page);
}

// Called on every loop() pass. Button to GND with pull-up: pressed = LOW.
void pollPageButton(uint32_t now) {
  static bool lastRaw = HIGH;
  static bool stable = HIGH;
  static uint32_t lastChangeMs = 0;
  bool raw = digitalRead(PAGE_BUTTON_PIN);
  if (raw != lastRaw) {
    lastRaw = raw;
    lastChangeMs = now;
  } else if (raw != stable && now - lastChangeMs >= PAGE_BUTTON_DEBOUNCE_MS) {
    stable = raw;
    if (stable == LOW) {
      showPage(page == 1 ? 2 : 1);
    }
  }
}

void drawUptime(uint64_t centiseconds) {
  char buf[16];
  // Right-aligned, fixed width -> digits don't jump around
  snprintf(buf, sizeof(buf), "%8llu.%02us",
           (unsigned long long)(centiseconds / 100),
           (unsigned)(centiseconds % 100));

  uptimeCanvas.fillScreen(ILI9341_BLACK);
  uptimeCanvas.setTextWrap(false);
  uptimeCanvas.setTextSize(1);
  uptimeCanvas.setTextColor(ILI9341_WHITE);
  uptimeCanvas.setCursor(0, 0);
  uptimeCanvas.print(buf);

  tft.drawRGBBitmap(UPTIME_X, UPTIME_Y, uptimeCanvas.getBuffer(),
                    UPTIME_W, UPTIME_H);
}

// ---- Modem handling ---------------------------------------------------------

void printModemTable() {
  Serial.printf("Modems: %u\n", modems.count());
  for (uint8_t i = 0; i < modems.count(); i++) {
    char mac[18];
    formatMac(mac, sizeof(mac), modems[i].mac);
    Serial.printf("  %c %s %s\n", ModemList::isLocal(modems[i].mac) ? '*' : ' ',
                  mac, modems[i].version);
  }
}

// One machine-readable line per received frame (serial command "log 0|1"):
//   F <ms> <src MAC> <dst MAC> <len> <description>
void logFrame(const uint8_t *frame, uint16_t len, const char *description) {
  if (!diag.logTraffic()) {
    return;
  }
  const uint8_t *d = &frame[0];
  const uint8_t *s = &frame[6];
  Serial.printf("F %lu %02x%02x%02x%02x%02x%02x %02x%02x%02x%02x%02x%02x %u %s\n",
                (unsigned long)millis(), s[0], s[1], s[2], s[3], s[4], s[5],
                d[0], d[1], d[2], d[3], d[4], d[5], len, description);
}

void onHomeplugFrame(const uint8_t *frame, uint16_t len) {
  uint16_t mm = homeplug::mmtype(frame);
  if ((mm & 0xFFFC) == 0xA034) {
    // The whole VS_SNIFFER family (REQ 0xA034, CNF 0xA035 = ack for our own toggle, IND 0xA036)
    // is ours: not logged, not counted as traffic.
    if (mm == homeplug::MMTYPE_VS_SNIFFER_IND) {
      handleSnifferInd(frame, len);
    }
    return;
  }

  homeplug::SoftwareVersion sw;
  if (homeplug::parseGetSwCnf(frame, len, sw)) {
    if (modems.update(sw, millis())) {
      printModemTable();
      panelDirty = true;
    }
    return;
  }

  homeplug::VsNetworkInfo vs;
  if (homeplug::parseVsNwInfoCnf(frame, len, vs)) {
    // Answer to our 1 s VS_NW_INFO.REQ (sent to the local modem only)
    if (ModemList::isLocal(vs.mac)) {
      homeplug::NetworkInfo info;
      memset(&info, 0, sizeof(info));
      memcpy(info.mac, vs.mac, 6);
      info.numNetworks = vs.numAvlns;
      info.tei = vs.ownTei;
      info.role = vs.role;  // same coding as CM_NW_INFO: 0 STA, 1 PCo, 2 CCo
      memcpy(info.ccoMac, vs.ccoMac, 6);
      // The own TEI is deliberately NOT compared: the ping-pong firmware rewrites it per frame
      // (bench 2026-09-17: it alternated every second), which would log and redraw constantly.
      bool changed = !network.valid ||
                     network.info.numNetworks != info.numNetworks ||
                     network.info.role != info.role;
      if (changed) {
        Serial.printf("Network: %s, role %u\n", info.numNetworks > 0 ? "joined" : "not joined",
                      info.role);
        char buf[24];
        strlcpy(buf, info.numNetworks > 0 ? "** joined" : "** not joined", sizeof(buf));
        addLog(buf, ILI9341_YELLOW);
        panelDirty = true;
      }
      network.info = info;
      network.valid = true;
      network.receivedMs = millis();
      if (changed) {
        updateSnifferState();  // switch the sniffer right away, not a cycle later
      }
    }
    return;
  }

  // A SLAC match starts a new session: drop the previous session's setup, so nothing of it can
  // turn coloured again if this session's SDP is missed (owner decision 2026-09-17 - clearing on
  // the handshake request would be too late, that comes after SDP).
  if ((mm & ~0x0003) == 0x607C) {
    sessionSetup = SessionSetup();
    panelDirty = true;
  }

  // Other MMEs, e.g. SLAC between car and charger
  traffic.mme++;
  char name[24];
  homeplug::describeMmtype(homeplug::mmtype(frame), name, sizeof(name));
  addLog(name, ILI9341_CYAN, &frame[6]);
  panelDirty = true;
  char description[32];
  snprintf(description, sizeof(description), "MME %s", name);
  logFrame(frame, len, description);
}

// TCP payload segments are handed to the V2GTP reassembler (v2gtp.h, backlog-0050); a
// completed V2GTP message is decoded (v2g_exi.h, backlog-0051) and applied to the display.
// Only UDP frames and decoded/failed V2G messages get a TFT log line - individual TCP
// segments (mostly bare ACKs, or the single data segment a small DIN message fits in) would
// otherwise flood the 12-line ring buffer with little to show for it. Every counted IPv6
// frame still gets its raw description on the serial port via logFrame(), unconditionally -
// that's the ground truth used by the capture-ratio measurements (backlog-0048).
void onIpv6Frame(const uint8_t *frame, uint16_t len) {
  traffic.ipv6++;
  panelDirty = true;

  // Ethernet header 14 bytes, IPv6 header 40 bytes, then TCP/UDP ports
  const char *proto = "IPv6";
  uint16_t srcPort = 0;
  uint16_t dstPort = 0;
  if (len >= 14 + 40 + 4) {
    uint8_t nextHeader = frame[14 + 6];
    proto = nextHeader == 6 ? "TCP" : nextHeader == 17 ? "UDP" : "IPv6";
    srcPort = (frame[54] << 8) | frame[55];
    dstPort = (frame[56] << 8) | frame[57];
  }
  char text[24];
  snprintf(text, sizeof(text), "%s %u>%u", proto, srcPort, dstPort);

  // For TCP also flags, sequence number, payload length and the V2GTP length
  // (if the payload starts with a V2GTP header 01 FE), e.g.
  //   TCP 15118>49152 fl=18 seq=1a2b3c4d pl=43 v2g=35
  char description[120];
  strlcpy(description, text, sizeof(description));
  uint16_t ipPayloadLen = len >= 14 + 40 ? (frame[18] << 8) | frame[19] : 0;

  if (strcmp(proto, "TCP") == 0 && len >= 54 + 20 && 54u + ipPayloadLen <= len) {
    uint32_t seq = ((uint32_t)frame[58] << 24) | (frame[59] << 16) | (frame[60] << 8) | frame[61];
    uint8_t headerLen = (frame[66] >> 4) * 4;
    uint8_t flags = frame[67];
    int payloadLen = (int)ipPayloadLen - headerLen;
    int v2gLen = -1;
    const uint8_t *payload = &frame[54 + headerLen];
    if (payloadLen >= 8 && payload[0] == 0x01 && payload[1] == 0xFE) {
      v2gLen = (int)(((uint32_t)payload[4] << 24) | (payload[5] << 16) |
                     (payload[6] << 8) | payload[7]);
    }
    snprintf(description, sizeof(description), "%s fl=%02x seq=%08lx pl=%d v2g=%d", text,
             flags, (unsigned long)seq, payloadLen, v2gLen);

    if (payloadLen > 0) {
      const uint8_t *msg;
      uint16_t msgLen;
      if (v2gtp.feed(&frame[6], seq, payload, (uint16_t)payloadLen, &msg, &msgLen)) {
        V2gValues v;
        bool ok = decodeV2gExiPayload(msg + 8, (uint16_t)(msgLen - 8), v);  // skip V2GTP header
        applyV2gValues(v, ok);
        char logtext[30];
        snprintf(logtext, sizeof(logtext), "%s%s", ok ? "" : "! ", v.msgName);
        addLog(logtext, ok ? ILI9341_GREEN : ILI9341_RED, &frame[6]);
        char extra[36];
        snprintf(extra, sizeof(extra), " -> %s", v.msgName);
        strlcat(description, extra, sizeof(description));
      }
    }
  } else if (strcmp(proto, "UDP") == 0) {
    char sdpText[24];
    if (parseSdp(frame, len, ipPayloadLen, sdpText, sizeof(sdpText))) {
      addLog(sdpText, ILI9341_GREEN, &frame[6]);
      strlcat(description, " -> ", sizeof(description));
      strlcat(description, sdpText, sizeof(description));
    } else {
      addLog(text, ILI9341_GREEN, &frame[6]);
    }
  }
  logFrame(frame, len, description);
}

// SDP (backlog_0009): V2GTP over UDP - header 01 FE, payload type (2), length (4), then
// request 0x9000 = security, transport; response 0x9001 = IPv6 address (16), port (2),
// security, transport. Returns true and a short log text, e.g. "SDP res no TLS TCP".
bool parseSdp(const uint8_t *frame, uint16_t len, uint16_t ipPayloadLen, char *text, size_t textLen) {
  static const uint16_t OFS_V2GTP = 14 + 40 + 8;  // Ethernet, IPv6, UDP headers
  if (ipPayloadLen < 8 + 8 || 54u + ipPayloadLen > len) {
    return false;
  }
  const uint8_t *p = &frame[OFS_V2GTP];
  if (p[0] != 0x01 || p[1] != 0xFE) {
    return false;
  }
  uint16_t type = (p[2] << 8) | p[3];
  uint32_t payloadLen = ((uint32_t)p[4] << 24) | ((uint32_t)p[5] << 16) | (p[6] << 8) | p[7];
  if (8u + 8u + payloadLen > ipPayloadLen) {
    return false;
  }
  const uint8_t *d = &p[8];
  SessionSetup &s = sessionSetup;
  if (type == 0x9000 && payloadLen == 2) {
    s.hasSdpReq = true;
    s.reqSecurity = d[0];
    s.reqTransport = d[1];
    s.lastActivityMs = millis();
    snprintf(text, textLen, "SDP req %s %s", sdpSecurityName(d[0]), sdpTransportName(d[1]));
  } else if (type == 0x9001 && payloadLen == 20) {
    s.hasSdpRes = true;
    memcpy(s.evseIp, d, 16);
    s.evsePort = (d[16] << 8) | d[17];
    s.resSecurity = d[18];
    s.resTransport = d[19];
    s.lastActivityMs = millis();
    snprintf(text, textLen, "SDP res %s %s", sdpSecurityName(d[18]), sdpTransportName(d[19]));
  } else {
    return false;
  }
  Serial.println(text);
  return true;
}

void onEthFrame(const uint8_t *frame, uint16_t len) {
  if (len < 14) {
    return;
  }
  if (diag.onFrame(frame, len)) {
    return;  // answer to a serial diagnosis command
  }
  switch (homeplug::etherType(frame)) {
    case homeplug::ETHERTYPE_HOMEPLUG:
      onHomeplugFrame(frame, len);
      break;
    case homeplug::ETHERTYPE_IPV6:
      onIpv6Frame(frame, len);
      break;
    default:
      traffic.other++;
      logFrame(frame, len, "OTHER");
      break;
  }
}

void checkModem() {
  uint16_t signature = qca.readSignature();
  bool present = signature == Qca7000::SIGNATURE;
  if (!present) {
    qca.dumpRegisters(Serial);
  }
  if (!modem.checked || signature != modem.signature || present != modem.present) {
    modem.checked = true;
    Serial.printf("QCA7005 signature: %04X (%s)\n", signature,
                  present ? "OK" : "missing");
    if (present) {
      modem.requestPending = true;  // ask immediately, not only after the interval
      addLog("** modem OK", ILI9341_YELLOW);
    } else {
      modems.clear();
      network.valid = false;
      addLog("** modem missing", ILI9341_RED);
    }
    modem.signature = signature;
    modem.present = present;
    panelDirty = true;
  }
}

// Called on every loop() pass, right after the SPI poll and before the panel is drawn, so a
// change of "beacons only" reaches the TFT in the same pass (owner requirement: <= 200 ms).
void updateBeaconActive() {
  uint32_t t = millis();  // not loop()'s `now`: lastBeaconMs may have been set after it
  bool active = sniffer.enabled && sniffer.beaconCount != 0 &&
                t - sniffer.lastBeaconMs < SNIFFER_BEACON_HOLD_MS;
  if (active != sniffer.beaconActive) {
    sniffer.beaconActive = active;
    panelDirty = true;
    Serial.printf("Beacon: %s at %lu ms (last beacon %lu ms)\n", active ? "on" : "off",
                  (unsigned long)t, (unsigned long)sniffer.lastBeaconMs);
  }

  uint8_t level = 0;
  if (active) {
    uint8_t stored = sniffer.beaconCount < 8 ? sniffer.beaconCount : 8;
    for (uint8_t i = 0; i < stored; i++) {
      if (t - sniffer.beaconTimes[i] < SNIFFER_METER_WINDOW_MS) level++;
    }
    if (level > SNIFFER_METER_SEGMENTS) level = SNIFFER_METER_SEGMENTS;
    sniffer.levelMin = min(sniffer.levelMin, level);
    sniffer.levelMax = max(sniffer.levelMax, level);
  }
  if (level != sniffer.beaconLevel) {
    sniffer.beaconLevel = level;
    if (sniffer.levelChanges < 255) sniffer.levelChanges++;
    if (meterShown >= 0 && active) {
      drawBeaconMeter(level);  // right away: a few filled rectangles, cheap
    }
  }
}

// Every QCA_REQUEST_INTERVAL_MS: GET_SW.REQ broadcast for the modem list.
void sendRequests(uint32_t now) {
  uint8_t frame[homeplug::MIN_ETH_FRAME_LEN];
  qca.sendEthFrame(frame, homeplug::composeGetSwReq(frame, MY_MAC));

  // Forget modems that stopped answering
  if (modems.expire(now, QCA_MAX_AGE_MS)) {
    printModemTable();
  }
  panelDirty = true;  // counters changed
}

// Sends the sniffer's desired state: on while not joined (or join status unknown), off while
// joined, always on after `sniff on`. Sent every status cycle, not only on a change -
// VS_SNIFFER.REQ has no retry of its own, and a modem reset switches the sniffer off.
void updateSnifferState() {
  bool joined = network.valid && network.info.numNetworks > 0;
  bool wanted = diag.snifferForced() || !joined;
  uint8_t frame[homeplug::MIN_ETH_FRAME_LEN];
  qca.sendEthFrame(frame, homeplug::composeVsSnifferReq(frame, MY_MAC, wanted));
  if (wanted != sniffer.enabled) {
    sniffer.enabled = wanted;
    Serial.printf("Sniffer: %s (%s)\n", wanted ? "enabled" : "disabled",
                  diag.snifferForced() ? "forced" : joined ? "joined" : "not joined");
    addLog(wanted ? "** sniffer on" : "** sniffer off", ILI9341_YELLOW);
    panelDirty = true;
  }
}

// Every QCA_STATUS_INTERVAL_MS: join status of the local modem, then the sniffer state.
void sendStatusRequests(uint32_t now) {
  static const uint8_t LOCAL_MODEM_MAC[6] = {0x04, 0x65, 0x65, 0xFF, 0xFF, 0x11};
  uint8_t frame[homeplug::MIN_ETH_FRAME_LEN];
  qca.sendEthFrame(frame, homeplug::composeVsNwInfoReq(frame, LOCAL_MODEM_MAC, MY_MAC));

  if (network.valid && now - network.receivedMs > QCA_STATUS_MAX_AGE_MS) {
    network.valid = false;
    Serial.println("Network: unknown (no VS_NW_INFO answer)");
    panelDirty = true;
  }
  updateSnifferState();
}

// ---- Arduino entry points ---------------------------------------------------

void setup() {
  Serial.setRxBufferSize(4096);  // "wr" command lines
  Serial.setTxBufferSize(8192);  // bursts of frame log lines
  Serial.begin(SERIAL_BAUD);

  pinMode(PAGE_BUTTON_PIN, INPUT_PULLUP);

  if (TFT_BL >= 0) {
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, HIGH);
  }

  SPI.begin(TFT_SCLK, TFT_MISO, TFT_MOSI, TFT_CS);
  tft.begin(SPI_FREQUENCY);
  tft.setRotation(1);   // landscape, 320x240
  showSplashScreen();   // also starts the QCA link (qca.begin()) - see there
  drawStaticScreen();

  checkModem();
  drawPanel();

  Serial.println("ccs32dave started");
}

void loop() {
  static uint32_t lastUptimeUpdate = 0;
  static uint32_t lastLogDraw = 0;
  static uint32_t lastPoll = 0;
  static uint32_t lastCheck = millis();
  static uint32_t lastRequest = 0;
  static uint32_t lastStatus = 0;
  static uint32_t lastPanelRefresh = 0;
  static uint64_t lastShown = UINT64_MAX;

  uint32_t now = millis();

  // Time-based panel state (the backlog-0055 stale-value fade, the backlog-0056 beacon-
  // freshness window) needs re-evaluating even when no new frame arrives to set panelDirty.
  if (now - lastPanelRefresh >= PANEL_REFRESH_INTERVAL_MS) {
    lastPanelRefresh = now;
    panelDirty = true;
  }

  // Once per second, only while .INDs arrive and "log 1": how many were received beacons vs.
  // everything else - so the beacon display can be checked against the serial log.
  // loopmax = longest loop() pass in that second (the part of the beacon->TFT delay we control).
  // dec/fail = V2G messages decoded / failed, err = SPI errors, rx = SPI frames, per second.
  static uint32_t lastSnifSummary = 0, lastInd = 0, lastBeacons = 0, loopMax = 0, prevLoopStart = now;
  static uint32_t lastDec = 0, lastFail = 0, lastErr = 0, lastRx = 0;
  loopMax = max(loopMax, now - prevLoopStart);
  prevLoopStart = now;
  if (now - lastSnifSummary >= 1000) {
    lastSnifSummary = now;
    uint32_t ind = sniffer.indCount - lastInd, beacons = sniffer.beaconCount - lastBeacons;
    uint32_t dec = v2gDisplay.decoded - lastDec, fail = v2gDisplay.failed - lastFail;
    if ((ind || dec || fail) && diag.logTraffic()) {
      Serial.printf("SNIF %lu ind=%lu beacons=%lu other=%lu dec=%lu fail=%lu rx=%lu err=%lu "
                    "loopmax=%lums meter=%u..%u changes=%u\n",
                    (unsigned long)now, (unsigned long)ind, (unsigned long)beacons,
                    (unsigned long)(ind - beacons), (unsigned long)dec, (unsigned long)fail,
                    (unsigned long)(qca.rxFrames() - lastRx), (unsigned long)(qca.errors() - lastErr),
                    (unsigned long)loopMax, sniffer.levelMin == 255 ? 0 : sniffer.levelMin,
                    sniffer.levelMax, sniffer.levelChanges);
    }
    lastInd = sniffer.indCount;
    lastBeacons = sniffer.beaconCount;
    lastDec = v2gDisplay.decoded;
    lastFail = v2gDisplay.failed;
    lastErr = qca.errors();
    lastRx = qca.rxFrames();
    loopMax = 0;
    sniffer.levelMin = 255;
    sniffer.levelMax = 0;
    sniffer.levelChanges = 0;
  }

  if (now - lastCheck >= QCA_CHECK_INTERVAL_MS) {
    lastCheck = now;
    checkModem();
  }

  if (modem.present) {
    bool pending = modem.requestPending;  // modem (re)appeared: ask right away
    modem.requestPending = false;
    if (pending || (diag.periodicRequests() && now - lastRequest >= QCA_REQUEST_INTERVAL_MS)) {
      lastRequest = now;
      sendRequests(now);
    }
    if (pending || (diag.periodicRequests() && now - lastStatus >= QCA_STATUS_INTERVAL_MS)) {
      lastStatus = now;
      sendStatusRequests(now);
    }
    if (now - lastPoll >= QCA_POLL_INTERVAL_MS) {
      lastPoll = now;
      qca.poll(onEthFrame);
    }
    diag.loop(now);
  }

  updateBeaconActive();
  pollPageButton(now);

  if (panelDirty) {
    panelDirty = false;
    drawHeaderHint();
    if (page == 1) {
      drawPanel();
    } else {
      drawPage2();
    }
  }

  if (page == 1 && logDirty && now - lastLogDraw >= LOG_DRAW_INTERVAL_MS) {
    logDirty = false;
    lastLogDraw = now;
    drawLog();
  }

  if (now - lastUptimeUpdate >= UPDATE_INTERVAL_MS) {
    lastUptimeUpdate = now;
    uint64_t centiseconds = uptimeCentiseconds();
    if (centiseconds != lastShown) {
      drawUptime(centiseconds);
      lastShown = centiseconds;
    }
  }
}
