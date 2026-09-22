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
// Pages 2 and 3 show one session's setup, so they do not age per value: they stay coloured while
// V2G messages (or SDP) keep arriving and gray as a whole once the session has been quiet this
// long (owner decision 2026-09-17). A new session clears them on the SLAC match, so nothing from
// the previous one can turn coloured again.
#define SESSION_QUIET_MS 3000

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

// 1 main page, 2 session setup (backlog_0008), 3 charge parameters (backlog_0013), then one
// page per topic: 4 timeline, 5 SLAC quality, 6 V/I chart, 7 identity, 8 energy, 9 ladder.
static const uint8_t PAGE_COUNT = 9;
uint8_t page = 1;
uint64_t uptimeShown = UINT64_MAX;  // last value drawn into the header box, UINT64_MAX = redraw

// Static parts of the main page, below the header
void drawMainStatic() {
  tft.drawFastHLine(0, COLUMNS_TOP_Y, tft.width(), COLOR_DARKGREY);
  tft.drawFastHLine(0, COLUMNS_BOTTOM_Y, tft.width(), COLOR_DARKGREY);
  tft.drawFastVLine(PANEL_DIVIDER_X, COLUMNS_TOP_Y, COLUMNS_BOTTOM_Y - COLUMNS_TOP_Y, COLOR_DARKGREY);
}

// The header carries the project name and the uptime box on pages 1 and 2. Page 3 takes it over
// for its own heading instead (owner request 2026-09-22) - there the page name is worth more than
// the project name, and the two freed rows let the values themselves grow.
void drawMainHeader() {
  tft.fillRect(0, 0, tft.width(), HEADER_LINE_Y, ILI9341_BLACK);
  tft.setTextWrap(false);

  tft.setCursor(6, 3);
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_YELLOW);
  tft.print("ccs32dave");

  tft.drawRect(UPTIME_X - 4, UPTIME_Y - 4, UPTIME_W + 8, UPTIME_H + 8, COLOR_DARKGREY);
  tft.drawFastHLine(0, HEADER_LINE_Y, tft.width(), COLOR_DARKGREY);
}

// The longest title, "ChargeParameterDiscovery", is 24 characters = 288 px at size 2, so a title
// starts at x=2 and leaves the last few pixels for the TLS hint (p3HintLine, size 1).
// Page 2 is one page in two stacked parts, and its two headings ("SDP" and "schema") are drawn
// alike: same colour, same size, same left edge. So its top heading is NOT the usual page title -
// it is the first part's heading, sitting where the title would be, and the line under the header
// is left out. The only rule the page draws is the separator between its two parts (owner
// request 2026-09-22).
static const uint16_t PAGE2_HEAD_COLOR = ILI9341_YELLOW;

void drawPage2Header() {
  tft.fillRect(0, 0, tft.width(), HEADER_LINE_Y + 1, ILI9341_BLACK);  // +1 erases the header line
  tft.setTextWrap(false);
  tft.setCursor(TEXT_X, 3);
  tft.setTextSize(2);
  tft.setTextColor(PAGE2_HEAD_COLOR);
  tft.print("SDP");
}

void drawTitleHeader(const char *title) {
  tft.fillRect(0, 0, tft.width(), HEADER_LINE_Y, ILI9341_BLACK);
  tft.setTextWrap(false);

  tft.setCursor(2, 3);
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_YELLOW);
  tft.print(title);

  tft.drawFastHLine(0, HEADER_LINE_Y, tft.width(), COLOR_DARKGREY);
}

const char *pageTitle(uint8_t p) {
  switch (p) {
    case 3: return "ChargeParameterDiscovery";
    case 4: return "Session timeline";
    case 5: return "SLAC quality";
    case 6: return "Voltage & current";
    case 7: return "Identity & keys";
    case 8: return "Energy";
    case 9: return "Message ladder";
    default: return "";
  }
}

void drawStaticScreen() {
  tft.fillScreen(ILI9341_BLACK);
  drawMainHeader();
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

// Session setup shown on pages 2 and 3 (backlog_0009/0010/0013). Kept until overwritten by the
// next session; both pages gray as a whole after SESSION_QUIET_MS without V2G traffic, and are
// cleared on a SLAC match.
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

  // Identity (backlog_0017, page 7). The SessionID is taken from any message that carries a
  // non-zero one - SessionSetupReq still sends all zeroes, the charger assigns it in its response.
  bool hasSessionId = false;
  uint8_t sessionId[8] = {};
  uint8_t sessionIdLen = 0;
  bool hasEvccid = false;
  uint8_t evccid[6] = {};
  uint8_t evccidLen = 0;

  // ChargeParameterDiscovery (backlog_0013, page 3) - the mirror of the V2gValues fields, kept
  // per session. The request's block and the response's block arrive separately and the response
  // is repeated while the charger is still "Ongoing", so each side has its own "has" flag.
  bool hasCpdReq = false;
  uint8_t energyTransferMode = 0;
  bool hasEvLimits = false;
  float evMaxVoltage = 0, evMaxCurrent = 0;
  bool hasFullSoc = false, hasBulkSoc = false;
  int8_t fullSoc = 0, bulkSoc = 0;
  bool evReady = false;
  uint8_t evErrorCode = 0;
  int8_t cpdSoc = 0;
  bool hasCpdRes = false;
  uint8_t evseProcessing = 0;
  uint8_t cpdResponseCode = 0;
  bool hasEvseLimits = false;
  float evseMaxVoltage = 0, evseMaxCurrent = 0;
  float evseMinVoltage = 0, evseMinCurrent = 0;
  bool hasEvseMaxPower = false;
  float evseMaxPower = 0;

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

// dinEVRequestedEnergyTransferType (dinEXIDatatypes.h) - what the car asks for in
// ChargeParameterDiscoveryReq. DIN 70121 in practice only uses DC_extended.
const char *energyTransferName(uint8_t t) {
  static const char *const NAMES[] = {"AC 1ph core", "AC 3ph core", "DC core",
                                      "DC extended", "DC combo core", "DC unique"};
  return t < 6 ? NAMES[t] : "?";
}

// dinDC_EVErrorCodeType, shortened to the 12 characters page 3's column holds at text size 2.
// In full (dinEXIDatatypes.h): FAILED_RESSTemperatureInhibit, FAILED_EVShiftPosition,
// FAILED_ChargerConnectorLockFault, FAILED_EVRESSMalfunction, FAILED_ChargingCurrentdifferential,
// FAILED_ChargingVoltageOutOfRange, FAILED_ChargingSystemIncompatibility, NoData.
const char *evErrorName(uint8_t c) {
  static const char *const NAMES[] = {"NO_ERROR",   "RESS temp",  "shift pos",  "conn lock",
                                      "RESS malfn", "curr diff",  "volt range", "reserved A",
                                      "reserved B", "reserved C", "incompat",   "no data"};
  return c < 12 ? NAMES[c] : "?";
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
// ---- Session data behind pages 4..9 (backlog_0014, 0015, 0016, 0020, 0021) --
//
// Everything here belongs to ONE charging session and is reset together in beginSession(), from
// the same SLAC-match branch that already clears SessionSetup. Two things happen before that
// point and would be wiped by it, so they wait outside: the start of SLAC (slacStartMs) and the
// attenuation profile (CM_ATTEN_CHAR.IND arrives about a second before the match). Both are
// copied into the fresh session by beginSession().

enum Phase : uint8_t {
  PH_SLAC, PH_JOIN, PH_SDP, PH_HANDSHAKE, PH_SETUP, PH_CABLECHECK, PH_PRECHARGE,
  PH_CURRENTDEMAND, PH_STOP, PH_COUNT
};

static const char *const PHASE_NAMES[PH_COUNT] = {
    "SLAC", "join", "SDP", "handshake", "setup..CPD", "CableCheck", "PreCharge",
    "CurrentDem", "stop"};

struct SessionData {
  uint32_t phaseStart[PH_COUNT] = {};  // 0 = phase not reached
  uint32_t phaseEnd[PH_COUNT] = {};
  uint8_t lastPhase = PH_COUNT;        // PH_COUNT = nothing decoded yet

  bool hasAtten = false;               // backlog_0015
  homeplug::AttenProfile atten;

  bool hasMatch = false;               // backlog_0017
  homeplug::SlacMatch match;

  double energyWs = 0, chargeAs = 0;   // backlog_0020, integrated over the present values
  float peakW = 0;
  uint32_t energyFirstMs = 0, energyLastMs = 0, energySampleMs = 0;

  uint32_t startMs = 0;                // the SLAC match that opened this session
};
SessionData session;

uint32_t slacStartMs = 0;  // first CM_SLAC_PARAM.REQ of the current SLAC run
uint32_t slacLastMs = 0;   // any SLAC frame, to tell a retry from a new run
bool hasPendingAtten = false;
homeplug::AttenProfile pendingAtten;

// backlog_0016: one V/I sample per VI_SAMPLE_MS at a fixed x position. The newest sample
// overwrites the oldest column (ECG style) instead of scrolling the whole plot, so an update
// costs one column of drawing instead of a full redraw - and doesn't flicker.
static const int16_t VI_PLOT_W = 300;
static const uint32_t VI_SAMPLE_MS = 400;  // 300 * 400 ms = 120 s of history
static const uint16_t VI_NONE = 0xFFFF;    // no fresh value at that moment
struct ViSample {
  uint16_t volt10 = VI_NONE, curr10 = VI_NONE;  // 0.1 V / 0.1 A
};
ViSample viHistory[VI_PLOT_W];
uint32_t viCount = 0;  // samples since the session started; x = viCount % VI_PLOT_W
uint32_t viLastMs = 0;

// backlog_0021: the message ladder. Consecutive messages sharing a base name collapse into one
// row with a count and a two-way arrow - without that, 89 CurrentDemand pairs would push
// everything else off the page.
static const uint8_t LADDER_ROWS = 15;
static const uint8_t DIR_CAR = 0, DIR_CHARGER = 1, DIR_BOTH = 2;
struct LadderEntry {
  char name[26];
  uint16_t count;
  uint8_t dir;
  uint32_t ms;
};
LadderEntry ladder[LADDER_ROWS];
uint8_t ladderCount = 0;
uint32_t ladderVersion = 0;  // bumped on every change; the page redraws only when it moved

void phaseMark(uint8_t ph, uint32_t now) {
  if (ph >= PH_COUNT) {
    return;
  }
  if (session.phaseStart[ph] == 0) {
    session.phaseStart[ph] = now;
  }
  session.phaseEnd[ph] = now;
  session.lastPhase = ph;
}

void beginSession(uint32_t now) {
  session = SessionData();
  session.startMs = now;
  viCount = 0;
  viLastMs = 0;
  for (ViSample &v : viHistory) {
    v = ViSample();
  }
  ladderCount = 0;
  ladderVersion++;
  if (slacStartMs != 0) {
    session.phaseStart[PH_SLAC] = slacStartMs;  // the SLAC run that led to this match
    session.phaseEnd[PH_SLAC] = now;
  }
  if (hasPendingAtten) {
    session.hasAtten = true;
    session.atten = pendingAtten;
  }
  phaseMark(PH_JOIN, now);
}

// Which phase a decoded V2G message belongs to. Everything from SessionSetup to
// ChargeParameterDiscovery counts as the setup phase - they always run as one block.
uint8_t phaseOfMessage(const char *n) {
  if (strncmp(n, "SupportedAppProtocol", 20) == 0) return PH_HANDSHAKE;
  if (strncmp(n, "CableCheck", 10) == 0) return PH_CABLECHECK;
  if (strncmp(n, "PreCharge", 9) == 0) return PH_PRECHARGE;
  if (strncmp(n, "CurrentDemand", 13) == 0) return PH_CURRENTDEMAND;
  if (strncmp(n, "PowerDelivery", 13) == 0 || strncmp(n, "SessionStop", 11) == 0 ||
      strncmp(n, "WeldingDetection", 16) == 0) {
    return PH_STOP;
  }
  return PH_SETUP;
}

// "CurrentDemandReq" -> "CurrentDemand", "ATTEN_CHAR.IND" -> "ATTEN_CHAR": the two directions of
// one exchange share a row.
void ladderBaseName(const char *name, char *out, size_t len) {
  strlcpy(out, name, len);
  size_t n = strlen(out);
  if (n > 3 && (strcmp(out + n - 3, "Req") == 0 || strcmp(out + n - 3, "Res") == 0)) {
    out[n - 3] = 0;
    return;
  }
  char *dot = strrchr(out, '.');
  if (dot) {
    *dot = 0;
  }
}

void ladderAdd(const char *name, uint8_t dir, uint32_t now) {
  char base[sizeof(ladder[0].name)];
  ladderBaseName(name, base, sizeof(base));
  if (ladderCount > 0) {
    LadderEntry &last = ladder[ladderCount - 1];
    if (strcmp(last.name, base) == 0) {
      last.count++;
      if (last.dir != dir) {
        last.dir = DIR_BOTH;
      }
      last.ms = now;
      ladderVersion++;
      return;
    }
  }
  if (ladderCount == LADDER_ROWS) {  // the oldest row falls off the top
    memmove(&ladder[0], &ladder[1], sizeof(ladder[0]) * (LADDER_ROWS - 1));
    ladderCount--;
  }
  LadderEntry &e = ladder[ladderCount++];
  strlcpy(e.name, base, sizeof(e.name));
  e.count = 1;
  e.dir = dir;
  e.ms = now;
  ladderVersion++;
}

// Who sends which SLAC message (backlog_0021). The REQ/CNF/IND/RSP suffix does not tell:
// CM_START_ATTEN_CHAR.IND and CM_MNBC_SOUND.IND come from the CAR, CM_ATTEN_CHAR.IND from the
// charger. MMTYPEs are the base code plus the suffix in the low two bits.
uint8_t slacDirection(uint16_t mm) {
  switch (mm) {
    case 0x6064:  // CM_SLAC_PARAM.REQ
    case 0x606A:  // CM_START_ATTEN_CHAR.IND
    case 0x606F:  // CM_ATTEN_CHAR.RSP
    case 0x6076:  // CM_MNBC_SOUND.IND
    case 0x607C:  // CM_SLAC_MATCH.REQ
      return DIR_CAR;
    default:
      return DIR_CHARGER;
  }
}

// backlog_0020. Only messages carrying BOTH present values feed this (CurrentDemandRes), so the
// product is of two values measured at the same moment. A gap longer than 5 s is not charging -
// it is one session being killed and the next starting - and must not be integrated over.
void energyAdd(float volts, float amps, uint32_t now) {
  if (session.energySampleMs != 0) {
    uint32_t dt = now - session.energySampleMs;
    if (dt > 0 && dt < 5000) {
      double seconds = dt / 1000.0;
      session.energyWs += (double)volts * (double)amps * seconds;
      session.chargeAs += (double)amps * seconds;
    }
  }
  float watts = volts * amps;
  if (watts > session.peakW) {
    session.peakW = watts;
  }
  if (session.energyFirstMs == 0) {
    session.energyFirstMs = now;
  }
  session.energyLastMs = now;
  session.energySampleMs = now;
}

// backlog_0016. Called from loop(): samples whatever the decoder currently holds, so the x axis
// is real time and not "one pixel per message" - a stalled session then shows as a flat gap.
void sampleVoltageCurrent(uint32_t now) {
  if (viLastMs != 0 && now - viLastMs < VI_SAMPLE_MS) {
    return;
  }
  viLastMs = now;
  ViSample &sample = viHistory[viCount % VI_PLOT_W];
  bool freshV = v2gDisplay.hasPresentVoltage && now - v2gDisplay.presentVoltageMs < 3000;
  bool freshI = v2gDisplay.hasPresentCurrent && now - v2gDisplay.presentCurrentMs < 3000;
  sample.volt10 = freshV ? (uint16_t)(v2gDisplay.presentVoltage * 10.0f + 0.5f) : VI_NONE;
  sample.curr10 = freshI ? (uint16_t)(v2gDisplay.presentCurrent * 10.0f + 0.5f) : VI_NONE;
  viCount++;
}

// backlog_0017: identity fields, whenever a message carries one.
void storeIdentity(const V2gValues &v) {
  SessionSetup &s = sessionSetup;
  if (v.hasSessionId) {
    bool nonZero = false;
    for (uint8_t i = 0; i < v.sessionIdLen; i++) {
      if (v.sessionId[i] != 0) nonZero = true;
    }
    if (nonZero || !s.hasSessionId) {  // SessionSetupReq still carries all zeroes
      memcpy(s.sessionId, v.sessionId, sizeof(s.sessionId));
      s.sessionIdLen = v.sessionIdLen;
      s.hasSessionId = true;
    }
  }
  if (v.hasEvccid) {
    memcpy(s.evccid, v.evccid, sizeof(s.evccid));
    s.evccidLen = v.evccidLen;
    s.hasEvccid = true;
  }
}

// backlog_0013: keeps the ChargeParameterDiscovery values of the running session for page 3.
// Serial output follows the same convention as the rest of this file - the line is printed only
// when the SHOWN values change, so the repeated "Ongoing" responses don't flood the log.
void storeChargeParams(const V2gValues &v) {
  SessionSetup &s = sessionSetup;
  char line[160] = "";
  if (v.hasCpdReq) {
    s.hasCpdReq = true;
    s.energyTransferMode = v.energyTransferMode;
    s.hasEvLimits = v.hasEvLimits;
    if (v.hasEvLimits) {
      s.evMaxVoltage = v.evMaxVoltage;
      s.evMaxCurrent = v.evMaxCurrent;
      s.hasFullSoc = v.hasFullSoc;
      s.fullSoc = v.fullSoc;
      s.hasBulkSoc = v.hasBulkSoc;
      s.bulkSoc = v.bulkSoc;
      s.evReady = v.evReady;
      s.evErrorCode = v.evErrorCode;
      s.cpdSoc = v.cpdSoc;
    }
    snprintf(line, sizeof(line), "CPD car: %s max %.1fV %.1fA SoC %d%% ready %d err %s",
             energyTransferName(s.energyTransferMode), s.evMaxVoltage, s.evMaxCurrent, s.cpdSoc,
             s.evReady ? 1 : 0, evErrorName(s.evErrorCode));
    static char lastReq[160] = "";
    if (strcmp(line, lastReq) != 0) {
      strlcpy(lastReq, line, sizeof(lastReq));
      Serial.println(line);
    }
  }
  if (v.hasCpdRes) {
    s.hasCpdRes = true;
    s.evseProcessing = v.evseProcessing;
    s.cpdResponseCode = v.responseCode;
    if (v.hasEvseLimits) {
      s.hasEvseLimits = true;
      s.evseMaxVoltage = v.evseMaxVoltage;
      s.evseMaxCurrent = v.evseMaxCurrent;
      s.evseMinVoltage = v.evseMinVoltage;
      s.evseMinCurrent = v.evseMinCurrent;
      s.hasEvseMaxPower = v.hasEvseMaxPower;
      s.evseMaxPower = v.evseMaxPower;
    }
    snprintf(line, sizeof(line),
             "CPD charger: %s RC %u max %.1fV %.1fA %.0fW min %.1fV %.1fA",
             s.evseProcessing == 0 ? "Finished" : "Ongoing", s.cpdResponseCode, s.evseMaxVoltage,
             s.evseMaxCurrent, s.hasEvseMaxPower ? s.evseMaxPower : 0.0f, s.evseMinVoltage,
             s.evseMinCurrent);
    static char lastRes[160] = "";
    if (strcmp(line, lastRes) != 0) {
      strlcpy(lastRes, line, sizeof(lastRes));
      Serial.println(line);
    }
  }
}

void applyV2gValues(const V2gValues &v, bool ok) {
  sessionSetup.lastActivityMs = millis();  // the session is alive - keeps page 2 coloured; also on
                                           // a failed decode, which is still V2G traffic
  if (ok) {
    storeHandshake(v);
    storeChargeParams(v);
    storeIdentity(v);
    // Pages 4, 8 and 9: this message's phase, its row in the ladder, its energy contribution.
    uint32_t msgMs = millis();
    phaseMark(phaseOfMessage(v.msgName), msgMs);
    size_t nameLen = strlen(v.msgName);
    bool fromCharger = nameLen >= 3 && strcmp(v.msgName + nameLen - 3, "Res") == 0;
    ladderAdd(v.msgName, fromCharger ? DIR_CHARGER : DIR_CAR, msgMs);
    if (v.hasPresentVoltage && v.hasPresentCurrent) {
      energyAdd(v.presentVoltage, v.presentCurrent, msgMs);
    }
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

TextLine headerHintLine = {126, 3, 2, 4};  // "TLS" next to "ccs32dave" on the main page
TextLine titleHintLine = {294, 7, 1, 4};   // the same hint behind a page's own heading. Pages
                                           // with a heading MUST use this one: the main page's
                                           // hint sits in the middle of the header and would
                                           // blank four characters out of their title.

// The page name is in the header ("SDP"), so the SDP block needs no heading of its own and the
// two rows it used go into the text (owner request 2026-09-22).
TextLine p2SdpCar = {TEXT_X, 26, 2};
TextLine p2SdpEvse = {TEXT_X, 46, 2};
TextLine p2SdpAddr = {TEXT_X, 66, 1};
static const int16_t PAGE2_DIVIDER_Y = 80;  // between the SDP block above and the schemas below
TextLine p2AphHead = {TEXT_X, 86, 2, 7};    // "schema", the lower section's own heading
TextLine p2AphHint = {TEXT_X + 90, 94, 1};  // its small print, on the heading's baseline
TextLine p2AphLines[V2gValues::MAX_APP_PROTOCOLS] = {
    {TEXT_X, 108, 1}, {TEXT_X, 119, 1}, {TEXT_X, 130, 1}, {TEXT_X, 141, 1}, {TEXT_X, 152, 1}};
TextLine p2AphResult = {TEXT_X, 166, 1};
TextLine p2AphSelected = {TEXT_X, 180, 3, 12};  // the negotiated protocol - the page's conclusion
TextLine p2Footer = {TEXT_X, 229, 1};

// TLS negotiated -> the DIN messages can't be decoded, so say it on both pages. "TLS?" if only
// the car's request is known (the charger's response may not reach us - backlog_0009).
void drawHeaderHint() {
  TextLine &line = page == 1 ? headerHintLine : titleHintLine;
  if (sessionSetup.hasSdpRes) {
    drawLine(line, ILI9341_RED, sessionSetup.resSecurity == SDP_SECURITY_TLS ? "TLS" : "");
  } else if (sessionSetup.hasSdpReq && sessionSetup.reqSecurity == SDP_SECURITY_TLS) {
    drawLine(line, ILI9341_ORANGE, "TLS?");
  } else {
    drawLine(line, ILI9341_RED, "");
  }
}

// Coloured while the session is alive, gray once it has been quiet - one decision for the whole
// page (2 and 3 alike), so their values never disagree about which session they belong to.
bool sessionQuiet() {
  return sessionSetup.lastActivityMs == 0 ||
         millis() - sessionSetup.lastActivityMs >= SESSION_QUIET_MS;
}

uint16_t sessionColor(uint16_t freshColor) {
  return sessionQuiet() ? COLOR_DARKGREY : freshColor;
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

  if (s.hasSdpReq) {
    snprintf(buf, sizeof(buf), "Car:  %s %s", sdpSecurityName(s.reqSecurity),
             sdpTransportName(s.reqTransport));
    drawLine(p2SdpCar, sessionColor(s.reqSecurity == SDP_SECURITY_NONE ? ILI9341_GREEN : ILI9341_RED),
             buf);
  } else {
    drawLine(p2SdpCar, COLOR_DARKGREY, "Car:  -");
  }
  if (s.hasSdpRes) {
    snprintf(buf, sizeof(buf), "EVSE: %s %s", sdpSecurityName(s.resSecurity),
             sdpTransportName(s.resTransport));
    drawLine(p2SdpEvse, sessionColor(s.resSecurity == SDP_SECURITY_NONE ? ILI9341_GREEN : ILI9341_RED),
             buf);
    const uint8_t *a = s.evseIp;
    snprintf(buf, sizeof(buf), "EVSE %x:%x:%x:%x:%x:%x:%x:%x port %u",
             (a[0] << 8) | a[1], (a[2] << 8) | a[3], (a[4] << 8) | a[5], (a[6] << 8) | a[7],
             (a[8] << 8) | a[9], (a[10] << 8) | a[11], (a[12] << 8) | a[13], (a[14] << 8) | a[15],
             s.evsePort);
    drawLine(p2SdpAddr, sessionColor(ILI9341_WHITE), buf);
  } else {
    drawLine(p2SdpEvse, COLOR_DARKGREY, "EVSE: -");
    drawLine(p2SdpAddr, COLOR_DARKGREY, "");
  }

  drawLine(p2AphHead, PAGE2_HEAD_COLOR, "schema");
  drawLine(p2AphHint, COLOR_DARKGREY, "offered by the car, > = selected");
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
    drawLine(p2AphLines[i], sessionColor(isSelected ? ILI9341_GREEN : ILI9341_WHITE), buf);
  }

  if (s.hasHandshakeResult) {
    bool okResult = s.handshakeResponseCode < 2;
    if (s.hasSelectedSchema) {
      snprintf(buf, sizeof(buf), "Charger: %s, schema ID %u",
               handshakeResultName(s.handshakeResponseCode), s.selectedSchemaId);
    } else {
      snprintf(buf, sizeof(buf), "Charger: %s", handshakeResultName(s.handshakeResponseCode));
    }
    uint16_t color = sessionColor(okResult ? ILI9341_GREEN : ILI9341_RED);
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

  drawPageFooter(p2Footer);
}


// Every page with its own heading carries the same footer: where you are, and what the single
// button will do next.
void drawPageFooter(TextLine &line) {
  char buf[48];
  snprintf(buf, sizeof(buf), "page %u/%u - button: %s", page, PAGE_COUNT,
           page == PAGE_COUNT ? "main page" : "next page");
  drawLine(line, COLOR_DARKGREY, buf);
}

// ---- Page 3: charge parameter discovery (backlog_0013) -----------------------
//
// Two columns: what the car asks for (left, yellow like the main page's TARGET block) against
// what the charger offers (right, green like PRESENT), so the two limit sets can be compared at
// a glance - the smaller of each pair is what the session will actually run at. Same session
// colouring as page 2: coloured while V2G traffic arrives, gray once the session is quiet.
//
// Layout revised 2026-09-22 after the first bench run: the page name moved into the header
// (drawPage3Header), which frees the title row and lets the six numbers that matter run at text
// size 3 (24 px) - readable across the bench - with size 2 for the rest. Column geometry: at
// size 3 a column holds 8 characters (8 * 18 px = 144), at size 2 twelve, at size 1 twenty-five.
static const int16_t P3_LEFT_X = 6;
static const int16_t P3_DIVIDER_X = 158;  // vertical divider between the two columns
static const int16_t P3_RIGHT_X = 166;
static const uint8_t P3_COLS_S1 = 25;  // padding must stop at the divider, not at the screen edge
static const uint8_t P3_COLS_S2 = 12;
static const uint8_t P3_COLS_S3 = 8;

TextLine p3CarHead = {P3_LEFT_X, 26, 2, P3_COLS_S2};
TextLine p3CarMode = {P3_LEFT_X, 46, 2, P3_COLS_S2};
TextLine p3CarMaxHead = {P3_LEFT_X, 68, 1, P3_COLS_S1};
TextLine p3CarVolt = {P3_LEFT_X, 78, 3, P3_COLS_S3};
TextLine p3CarCurr = {P3_LEFT_X, 106, 3, P3_COLS_S3};
TextLine p3CarSoc = {P3_LEFT_X, 136, 2, P3_COLS_S2};
TextLine p3CarReady = {P3_LEFT_X, 156, 2, P3_COLS_S2};
TextLine p3CarSocTargets = {P3_LEFT_X, 178, 1, P3_COLS_S1};
TextLine p3CarErrorHead = {P3_LEFT_X, 190, 1, P3_COLS_S1};
TextLine p3CarError = {P3_LEFT_X, 200, 2, P3_COLS_S2};

TextLine p3EvseHead = {P3_RIGHT_X, 26, 2, P3_COLS_S2};
TextLine p3EvseProc = {P3_RIGHT_X, 46, 2, P3_COLS_S2};
TextLine p3EvseMaxHead = {P3_RIGHT_X, 68, 1, P3_COLS_S1};
TextLine p3EvseVolt = {P3_RIGHT_X, 78, 3, P3_COLS_S3};
TextLine p3EvseCurr = {P3_RIGHT_X, 106, 3, P3_COLS_S3};
TextLine p3EvsePower = {P3_RIGHT_X, 134, 3, P3_COLS_S3};
TextLine p3EvseMinHead = {P3_RIGHT_X, 164, 1, P3_COLS_S1};
TextLine p3EvseMin = {P3_RIGHT_X, 174, 2, P3_COLS_S2};
TextLine p3EvseRcHead = {P3_RIGHT_X, 190, 1, P3_COLS_S1};
TextLine p3EvseRc = {P3_RIGHT_X, 200, 2, P3_COLS_S2};

TextLine p3Footer = {P3_LEFT_X, 229, 1};

// DIN sends power in watts with its own multiplier, so a 10 kW station may arrive as
// 10 * 10^3 W - printed as kW once it gets long.
void formatPower(char *buf, size_t len, float watts) {
  if (fabsf(watts) >= 1000.0f) {
    snprintf(buf, len, "%.1fkW", watts / 1000.0f);
  } else {
    snprintf(buf, len, "%.0fW", watts);
  }
}

void drawPage3() {
  const SessionSetup &s = sessionSetup;
  char buf[65];

  // --- left column: the car's request ---
  drawLine(p3CarHead, COLOR_DARKGREY, "CAR wants");
  drawLine(p3CarMode, s.hasCpdReq ? sessionColor(ILI9341_WHITE) : COLOR_DARKGREY,
           s.hasCpdReq ? energyTransferName(s.energyTransferMode) : "-");
  drawLine(p3CarMaxHead, COLOR_DARKGREY, "maximum");
  if (s.hasEvLimits) {
    snprintf(buf, sizeof(buf), "%.1fV", s.evMaxVoltage);
    drawLine(p3CarVolt, sessionColor(ILI9341_YELLOW), buf);
    snprintf(buf, sizeof(buf), "%.1fA", s.evMaxCurrent);
    drawLine(p3CarCurr, sessionColor(ILI9341_YELLOW), buf);
    snprintf(buf, sizeof(buf), "SoC %d%%", s.cpdSoc);
    drawLine(p3CarSoc, sessionColor(ILI9341_CYAN), buf);
    snprintf(buf, sizeof(buf), "ready %s", s.evReady ? "yes" : "no");
    drawLine(p3CarReady, sessionColor(s.evReady ? ILI9341_GREEN : ILI9341_ORANGE), buf);
    // Both SoC targets are optional in DIN - each one shows "-" on its own when omitted.
    char full[8], bulk[8];
    if (s.hasFullSoc) snprintf(full, sizeof(full), "%d%%", s.fullSoc); else strlcpy(full, "-", sizeof(full));
    if (s.hasBulkSoc) snprintf(bulk, sizeof(bulk), "%d%%", s.bulkSoc); else strlcpy(bulk, "-", sizeof(bulk));
    snprintf(buf, sizeof(buf), "full %s  bulk %s", full, bulk);
    drawLine(p3CarSocTargets, sessionColor(ILI9341_WHITE), buf);
    drawLine(p3CarErrorHead, COLOR_DARKGREY, "EV error code");
    bool noError = s.evErrorCode == 0;
    drawLine(p3CarError, sessionColor(noError ? ILI9341_GREEN : ILI9341_RED),
             evErrorName(s.evErrorCode));
  } else {
    drawLine(p3CarVolt, COLOR_DARKGREY, "-");
    drawLine(p3CarCurr, COLOR_DARKGREY, "-");
    drawLine(p3CarSoc, COLOR_DARKGREY, "SoC -");
    drawLine(p3CarReady, COLOR_DARKGREY, "ready -");
    drawLine(p3CarSocTargets, COLOR_DARKGREY, "full -  bulk -");
    drawLine(p3CarErrorHead, COLOR_DARKGREY, "EV error code");
    drawLine(p3CarError, COLOR_DARKGREY, "-");
  }

  // --- right column: the charger's response ---
  drawLine(p3EvseHead, COLOR_DARKGREY, "CHARGER");
  if (s.hasCpdRes) {
    // "Ongoing" means the charger is still working out its parameters; the limits below may
    // still be missing then, and the car keeps repeating the request until "Finished".
    bool finished = s.evseProcessing == 0;
    drawLine(p3EvseProc, sessionColor(finished ? ILI9341_GREEN : ILI9341_ORANGE),
             finished ? "Finished" : "Ongoing");
  } else {
    drawLine(p3EvseProc, COLOR_DARKGREY, "-");
  }
  drawLine(p3EvseMaxHead, COLOR_DARKGREY, "maximum");
  drawLine(p3EvseMinHead, COLOR_DARKGREY, "minimum");
  if (s.hasEvseLimits) {
    snprintf(buf, sizeof(buf), "%.1fV", s.evseMaxVoltage);
    drawLine(p3EvseVolt, sessionColor(ILI9341_GREEN), buf);
    snprintf(buf, sizeof(buf), "%.1fA", s.evseMaxCurrent);
    drawLine(p3EvseCurr, sessionColor(ILI9341_GREEN), buf);
    if (s.hasEvseMaxPower) {
      formatPower(buf, sizeof(buf), s.evseMaxPower);
      drawLine(p3EvsePower, sessionColor(ILI9341_GREEN), buf);
    } else {
      drawLine(p3EvsePower, COLOR_DARKGREY, "-");
    }
    snprintf(buf, sizeof(buf), "%.0fV %.0fA", s.evseMinVoltage, s.evseMinCurrent);
    drawLine(p3EvseMin, sessionColor(ILI9341_WHITE), buf);
  } else {
    drawLine(p3EvseVolt, COLOR_DARKGREY, "-");
    drawLine(p3EvseCurr, COLOR_DARKGREY, "-");
    drawLine(p3EvsePower, COLOR_DARKGREY, "-");
    drawLine(p3EvseMin, COLOR_DARKGREY, "-");
  }

  // Response code of the discovery itself - same wording as the main page's rc line.
  drawLine(p3EvseRcHead, COLOR_DARKGREY, "response");
  if (!s.hasCpdRes) {
    drawLine(p3EvseRc, COLOR_DARKGREY, "-");
  } else if (s.cpdResponseCode == 0) {
    drawLine(p3EvseRc, sessionColor(ILI9341_GREEN), "OK");
  } else {
    snprintf(buf, sizeof(buf), "RC %u", s.cpdResponseCode);
    drawLine(p3EvseRc, sessionColor(ILI9341_RED), buf);
  }

  drawPageFooter(p3Footer);
}

// ---- Page 4: session timeline (backlog_0014) --------------------------------
//
// One row per phase: name, a bar whose length is that phase's share of the longest one, and the
// duration. Green = finished, yellow = running, red = it was the last thing that happened and
// the session has gone quiet since (that is where it died), dark grey = never reached.
static const int16_t P4_ROW_Y = 26;
static const int16_t P4_ROW_PITCH = 22;
static const int16_t P4_BAR_X = 130;
static const int16_t P4_BAR_W = 120;
static const int16_t P4_BAR_H = 10;

TextLine p4Names[PH_COUNT];
TextLine p4Times[PH_COUNT];
int16_t p4DrawnWidth[PH_COUNT];    // last bar geometry, so a bar is repainted only when it moves
uint16_t p4DrawnColor[PH_COUNT];
TextLine p4Footer = {6, 229, 1};

// Set up once at boot - nine rows of identical geometry aren't worth writing out by hand.
void initPage4Lines() {
  for (uint8_t i = 0; i < PH_COUNT; i++) {
    int16_t y = P4_ROW_Y + i * P4_ROW_PITCH;
    p4Names[i] = TextLine{4, y, 2, 10};
    p4Times[i] = TextLine{256, (int16_t)(y + 4), 1, 10};
  }
}

void formatDuration(char *buf, size_t len, uint32_t ms) {
  if (ms >= 100000) {
    snprintf(buf, len, "%lus", (unsigned long)(ms / 1000));
  } else {
    snprintf(buf, len, "%lu.%lus", (unsigned long)(ms / 1000), (unsigned long)((ms % 1000) / 100));
  }
}

void drawPage4() {
  char buf[32];
  uint32_t now = millis();
  bool quiet = sessionQuiet();

  // The running phase is the last one a message was booked into, as long as the session lives.
  uint32_t durations[PH_COUNT];
  uint32_t longest = 1;
  for (uint8_t i = 0; i < PH_COUNT; i++) {
    bool running = !quiet && i == session.lastPhase;
    if (session.phaseStart[i] == 0) {
      durations[i] = 0;
    } else if (running) {
      durations[i] = now - session.phaseStart[i];
    } else {
      durations[i] = session.phaseEnd[i] - session.phaseStart[i];
    }
    if (durations[i] > longest) {
      longest = durations[i];
    }
  }

  for (uint8_t i = 0; i < PH_COUNT; i++) {
    bool reached = session.phaseStart[i] != 0;
    bool running = reached && !quiet && i == session.lastPhase;
    bool died = reached && quiet && i == session.lastPhase;
    uint16_t color = COLOR_DARKGREY;
    if (running) {
      color = ILI9341_YELLOW;
    } else if (died) {
      color = ILI9341_RED;
    } else if (reached) {
      color = ILI9341_GREEN;
    }
    drawLine(p4Names[i], color, PHASE_NAMES[i]);

    if (reached) {
      formatDuration(buf, sizeof(buf), durations[i]);
    } else {
      strlcpy(buf, "-", sizeof(buf));
    }
    drawLine(p4Times[i], color, buf);

    // The bar is redrawn only when its length changes - otherwise the running phase would
    // repaint the whole column on every message.
    int16_t w = reached ? (int16_t)((uint32_t)P4_BAR_W * durations[i] / longest) : 0;
    if (reached && w < 2) {
      w = 2;
    }
    int16_t y = P4_ROW_Y + i * P4_ROW_PITCH + 3;
    if (w != p4DrawnWidth[i] || color != p4DrawnColor[i]) {
      tft.fillRect(P4_BAR_X, y, P4_BAR_W, P4_BAR_H, ILI9341_BLACK);
      if (w > 0) {
        tft.fillRect(P4_BAR_X, y, w, P4_BAR_H, color);
      }
      p4DrawnWidth[i] = w;
      p4DrawnColor[i] = color;
    }
  }
  drawPageFooter(p4Footer);
}

// ---- Page 5: SLAC quality (backlog_0015) ------------------------------------
//
// The attenuation profile of CM_ATTEN_CHAR.IND as a bar per carrier group. Low bars (green) mean
// good coupling; the shape is a fingerprint of the cable and coupler and changes when they do.
static const int16_t P5_PLOT_X = 24;
static const int16_t P5_PLOT_Y = 84;
static const int16_t P5_PLOT_W = 288;
static const int16_t P5_PLOT_H = 126;

TextLine p5Sounds = {6, 26, 2, 14};
TextLine p5Duration = {180, 26, 2, 11};
TextLine p5Average = {6, 48, 2, 14};
TextLine p5Peak = {180, 48, 2, 11};
TextLine p5AxisHead = {6, 70, 1};
TextLine p5Footer = {6, 229, 1};

uint32_t p5DrawnKey = 0;  // changes whenever the plotted profile changes

void drawPage5() {
  char buf[40];
  const homeplug::AttenProfile &a = session.atten;

  if (session.hasAtten) {
    snprintf(buf, sizeof(buf), "sounds %u", a.numSounds);
    drawLine(p5Sounds, sessionColor(a.numSounds >= 10 ? ILI9341_GREEN : ILI9341_ORANGE), buf);
  } else {
    drawLine(p5Sounds, COLOR_DARKGREY, "sounds -");
  }

  // The SLAC duration comes from the timeline, which measured the same run.
  if (session.phaseStart[PH_SLAC] != 0) {
    char d[16];
    formatDuration(d, sizeof(d), session.phaseEnd[PH_SLAC] - session.phaseStart[PH_SLAC]);
    snprintf(buf, sizeof(buf), "SLAC %s", d);
    drawLine(p5Duration, sessionColor(ILI9341_WHITE), buf);
  } else {
    drawLine(p5Duration, COLOR_DARKGREY, "SLAC -");
  }

  uint16_t sum = 0, peak = 0;
  for (uint8_t i = 0; i < a.numGroups; i++) {
    sum += a.attenuation[i];
    if (a.attenuation[i] > peak) {
      peak = a.attenuation[i];
    }
  }
  uint8_t average = a.numGroups ? (uint8_t)(sum / a.numGroups) : 0;
  if (session.hasAtten) {
    snprintf(buf, sizeof(buf), "avg %u dB", average);
    drawLine(p5Average, sessionColor(ILI9341_CYAN), buf);
    snprintf(buf, sizeof(buf), "max %u dB", peak);
    drawLine(p5Peak, sessionColor(ILI9341_CYAN), buf);
    snprintf(buf, sizeof(buf), "attenuation of %u carrier groups, dB", a.numGroups);
    drawLine(p5AxisHead, COLOR_DARKGREY, buf);
  } else {
    drawLine(p5Average, COLOR_DARKGREY, "avg -");
    drawLine(p5Peak, COLOR_DARKGREY, "max -");
    drawLine(p5AxisHead, COLOR_DARKGREY, "waiting for CM_ATTEN_CHAR.IND");
  }

  // Redraw the bars only when the profile itself changed.
  uint32_t key = session.hasAtten ? (session.startMs ^ ((uint32_t)a.numGroups << 8) ^
                                     ((uint32_t)a.numSounds << 16) ^ ((uint32_t)sum << 4))
                                  : 0;
  if (key == p5DrawnKey) {
    drawPageFooter(p5Footer);
    return;
  }
  p5DrawnKey = key;
  tft.fillRect(P5_PLOT_X, P5_PLOT_Y, P5_PLOT_W, P5_PLOT_H, ILI9341_BLACK);
  tft.drawFastHLine(P5_PLOT_X, P5_PLOT_Y + P5_PLOT_H, P5_PLOT_W, COLOR_DARKGREY);
  if (!session.hasAtten || a.numGroups == 0) {
    drawPageFooter(p5Footer);
    return;
  }

  // Full scale is the next multiple of 20 dB above the worst group, so the shape fills the plot.
  uint8_t fullScale = (uint8_t)(((peak / 20) + 1) * 20);
  int16_t barPitch = P5_PLOT_W / a.numGroups;
  if (barPitch < 1) {
    barPitch = 1;
  }
  int16_t barWidth = barPitch > 2 ? barPitch - 1 : barPitch;
  for (uint8_t i = 0; i < a.numGroups; i++) {
    uint8_t db = a.attenuation[i];
    int16_t h = (int16_t)((uint32_t)P5_PLOT_H * db / fullScale);
    if (h > P5_PLOT_H) {
      h = P5_PLOT_H;
    }
    uint16_t color = db < 30 ? ILI9341_GREEN : (db < 45 ? ILI9341_YELLOW : ILI9341_RED);
    tft.fillRect(P5_PLOT_X + i * barPitch, P5_PLOT_Y, barWidth, h, sessionColor(color));
  }
  // Scale marks: 0 dB at the top of the plot, full scale at its baseline.
  tft.setTextSize(1);
  tft.setTextColor(COLOR_DARKGREY, ILI9341_BLACK);
  tft.setCursor(2, P5_PLOT_Y);
  tft.print("0");
  tft.setCursor(2, P5_PLOT_Y + P5_PLOT_H - 8);
  tft.print(fullScale);
  drawPageFooter(p5Footer);
}

// ---- Page 6: voltage and current over time (backlog_0016) -------------------
//
// Sweeping plot: sample n is drawn at x = n % width, so a new sample costs one column instead of
// a full repaint, and the position of the gap shows where "now" is. Voltage green against the
// left scale, current yellow against the right one.
static const int16_t P6_PLOT_X = 10;
static const int16_t P6_PLOT_Y = 60;
static const int16_t P6_PLOT_H = 150;

TextLine p6Volt = {6, 26, 2, 8};
TextLine p6Curr = {110, 26, 2, 8};
TextLine p6Power = {214, 26, 2, 8};
TextLine p6ScaleV = {6, 48, 1, 16};
TextLine p6ScaleI = {214, 48, 1, 17};
TextLine p6Footer = {6, 229, 1};

uint32_t p6DrawnCount = 0;
uint16_t p6ScaleVolt10 = 0, p6ScaleCurr10 = 0;

// Round up to something a human reads off an axis: 1, 2 or 5 times a power of ten.
uint16_t niceScale(uint16_t value) {
  uint16_t step = 10;
  while (step < 10000) {
    if (value <= step) return step;
    if (value <= step * 2) return step * 2;
    if (value <= step * 5) return step * 5;
    step *= 10;
  }
  return 10000;
}

int16_t p6PlotY(uint16_t value10, uint16_t scale10) {
  if (scale10 == 0) {
    return P6_PLOT_Y + P6_PLOT_H;
  }
  int32_t h = (int32_t)P6_PLOT_H * value10 / scale10;
  if (h > P6_PLOT_H) {
    h = P6_PLOT_H;
  }
  return P6_PLOT_Y + P6_PLOT_H - (int16_t)h;
}

// One column of the sweep: erase it, put the grid back, then plot whatever the sample holds.
void drawViColumn(int16_t index) {
  int16_t x = P6_PLOT_X + index;
  const ViSample &sample = viHistory[index];
  tft.drawFastVLine(x, P6_PLOT_Y, P6_PLOT_H, ILI9341_BLACK);
  for (uint8_t g = 1; g < 4; g++) {  // three horizontal grid lines, as dotted pixels
    if ((index % 6) == 0) {
      tft.drawPixel(x, P6_PLOT_Y + P6_PLOT_H * g / 4, COLOR_DARKGREY);
    }
  }
  if (sample.volt10 != VI_NONE) {
    int16_t y = p6PlotY(sample.volt10, p6ScaleVolt10);
    tft.drawPixel(x, y, ILI9341_GREEN);
    tft.drawPixel(x, y + 1, ILI9341_GREEN);
  }
  if (sample.curr10 != VI_NONE) {
    int16_t y = p6PlotY(sample.curr10, p6ScaleCurr10);
    tft.drawPixel(x, y, ILI9341_YELLOW);
    tft.drawPixel(x, y + 1, ILI9341_YELLOW);
  }
}

void drawPage6() {
  char buf[32];

  if (v2gDisplay.hasPresentVoltage) {
    snprintf(buf, sizeof(buf), "%.1fV", v2gDisplay.presentVoltage);
    drawLine(p6Volt, staleColor(ILI9341_GREEN, true, v2gDisplay.presentVoltageMs), buf);
  } else {
    drawLine(p6Volt, COLOR_DARKGREY, "-");
  }
  if (v2gDisplay.hasPresentCurrent) {
    snprintf(buf, sizeof(buf), "%.1fA", v2gDisplay.presentCurrent);
    drawLine(p6Curr, staleColor(ILI9341_YELLOW, true, v2gDisplay.presentCurrentMs), buf);
  } else {
    drawLine(p6Curr, COLOR_DARKGREY, "-");
  }
  if (v2gDisplay.hasPresentVoltage && v2gDisplay.hasPresentCurrent) {
    formatPower(buf, sizeof(buf), v2gDisplay.presentVoltage * v2gDisplay.presentCurrent);
    drawLine(p6Power, staleColor(ILI9341_CYAN, true, v2gDisplay.presentCurrentMs), buf);
  } else {
    drawLine(p6Power, COLOR_DARKGREY, "-");
  }

  // Scales follow the data, rounded up to a readable step.
  uint16_t maxV = 0, maxI = 0;
  uint32_t stored = viCount < VI_PLOT_W ? viCount : VI_PLOT_W;
  for (uint32_t i = 0; i < stored; i++) {
    if (viHistory[i].volt10 != VI_NONE && viHistory[i].volt10 > maxV) maxV = viHistory[i].volt10;
    if (viHistory[i].curr10 != VI_NONE && viHistory[i].curr10 > maxI) maxI = viHistory[i].curr10;
  }
  uint16_t scaleV = niceScale(maxV ? maxV : 1);
  uint16_t scaleI = niceScale(maxI ? maxI : 1);

  bool rescaled = scaleV != p6ScaleVolt10 || scaleI != p6ScaleCurr10;
  if (rescaled) {
    p6ScaleVolt10 = scaleV;
    p6ScaleCurr10 = scaleI;
    snprintf(buf, sizeof(buf), "0..%uV", scaleV / 10);
    drawLine(p6ScaleV, ILI9341_GREEN, buf);
    snprintf(buf, sizeof(buf), "0..%uA  120s ->", scaleI / 10);
    drawLine(p6ScaleI, ILI9341_YELLOW, buf);
  }
  if (rescaled || viCount < p6DrawnCount) {  // a new session restarts the sweep
    tft.fillRect(P6_PLOT_X, P6_PLOT_Y, VI_PLOT_W, P6_PLOT_H, ILI9341_BLACK);
    tft.drawFastHLine(P6_PLOT_X, P6_PLOT_Y + P6_PLOT_H, VI_PLOT_W, COLOR_DARKGREY);
    for (uint32_t i = 0; i < stored; i++) {
      drawViColumn((int16_t)i);
    }
    p6DrawnCount = viCount;
  }
  for (uint32_t n = p6DrawnCount; n < viCount; n++) {
    drawViColumn((int16_t)(n % VI_PLOT_W));
  }
  if (viCount > p6DrawnCount || rescaled) {
    // A two-column gap ahead of the cursor marks where the newest sample sits.
    int16_t gap = (int16_t)(viCount % VI_PLOT_W);
    tft.drawFastVLine(P6_PLOT_X + gap, P6_PLOT_Y, P6_PLOT_H, ILI9341_BLACK);
    tft.drawFastVLine(P6_PLOT_X + (gap + 1) % VI_PLOT_W, P6_PLOT_Y, P6_PLOT_H, ILI9341_BLACK);
  }
  p6DrawnCount = viCount;
  drawPageFooter(p6Footer);
}

// ---- Page 7: identity and keys (backlog_0017) -------------------------------

TextLine p7CarHead = {6, 26, 1};
TextLine p7Car = {6, 36, 2, 17};
TextLine p7EvseHead = {6, 62, 1};
TextLine p7Evse = {6, 72, 2, 17};
TextLine p7SessionHead = {6, 98, 1};
TextLine p7Session = {6, 108, 2, 17};
TextLine p7NidHead = {6, 134, 1};
TextLine p7Nid = {6, 144, 2, 17};
TextLine p7NmkHead = {6, 170, 1};
TextLine p7Nmk1 = {6, 180, 2, 16};   // 16 hex characters at size 2 are 192 px - one row each,
TextLine p7Nmk2 = {6, 202, 2, 16};   // side by side the second half would run off the screen
TextLine p7Footer = {6, 229, 1};

void formatHex(char *buf, size_t len, const uint8_t *data, uint8_t count) {
  buf[0] = '\0';
  for (uint8_t i = 0; i < count && (size_t)(i * 2 + 1) < len; i++) {
    snprintf(buf + i * 2, len - i * 2, "%02X", data[i]);
  }
}

void drawPage7() {
  char buf[48];
  const SessionSetup &s = sessionSetup;

  drawLine(p7CarHead, COLOR_DARKGREY, "car (EVCCID)");
  if (s.hasEvccid) {
    formatMac(buf, sizeof(buf), s.evccid);
    drawLine(p7Car, sessionColor(ILI9341_YELLOW), buf);
  } else if (session.hasMatch) {
    formatMac(buf, sizeof(buf), session.match.pevMac);  // the SLAC match names it too
    drawLine(p7Car, sessionColor(ILI9341_YELLOW), buf);
  } else {
    drawLine(p7Car, COLOR_DARKGREY, "-");
  }

  drawLine(p7EvseHead, COLOR_DARKGREY, "charger (EVSE)");
  if (session.hasMatch) {
    formatMac(buf, sizeof(buf), session.match.evseMac);
    drawLine(p7Evse, sessionColor(ILI9341_GREEN), buf);
  } else {
    drawLine(p7Evse, COLOR_DARKGREY, "-");
  }

  drawLine(p7SessionHead, COLOR_DARKGREY, "V2G SessionID");
  if (s.hasSessionId) {
    formatHex(buf, sizeof(buf), s.sessionId, s.sessionIdLen);
    drawLine(p7Session, sessionColor(ILI9341_WHITE), buf);
  } else {
    drawLine(p7Session, COLOR_DARKGREY, "-");
  }

  // NID and NMK come from CM_SLAC_MATCH.CNF - the key the whole AVLN then runs on.
  drawLine(p7NidHead, COLOR_DARKGREY, "NID");
  drawLine(p7NmkHead, COLOR_DARKGREY, "NMK");
  if (session.hasMatch) {
    formatHex(buf, sizeof(buf), session.match.nid, 7);
    drawLine(p7Nid, sessionColor(ILI9341_CYAN), buf);
    formatHex(buf, sizeof(buf), session.match.nmk, 8);
    drawLine(p7Nmk1, sessionColor(ILI9341_CYAN), buf);
    formatHex(buf, sizeof(buf), session.match.nmk + 8, 8);
    drawLine(p7Nmk2, sessionColor(ILI9341_CYAN), buf);
  } else {
    drawLine(p7Nid, COLOR_DARKGREY, "-");
    drawLine(p7Nmk1, COLOR_DARKGREY, "-");
    drawLine(p7Nmk2, COLOR_DARKGREY, "");
  }
  drawPageFooter(p7Footer);
}

// ---- Page 8: energy receipt (backlog_0020) ----------------------------------

TextLine p8EnergyHead = {6, 26, 1};
TextLine p8Energy = {6, 36, 3, 10};
TextLine p8ChargeHead = {6, 70, 1};
TextLine p8Charge = {6, 80, 3, 10};
TextLine p8AvgHead = {6, 114, 1, 12};
TextLine p8PeakHead = {166, 114, 1, 12};
TextLine p8Avg = {6, 124, 3, 8};
TextLine p8Peak = {166, 124, 3, 8};
TextLine p8DurationHead = {6, 158, 1, 12};
TextLine p8PhaseHead = {166, 158, 1, 12};
TextLine p8Duration = {6, 168, 2, 12};
TextLine p8Phase = {166, 168, 2, 12};
TextLine p8Footer = {6, 229, 1};

void drawPage8() {
  char buf[32];
  bool any = session.energyFirstMs != 0;
  uint32_t chargingMs = any ? session.energyLastMs - session.energyFirstMs : 0;
  double kwh = session.energyWs / 3600000.0;
  double ah = session.chargeAs / 3600.0;
  float avgW = chargingMs > 0 ? (float)(session.energyWs / (chargingMs / 1000.0)) : 0.0f;

  drawLine(p8EnergyHead, COLOR_DARKGREY, "ENERGY DELIVERED");
  if (any) {
    snprintf(buf, sizeof(buf), "%.3fkWh", kwh);
    drawLine(p8Energy, sessionColor(ILI9341_GREEN), buf);
  } else {
    drawLine(p8Energy, COLOR_DARKGREY, "-");
  }

  drawLine(p8ChargeHead, COLOR_DARKGREY, "CHARGE");
  if (any) {
    snprintf(buf, sizeof(buf), "%.2fAh", ah);
    drawLine(p8Charge, sessionColor(ILI9341_YELLOW), buf);
  } else {
    drawLine(p8Charge, COLOR_DARKGREY, "-");
  }

  drawLine(p8AvgHead, COLOR_DARKGREY, "AVERAGE");
  drawLine(p8PeakHead, COLOR_DARKGREY, "PEAK");
  if (any) {
    formatPower(buf, sizeof(buf), avgW);
    drawLine(p8Avg, sessionColor(ILI9341_CYAN), buf);
    formatPower(buf, sizeof(buf), session.peakW);
    drawLine(p8Peak, sessionColor(ILI9341_CYAN), buf);
  } else {
    drawLine(p8Avg, COLOR_DARKGREY, "-");
    drawLine(p8Peak, COLOR_DARKGREY, "-");
  }

  drawLine(p8DurationHead, COLOR_DARKGREY, "CHARGING");
  drawLine(p8PhaseHead, COLOR_DARKGREY, "PHASE");
  if (any) {
    formatDuration(buf, sizeof(buf), chargingMs);
    drawLine(p8Duration, sessionColor(ILI9341_WHITE), buf);
  } else {
    drawLine(p8Duration, COLOR_DARKGREY, "-");
  }
  drawLine(p8Phase, sessionColor(ILI9341_WHITE),
           session.lastPhase < PH_COUNT ? PHASE_NAMES[session.lastPhase] : "-");
  drawPageFooter(p8Footer);
}

// ---- Page 9: message ladder (backlog_0021) ----------------------------------
//
// Car on the left rail, charger on the right, newest at the bottom. Repeats of the same exchange
// collapse into one row with a count and a two-way arrow, so the 89 CurrentDemand pairs of a
// charge loop take one line instead of pushing the session start off the page.
static const int16_t P9_RAIL_L = 44;
static const int16_t P9_RAIL_R = 276;
static const int16_t P9_ROW_Y = 38;
static const int16_t P9_ROW_PITCH = 12;

TextLine p9Rows[LADDER_ROWS];
TextLine p9Footer = {6, 229, 1};
uint8_t p9DrawnDir[LADDER_ROWS];
uint32_t p9DrawnVersion = 0xFFFFFFFF;
uint32_t p9LastDrawMs = 0;

void initPage9Lines() {
  for (uint8_t i = 0; i < LADDER_ROWS; i++) {
    p9Rows[i] = TextLine{P9_RAIL_L + 4, (int16_t)(P9_ROW_Y + i * P9_ROW_PITCH), 1, 38};
    p9DrawnDir[i] = 0xFF;
  }
}

// The arrow under one row's name: a line between the rails with a head at the destination.
void drawLadderArrow(int16_t y, uint8_t dir, uint16_t color) {
  tft.fillRect(P9_RAIL_L, y, P9_RAIL_R - P9_RAIL_L + 1, 4, ILI9341_BLACK);
  tft.drawFastHLine(P9_RAIL_L, y + 1, P9_RAIL_R - P9_RAIL_L, color);
  if (dir == DIR_CHARGER || dir == DIR_BOTH) {  // pointing at the car
    tft.fillTriangle(P9_RAIL_L, y + 1, P9_RAIL_L + 5, y - 1, P9_RAIL_L + 5, y + 3, color);
  }
  if (dir == DIR_CAR || dir == DIR_BOTH) {  // pointing at the charger
    tft.fillTriangle(P9_RAIL_R, y + 1, P9_RAIL_R - 5, y - 1, P9_RAIL_R - 5, y + 3, color);
  }
}

void drawPage9() {
  char buf[48];
  uint32_t now = millis();
  // The count of the newest row ticks on every message; 5 Hz is enough to follow it.
  if (ladderVersion == p9DrawnVersion || now - p9LastDrawMs < 200) {
    drawPageFooter(p9Footer);
    return;
  }
  p9DrawnVersion = ladderVersion;
  p9LastDrawMs = now;

  for (uint8_t i = 0; i < LADDER_ROWS; i++) {
    int16_t y = P9_ROW_Y + i * P9_ROW_PITCH;
    if (i >= ladderCount) {
      drawLine(p9Rows[i], COLOR_DARKGREY, "");
      if (p9DrawnDir[i] != 0xFF) {
        tft.fillRect(P9_RAIL_L, y + 8, P9_RAIL_R - P9_RAIL_L + 1, 4, ILI9341_BLACK);
        p9DrawnDir[i] = 0xFF;
      }
      continue;
    }
    const LadderEntry &e = ladder[i];
    if (e.count > 1) {
      snprintf(buf, sizeof(buf), "%.25s x%u", e.name, e.count);
    } else {
      strlcpy(buf, e.name, sizeof(buf));
    }
    uint16_t color = e.dir == DIR_CAR ? ILI9341_YELLOW
                                      : (e.dir == DIR_CHARGER ? ILI9341_GREEN : ILI9341_CYAN);
    color = sessionColor(color);
    drawLine(p9Rows[i], color, buf);
    if (p9DrawnDir[i] != e.dir) {
      drawLadderArrow(y + 8, e.dir, color);
      p9DrawnDir[i] = e.dir;
    }
  }
  drawPageFooter(p9Footer);
}

// Everything below the header is redrawn from scratch after a page switch.
void showPage(uint8_t p) {
  static TextLine *const LINES[] = {
      &statusLine, &networkLine, &modemLines[0], &modemLines[1], &modemLines[2],
      &targetLabelLine, &targetVoltLine, &targetCurrLine, &presentLabelLine, &presentVoltLine,
      &presentCurrLine, &socLabelLine, &socValueLine, &msgLine, &rcLine, &counterLine,
      &p2SdpCar, &p2SdpEvse, &p2SdpAddr, &p2AphHead, &p2AphHint, &p2AphLines[0], &p2AphLines[1],
      &p2AphLines[2], &p2AphLines[3], &p2AphLines[4], &p2AphResult, &p2AphSelected, &p2Footer,
      &p3CarHead, &p3CarMode, &p3CarMaxHead, &p3CarVolt, &p3CarCurr, &p3CarSoc, &p3CarReady,
      &p3CarSocTargets, &p3CarErrorHead, &p3CarError, &p3EvseHead, &p3EvseProc, &p3EvseMaxHead,
      &p3EvseVolt, &p3EvseCurr, &p3EvsePower, &p3EvseMinHead, &p3EvseMin, &p3EvseRcHead,
      &p3EvseRc, &p3Footer, &headerHintLine, &titleHintLine, &p4Footer,
      &p5Sounds, &p5Duration, &p5Average, &p5Peak, &p5AxisHead, &p5Footer,
      &p6Volt, &p6Curr, &p6Power, &p6ScaleV, &p6ScaleI, &p6Footer,
      &p7CarHead, &p7Car, &p7EvseHead, &p7Evse, &p7SessionHead, &p7Session,
      &p7NidHead, &p7Nid, &p7NmkHead, &p7Nmk1, &p7Nmk2, &p7Footer,
      &p8EnergyHead, &p8Energy, &p8ChargeHead, &p8Charge, &p8AvgHead, &p8PeakHead, &p8Avg,
      &p8Peak, &p8DurationHead, &p8PhaseHead, &p8Duration, &p8Phase, &p8Footer, &p9Footer};
  page = p;
  tft.fillRect(0, HEADER_LINE_Y + 1, tft.width(), tft.height() - HEADER_LINE_Y - 1, ILI9341_BLACK);
  // The header differs per page (project name + uptime, or page 3's own heading), so it is
  // repainted on every switch - including the TLS hint lines, which live in it.
  if (page == 2) {
    drawPage2Header();
  } else if (page > 2) {
    drawTitleHeader(pageTitle(page));
  } else {
    drawMainHeader();
    uptimeShown = UINT64_MAX;  // force the uptime back onto the screen
  }
  for (TextLine *line : LINES) {
    line->drawn = false;
  }
  for (uint8_t i = 0; i < PH_COUNT; i++) {
    p4Names[i].drawn = false;
    p4Times[i].drawn = false;
    p4DrawnWidth[i] = -1;
  }
  for (uint8_t i = 0; i < LADDER_ROWS; i++) {
    p9Rows[i].drawn = false;
    p9DrawnDir[i] = 0xFF;
  }
  // Everything the graphical pages cache about what is already on the screen.
  p5DrawnKey = 0;
  p6ScaleVolt10 = 0;
  p6ScaleCurr10 = 0;
  p6DrawnCount = 0;
  p9DrawnVersion = 0xFFFFFFFF;
  p9LastDrawMs = 0;
  meterShown = -1;
  if (page == 1) {
    drawMainStatic();
    logDirty = true;
  } else if (page == 2) {
    // divider between the SDP block and the schema block
    tft.drawFastHLine(0, PAGE2_DIVIDER_Y, tft.width(), COLOR_DARKGREY);
  } else if (page == 3) {
    // the heading is in the header, so only the column divider is left
    tft.drawFastVLine(P3_DIVIDER_X, HEADER_LINE_Y + 1, 222 - HEADER_LINE_Y, COLOR_DARKGREY);
  } else if (page == 9) {
    // the two rails of the ladder, with the ends they belong to
    tft.setTextSize(1);
    tft.setTextColor(COLOR_DARKGREY, ILI9341_BLACK);
    tft.setCursor(P9_RAIL_L - 9, 26);
    tft.print("CAR");
    tft.setCursor(P9_RAIL_R - 21, 26);
    tft.print("CHARGER");
    tft.drawFastVLine(P9_RAIL_L, 36, 184, COLOR_DARKGREY);
    tft.drawFastVLine(P9_RAIL_R, 36, 184, COLOR_DARKGREY);
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
      showPage(page >= PAGE_COUNT ? 1 : page + 1);  // the board has only this one button
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

  // SLAC (backlog_0014/0015/0017). The whole run happens BEFORE the match that opens the
  // session, so its start and its attenuation profile are kept outside the session data and
  // copied in by beginSession().
  uint32_t mmeMs = millis();
  if (mm >= 0x6064 && mm < 0x607C) {
    if (mm == 0x6064 && (slacStartMs == 0 || mmeMs - slacLastMs > 5000)) {
      slacStartMs = mmeMs;  // the first CM_SLAC_PARAM.REQ of a run, not one of its retries
    }
    slacLastMs = mmeMs;
    if (session.startMs != 0) {
      phaseMark(PH_SLAC, mmeMs);
    }
  } else if (session.startMs != 0 && session.phaseStart[PH_SDP] == 0) {
    phaseMark(PH_JOIN, mmeMs);  // CC_ASSOC, SET_KEY, TEI map: still forming the network
  }
  if (homeplug::parseAttenCharInd(frame, len, pendingAtten)) {
    hasPendingAtten = true;
    if (session.startMs != 0) {  // a profile measured inside a running session
      session.hasAtten = true;
      session.atten = pendingAtten;
    }
    uint16_t sum = 0, peak = 0;
    for (uint8_t i = 0; i < pendingAtten.numGroups; i++) {
      sum += pendingAtten.attenuation[i];
      if (pendingAtten.attenuation[i] > peak) peak = pendingAtten.attenuation[i];
    }
    Serial.printf("SLAC atten: sounds %u groups %u avg %u max %u dB\n", pendingAtten.numSounds,
                  pendingAtten.numGroups, pendingAtten.numGroups ? sum / pendingAtten.numGroups : 0,
                  peak);
  }

  // A SLAC match starts a new session: drop the previous session's setup, so nothing of it can
  // turn coloured again if this session's SDP is missed (owner decision 2026-09-17 - clearing on
  // the handshake request would be too late, that comes after SDP).
  if ((mm & ~0x0003) == 0x607C) {
    sessionSetup = SessionSetup();
    beginSession(mmeMs);
    // Only the CNF carries both MACs, the NID and the NMK - the REQ that precedes it does not.
    homeplug::SlacMatch matched;
    if (homeplug::parseSlacMatchCnf(frame, len, matched)) {
      session.hasMatch = true;
      session.match = matched;
      char pev[18], evse[18], nid[16], nmk[36];
      formatMac(pev, sizeof(pev), matched.pevMac);
      formatMac(evse, sizeof(evse), matched.evseMac);
      formatHex(nid, sizeof(nid), matched.nid, 7);
      formatHex(nmk, sizeof(nmk), matched.nmk, 16);
      Serial.printf("SLAC match: pev %s evse %s nid %s nmk %s\n", pev, evse, nid, nmk);
    }
    panelDirty = true;
  }

  // Other MMEs, e.g. SLAC between car and charger
  traffic.mme++;
  char name[24];
  homeplug::describeMmtype(homeplug::mmtype(frame), name, sizeof(name));
  if (mm >= 0x6064 && mm <= 0x607F) {
    ladderAdd(name, slacDirection(mm), mmeMs);  // page 9 shows SLAC, not every housekeeping MME
  }
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
    phaseMark(PH_SDP, s.lastActivityMs);
    ladderAdd("SDPReq", DIR_CAR, s.lastActivityMs);
    snprintf(text, textLen, "SDP req %s %s", sdpSecurityName(d[0]), sdpTransportName(d[1]));
  } else if (type == 0x9001 && payloadLen == 20) {
    s.hasSdpRes = true;
    memcpy(s.evseIp, d, 16);
    s.evsePort = (d[16] << 8) | d[17];
    s.resSecurity = d[18];
    s.resTransport = d[19];
    s.lastActivityMs = millis();
    phaseMark(PH_SDP, s.lastActivityMs);
    ladderAdd("SDPRes", DIR_CHARGER, s.lastActivityMs);
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

  // The rows of pages 4 and 9 share one geometry each - built here instead of written out.
  initPage4Lines();
  initPage9Lines();

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
  static uint32_t lastPageTickMs = 0;
  static uint32_t lastLogDraw = 0;
  static uint32_t lastPoll = 0;
  static uint32_t lastCheck = millis();
  static uint32_t lastRequest = 0;
  static uint32_t lastStatus = 0;
  static uint32_t lastPanelRefresh = 0;


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
  uint8_t requestedPage = diag.takePageRequest();  // "page <n>" over the serial line
  if (requestedPage >= 1 && requestedPage <= PAGE_COUNT && requestedPage != page) {
    showPage(requestedPage);
  }
  sampleVoltageCurrent(now);  // backlog_0016: the chart's time base, independent of the page shown

  if (panelDirty) {
    panelDirty = false;
    drawHeaderHint();
    switch (page) {
      case 1: drawPanel(); break;
      case 2: drawPage2(); break;
      case 3: drawPage3(); break;
      case 4: drawPage4(); break;
      case 5: drawPage5(); break;
      case 6: drawPage6(); break;
      case 7: drawPage7(); break;
      case 8: drawPage8(); break;
      default: drawPage9(); break;
    }
  }

  if (page == 1 && logDirty && now - lastLogDraw >= LOG_DRAW_INTERVAL_MS) {
    logDirty = false;
    lastLogDraw = now;
    drawLog();
  }

  // Pages 4 and 6 keep moving between messages (a running phase gets longer, the sweep
  // advances), so they are refreshed on a timer instead of only when a frame arrives.
  if ((page == 4 || page == 6) && now - lastPageTickMs >= 200) {
    lastPageTickMs = now;
    panelDirty = true;
  }

  // Only the main page has the uptime box - every other page uses the header for its heading.
  if (page == 1 && now - lastUptimeUpdate >= UPDATE_INTERVAL_MS) {
    lastUptimeUpdate = now;
    uint64_t centiseconds = uptimeCentiseconds();
    if (centiseconds != uptimeShown) {
      drawUptime(centiseconds);
      uptimeShown = centiseconds;
    }
  }
}
