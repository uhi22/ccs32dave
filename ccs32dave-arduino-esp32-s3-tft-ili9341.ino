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

#define QCA_SPI_FREQUENCY       2000000UL
#define QCA_POLL_INTERVAL_MS    10     // fetch received frames
#define QCA_CHECK_INTERVAL_MS   1000   // signature check
#define QCA_REQUEST_INTERVAL_MS 5000   // GET_SW.REQ and CM_NW_INFO.REQ
#define QCA_MAX_AGE_MS          (3 * QCA_REQUEST_INTERVAL_MS + 500)

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
  if (splashLocalVersion[0] || homeplug::etherType(frame) != homeplug::ETHERTYPE_HOMEPLUG) {
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
      requested = true;
    }
    if (requested && !shown) {
      qca.poll(splashHandleFrame);
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

void drawStaticScreen() {
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextWrap(false);

  tft.setCursor(6, 3);
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_YELLOW);
  tft.print("ccs32dave");

  tft.drawRect(UPTIME_X - 4, UPTIME_Y - 4, UPTIME_W + 8, UPTIME_H + 8, COLOR_DARKGREY);

  tft.drawFastHLine(0, HEADER_LINE_Y, tft.width(), COLOR_DARKGREY);
  tft.drawFastHLine(0, COLUMNS_TOP_Y, tft.width(), COLOR_DARKGREY);
  tft.drawFastHLine(0, COLUMNS_BOTTOM_Y, tft.width(), COLOR_DARKGREY);
  tft.drawFastVLine(PANEL_DIVIDER_X, COLUMNS_TOP_Y, COLUMNS_BOTTOM_Y - COLUMNS_TOP_Y, COLOR_DARKGREY);
}

// V2G values decoded from the reassembled DIN EXI traffic (backlog-0050/0051). Persist across
// messages - e.g. CurrentDemandRes doesn't repeat the target values, so the display keeps
// showing the last one seen until a newer message updates it (or the modem disappears).
struct V2gDisplay {
  bool hasTarget = false;
  float targetVoltage = 0, targetCurrent = 0;      // from CurrentDemandReq (PEV -> EVSE)
  bool hasPresent = false;
  float presentVoltage = 0, presentCurrent = 0;    // from CurrentDemandRes (EVSE -> PEV)
  bool hasSoc = false;
  int8_t soc = 0;                                  // DC_EVStatus.EVRESSSOC, from the PEV
  bool hasResponseCode = false;
  uint8_t responseCode = 0;
  char lastMsg[28] = "";
  bool lastOk = false;
  uint32_t decoded = 0, failed = 0;                // counters for the status line
};
V2gDisplay v2gDisplay;

// Applies one decode result (ok=false: v.msgName carries a short error tag instead) to the
// persistent display state above. Prints to serial only when the shown VALUES actually
// change (not on every message) - same convention as printModemTable() / the "Network:"
// line elsewhere in this file, so the TFT's content can be checked without seeing the screen.
void applyV2gValues(const V2gValues &v, bool ok) {
  if (ok) {
    v2gDisplay.decoded++;
    bool changed = false;
    if (v.hasTargetVoltage && v.targetVoltage != v2gDisplay.targetVoltage) { v2gDisplay.targetVoltage = v.targetVoltage; changed = true; }
    if (v.hasTargetCurrent && v.targetCurrent != v2gDisplay.targetCurrent) { v2gDisplay.targetCurrent = v.targetCurrent; changed = true; }
    if (v.hasTargetVoltage || v.hasTargetCurrent) v2gDisplay.hasTarget = true;
    if (v.hasPresentVoltage && v.presentVoltage != v2gDisplay.presentVoltage) { v2gDisplay.presentVoltage = v.presentVoltage; changed = true; }
    if (v.hasPresentCurrent && v.presentCurrent != v2gDisplay.presentCurrent) { v2gDisplay.presentCurrent = v.presentCurrent; changed = true; }
    if (v.hasPresentVoltage || v.hasPresentCurrent) v2gDisplay.hasPresent = true;
    if (v.hasSoc && v.soc != v2gDisplay.soc) { v2gDisplay.soc = v.soc; changed = true; }
    if (v.hasSoc) v2gDisplay.hasSoc = true;
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

void drawPanel() {
  char buf[65];

  drawLine(statusLine, modem.present ? ILI9341_GREEN : ILI9341_RED,
           modem.present ? "Modem OK" : "No modem");

  // Network join status from CM_NW_INFO.CNF of the local modem
  static const char *const ROLE[] = {"STA", "PCo", "CCo"};
  if (!modem.present || !network.valid) {
    drawLine(networkLine, COLOR_DARKGREY, "Net ?");
  } else if (network.info.numNetworks == 0) {
    drawLine(networkLine, ILI9341_YELLOW, "not joined");
  } else {
    snprintf(buf, sizeof(buf), "joined %s TEI%u",
             network.info.role < 3 ? ROLE[network.info.role] : "?",
             network.info.tei);
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
  if (v2gDisplay.hasTarget) {
    snprintf(buf, sizeof(buf), "%.1fV", v2gDisplay.targetVoltage);
  } else {
    strlcpy(buf, "-", sizeof(buf));
  }
  drawLine(targetVoltLine, ILI9341_YELLOW, buf);
  if (v2gDisplay.hasTarget) {
    snprintf(buf, sizeof(buf), "%.1fA", v2gDisplay.targetCurrent);
  } else {
    strlcpy(buf, "-", sizeof(buf));
  }
  drawLine(targetCurrLine, ILI9341_YELLOW, buf);

  drawLine(presentLabelLine, COLOR_DARKGREY, "PRESENT");
  if (v2gDisplay.hasPresent) {
    snprintf(buf, sizeof(buf), "%.1fV", v2gDisplay.presentVoltage);
  } else {
    strlcpy(buf, "-", sizeof(buf));
  }
  drawLine(presentVoltLine, ILI9341_GREEN, buf);
  if (v2gDisplay.hasPresent) {
    snprintf(buf, sizeof(buf), "%.1fA", v2gDisplay.presentCurrent);
  } else {
    strlcpy(buf, "-", sizeof(buf));
  }
  drawLine(presentCurrLine, ILI9341_GREEN, buf);

  drawLine(socLabelLine, COLOR_DARKGREY, "SoC");
  if (v2gDisplay.hasSoc) {
    snprintf(buf, sizeof(buf), "%d%%", v2gDisplay.soc);
  } else {
    strlcpy(buf, "-", sizeof(buf));
  }
  drawLine(socValueLine, ILI9341_CYAN, buf);

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
  homeplug::SoftwareVersion sw;
  if (homeplug::parseGetSwCnf(frame, len, sw)) {
    if (modems.update(sw, millis())) {
      printModemTable();
      panelDirty = true;
    }
    return;
  }

  homeplug::NetworkInfo info;
  if (homeplug::parseNwInfoCnf(frame, len, info)) {
    // All modems answer the broadcast; only the local modem's view counts
    if (ModemList::isLocal(info.mac)) {
      bool changed = !network.valid ||
                     network.info.numNetworks != info.numNetworks ||
                     network.info.tei != info.tei ||
                     network.info.role != info.role;
      if (changed) {
        Serial.printf("Network: %s, TEI %u, role %u\n",
                      info.numNetworks > 0 ? "joined" : "not joined",
                      info.tei, info.role);
        char buf[24];
        if (info.numNetworks > 0) {
          snprintf(buf, sizeof(buf), "** joined TEI %u", info.tei);
        } else {
          strlcpy(buf, "** not joined", sizeof(buf));
        }
        addLog(buf, ILI9341_YELLOW);
        panelDirty = true;
      }
      network.info = info;
      network.valid = true;
      network.receivedMs = millis();
    }
    return;
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
    addLog(text, ILI9341_GREEN, &frame[6]);
  }
  logFrame(frame, len, description);
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

void sendRequests(uint32_t now) {
  uint8_t frame[homeplug::MIN_ETH_FRAME_LEN];
  qca.sendEthFrame(frame, homeplug::composeGetSwReq(frame, MY_MAC));
  qca.sendEthFrame(frame, homeplug::composeNwInfoReq(frame, MY_MAC));

  // Forget modems and network info that stopped answering
  if (modems.expire(now, QCA_MAX_AGE_MS)) {
    printModemTable();
  }
  if (network.valid && now - network.receivedMs > QCA_MAX_AGE_MS) {
    network.valid = false;
  }
  panelDirty = true;  // counters changed
}

// ---- Arduino entry points ---------------------------------------------------

void setup() {
  Serial.setRxBufferSize(4096);  // "wr" command lines
  Serial.setTxBufferSize(8192);  // bursts of frame log lines
  Serial.begin(SERIAL_BAUD);

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
  static uint64_t lastShown = UINT64_MAX;

  uint32_t now = millis();

  if (now - lastCheck >= QCA_CHECK_INTERVAL_MS) {
    lastCheck = now;
    checkModem();
  }

  if (modem.present) {
    if (modem.requestPending ||
        (diag.periodicRequests() && now - lastRequest >= QCA_REQUEST_INTERVAL_MS)) {
      modem.requestPending = false;
      lastRequest = now;
      sendRequests(now);
    }
    if (now - lastPoll >= QCA_POLL_INTERVAL_MS) {
      lastPoll = now;
      qca.poll(onEthFrame);
    }
    diag.loop(now);
  }

  if (panelDirty) {
    panelDirty = false;
    drawPanel();
  }

  if (logDirty && now - lastLogDraw >= LOG_DRAW_INTERVAL_MS) {
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
