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

#define LOG_TRAFFIC 1  // print every received frame on the serial port

// Locally administered MAC used as source of our requests
static const uint8_t MY_MAC[6] = {0xFE, 0xED, 0xBE, 0xEF, 0xAF, 0xFE};

Adafruit_ILI9341 tft(TFT_CS, TFT_DC, TFT_RST);

SPIClass qcaSpi(HSPI);
Qca7000 qca(qcaSpi, QCA_CS, QCA_SPI_FREQUENCY);

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

// Frame log, drawn off-screen and sent as one bitmap
static const uint8_t LOG_LINES = 14;
static const int16_t LOG_PITCH = 10;
static const int16_t LOG_X = TEXT_X;
static const int16_t LOG_Y = 80;
static const int16_t LOG_W = 51 * 6;
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
    char mac[9] = "";
    if (e.hasMac) {
      snprintf(mac, sizeof(mac), "%02X:%02X:%02X", e.mac[0], e.mac[1], e.mac[2]);
    }
    char count[8] = "";
    if (e.count > 1) {
      snprintf(count, sizeof(count), " x%u", e.count);
    }
    char buf[64];
    snprintf(buf, sizeof(buf), "%5lu.%02u %-20.20s %-8s%s",
             (unsigned long)(e.centiseconds / 100), (unsigned)(e.centiseconds % 100),
             e.text, mac, count);
    logCanvas.setTextColor(e.color);
    logCanvas.setCursor(0, i * LOG_PITCH);
    logCanvas.print(buf);
  }

  tft.drawRGBBitmap(LOG_X, LOG_Y, logCanvas.getBuffer(), LOG_W, LOG_H);
}

// ---- Screen -----------------------------------------------------------------

void drawStaticScreen() {
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextWrap(false);

  tft.setCursor(6, 3);
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_YELLOW);
  tft.print("ESP32-S3 QCA7005");

  tft.drawRect(UPTIME_X - 4, UPTIME_Y - 4, UPTIME_W + 8, UPTIME_H + 8, COLOR_DARKGREY);

  tft.drawFastHLine(0, HEADER_LINE_Y, tft.width(), COLOR_DARKGREY);
  tft.drawFastHLine(0, LOG_Y - 4, tft.width(), COLOR_DARKGREY);
  tft.drawFastHLine(0, LOG_Y + LOG_H + 3, tft.width(), COLOR_DARKGREY);
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
#if LOG_TRAFFIC
  char src[18];
  formatMac(src, sizeof(src), &frame[6]);
  Serial.printf("RX %s from %s, %u bytes\n", name, src, len);
#endif
}

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
  addLog(text, ILI9341_GREEN, &frame[6]);
#if LOG_TRAFFIC
  Serial.printf("RX %s, %u bytes\n", text, len);
#endif
}

void onEthFrame(const uint8_t *frame, uint16_t len) {
  if (len < 14) {
    return;
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
  Serial.begin(115200);

  if (TFT_BL >= 0) {
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, HIGH);
  }

  SPI.begin(TFT_SCLK, TFT_MISO, TFT_MOSI, TFT_CS);
  tft.begin(SPI_FREQUENCY);
  tft.setRotation(1);   // landscape, 320x240
  drawStaticScreen();

  qca.begin(QCA_SCLK, QCA_MISO, QCA_MOSI);
  checkModem();
  drawPanel();

  Serial.println("ESP32-S3 TFT + QCA7005 demo started");
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
    if (modem.requestPending || now - lastRequest >= QCA_REQUEST_INTERVAL_MS) {
      modem.requestPending = false;
      lastRequest = now;
      sendRequests(now);
    }
    if (now - lastPoll >= QCA_POLL_INTERVAL_MS) {
      lastPoll = now;
      qca.poll(onEthFrame);
    }
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
