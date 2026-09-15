#include "diag.h"
#include "homeplug.h"

// Requests go to the local modem only (not broadcast), so they are not sent on
// the powerline and the other modems don't answer
static const uint8_t LOCAL_MODEM_MAC[6] = {0x04, 0x65, 0x65, 0xFF, 0xFF, 0x11};

static constexpr uint32_t TIMEOUT_MS = 700;
static constexpr uint8_t MAX_RETRIES = 2;

// Ping-pong control block of the special firmware (self-arming both-direction transparency)
static constexpr uint32_t PP_BLOCK_ADDR = 0x18000;
static constexpr uint32_t PP_BLOCK_LEN = 0x44;
static constexpr uint32_t OWN_TEI_ADDR = 0x8CAE4;

static uint32_t le32(const uint8_t *p) {
  return p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24);
}

static int hexNibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

Diag::Diag(Qca7000 &qca, Stream &io, const uint8_t hostMac[6]) : _qca(qca), _io(io) {
  memcpy(_hostMac, hostMac, 6);
}

void Diag::loop(uint32_t nowMs) {
  if (_pending != Kind::None) {
    if (nowMs - _sentMs < TIMEOUT_MS) {
      return;
    }
    if (_retries < MAX_RETRIES) {
      _retries++;
      sendPending();
      return;
    }
    static const char *const TAG[] = {"", "RD", "WR", "NWI", "PP", "PP"};
    _io.printf("%s 0x%X timeout\n", TAG[(int)_pending], (unsigned)_addr);
    _pending = Kind::None;
  }

  // Commands are read only while nothing is pending; the rest waits in the UART buffer
  while (_pending == Kind::None && _io.available()) {
    char c = _io.read();
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      _line[_lineLen] = '\0';
      _lineLen = 0;
      handleLine(_line);
    } else if (_lineLen < sizeof(_line) - 1) {
      _line[_lineLen++] = c;
    }
  }
}

void Diag::handleLine(char *line) {
  char *cmd = strtok(line, " ");
  if (!cmd) {
    return;
  }
  char *a1 = strtok(nullptr, " ");
  char *a2 = strtok(nullptr, " ");

  if (strcmp(cmd, "rd") == 0 && a1 && a2) {
    _addr = strtoul(a1, nullptr, 0);
    _len = strtoul(a2, nullptr, 0);
    if (_len == 0 || _len > homeplug::MEM_MAX_LEN) {
      _io.println("RD ERR len");
      return;
    }
    _pending = Kind::Read;
  } else if (strcmp(cmd, "wr") == 0 && a1 && a2) {
    _addr = strtoul(a1, nullptr, 0);
    size_t hexLen = strlen(a2);
    _len = hexLen / 2;
    bool ok = hexLen % 2 == 0 && _len > 0 && _len <= homeplug::MEM_MAX_LEN &&
              _len % 4 == 0 && _addr % 4 == 0;
    for (uint32_t i = 0; ok && i < _len; i++) {
      int hi = hexNibble(a2[2 * i]);
      int lo = hexNibble(a2[2 * i + 1]);
      ok = hi >= 0 && lo >= 0;
      _data[i] = (hi << 4) | lo;
    }
    if (!ok) {
      _io.println("WR ERR args");
      return;
    }
    _pending = Kind::Write;
  } else if (strcmp(cmd, "nwi") == 0) {
    _addr = 0;
    _pending = Kind::NwInfo;
  } else if (strcmp(cmd, "pp") == 0) {
    _addr = PP_BLOCK_ADDR;
    _len = PP_BLOCK_LEN;
    _pending = Kind::PingPongBlock;
  } else if (strcmp(cmd, "stat") == 0) {
    // SPI counters and the modem's SPI registers (see Qca7000::dumpRegisters)
    _io.printf("STAT tx=%u rx=%u err=%u txrej=%u", (unsigned)_qca.txFrames(),
               (unsigned)_qca.rxFrames(), (unsigned)_qca.errors(), (unsigned)_qca.txRejected());
    for (uint8_t reg = 0x00; reg <= 0x1B; reg++) {
      _io.printf(" r%02X=%04X", reg, _qca.readRegister(reg));
    }
    _io.println();
    return;
  } else if (strcmp(cmd, "qreset") == 0) {
    _qca.softReset();
    _io.println("QRESET done");
    return;
  } else if (strcmp(cmd, "bcast") == 0 && a1) {
    _periodicRequests = atoi(a1) != 0;
    _io.printf("BCAST %d\n", _periodicRequests);
    return;
  } else if (strcmp(cmd, "log") == 0 && a1) {
    _logTraffic = atoi(a1) != 0;
    _io.printf("LOG %d\n", _logTraffic);
    return;
  } else {
    _io.println("HELP rd <addr> <len> | wr <addr> <hex> | nwi | pp | bcast 0|1 | log 0|1");
    return;
  }
  _retries = 0;
  sendPending();
}

void Diag::sendPending() {
  uint8_t frame[28 + homeplug::MEM_MAX_LEN];
  uint16_t len = 0;
  switch (_pending) {
    case Kind::Read:
    case Kind::PingPongBlock:
    case Kind::PingPongOwnTei:
      len = homeplug::composeRdMemReq(frame, LOCAL_MODEM_MAC, _hostMac, _addr, _len);
      break;
    case Kind::Write:
      len = homeplug::composeWrMemReq(frame, LOCAL_MODEM_MAC, _hostMac, _addr, _data, _len);
      break;
    case Kind::NwInfo:
      len = homeplug::composeVsNwInfoReq(frame, LOCAL_MODEM_MAC, _hostMac);
      break;
    case Kind::None:
      return;
  }
  _qca.sendEthFrame(frame, len);
  _sentMs = millis();
}

bool Diag::onFrame(const uint8_t *frame, uint16_t len) {
  if (_pending == Kind::None) {
    return false;
  }

  if (_pending == Kind::Read || _pending == Kind::PingPongBlock ||
      _pending == Kind::PingPongOwnTei) {
    homeplug::MemReadResult r;
    if (!homeplug::parseRdMemCnf(frame, len, r) || memcmp(r.mac, LOCAL_MODEM_MAC, 6) != 0) {
      return false;
    }
    if (_pending == Kind::Read) {
      _io.printf("RD 0x%X st=%u ", (unsigned)_addr, r.status);
      for (uint32_t i = 0; i < r.len; i++) {
        _io.printf("%02x", r.data[i]);
      }
      _io.println();
      _pending = Kind::None;
    } else if (_pending == Kind::PingPongBlock) {
      if (r.status != 0 || r.len < PP_BLOCK_LEN) {
        _io.printf("PP ERR st=%u\n", r.status);
        _pending = Kind::None;
        return true;
      }
      memcpy(_ppBlock, r.data, PP_BLOCK_LEN);
      _addr = OWN_TEI_ADDR;
      _len = 4;
      _pending = Kind::PingPongOwnTei;
      _retries = 0;
      sendPending();
    } else {
      finishPingPong(r.status == 0 && r.len >= 4 ? r.data : nullptr);
      _pending = Kind::None;
    }
    return true;
  }

  if (_pending == Kind::Write) {
    uint8_t mac[6];
    uint8_t status;
    if (!homeplug::parseWrMemCnf(frame, len, mac, status) ||
        memcmp(mac, LOCAL_MODEM_MAC, 6) != 0) {
      return false;
    }
    _io.printf("WR 0x%X st=%u\n", (unsigned)_addr, status);
    _pending = Kind::None;
    return true;
  }

  // Kind::NwInfo
  homeplug::VsNetworkInfo info;
  if (!homeplug::parseVsNwInfoCnf(frame, len, info) ||
      memcmp(info.mac, LOCAL_MODEM_MAC, 6) != 0) {
    return false;
  }
  _io.printf("NWI avlns=%u own=%u role=%u cco=%u ccomac=%02x%02x%02x%02x%02x%02x sta=",
             info.numAvlns, info.ownTei, info.role, info.ccoTei, info.ccoMac[0],
             info.ccoMac[1], info.ccoMac[2], info.ccoMac[3], info.ccoMac[4], info.ccoMac[5]);
  for (uint8_t i = 0; i < info.numStations; i++) {
    const uint8_t *m = info.stations[i].mac;
    _io.printf("%s%02x%02x%02x%02x%02x%02x/%u", i ? "," : "", m[0], m[1], m[2], m[3], m[4],
               m[5], info.stations[i].tei);
  }
  _io.println();
  _pending = Kind::None;
  return true;
}

void Diag::finishPingPong(const uint8_t *ownTei) {
  const uint8_t *b = _ppBlock;
  // state/sentinel/seeds exist only in the self-arming build (magic 0x5AC1AC05)
  _io.printf("PP magic=0x%08X matches=%u frames=%u setkey=%u armed=%u teia=%u teib=%u "
             "state=%u flips=%u data=%u sentinel=0x%08X seeds=%u own=",
             (unsigned)le32(&b[0x00]), (unsigned)le32(&b[0x04]), (unsigned)le32(&b[0x08]),
             (unsigned)le32(&b[0x2C]), b[0x30], b[0x31], b[0x32], b[0x33],
             (unsigned)le32(&b[0x34]), (unsigned)le32(&b[0x38]), (unsigned)le32(&b[0x3C]),
             (unsigned)le32(&b[0x40]));
  if (ownTei) {
    _io.printf("%u\n", ownTei[0]);
  } else {
    _io.println("?");
  }
}
