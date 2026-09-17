// Diagnosis over the serial port: reads and writes the local modem's RAM and
// queries its network info, using the Qualcomm vendor MMEs of the special
// firmware (VS_RD_MEM, VS_WR_MEM, VS_NW_INFO). Replaces the Ethernet-based
// tools (rd_mem.py / wr_mem.py / nw_info.py / pingpong_ctl.py) used to develop
// that firmware on its original testbench, which had a direct Ethernet link
// to the modem - this board doesn't.
//
// One command per line, one answer line per command. Every answer line starts
// with a fixed tag, so a script can wait for it:
//
//   rd <addr> <len>      RD <addr> st=<status> <hex bytes>    (len <= 1024)
//   wr <addr> <hex>      WR <addr> st=<status>                (addr, len multiple of 4)
//   nwi                  NWI avlns=<n> own=<tei> role=<r> cco=<tei> ccomac=<mac> sta=<mac>/<tei>,...
//   pp                   PP magic=.. matches=.. frames=.. setkey=.. armed=.. teia=.. teib=.. state=..
//                           flips=.. data=.. sentinel=.. seeds=.. own=..
//                        (state: 1 not joined, 2 no CCo TEI, 3 no other station, 4 running;
//                         state/sentinel/seeds only in the self-arming build, magic 0x5AC1AC05)
//   stat                 STAT tx=.. rx=.. err=.. txrej=.. r00=<hex> .. r1B=<hex>   SPI counters + registers
//   qreset               QRESET done   restart the modem over SPI (SLAVE_RESET)
//   bcast 0|1            BCAST <0|1>   periodic GET_SW.REQ / CM_NW_INFO.REQ broadcasts
//   log 0|1              LOG <0|1>     one "F ..." line per received frame
//   sniff on|auto        SNIFF <on|auto>   test aid: keep the VS_SNIFFER stream on even while
//                                      joined (auto = on only while not joined)
//   help                 HELP ...
// A command without answer from the modem ends with "<TAG> <addr> timeout".
// Numbers may be given as hex (0x prefix) or decimal.

#pragma once

#include <Arduino.h>
#include "qca7000.h"

class Diag {
public:
  Diag(Qca7000 &qca, Stream &io, const uint8_t hostMac[6]);

  // Call from loop(): reads command lines and handles timeouts
  void loop(uint32_t nowMs);

  // Call for every received Ethernet frame. Returns true if the frame was an
  // answer to a diagnosis request (the caller should not count or log it).
  bool onFrame(const uint8_t *frame, uint16_t len);

  bool periodicRequests() const { return _periodicRequests; }
  bool logTraffic() const { return _logTraffic; }
  bool snifferForced() const { return _snifferForced; }

private:
  enum class Kind : uint8_t { None, Read, Write, NwInfo, PingPongBlock, PingPongOwnTei };

  void handleLine(char *line);
  void sendPending();
  void finishPingPong(const uint8_t *ownTei);

  Qca7000 &_qca;
  Stream &_io;
  uint8_t _hostMac[6];

  char _line[2200];  // "wr <addr> " + 1024 bytes as hex
  uint16_t _lineLen = 0;

  Kind _pending = Kind::None;
  uint32_t _addr = 0;
  uint32_t _len = 0;
  uint8_t _data[1024];
  uint8_t _retries = 0;
  uint32_t _sentMs = 0;
  uint8_t _ppBlock[0x40];

  bool _periodicRequests = true;
  bool _logTraffic = true;
  bool _snifferForced = false;
};
