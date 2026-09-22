// Decodes one complete V2GTP-EXI payload (DIN 70121, plus the app-handshake that precedes it)
// into the handful of fields the TFT shows (backlog-0051). Wraps the OpenV2G decoder in
// src/exi/ (ported from C:\UwesTechnik\ccs32berta, decoder-only - see that project's
// GNU LGPL-licensed source for the codec itself).
#pragma once

#include <Arduino.h>

struct V2gValues {
  char msgName[28] = "";  // e.g. "CurrentDemandReq", or an error tag on failure

  bool hasTargetVoltage = false, hasTargetCurrent = false;
  float targetVoltage = 0, targetCurrent = 0;  // from CurrentDemandReq (EVTarget*)

  bool hasPresentVoltage = false, hasPresentCurrent = false;
  float presentVoltage = 0, presentCurrent = 0;  // from CurrentDemandRes (EVSEPresent*)

  bool hasSoc = false;
  int8_t soc = 0;  // DC_EVStatus.EVRESSSOC, percent - reported by the EV in its DC requests

  bool hasResponseCode = false;
  uint8_t responseCode = 0;  // dinresponseCodeType - 0 = OK, see dinEXIDatatypes.h

  // App handshake (backlog_0010). The decoder keeps at most 5 offered protocols.
  struct AppProtocol {
    char ns[48];  // e.g. "urn:din:70121:2012:MsgDef" (truncated if longer)
    uint8_t versionMajor, versionMinor, schemaId, priority;
  };
  static constexpr uint8_t MAX_APP_PROTOCOLS = 5;
  uint8_t appProtocolCount = 0;  // > 0: this was a supportedAppProtocolReq
  AppProtocol appProtocols[MAX_APP_PROTOCOLS];
  bool hasHandshakeResult = false;  // this was a supportedAppProtocolRes
  uint8_t handshakeResponseCode = 0;  // 0 OK, 1 OK with minor deviation, 2 failed
  bool hasSelectedSchema = false;
  uint8_t selectedSchemaId = 0;
};

// `data`/`len` is the EXI payload only (the V2GTP message minus its 8-byte header). Returns
// true and fills `out` for a recognized DIN body message or the app-handshake; false (with
// out.msgName set to a short error tag, e.g. "DIN err -110") otherwise - a wrong guess at the
// message boundary (see V2gtpReassembler) typically shows up here as a decode error.
bool decodeV2gExiPayload(const uint8_t *data, uint16_t len, V2gValues &out);
