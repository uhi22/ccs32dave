// HomePlug AV management messages (MMEs).
// Ported from github.com/uhi22/ccs32berta (homeplug.ino).

#pragma once

#include <Arduino.h>

namespace homeplug {

constexpr uint16_t ETHERTYPE_HOMEPLUG = 0x88E1;
constexpr uint16_t ETHERTYPE_IPV6 = 0x86DD;
constexpr uint16_t MIN_ETH_FRAME_LEN = 60;
constexpr uint8_t VERSION_MAX_LEN = 64;

struct SoftwareVersion {
  uint8_t mac[6];                        // MAC of the answering modem
  char version[VERSION_MAX_LEN + 1];     // e.g. "MAC-QCA7005-1.1.0.730-04-20140815-CS"
};

// Content of CM_NW_INFO.CNF: the networks (AVLNs) the answering station is member of
struct NetworkInfo {
  uint8_t mac[6];         // MAC of the answering modem
  uint8_t numNetworks;    // 0 = not joined
  // First network, valid if numNetworks > 0
  uint8_t nid[7];
  uint8_t snid;
  uint8_t tei;
  uint8_t role;           // 0 = STA, 1 = PCo, 2 = CCo
  uint8_t ccoMac[6];
};

uint16_t etherType(const uint8_t *frame);

// MMTYPE of a HomePlug frame (base type + REQ/CNF/IND/RSP in the low 2 bits)
uint16_t mmtype(const uint8_t *frame);

// Writes a readable MMTYPE, e.g. "SLAC_PARAM.REQ" or "MME 6044.CNF", into buf
void describeMmtype(uint16_t mmtype, char *buf, size_t bufLen);

// Builds a broadcast GET_SW.REQ (Qualcomm vendor MME) into frame, which must
// hold MIN_ETH_FRAME_LEN bytes. Returns the frame length.
uint16_t composeGetSwReq(uint8_t *frame, const uint8_t sourceMac[6]);

// Returns true and fills out if frame is a valid GET_SW.CNF
bool parseGetSwCnf(const uint8_t *frame, uint16_t len, SoftwareVersion &out);

// Builds a broadcast CM_NW_INFO.REQ into frame (MIN_ETH_FRAME_LEN bytes).
// Returns the frame length.
uint16_t composeNwInfoReq(uint8_t *frame, const uint8_t sourceMac[6]);

// Returns true and fills out if frame is a valid CM_NW_INFO.CNF
bool parseNwInfoCnf(const uint8_t *frame, uint16_t len, NetworkInfo &out);

// ---- Qualcomm vendor MMEs for diagnosis (special firmware on the local modem) ----

constexpr uint16_t MEM_MAX_LEN = 1024;  // VS_RD_MEM / VS_WR_MEM limit per request

// VS_RD_MEM.REQ (0xA008) to dst. Returns the frame length.
uint16_t composeRdMemReq(uint8_t *frame, const uint8_t dst[6], const uint8_t src[6],
                         uint32_t addr, uint32_t len);

// VS_WR_MEM.REQ (0xA004) to dst; frame must hold 28 + n bytes (at least 60).
// addr and n must be multiples of 4. Returns the frame length.
uint16_t composeWrMemReq(uint8_t *frame, const uint8_t dst[6], const uint8_t src[6],
                         uint32_t addr, const uint8_t *data, uint16_t n);

struct MemReadResult {
  uint8_t mac[6];       // answering modem
  uint8_t status;       // 0 = OK
  uint32_t addr;
  uint32_t len;
  const uint8_t *data;  // points into the frame, len bytes
};

// VS_RD_MEM.CNF (0xA009)
bool parseRdMemCnf(const uint8_t *frame, uint16_t len, MemReadResult &out);

// VS_WR_MEM.CNF (0xA005); status 0 = OK
bool parseWrMemCnf(const uint8_t *frame, uint16_t len, uint8_t mac[6], uint8_t &status);

// VS_NW_INFO.REQ (0xA038) to dst. Returns the frame length.
uint16_t composeVsNwInfoReq(uint8_t *frame, const uint8_t dst[6], const uint8_t src[6]);

struct VsNetworkInfo {
  static constexpr uint8_t MAX_STATIONS = 8;
  uint8_t mac[6];       // answering modem
  uint8_t numAvlns;     // 0 = not joined
  uint8_t ownTei;
  uint8_t role;
  uint8_t ccoTei;
  uint8_t ccoMac[6];
  uint8_t numStations;  // stations stored (<= MAX_STATIONS)
  struct Station {
    uint8_t mac[6];
    uint8_t tei;
  } stations[MAX_STATIONS];
};

// VS_NW_INFO.CNF (0xA039)
bool parseVsNwInfoCnf(const uint8_t *frame, uint16_t len, VsNetworkInfo &out);

}  // namespace homeplug
