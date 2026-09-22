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

// VS_SNIFFER.REQ (0xA034) broadcast; 1-byte payload, enable=true starts the .IND stream
// (0xA036), enable=false stops it. Stock vendor MME, no firmware patch needed - see
// qca7000-transparency repo's helpers/vs_sniffer.py for how it was found/confirmed on the old
// DUT. Returns the frame length.
uint16_t composeVsSnifferReq(uint8_t *frame, const uint8_t sourceMac[6], bool enable);

// MMTYPE of VS_SNIFFER.IND (0xA036): per-delimiter (beacon/SOF) metadata, no application
// payload. Streams continuously (~30-48/s) whenever the MAC observes RF activity while
// enabled - treat as a "beacon activity" heartbeat, not as a frame worth logging individually.
constexpr uint16_t MMTYPE_VS_SNIFFER_IND = 0xA036;

// True if frame is a VS_SNIFFER.IND for a beacon the local modem RECEIVED. The stream also
// reports every other delimiter, including the local modem's own transmissions (our periodic
// broadcasts), which must not count as "a CCo is in sight". Wire layout, read off real records
// on the bench 2026-09-15:
//   0x15      0 = own transmission, 1 = received (inferred: the 0 records were SOFs with STEI 0
//             and DTEI 0xFF, i.e. broadcasts from an unassociated station - the local modem)
//   0x16      system time, 8 bytes LE;  0x1E  beacon time, 4 bytes
//   0x22      HomePlug AV frame control, 16 bytes; delimiter type = low 3 bits of the first
//             byte (0 = beacon, 1 = SOF)
//   0x32      beacon payload, starting with the 7-byte NID
bool isReceivedBeaconInd(const uint8_t *frame, uint16_t len);

}  // namespace homeplug
