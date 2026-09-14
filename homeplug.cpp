#include "homeplug.h"

namespace homeplug {

static constexpr uint16_t MMTYPE_GET_SW = 0xA000;
static constexpr uint16_t MMTYPE_NW_INFO = 0x6038;

static constexpr uint16_t MMTYPE_REQ = 0x0000;
static constexpr uint16_t MMTYPE_CNF = 0x0001;

static const uint8_t MAC_BROADCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static const uint8_t QUALCOMM_OUI[3] = {0x00, 0xB0, 0x52};

// Offsets in a HomePlug AV management frame
static constexpr uint8_t OFS_MMV = 14;
static constexpr uint8_t OFS_MMTYPE = 15;         // little endian
static constexpr uint8_t OFS_OUI = 17;            // vendor MMEs (MMV 0)
static constexpr uint8_t OFS_SW_VERSION_LEN = 22; // GET_SW.CNF
static constexpr uint8_t OFS_SW_VERSION = 23;     // GET_SW.CNF

// MMV 1 (HomePlug AV 1.1) has a 2 byte fragmentation header (FMI) after the MMTYPE
static constexpr uint8_t MMV_AV10 = 0x00;
static constexpr uint8_t MMV_AV11 = 0x01;

// CM_NW_INFO.CNF: NumNWs, then per network 18 bytes
static constexpr uint8_t NW_INFO_ENTRY_LEN = 18;

struct MmtypeName {
  uint16_t base;
  const char *name;
};

static const MmtypeName MMTYPE_NAMES[] = {
    {0x0030, "CC_ASSOC"},       {0x0038, "CC_SET_TEI_MAP"},
    {0x6000, "UNASSOCIATED_STA"}, {0x6004, "ENCRYPTED_PAYLOAD"},
    {0x6008, "SET_KEY"},        {0x600C, "GET_KEY"},
    {0x6038, "NW_INFO"},        {0x6044, "MME_ERROR"},
    {0x6048, "NW_STATS"},       {0x6064, "SLAC_PARAM"},
    {0x6068, "START_ATTEN_CHAR"}, {0x606C, "ATTEN_CHAR"},
    {0x6074, "MNBC_SOUND"},     {0x6078, "VALIDATE"},
    {0x607C, "SLAC_MATCH"},     {0x6084, "ATTEN_PROFILE"},
    {0xA000, "GET_SW"},
};

uint16_t etherType(const uint8_t *frame) {
  return (frame[12] << 8) | frame[13];
}

uint16_t mmtype(const uint8_t *frame) {
  return frame[OFS_MMTYPE] | (frame[OFS_MMTYPE + 1] << 8);
}

void describeMmtype(uint16_t mm, char *buf, size_t bufLen) {
  static const char *const SUFFIX[4] = {"REQ", "CNF", "IND", "RSP"};
  uint16_t base = mm & ~0x0003;
  for (const MmtypeName &entry : MMTYPE_NAMES) {
    if (entry.base == base) {
      snprintf(buf, bufLen, "%s.%s", entry.name, SUFFIX[mm & 0x0003]);
      return;
    }
  }
  snprintf(buf, bufLen, "MME %04X.%s", base, SUFFIX[mm & 0x0003]);
}

// Header up to and including the MMTYPE; returns the offset of the payload
static uint8_t composeHeader(uint8_t *frame, const uint8_t sourceMac[6],
                             uint8_t mmv, uint16_t mm) {
  memset(frame, 0, MIN_ETH_FRAME_LEN);
  memcpy(&frame[0], MAC_BROADCAST, 6);
  memcpy(&frame[6], sourceMac, 6);
  frame[12] = ETHERTYPE_HOMEPLUG >> 8;
  frame[13] = ETHERTYPE_HOMEPLUG & 0xFF;
  frame[OFS_MMV] = mmv;
  frame[OFS_MMTYPE] = mm & 0xFF;
  frame[OFS_MMTYPE + 1] = mm >> 8;
  return (mmv == MMV_AV10) ? 17 : 19;  // FMI bytes stay 0
}

// Offset of the payload in a received MME, depending on its MMV
static uint8_t payloadOffset(const uint8_t *frame) {
  return (frame[OFS_MMV] == MMV_AV10) ? 17 : 19;
}

uint16_t composeGetSwReq(uint8_t *frame, const uint8_t sourceMac[6]) {
  uint8_t ofs = composeHeader(frame, sourceMac, MMV_AV10, MMTYPE_GET_SW | MMTYPE_REQ);
  memcpy(&frame[ofs], QUALCOMM_OUI, 3);
  return MIN_ETH_FRAME_LEN;
}

bool parseGetSwCnf(const uint8_t *frame, uint16_t len, SoftwareVersion &out) {
  if (len < OFS_SW_VERSION || etherType(frame) != ETHERTYPE_HOMEPLUG ||
      mmtype(frame) != (MMTYPE_GET_SW | MMTYPE_CNF)) {
    return false;
  }
  uint8_t versionLen = frame[OFS_SW_VERSION_LEN];
  if (versionLen == 0 || versionLen > VERSION_MAX_LEN ||
      OFS_SW_VERSION + versionLen > len) {
    return false;
  }

  memcpy(out.mac, &frame[6], 6);
  for (uint8_t i = 0; i < versionLen; i++) {
    char c = frame[OFS_SW_VERSION + i];
    out.version[i] = (c < 0x20 || c > 0x7E) ? ' ' : c;  // unprintable -> space
  }
  // The string is zero-padded; drop the padding
  while (versionLen > 0 && out.version[versionLen - 1] == ' ') {
    versionLen--;
  }
  out.version[versionLen] = '\0';
  return true;
}

uint16_t composeNwInfoReq(uint8_t *frame, const uint8_t sourceMac[6]) {
  composeHeader(frame, sourceMac, MMV_AV11, MMTYPE_NW_INFO | MMTYPE_REQ);
  return MIN_ETH_FRAME_LEN;
}

bool parseNwInfoCnf(const uint8_t *frame, uint16_t len, NetworkInfo &out) {
  if (len < 20 || etherType(frame) != ETHERTYPE_HOMEPLUG ||
      mmtype(frame) != (MMTYPE_NW_INFO | MMTYPE_CNF)) {
    return false;
  }
  uint8_t ofs = payloadOffset(frame);
  memset(&out, 0, sizeof(out));
  memcpy(out.mac, &frame[6], 6);
  out.numNetworks = frame[ofs];
  if (out.numNetworks > 0) {
    const uint8_t *nw = &frame[ofs + 1];
    if (ofs + 1 + NW_INFO_ENTRY_LEN > len) {
      return false;
    }
    memcpy(out.nid, &nw[0], 7);
    out.snid = nw[7];
    out.tei = nw[8];
    out.role = nw[9];
    memcpy(out.ccoMac, &nw[10], 6);
  }
  return true;
}

}  // namespace homeplug
