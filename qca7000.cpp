#include "qca7000.h"

// SPI command word: bit 15 = read, bit 14 = internal register, bits 13..8 = register
// (e.g. signature read = 0xDA00). Read/write without the internal bit accesses
// the Ethernet frame buffers.
static constexpr uint16_t CMD_READ = 0x8000;
static constexpr uint16_t CMD_INTERNAL = 0x4000;

static constexpr uint8_t REG_BFR_SIZE = 0x01;
static constexpr uint8_t REG_WRBUF_SPC_AVA = 0x02;
static constexpr uint8_t REG_RDBUF_BYTE_AVA = 0x03;
static constexpr uint8_t REG_SIGNATURE = 0x1A;

// Framing of each Ethernet frame on SPI:
//   [4 byte length, big endian; receive direction only]
//   AA AA AA AA | frame length (2 byte, little endian) | 00 00 | frame | 55 55
static constexpr uint8_t SOF_BYTE = 0xAA;
static constexpr uint8_t EOF_BYTE = 0x55;
static constexpr uint16_t FRAME_OVERHEAD = 10;  // SOF(4) + length(2) + reserved(2) + EOF(2)
static constexpr uint16_t RX_LENGTH_FIELD = 4;

Qca7000::Qca7000(SPIClass &spi, int8_t pinCs, uint32_t spiFrequency)
    : _spi(spi), _pinCs(pinCs), _settings(spiFrequency, MSBFIRST, SPI_MODE3) {}

void Qca7000::begin(int8_t pinSclk, int8_t pinMiso, int8_t pinMosi) {
  pinMode(_pinCs, OUTPUT);
  digitalWrite(_pinCs, HIGH);
  _spi.begin(pinSclk, pinMiso, pinMosi, -1);
}

void Qca7000::select() {
  _spi.beginTransaction(_settings);
  digitalWrite(_pinCs, LOW);
}

void Qca7000::deselect() {
  digitalWrite(_pinCs, HIGH);
  _spi.endTransaction();
}

uint16_t Qca7000::readRegister(uint8_t reg, uint16_t *misoDuringCommand) {
  select();
  uint16_t cmdMiso = _spi.transfer16(CMD_READ | CMD_INTERNAL | (uint16_t)(reg << 8));
  uint16_t value = _spi.transfer16(0x0000);
  deselect();
  if (misoDuringCommand) {
    *misoDuringCommand = cmdMiso;
  }
  return value;
}

void Qca7000::dumpRegisters(Print &out) {
  uint16_t cmdMiso;
  uint16_t sig = readRegister(REG_SIGNATURE, &cmdMiso);
  out.printf("QCA SIGNATURE=%04X (MISO during command: %04X)\n", sig, cmdMiso);
  out.print("QCA regs:");
  for (uint8_t reg = 0x00; reg <= 0x1B; reg++) {
    out.printf(" %02X=%04X", reg, readRegister(reg));
  }
  out.println();
}

void Qca7000::writeRegister(uint8_t reg, uint16_t value) {
  select();
  _spi.transfer16(CMD_INTERNAL | (uint16_t)(reg << 8));
  _spi.transfer16(value);
  deselect();
}

uint16_t Qca7000::readSignature() {
  return readRegister(REG_SIGNATURE);
}

bool Qca7000::sendEthFrame(const uint8_t *frame, uint16_t len) {
  if (len > MAX_ETH_FRAME_LEN) {
    _errors++;
    return false;
  }
  uint16_t spiLen = len + FRAME_OVERHEAD;
  if (readRegister(REG_WRBUF_SPC_AVA) < spiLen) {
    _errors++;
    return false;
  }
  writeRegister(REG_BFR_SIZE, spiLen);

  uint8_t *p = _txBuffer;
  *p++ = 0x00;  // external write
  *p++ = 0x00;
  memset(p, SOF_BYTE, 4);
  p += 4;
  *p++ = len & 0xFF;
  *p++ = len >> 8;
  *p++ = 0x00;  // reserved
  *p++ = 0x00;
  memcpy(p, frame, len);
  p += len;
  *p++ = EOF_BYTE;
  *p++ = EOF_BYTE;

  select();
  _spi.writeBytes(_txBuffer, p - _txBuffer);
  deselect();
  _txFrames++;
  return true;
}

uint16_t Qca7000::poll(FrameHandler handler) {
  uint16_t avail = readRegister(REG_RDBUF_BYTE_AVA);
  if (avail == 0) {
    return 0;
  }
  if (avail > HW_BUFFER_LEN) {
    // Implausible value, e.g. modem not connected
    _errors++;
    return 0;
  }

  // Tell the modem how much to deliver, then read it with an external read
  writeRegister(REG_BFR_SIZE, avail);
  memset(_rxBuffer, 0, avail);
  select();
  _spi.transfer16(CMD_READ);
  _spi.transfer(_rxBuffer, avail);
  deselect();

  // The buffer may contain several Ethernet frames
  uint16_t frames = 0;
  uint16_t pos = 0;
  while (pos + RX_LENGTH_FIELD + FRAME_OVERHEAD <= avail) {
    const uint8_t *p = &_rxBuffer[pos];
    uint32_t outerLen = ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
                        ((uint32_t)p[2] << 8) | p[3];
    uint16_t frameLen = p[8] | (p[9] << 8);
    bool valid = p[4] == SOF_BYTE && p[5] == SOF_BYTE &&
                 p[6] == SOF_BYTE && p[7] == SOF_BYTE &&
                 outerLen == (uint32_t)frameLen + FRAME_OVERHEAD &&
                 pos + RX_LENGTH_FIELD + outerLen <= avail &&
                 p[12 + frameLen] == EOF_BYTE && p[13 + frameLen] == EOF_BYTE;
    if (!valid) {
      _errors++;
      break;
    }
    _rxFrames++;
    frames++;
    if (handler) {
      handler(&p[12], frameLen);
    }
    pos += RX_LENGTH_FIELD + outerLen;
  }
  return frames;
}
