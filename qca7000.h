// QCA7000 / QCA7005 HomePlug Green PHY modem, SPI driver.
// Ported from github.com/uhi22/ccs32berta (qca7000.ino).
// Protocol reference: chargebyte application note AN4 rev5, Linux qcaspi driver.

#pragma once

#include <Arduino.h>
#include <SPI.h>

class Qca7000 {
public:
  static constexpr uint16_t SIGNATURE = 0xAA55;
  static constexpr uint16_t MAX_ETH_FRAME_LEN = 1522;

  // Called for each Ethernet frame received from the modem
  using FrameHandler = void (*)(const uint8_t *frame, uint16_t len);

  Qca7000(SPIClass &spi, int8_t pinCs, uint32_t spiFrequency = 2000000);

  void begin(int8_t pinSclk, int8_t pinMiso, int8_t pinMosi);

  // Reads the signature register; a working modem returns SIGNATURE (0xAA55)
  uint16_t readSignature();

  // Sends one Ethernet frame (without FCS). Returns false if the modem's
  // write buffer has no space or the frame is too long.
  bool sendEthFrame(const uint8_t *frame, uint16_t len);

  // Fetches pending receive data from the modem and calls handler for each
  // Ethernet frame in it. Returns the number of frames received.
  uint16_t poll(FrameHandler handler);

  // Diagnosis: prints all internal registers (0x00..0x1B) and what MISO
  // carried during the command word of the signature read
  void dumpRegisters(Print &out);

  uint32_t txFrames() const { return _txFrames; }
  uint32_t rxFrames() const { return _rxFrames; }
  uint32_t errors() const { return _errors; }

private:
  // Size of the modem's SPI buffers (Linux driver: QCASPI_HW_BUF_LEN)
  static constexpr uint16_t HW_BUFFER_LEN = 3163;

  uint16_t readRegister(uint8_t reg, uint16_t *misoDuringCommand = nullptr);
  void writeRegister(uint8_t reg, uint16_t value);
  void select();
  void deselect();

  SPIClass &_spi;
  int8_t _pinCs;
  SPISettings _settings;

  uint32_t _txFrames = 0;
  uint32_t _rxFrames = 0;
  uint32_t _errors = 0;

  uint8_t _rxBuffer[HW_BUFFER_LEN];
  uint8_t _txBuffer[MAX_ETH_FRAME_LEN + 12];
};
