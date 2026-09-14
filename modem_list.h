// Table of the modems that answered GET_SW.REQ.
// The local modem (MAC ends with FF:FF:11) is always kept at index 0.

#pragma once

#include <Arduino.h>
#include "homeplug.h"

class ModemList {
public:
  static constexpr uint8_t MAX_MODEMS = 3;

  struct Entry {
    uint8_t mac[6];
    char version[homeplug::VERSION_MAX_LEN + 1];
    uint32_t lastSeenMs;
  };

  static bool isLocal(const uint8_t mac[6]);

  // Adds or refreshes a modem. Returns true if the list content changed
  // (new modem or changed version).
  bool update(const homeplug::SoftwareVersion &sw, uint32_t nowMs);

  // Removes modems that didn't answer for maxAgeMs. Returns true if any was removed.
  bool expire(uint32_t nowMs, uint32_t maxAgeMs);

  void clear() { _count = 0; }
  uint8_t count() const { return _count; }
  const Entry &operator[](uint8_t i) const { return _entries[i]; }

  // Answers dropped because the table was full
  uint32_t dropped() const { return _dropped; }

private:
  void removeAt(uint8_t index);

  Entry _entries[MAX_MODEMS];
  uint8_t _count = 0;
  uint32_t _dropped = 0;
};
