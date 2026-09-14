#include "modem_list.h"

bool ModemList::isLocal(const uint8_t mac[6]) {
  return mac[3] == 0xFF && mac[4] == 0xFF && mac[5] == 0x11;
}

bool ModemList::update(const homeplug::SoftwareVersion &sw, uint32_t nowMs) {
  for (uint8_t i = 0; i < _count; i++) {
    Entry &e = _entries[i];
    if (memcmp(e.mac, sw.mac, 6) == 0) {
      e.lastSeenMs = nowMs;
      if (strcmp(e.version, sw.version) == 0) {
        return false;
      }
      strlcpy(e.version, sw.version, sizeof(e.version));
      return true;
    }
  }

  // New modem
  bool local = isLocal(sw.mac);
  if (_count == MAX_MODEMS) {
    if (!local) {
      _dropped++;
      return false;
    }
    removeAt(_count - 1);  // make room for the local modem
    _dropped++;
  }
  uint8_t index = _count;
  if (local) {
    memmove(&_entries[1], &_entries[0], _count * sizeof(Entry));
    index = 0;
  }
  Entry &e = _entries[index];
  memcpy(e.mac, sw.mac, 6);
  strlcpy(e.version, sw.version, sizeof(e.version));
  e.lastSeenMs = nowMs;
  _count++;
  return true;
}

bool ModemList::expire(uint32_t nowMs, uint32_t maxAgeMs) {
  bool changed = false;
  for (uint8_t i = _count; i > 0; i--) {
    if (nowMs - _entries[i - 1].lastSeenMs > maxAgeMs) {
      removeAt(i - 1);
      changed = true;
    }
  }
  return changed;
}

void ModemList::removeAt(uint8_t index) {
  memmove(&_entries[index], &_entries[index + 1],
          (_count - index - 1) * sizeof(Entry));
  _count--;
}
