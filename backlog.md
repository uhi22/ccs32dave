# Backlog

## backlog_0001: integrate QCA7005

**Status:** in progress. Step 1 is done and tested with hardware (2026-09-14); step 2 is next.

### Goal

Connect a QCA7005 HomePlug Green PHY modem (software-compatible with the QCA7000) to the ESP32-S3 via SPI.

1. **Step 1:** read the modem's software version and show it on the TFT.
2. **Step 2:** decode EXI V2G messages (DIN 70121) and show their status and content on the TFT.

### Decisions (2026-09-14)

| Topic | Decision |
|---|---|
| Pins | Separate SPI bus (HSPI): CLK GPIO 4, MOSI GPIO 5, MISO GPIO 6, CS GPIO 15. **INT not connected**; the modem is polled. |
| Screen layout | The font demo is replaced by a modem panel; title and uptime stay |
| Code structure | Clean C++ modules (`qca7000.h/.cpp`, `homeplug.h/.cpp`) instead of `.ino` files sharing globals |
| Role on the PLC link | **Passive listener**: observe the traffic between a car and a charger and decode it. No PEV/EVSE state machine. |
| EXI scope | DIN 70121 (plus app handshake) is enough for now |

### Step 1: software version (implemented)

- [x] `qca7000` module: signature, buffer registers, sending and receiving Ethernet frames. Ported from `ccs32berta/qca7000.ino`. Fixes compared to the original: the SOF check used `=` instead of `==`, the buffer length is now checked, and the EOF is checked.
- [x] `homeplug` module: `GET_SW.REQ` / `GET_SW.CNF`
- [x] TFT modem panel: status and signature, MAC, version (2 lines), TX/RX/error/answer counters
- [x] Wiring documented in `readme.md`
- [x] Test with real hardware: signature `AA55`; `GET_SW.CNF` from `04:65:65:FF:FF:FF` with version `MAC-QCA7005-1.1.0.730-04-20140815-CS`
- [ ] Check on the TFT that the version is shown and the uptime keeps updating smoothly while the modem is polled

Lessons learned:
- **Modem powered?** Without supply, the signature read returned `0000`.
- **Command word layout:** the register address is in bits 13..8 of the SPI command word (signature read = `0xDA00`, Linux: `SPI_REG_SIGNATURE 0x1A00`), not in the low byte. With the wrong layout, every register returns the same value. `Qca7000::dumpRegisters()` prints all registers to the serial port while the signature is wrong.

### Step 2: passive DIN 70121 listener (to plan)

Reference code in `C:\UwesTechnik\ccs32berta`:

| File | What to take from it |
|---|---|
| `ipv6.ino` | IPv6/UDP/TCP parsing, V2GTP header (`01 FE`, payload type `0x8001` = EXI) |
| `src/exi/` | EXI codec (OpenV2G style): app handshake (`appHand*`) and DIN (`din*`). Only the decoders are needed |
| `src/exi/projectExiConnector.*` | Glue between the sketch and the EXI decoder |
| `homeplug.ino` | MME constants, SLAC message layout (`CM_SLAC_MATCH`, `CM_SET_KEY`) |

Open questions and ideas:

- **Joining the car's and charger's network (decided):** the QCA7005 gets a special firmware that sniffs the key and joins the network by itself. Nothing to do in this project; the ESP32 just receives the traffic the modem forwards.
- **Unicast traffic is not forwarded (observed 2026-09-14, firmware `PINGPONG-RELEASE-1`):** over two charging sessions on the testbench, the local modem forwarded SLAC (broadcast), `SET_KEY.REQ` and the **SDP request** (UDP multicast to port 15118). It forwarded **no SDP response and no TCP**, i.e. no unicast frames between the PEV and the EVSE. Without them, no DIN messages can be decoded. **This needs a change in the special firmware** (forward all frames to the host). Step 2 is blocked until then.
- **TCP:** as a passive listener, no TCP stack is needed. Parse IPv6 → TCP → V2GTP and decode the payload, but handle TCP segments that are split or repeated.
- **Display:** a message log (last N messages: direction, name, response code) and/or key values (EVSE voltage/current, SoC, target values).
- **Memory:** the DIN decoder structures are large. Check RAM usage on the ESP32-S3; decode into a static document instead of the stack.

## backlog_0002: cyclic software version polling with multiple modems

**Status:** done and tested on the testbench (2026-09-14); only the check on the TFT is still open

### Goal

Send `GET_SW.REQ` cyclically, not only until the first answer. Once the QCA7005 has joined the network (see the special firmware in backlog_0001), **all modems** in the network answer the broadcast, e.g. the local modem, the car's modem and the charger's modem. So expect several `GET_SW.CNF` per request and show all of them.

### Current behavior (after backlog_0001 step 1)

- `GET_SW.REQ` is only sent while no answer has been received. After the first answer, polling stops.
- Only the **first** answer (MAC and version) is stored and shown. Later answers from other modems are only printed on the serial port.
- The `answers` counter adds up over the whole runtime and is only reset when the modem disappears.

### Proposal

- **Polling cycle:** send `GET_SW.REQ` every N seconds (proposal: 5 s), independent of earlier answers, as long as the signature is OK.
- **Modem table:** collect the answers in a small table (proposal: max. 4 entries), keyed by MAC:
  - MAC, version string
  - time of the last answer
  - flag "local modem" (see open questions)
- **Aging:** remove entries that didn't answer in the last 2–3 polling cycles, e.g. after the car was unplugged and the network is gone.
- **Display:** number of modems (e.g. `Modems: 3`), plus one line per modem with MAC and a short version, e.g. only the part after `MAC-QCA7005-`. The local modem's full version stays visible.
- **Serial port:** print the table when it changes (modem added, removed, version changed).
- **Network state:** derive a simple status from the count: 1 modem = "not joined", 2 or more = "joined".

### Answers (2026-09-14)

- **Local modem:** its MAC ends with `FF:FF:11`.
- **Number of modems:** at most 3 are expected, so 3 version strings are enough.
- **Join status:** the special firmware can't report it, so it is polled with MMEs. Implemented with the standard `CM_NW_INFO.REQ` (0x6038, MMV 1).

### Implementation

- `modem_list.h/.cpp`: table of max. 3 modems, local modem first, entries expire after 3 missed cycles
- Every 5 s: broadcast `GET_SW.REQ` + `CM_NW_INFO.REQ`
- The join status is taken only from the local modem's `CM_NW_INFO.CNF` (number of networks > 0)
- The TFT shows the join status (role, TEI), the modem lines `*MAC version`, a frame log of 14 lines and the counters. The headline is smaller, and the uptime is a small box right of it (layout change 2026-09-14, to keep the important information visible). The serial port prints table and join changes, plus every received frame (`LOG_TRAFFIC`).

### Test on the testbench (2026-09-14, new session every 40 s)

| MAC | Role | Version |
|---|---|---|
| `04:65:65:FF:FF:11` | local modem (sniffer) | `PINGPONG-RELEASE-1` |
| `04:65:65:FF:FF:FF` | PEV modem | `MAC-QCA7005-1.1.0.730-04-20140815-CS` |
| `98:48:27:5A:3C:E4` | EVSE modem, CCo | `MAC-QCA7420-1.4.0.20-00-20171027-CS` |
| `02:34:28:88:10:57` | PEV host | – |
| `B8:27:EB:E6:39:89` | EVSE host | – |

Sequence per session, from the serial log:
1. The local modem resets (signature `0000` for up to 1 s); the table is cleared.
2. `CM_NW_INFO.CNF`: not joined. `UNASSOCIATED_STA.IND` from the PEV modem.
3. SLAC: `SLAC_PARAM`, `START_ATTEN_CHAR`, 10× `MNBC_SOUND`, `ATTEN_CHAR`, `SLAC_MATCH`
4. `CC_ASSOC` / `CC_SET_TEI_MAP` / `ENCRYPTED_PAYLOAD` from the CCo, then joined, TEI 2 (the NID changes each session)
5. All 3 modems answer `GET_SW.REQ`; `SET_KEY.REQ` from the EVSE host; one SDP request (UDP → port 15118)

The `CM_NW_INFO.CNF` layout (MMV 1, FMI, NumNWs, then NID(7), SNID, TEI, role, CCo MAC) was confirmed with the raw bytes.

### Acceptance criteria

- [x] `GET_SW.REQ` is sent cyclically while the modem is present
- [x] Several answers per request are collected without duplicates (keyed by MAC)
- [x] Modems that stop answering disappear from the table (seen at each modem reset)
- [x] Join status polled via MME and shown
- [x] Check on the TFT: modem lines, join status, and that the uptime keeps updating smoothly during SLAC traffic
