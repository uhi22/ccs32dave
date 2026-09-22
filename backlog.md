# Backlog

## backlog_0001: integrate QCA7005

**Status:** Step 1 and Step 2 both done + hardware-tested (2026-09-14/15). The "no unicast" blocker
is resolved (see below) and DIN 70121 EXI decoding is live on real traffic.

A serial diagnosis interface was added (`diag.h/.cpp`): the ESP32 sends the modem's Qualcomm vendor
MMEs (VS_RD_MEM/VS_WR_MEM/VS_NW_INFO) over SPI and answers on the serial port, and prints one line
per received frame (`F <ms> <src> <dst> <len> <desc>`, with TCP seq / payload-len / V2GTP-len). It's
driven from a script on the host PC that talks to it over the serial port.

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

### Step 2: passive DIN 70121 listener (done, 2026-09-15)

Reference code in `C:\UwesTechnik\ccs32berta` (`src/exi/`, OpenV2G-style EXI codec) was ported in:
decoder-only files copied to `src/exi/` here (no encoder - only decoders are needed), compiled the
same way ccs32berta does (Arduino compiles `.c`/`.cpp` in sketch subfolders; include with the
`src/exi/...` relative path). `projectExiConnector.c` itself was **not** copied (it's mostly
encoder-side plus a couple of globals this project doesn't need) - `v2g_exi.cpp` is a from-scratch,
smaller glue layer instead (below). One trimmed-away piece had to be stubbed back in:
`debugAddStringAndInt()`, a no-op debug hook the decoder calls internally.

- **`v2gtp.h/.cpp`:** TCP reassembly + V2GTP framing, keyed by source MAC (no TCP stack - just
  enough to find message boundaries). Tracks the expected sequence number per peer; a gap or
  retransmit resets that peer's partial buffer and resyncs on the next `01 FE` header found in the
  incoming bytes. In every session observed, one TCP segment carries exactly one V2GTP message, so
  coalesced multi-message segments aren't handled (documented, not silently wrong).
- **`v2g_exi.h/.cpp`:** decodes one complete EXI payload - DIN first (the common case once joined),
  falling back to the app-handshake schema (`SupportedAppProtocolReq/Res`, used once per session
  before DIN starts). Extracts message name (34 DIN body variants recognized) and, where present:
  `EVTargetVoltage`/`EVTargetCurrent` (`CurrentDemandReq`), `EVSEPresentVoltage`/`EVSEPresentCurrent`
  (`CurrentDemandRes`/`PreChargeRes`), `DC_EVStatus.EVRESSSOC` (SoC, several EV-sent messages),
  `ResponseCode`. `dinEXIDocument` is ~28 KB (measured via host `sizeof`) - kept as a static global,
  not on the stack.
- **TFT:** two new lines between the modem panel and the frame log (`Tgt`/`Pres` V+A, SoC, last
  message name); the frame log itself now shows decoded message names for TCP traffic (green =
  decoded OK, red = decode error) instead of raw per-segment TCP lines, to avoid flooding the
  12-line ring buffer (shrunk from 14 to make room). Every counted IPv6 frame still gets its raw
  description on the serial port unconditionally (`logFrame()`), independent of what's shown.
- **Validated offline first** (validate before trusting hardware), against a real captured DIN
  session: a host-gcc build of the same decoder reproduced the EVSE's own logged values exactly
  (`EVTargetVoltage 230.0`/`EVTargetCurrent 10.0`). A separate host unit
  test of `V2gtpReassembler` (split segments, duplicates, gaps+resync, junk bytes) caught and fixed
  two real bugs in `resyncBuffer()`'s return value before ever touching hardware. Both test
  harnesses live outside this repo.
- **Live on real hardware, 2026-09-15:** a full EVSE session decoded 785 DIN messages with **0
  decode errors** (`PowerDeliveryReq/Res`, `PreChargeReq/Res`, `CurrentDemandReq/Res` × ~389 each),
  values matching pyPLC's own log (`230.0 V / 10.0 A`). No helper on the host PC needed for either
  the traffic or the decode.
- **Not yet done:** ISO 15118-2 (only DIN 70121 + the app handshake are decoded; out of scope for
  now per the 2026-09-15 decision). SoC has read 0% in every session so far - the Foccci PEV
  simulator likely doesn't report a real battery SoC; not a decoder bug (the field decodes fine,
  see the offline validation values above).

Reference code and open earlier questions (kept for the record): `ipv6.ino` (IPv6/UDP/TCP parsing,
V2GTP header) and `homeplug.ino` (MME constants, SLAC layout) from ccs32berta informed the design
but weren't ported directly - this project's own `homeplug.h/.cpp` already covers the needed MMEs.

- **Unicast traffic is not forwarded (observed 2026-09-14):** over two charging sessions on the testbench, the local modem forwarded SLAC (broadcast), `SET_KEY.REQ` and the **SDP request** (UDP multicast to port 15118). It forwarded **no SDP response and no TCP**, i.e. no unicast frames between the PEV and the EVSE. Without them, no DIN messages can be decoded.
  - **RESOLVED 2026-09-15:** with an updated special firmware on the local modem, which forwards both directions on its own, without a helper on the host PC. Confirmed: both TCP directions reach the ESP32, balanced (~561 PEV + ~560 EVSE data segments in 40 s).

## backlog_0003: splash screen (done, 2026-09-15)

**Status:** done + hardware-tested.

Owner request: TFT headline changed from "ESP32-S3 QCA7005" to "ccs32dave"; a welcome screen at
boot for ~5 s (project name, GitHub link, build date/time, and - if it answers in time - the local
modem's own software version), with an animation, before switching to the main screen.

- `showSplashScreen()`, called from `setup()` right after the TFT starts (also starts the QCA link
  early - `qca.begin()` - so the modem has the full 5 s to answer). Blocking (nothing else needs to
  run during it).
- Typewriter reveal of the title (size-3 "ccs32dave"), subtitle, GitHub link
  (`github.com/uhi22/ccs32dave`), `Built <__DATE__> <__TIME__>`, a bordered progress bar filling
  over the 5 s, and two pulsing lightning-bolt icons either side of the title.
- Sends one `GET_SW.REQ` as soon as the modem's signature reads OK, and polls for the local
  modem's own `GET_SW.CNF` (MAC ending `FF:FF:11`) for the rest of the splash window; if it
  arrives, "Local modem: detecting..." is replaced with the real version and the modem table is
  seeded so the main screen shows it immediately, no extra wait.
- Confirmed live: local modem version captured and shown ~5.0 s after boot,
  matching `SPLASH_DURATION_MS`.

## backlog_0004: two-column layout - prominent V2G value panel (done, 2026-09-15)

**Status:** done + hardware-tested.

Owner request/decision: the target/present values needed much more prominence than the two shared
text lines from backlog_0001 step 2 gave them. Since the log doesn't need the screen's full width,
the screen below the modem panel now splits into two columns instead: the frame log (left, kept at
its original 14 lines and full 20-character message-name field, but narrower - a 5-char whole-
second timestamp and a 2-byte MAC suffix instead of the previous 8-char/3-byte ones) and a "big
stacked numbers" V2G panel (right): `TARGET`/`PRESENT` voltage+current in size-2 text, `SoC`, the
last decoded message name, and its response code (owner decision: show everything, not just V+A).
A vertical divider separates the two columns. Confirmed live against a real EVSE session: all panel
fields update correctly frame by frame.

## backlog_0005: fade out stale V2G values

**Status:** done.

Target/present voltage+current and SoC turn mid-gray after 2 s without a fresh value, dark gray after 4 s. Every value ages on its own: during pre-charge only the voltages are sent, so the currents gray out while the voltages keep updating.

## backlog_0006: show whether a CCo is in sight

**Status:** done.

When not joined, the network line shows "beacons" (orange) plus a 5-segment meter (beacons received in the last ~200 ms) while a CCo's beacons are received, otherwise "not joined". Reacts within 200 ms. Uses the modem's `VS_SNIFFER` indications (received beacons only). The sniffer runs while the modem is not joined (from startup on) and is switched off once it is joined; join status is polled every second with `VS_NW_INFO`.

## backlog_0007: serial diagnosis interface for the MMEs (done, 2026-09-15)

**Status:** done + hardware-tested.

This board has no JTAG and no Ethernet link to the QCA, so the modem's vendor MMEs can only be
reached over SPI, through the ESP32.

- `diag.h/.cpp` (this repo): a serial command interface - one command per line, one answer line
  per command, tagged so a script can wait for it (`RD`/`WR`/`NWI`/`PP`/`STAT`/`QRESET`/`BCAST`/
  `LOG`). Sends `VS_RD_MEM`/`VS_WR_MEM`/`CM_NW_INFO`/`GET_SW` over SPI on the ESP32's behalf.
  `bcast 0` also silences the sketch's own periodic GET_SW/NW_INFO/sniffer broadcasts
  (`backlog_0006`) for careful low-level testing.
- **DONE 2026-09-15:** implemented and tested.

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
| `04:65:65:FF:FF:11` | local modem (sniffer) | special firmware |
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

## backlog_0008: second screen page ("page2")

**Status:** implemented and compile-checked 2026-09-17; not yet tested on the bench.

The session-setup details (backlog_0009, backlog_0010) don't fit on the main page. Add a second
full-screen page for them. The header (title, uptime) stays on both pages. Only the visible page is
drawn; data for both pages is collected all the time, and switching redraws the page completely.

**Decided (owner, 2026-09-17):** a separate push button on **GPIO 16** (input with internal
pull-up, button to GND) toggles the page; 30 ms software debounce. No automatic switching, no
serial command. The BOOT button was rejected (hard to reach).

## backlog_0009: page2 - SDP: encrypted (TLS) or unencrypted

**Status:** implemented and compile-checked 2026-09-17; not yet tested on the bench.

Show what the car asks for in the SDP request (security: TLS / no TLS, transport: TCP / UDP) and
what the charger answers in the SDP response (security, transport, its IPv6 address and TCP port).
Both are V2GTP messages over UDP port 15118 (request payload type `0x9000`, response `0x9001`).
If TLS is chosen, the DIN/ISO messages can't be decoded: the main page then shows a red "TLS" in
its header (owner decision 2026-09-17).
Page 2 stays coloured while the session is alive (any SDP or V2G message), grays as a whole after 3 s of quiet, and is cleared on the SLAC match so nothing of the previous session can reappear.

**Solved 2026-09-17 with an updated firmware on the local modem.** Both SDP messages now arrive in
the same session (verified: request + response three times in a row, the car retried, then the
handshake request and response).

Independent of this, if the modem hasn't joined yet (seen with 120 s sessions) the whole session
start is missed - the auto-join first has to overhear the SLAC match.

## backlog_0010: page2 - offered and selected XML schemas

**Status:** implemented and compile-checked 2026-09-17; not yet tested on the bench.

From the app handshake: list every schema the car offers in `supportedAppProtocolReq` (namespace,
e.g. `urn:din:70121:2012:MsgDef`, version major/minor, SchemaID, priority), and show which one the
charger selects in `supportedAppProtocolRes` (SchemaID, resolved to the namespace, plus the
response code: OK / OK with minor deviation / failed). The handshake is already decoded
(`v2g_exi.cpp`), only the message name is used so far.

## backlog_0011: never sniff while joined

**Status:** done (rule implemented in backlog_0006).

Sniffing while the modem is a member of the network disturbs the real charging session. Measured
2026-09-17 inside one running session: the charger's own message rate fell from 318 per 20 s to 94
and then 18 with the sniffer forced on, and recovered to 163 after switching back. While not
joined, the sniffer does not disturb anything (sessions ran normally with it active during SLAC).
`sniff on` (diag) forces it anyway and is a test aid only.

## backlog_0012: SPI clock raised to 4 MHz

**Status:** done.

2 MHz was the ported default. Measured 2026-09-17 against the charger's own message count, one
charging phase each: 4 MHz captured 458 of ~486 messages (94%) with 0 SPI errors and 77 ms worst
loop; 8 MHz 91% with 5 errors; 2 MHz no errors either. 4 MHz gives headroom for the sniffer stream
(~19 kB/s) without the 8 MHz errors, so 4 MHz it is. The worst-case loop time comes from drawing
and serial output, not from the SPI clock.

## backlog_0013: page3 - charge parameter discovery

**Status:** implemented, and run on the bench 2026-09-22 (pyPLC EVSE loop + Foccci PEV).

A third page, reached with the same button (main -> page 2 -> page 3 -> main), shows what
`ChargeParameterDiscoveryReq` and `-Res` negotiate. Two columns, so the limiting side is visible
without arithmetic: the car's request left (yellow, like the main page's TARGET block), the
charger's response right (green, like PRESENT).

Fields chosen by the owner 2026-09-22:

- Car: requested energy transfer mode, `EVMaximumVoltageLimit` / `EVMaximumCurrentLimit`,
  `FullSOC` / `BulkSOC`, and the request's `DC_EVStatus` (`EVReady`, `EVErrorCode`, `EVRESSSOC`).
- Charger: `EVSEProcessing` plus the response code, `EVSEMaximum` voltage / current / power, and
  `EVSEMinimum` voltage / current.
- Deliberately left out: the car's max power and energy request/capacity, peak current ripple and
  current regulation tolerance, `EVSEEnergyToBeDelivered`, the `DC_EVSEStatus` detail
  (status code, isolation status, notification) and the `SAScheduleList` PMax schedule. The
  remaining lines on the page are free for those if a bench run shows they're wanted.

Everything optional in DIN (`FullSOC`, `BulkSOC`, `EVSEMaximumPowerLimit`) shows `-` when the
message omits it, and the charger's whole parameter block may be missing while it still answers
`Ongoing` - then only the processing state and response code are shown.

The values live in `SessionSetup` alongside the SDP and handshake data, so they inherit its
lifetime: cleared on the SLAC match, grayed as a whole after 3 s of V2G silence. That timer and
its helpers lost their "page2" name (`SESSION_QUIET_MS`, `sessionQuiet()`, `sessionColor()`),
since page 3 uses them too.

**Bench run 2026-09-22** (`run_evse_loop.sh`, `SESSION_S=40`, Foccci as PEV): the values on page 3
matched an independent decode of the same session - the car's own request as pyPLC logged it, and
the charger's 63-byte response re-decoded through pyPLC's `exiConnector`:

| | car (Foccci) | charger (pyPLC) |
|---|---|---|
| mode | `DC extended` (type 3) | - |
| maximum | 410.0 V / 125.0 A | 450.0 V / 200.0 A / 10.0 kW |
| minimum | - | 200 V / 1 A |
| SoC | 0 % (the Foccci reports no real SoC, see backlog_0051) | - |
| targets | full 100 %, bulk 80 % | - |
| status | `EVReady` 0 -> `ready no`, `NO_ERROR` | `Finished`, response `OK` |

**Layout revised after that run** (owner request): the page name moved into the header, replacing
`ccs32dave` and the uptime box on this page only (`drawMainHeader()` / `drawPage3Header()`, and
the uptime is skipped while page 3 is up). The freed rows let the six numbers that matter run at
text size 3 and the rest at size 2. Two consequences: the `TLS` hint moved to the right edge of
the heading in size 1 (it must stay visible - with TLS there is no discovery data at all), and the
DC error names were shortened to the 12 characters a size-2 column holds.

## backlog_0014: page4 - session timeline

**Status:** implemented and run on the bench 2026-09-22.

Every frame the ESP32 receives already carries a millisecond timestamp, so the phases of a session
and their durations can be shown as bars: SLAC, network join, SDP, app handshake, setup..CPD,
CableCheck, PreCharge, CurrentDemand, stop. The running phase is highlighted, a phase that stopped
without its successor turns red. Measured on the bench 2026-09-22: SLAC takes 1.4 s from
`CM_SLAC_PARAM.REQ` to `CM_SLAC_MATCH.CNF`.

Why it earns a page: it answers "where did the session stop, and what took so long" directly,
instead of leaving it to be inferred from the last message name. With the 40 s `run_evse_loop.sh`
cycle most sessions are cut in CableCheck - the timeline says so at a glance.

## backlog_0015: page5 - SLAC attenuation profile

**Status:** implemented and verified on the bench 2026-09-22, but see the caveat at the end.

`CM_ATTEN_CHAR.IND` (MMTYPE 0x606C) carries the attenuation the EVSE measured over the 10
`CM_MNBC_SOUND.IND` bursts, per OFDM carrier group, in dB. Observed on the bench: 129 bytes total,
which is exactly the 19-byte MME header plus APPLICATION_TYPE(1) SECURITY_TYPE(1)
SOURCE_ADDRESS(6) RUN_ID(8) SOURCE_ID(17) RESP_ID(17) NUM_SOUNDS(1) NUM_GROUPS(1) and 58 group
bytes - so NumGroups = 58 on this bench.

Drawn as a bar graph it is a fingerprint of the RF coupling: it changes shape when a cable or
coupler is moved. Plus the sound count (10/10 seen), the SLAC duration and the average
attenuation. This is the only view of the physical layer the project would have - the beacon
meter on page 1 is a crude proxy by comparison.

## backlog_0016: page6 - live V/I chart

**Status:** implemented and run on the bench 2026-09-22.

Two traces, present voltage and present current, over the last couple of minutes, drawn as a
sweeping (ECG-style) plot so no full-screen redraw is needed. Shows the shape of the session -
the precharge ramp, the current ramp, the CC->CV knee - which the numeric panel never conveys.
The data rate is there: 87 `CurrentDemandRes` in a 50 s bench capture, and much faster on a real
charger.

## backlog_0017: page7 - identity and keys

**Status:** implemented and verified byte-for-byte on the bench 2026-09-22.

One page for "who is talking to whom, under which keys": EVCCID (the car's MAC, e.g.
`02:34:28:88:10:57`), the EVSE's MAC, the V2G SessionID (assigned by the charger in
`SessionSetupRes`; the request carries all zeroes), and the NID + NMK of the PLC network. The
schema and version stay on page 2, and the EVSE's IPv6 address and port stay in its SDP block.

The payment option and selected service were implemented, then **removed again on 2026-09-22**
(owner: not of interest). The decoding went with them, so bringing them back means re-adding the
`ServicePaymentSelectionReq` extraction in `v2g_exi.cpp` - it is three fields.

`CM_SLAC_MATCH.CNF` (observed: 109 bytes = 19 header + APPLICATION_TYPE(1) SECURITY_TYPE(1)
MVFLength(2) PEV_ID(17) PEV_MAC(6) EVSE_ID(17) EVSE_MAC(6) RunID(8) RSVD(8) NID(7) RSVD(1)
NMK(16)) carries both MACs, the NID and the NMK in one frame. For the qca7000-transparency work
next door that key material is the interesting part; a SessionID that changes per cycle also
proves sessions aren't being resumed.

## backlog_0018: protocol timing / latency page

**Status:** proposed 2026-09-22, NOT implemented (owner selected 0014-0017, 0020, 0021 first).

Request-to-response round-trip per message type (min / average / maximum) and the period of the
CurrentDemand loop. DIN and ISO 15118 both have real timeouts, so this would catch a sluggish
charger or a retrying car before it turns into a failed session. Retries are already visible on
the wire (the bench capture shows 3x `CM_START_ATTEN_CHAR.IND` and 3x SDP request).

## backlog_0019: session history / soak-test table

**Status:** proposed 2026-09-22, NOT implemented (owner selected 0014-0017, 0020, 0021 first).

The last ~8 sessions as rows: start, phase reached, duration, whether SLAC completed, maximum
power, end reason. With the 40 s session loop this turns the display into a regression view
("6 of 8 reached CurrentDemand") that no single-session page can give. Would need the per-session
summary to survive the SLAC-match clear that resets the other pages.

## backlog_0020: page8 - energy receipt

**Status:** implemented and run on the bench 2026-09-22.

Integrate the present voltage x present current over the session: energy in kWh, charge in Ah,
average and peak power, and the session duration. The "what did this session actually deliver"
summary, computed from the `CurrentDemandRes` stream the decoder already produces.

## backlog_0021: page9 - ladder diagram

**Status:** implemented and run on the bench 2026-09-22.

A sequence diagram: the car on one rail, the charger on the other, one arrow per message with its
name, scrolling as the session runs. Direction and order at a glance - page 1's frame log is a
flat list that doesn't show who spoke. Direction comes from the message suffix (`...Req` = car,
`...Res` = charger) for V2G, and from the source MAC for the SLAC frames.

## backlog_0022: what the six new pages cost, and how they were checked

**Status:** notes on the 2026-09-22 implementation of backlog_0014..0017, 0020, 0021.

**Session data.** Everything the new pages show belongs to one session and is reset together in
`beginSession()`, called from the same SLAC-match branch that already cleared `SessionSetup`. Two
things happen *before* that point and would be wiped by it, so they wait outside the session
struct and are copied in: the start of the SLAC run (`slacStartMs`, from the first
`CM_SLAC_PARAM.REQ` that is not a retry) and the attenuation profile - `CM_ATTEN_CHAR.IND`
arrives about a second before the match.

**Two new wire parsers** in `homeplug.cpp`, both verified against pyPLC's own values:

- `parseAttenCharInd()` (`CM_ATTEN_CHAR.IND`, 0x606E). pyPLC sends 58 groups, 10 sounds, every
  group 9 dB except the last three (0x0F, 0x13, 0x19). The ESP32 read `sounds 10 groups 58 avg 9
  max 25 dB` - the average of that profile is 9.55 and the worst group is the final byte, so the
  offsets are right to the last byte of the frame.
- `parseSlacMatchCnf()` (`CM_SLAC_MATCH.CNF`, 0x607D). Two consecutive sessions matched the RPi's
  log exactly: `nid 0102C352ACFD07 nmk 777702F877..` / `nid 0102F39550A707 nmk 77771D3777..`
  against pyPLC's `01 02 C3 52 AC FD 07` / `77 77 02 F8 77 ..` and `01 02 F3 95 50 A7 07` /
  `77 77 1D 37 77 ..`. pyPLC randomises only bytes 2 and 3 of an otherwise all-0x77 NMK
  (`runEvseSlacHandler`), which is exactly the pattern that came out.

**Caveat for page 5:** on this bench the attenuation profile is **simulated, not measured** -
pyPLC has no values from the AR7420 and fills a constant 9 dB plus three rising values
(`composeAttenChar`). The page will therefore show the same flat curve in every session here. It
only becomes the coupling fingerprint the page was built for against a real charger.

**Drawing cost.** The pages that draw graphics cache what is already on the screen and repaint
only the difference: page 4 keeps each bar's last width, page 5 a key over the profile, page 6
draws one column per new sample (sweep, not scroll), page 9 redraws at most at 5 Hz and only when
the ladder actually changed. Pages 4 and 6 also need a 200 ms tick of their own, because a running
phase grows and the sweep advances even when no frame arrives.

**Navigation.** Nine pages on one button is tedious, so `diag` gained `page <n>` (answer
`PAGE <n>`), which jumps straight to a page over the serial line. It was how the pages were
stepped through during the bench run.

**Measured cost** of all six pages together: flash 414 kB -> 428 kB (32 % of 1.3 MB), globals
69 kB -> 77 kB (23 % of 320 kB). A 95 s run over two sessions produced 0 decode errors and no
resets.

## backlog_0023: page layout revision after the first nine-page bench run

**Status:** done 2026-09-22.

Three owner observations after looking at the pages on the bench:

- **A black rectangle sat in the middle of every heading.** The TLS hint (`headerHintLine`) is a
  four-character field at x=126 in the header, which is fine next to `ccs32dave` on the main page
  but blanks four characters out of a page's own heading. Only page 3 had been moved to the hint
  line at the right edge; pages 4-9 inherited the main page's. Now the rule is by page: the main
  page uses `headerHintLine`, **everything with a heading uses `titleHintLine`** (renamed from
  `p3HintLine`, since it is no longer page 3's).
- **Page 2 became one page in two stacked parts.** `ccs32dave` and the uptime are gone there. Its
  two headings, `SDP` and `schema`, are drawn alike - same yellow, same size 2, same left edge -
  and the top one deliberately is NOT the usual page title: page 2 has its own header function
  (`drawPage2Header()`) which puts the first part's heading where a title would go and leaves out
  the line under the header, so the only rule on the page is the separator between its two parts
  (owner request 2026-09-22, refined after a first version that kept the title styling). Note the
  `HEADER_LINE_Y + 1` in its fillRect: the clear has to take the header line's own row with it,
  or the line survives a switch from another page. The two rows this freed went into the text -
  the SDP lines moved up and the negotiated protocol is now size 3. Only the main page still has
  the uptime box.
- **The payment option left page 7** (see backlog_0017), which freed a row. The NMK halves now get
  one row each - side by side the second half started at x=170 with 16 size-2 characters, i.e.
  42 px past the right edge, so it had been clipped since it was written.
