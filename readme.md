# ESP32-S3 with TFT display

Arduino project that drives an **ILI9341 SPI TFT** (240×320) and a **QCA7005 HomePlug Green PHY modem** (compatible with QCA7000) from an **ESP32-S3**.

The display shows:

- The local modem's status (SPI signature)
- Whether the local modem has joined a powerline network, with its TEI and role (HomePlug `CM_NW_INFO`)
- Up to 3 modems in the network with MAC and software version (HomePlug `GET_SW`). The local modem, whose MAC ends in `FF:FF:11`, is marked with `*`.
- A log of the last 14 received frames (HomePlug MMEs such as SLAC, and IPv6) and events
- Traffic and SPI counters
- In the header: the uptime in seconds with **10 ms resolution**, refreshed every **50 ms** (20 Hz)

```
+-----------------------------------------------------+
| ESP32-S3 QCA7005 (yellow, size 2)    [    1234.56s] |
|-----------------------------------------------------|
| Modem OK (green/red)     joined STA TEI2  (size 2)  |
| *04:65:65:FF:FF:11 PINGPONG-RELEASE-1        (cyan) |
|  98:48:27:5A:3C:E4 QCA7420-1.4.0.20-00-20171027-CS  |
|  04:65:65:FF:FF:FF QCA7005-1.1.0.730-04-20140815-CS |
|-----------------------------------------------------|
|    12.30 ** modem OK                       (yellow) |
|    13.10 SLAC_PARAM.REQ       88:10:57       (cyan) |
|    13.15 SLAC_PARAM.CNF       E6:39:89              |
|    13.60 START_ATTEN_CHAR.IND 88:10:57 x3           |
|    14.40 MNBC_SOUND.IND       88:10:57 x10          |
|    14.50 ATTEN_CHAR.IND       E6:39:89              |
|    14.60 SLAC_MATCH.CNF       E6:39:89              |
|    15.20 ** joined TEI 2                   (yellow) |
|    16.00 UDP 59230>15118      88:10:57      (green) |
|     ... (14 lines, newest at the bottom)            |
|-----------------------------------------------------|
| MME 42 IPv6 2  SPI TX 24 RX 80 err 0 AA55   (grey)  |
+-----------------------------------------------------+
```

Planned next steps are in [backlog.md](backlog.md).

## Project layout

```
readme.md                                    this file
backlog.md                                   planned work
ccs32dave-arduino-esp32-s3-tft-ili9341.ino   Arduino sketch: display, scheduling
qca7000.h/.cpp                               QCA7000/QCA7005 SPI driver
homeplug.h/.cpp                              HomePlug messages (GET_SW, NW_INFO, MME names)
modem_list.h/.cpp                            table of the modems in the network
```

The sketch folder is the repository root. Arduino requires the `.ino` file to have the same name as its folder.

The modem code is ported from [ccs32berta](https://github.com/uhi22/ccs32berta) and cleaned up into small C++ modules.

## Hardware

- ESP32-S3 board with an ESP32-S3-WROOM-1 module and two USB-C ports labeled "USB" and "COM" (DevKitC-1 style). Tested with this board.
- ILI9341 SPI TFT module **without CS and MISO**, with the 7 pins `GND VCC CLK MOSI RES DC BLK`. This is the same module as in the STM32 project `TFT-with-CAN`.
- QCA7005 (or QCA7000) HomePlug modem with SPI access (3.3 V logic)
- Jumper wires

## Wiring

The TFT and the modem use **two separate SPI buses**. The TFT has no CS, so it would react to the modem's SPI traffic.

### TFT display (FSPI bus)

| TFT pin | ESP32-S3 pin | Sketch define | Notes                                                    |
|---------|--------------|---------------|----------------------------------------------------------|
| GND     | GND          |               |                                                          |
| VCC     | 3V3          |               | Supply for the display controller                        |
| CLK     | GPIO 12      | `TFT_SCLK`    | FSPI SCK                                                 |
| MOSI    | GPIO 11      | `TFT_MOSI`    | FSPI MOSI                                                |
| RES     | GPIO 8       | `TFT_RST`     | Hardware reset                                           |
| DC      | GPIO 9       | `TFT_DC`      | Data/command select                                      |
| BLK     | 5V           | `TFT_BL = -1` | Backlight, always on. Or use a GPIO (see notes)          |
| *(CS)*  | –            | `TFT_CS = -1` | Not on the module                                        |
| *(MISO)*| –            | `TFT_MISO = -1` | Not on the module                                      |

The chosen GPIOs are free on every ESP32-S3 module variant. The wiring avoids GPIO 26–32 (SPI flash), GPIO 33–37 (octal PSRAM on N8R8 / N16R8 modules), GPIO 19/20 (USB D-/D+), GPIO 0/3/45/46 (strapping pins) and GPIO 43/44 (UART0, used by the "COM" port).

#### TFT notes

- **No CS:** the controller's chip select is tied active on the module, so the display must be the **only device on this SPI bus**. In the sketch, `TFT_CS` and `TFT_MISO` are set to `-1`, and the Adafruit driver then skips CS handling. The display is write-only, so MISO isn't needed.
- **SPI mode:** mode 0 (clock idle low, sample on the first edge), the same as in the STM32 project. The Adafruit driver uses mode 0 by default.
- **Backlight:** to switch the backlight from software, connect `BLK` to a free GPIO (e.g. GPIO 7) and set `#define TFT_BL 7` in the sketch. The backlight draws several tens of mA. If the module has no backlight transistor, use a small NPN/MOSFET switch instead of driving it directly from the GPIO.
- Keep the wires short (< 15 cm). The SPI clock runs at 40 MHz; the STM32 project used 18 MHz. If the picture shows glitches, lower `SPI_FREQUENCY` to `20000000`.
- To use different pins, change the `#define TFT_*` lines at the top of the sketch. The ESP32-S3 can route SPI to any free GPIO.

### QCA7005 modem (HSPI bus)

| QCA7005 pin | ESP32-S3 pin | Sketch define | Notes                          |
|-------------|--------------|---------------|--------------------------------|
| GND         | GND          |               |                                |
| SPI_CLK     | GPIO 4       | `QCA_SCLK`    |                                |
| SPI_SI      | GPIO 5       | `QCA_MOSI`    | ESP32 → modem                  |
| SPI_SO      | GPIO 6       | `QCA_MISO`    | Modem → ESP32                  |
| SPI_CS      | GPIO 15      | `QCA_CS`      | Active low                     |
| INT         | –            |               | Not connected; the sketch polls |

#### QCA7005 notes

- **SPI settings:** mode 3 (clock idle high) at 2 MHz, as in ccs32berta.
- **Supply:** check the modem board's supply voltage before connecting. The SPI lines must be 3.3 V logic.
- **No interrupt:** the sketch polls the modem's receive buffer every 10 ms instead of using INT.
- **Diagnosis:** if the display shows `Modem: missing`, the signature read didn't return `AA55`. `FFFF` or `0000` usually means a wiring problem (MISO, CS, CLK) or no supply. While the signature is wrong, the serial port prints all internal registers every second (`QCA regs: ...`). If they all have the same value, the modem isn't decoding the commands.
- **Tested with:** local QCA7005 running the special sniffer firmware `PINGPONG-RELEASE-1` (MAC `04:65:65:FF:FF:11`), on a testbench with a PEV modem (QCA7005, `MAC-QCA7005-1.1.0.730-04-20140815-CS`) and an EVSE modem (QCA7420, `MAC-QCA7420-1.4.0.20-00-20171027-CS`). Serial output after a reset, once the local modem has joined the network:
  ```
  QCA7005 signature: AA55 (OK)
  ESP32-S3 TFT + QCA7005 demo started
  Modems: 1
    * 04:65:65:FF:FF:11 PINGPONG-RELEASE-1
  Network: joined, TEI 2, role 0
  Modems: 3
    * 04:65:65:FF:FF:11 PINGPONG-RELEASE-1
      98:48:27:5A:3C:E4 MAC-QCA7420-1.4.0.20-00-20171027-CS
      04:65:65:FF:FF:FF MAC-QCA7005-1.1.0.730-04-20140815-CS
  ```
  At the start of each new charging session the local modem resets, and the signature reads `0000` for up to 1 s.

## Software

### Requirements

- Arduino IDE 2.x **or** just `arduino-cli`. The IDE isn't needed; see [Command line only](#command-line-only-no-arduino-ide)
- **esp32** board package by Espressif, version 3.x (tested with 3.0.2)
- Libraries, from the Library Manager:
  - **Adafruit ILI9341** (tested with 1.6.3)
  - **Adafruit GFX Library** (tested with 1.12.6)
  - **Adafruit BusIO**, which is installed automatically as a dependency

The sketch uses the Adafruit libraries rather than TFT_eSPI. TFT_eSPI has known compatibility problems with esp32 core 3.x on the ESP32-S3.

### Board settings (Arduino IDE)

- Board: **ESP32S3 Dev Module**
- Everything else can stay at the defaults

### USB port

Connect the board's **"COM"** USB-C port. It has a USB-to-serial chip on UART0, which:

- puts the ESP32 into download mode automatically, so you don't need to press BOOT/RESET
- shows `Serial` output without the "USB CDC On Boot" setting
- stays available when the sketch crashes

The "USB" port is the ESP32-S3's native USB and isn't needed here. If Windows shows no COM port, install the driver for the USB-to-serial chip (CH343 or CP210x).

### Command line only (no Arduino IDE)

Everything (setup, build, upload, serial monitor) works with `arduino-cli` alone.

**1. Install arduino-cli**

```sh
winget install ArduinoSA.CLI
```

Or download it from <https://arduino.github.io/arduino-cli/latest/installation/>. If the Arduino IDE 2.x is already installed, it contains a copy of the CLI:
`%LOCALAPPDATA%\Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe`

**2. Install the ESP32 board package and the libraries** (one time)

```sh
arduino-cli config init
arduino-cli config add board_manager.additional_urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli core update-index
arduino-cli core install esp32:esp32
arduino-cli lib install "Adafruit ILI9341" "Adafruit GFX Library"
```

`arduino-cli config init` fails if a config file already exists, e.g. because the Arduino IDE created one. That's fine: continue with the next command. To get the tested version, use `esp32:esp32@3.0.2`.

**3. Build and upload**

Run these commands in the sketch folder `ccs32dave-arduino-esp32-s3-tft-ili9341`:

```sh
arduino-cli board list
arduino-cli compile --upload -p COM11 --fqbn esp32:esp32:esp32s3 .
```

Replace `COM11` with the port shown by `arduino-cli board list`. The FQBN `esp32:esp32:esp32s3` is the "ESP32S3 Dev Module" from the IDE, with the default settings.

**4. Serial monitor**

```sh
arduino-cli monitor -p COM11 -c baudrate=115200
```

After a reset it shows `ESP32-S3 TFT + QCA7005 demo started`. Exit with Ctrl+C. Close any serial monitor (CLI or IDE) before uploading, because it blocks the port.

## How it works

### Startup

`setup()` starts the TFT on the FSPI bus (40 MHz, landscape 320×240) and draws the static labels. Then it starts the modem on the HSPI bus and does the first modem check.

### Main loop

`loop()` is a small non-blocking scheduler:

| Interval | Task |
|---|---|
| 1 s | `checkModem()`: reads the signature register (`0xAA55` = modem present). If the modem disappears, the modem table and network status are cleared. When it comes back, the requests are sent immediately. |
| 5 s | `sendRequests()`: sends a broadcast `GET_SW.REQ` and a broadcast `CM_NW_INFO.REQ`. Modems and network info that didn't answer for 3 cycles (15.5 s) are removed. |
| 10 ms | `qca.poll()`: if the modem reports received data (`RDBUF_BYTE_AVA`), reads it, splits it into Ethernet frames and passes each one to `onEthFrame()` (see below) |
| 50 ms | Uptime display |
| on change | `drawPanel()`: redraws only the text lines whose content changed |
| on change, max. every 200 ms | `drawLog()`: redraws the frame log |

`onEthFrame()` sorts the received frames:

- **`GET_SW.CNF`:** added to the modem table. Once the local modem has joined a network, every modem in it answers.
- **`CM_NW_INFO.CNF`:** only the answer of the local modem sets the join status (number of networks > 0 means joined).
- **Other HomePlug MMEs:** counted and added to the frame log (e.g. `SLAC_MATCH.CNF`).
- **IPv6:** counted, protocol and ports added to the frame log (e.g. `UDP 59230>15118` = SDP).

With `LOG_TRAFFIC 1`, every counted frame is also printed on the serial port. Changes of the modem table and join status are always printed.

### Modules

- **`qca7000`:** SPI protocol of the modem (chargebyte AN4). Internal register reads/writes, and the framing `AA AA AA AA | length | 00 00 | frame | 55 55` for sending and receiving Ethernet frames. It counts TX/RX frames and errors.
- **`homeplug`:** builds `GET_SW.REQ` (Qualcomm vendor MME `0xA000`, OUI `00:B0:52`) and `CM_NW_INFO.REQ` (`0x6038`, MMV 1). It parses `GET_SW.CNF` into MAC and version string, and `CM_NW_INFO.CNF` into number of networks, NID, TEI, role and CCo MAC. It also turns MMTYPEs into readable names.
- **`modem_list`:** table of up to 3 modems, keyed by MAC, with the local modem always first. Entries expire when a modem stops answering.

### Display

- **Status and modem lines:** each line is padded to its full width and drawn with a background color, so old text is erased without clearing the screen.
- **Frame log:** a ring buffer of 14 entries: uptime, MME name or IPv6 protocol/ports, and the last 3 bytes of the source MAC. Consecutive identical entries are combined (`x10`). Events (modem OK/missing, joined/not joined) are shown in yellow. The log is rendered into a 306×140 `GFXcanvas16` (about 86 KB RAM) and sent as one bitmap, at most every 200 ms.
- **Uptime:** read from `esp_timer_get_time()`, which is 64-bit microseconds, so it doesn't overflow after 49 days like `millis()`. The fixed-width string `%8llu.%02us` is rendered into a 72×8 canvas and sent in one `drawRGBBitmap()` call. This avoids flicker.
