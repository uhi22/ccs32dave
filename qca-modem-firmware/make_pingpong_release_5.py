#!/usr/bin/env python3
"""
make_pingpong_release_5.py - build the QCA7005 flash image PINGPONG-RELEASE-5 for ccs32dave.

This script contains no firmware. It needs an original 2 MB SPI flash image with the QCA7005
firmware MAC-QCA7005-1.1.0.730-04-20140815-CS, which you obtain yourself. Apart from the per-device
parameter blocks (PIB: MAC address, keys), the image must be identical to the one this release
was tested with; the script checks this and refuses anything else. The file name doesn't
matter; the default is original_1.1.0.730-04-CS-20140815.bin.
It applies the ccs32dave changes to the modem firmware in that dump, sets a new MAC address and
writes a complete 2 MB image for the QCA7005's SPI flash. Only Python 3 is needed, no packages.

Usage:
    python make_pingpong_release_5.py
    python make_pingpong_release_5.py other_name.bin --mac 04:65:65:FF:FF:12 --out my.bin

Default output: qca_flash_PINGPONG-RELEASE-5.bin. Default MAC: 04:65:65:FF:FF:11. The MAC must differ from every other
modem on the same line: images of this firmware often carry 04:65:65:FF:FF:FF, which many other
boards (e.g. Foccci) also use, and two modems with the same MAC break the traffic in confusing ways.

Flashing is up to you (any SPI flash programmer). One way: a Raspberry Pi with flashrom, the
QCA7005 held in reset while flashing. Read and keep a backup of the original flash first.

Tested on (the compressor must produce a bit-exact result, the script checks this):
    - Windows 10 x64, Python 3.11.7 (python.org)
    - Ubuntu 24.04 (WSL) x86_64, Python 3.12.3
    - Raspbian 11 (bullseye) armv7l, Python 3.9.2

DISCLAIMER - use at your own risk, no warranty of any kind:
  * Flashing can leave the modem unusable until it is flashed again with a working image.
  * This firmware joins the network of any charging session it observes and forwards that
    session's traffic to its host. It is meant for analysis on your own equipment. Use it only
    where you are allowed to, and be aware that an extra station in the network can affect the
    charging session.

License: GNU GPL v3, see LICENSE.md in the repository root.
"""

import argparse
import base64
import hashlib
import lzma
import platform
import struct
import sys

TAG = "PINGPONG-RELEASE-5"
INPUT_NAME = "original_1.1.0.730-04-CS-20140815.bin"
FW_VERSION = "MAC-QCA7005-1.1.0.730-04-20140815-CS"
REF_INPUT_SHA256 = "fb8b44adfd73b3ed11e048787d244fbca5057b192453f4164f08d6f5c938c2db"
MASKED_SHA256 = "8175b217a4a793256593d530ff04e45e0b0f9328dbbfc5f546bbb764076b910a"
INPUT_LEN = 2097152
STOCK_SHA256 = "6f4656c7354111cf253a78a25cb9ba4e4cb4b9ad0daac7ea6014fb60717ae382"
PATCHED_SHA256 = "75984da20422801b597d081b28913474286838440b13a7a0675f19400c55bc50"
IMAGE_LEN = 487732
STREAM_LEN = 307301
STREAM_SHA256 = "22b2a98840712c3f8410892418aa3733a906532df6db79142bf7d5e8b7ba4b8b"
FIRMWARE_SHA256 = "2d15bce5c4f223674ad7fd390c4bdc5b544c383ec6c1c8744de8d15cd2f6b645"
OUTPUT_SHA256 = "0c89e04318ca7337c8648f0c4e4c55ee18a0c2b7bb6284024cc9b346dcc7f453"
DEFAULT_MAC = "04:65:65:FF:FF:11"
OUT_NAME = "qca_flash_PINGPONG-RELEASE-5.bin"
ISSUES_URL = "https://github.com/uhi22/ccs32dave/issues"
TESTED = [
    'Windows 10 x64, Python 3.11.7 (python.org)',
    'Ubuntu 24.04 (WSL) x86_64, Python 3.12.3',
    'Raspbian 11 (bullseye) armv7l, Python 3.9.2',
]

SLOTS = (0x018bf0, 0x148bf0)
PIB_HEADERS = (0x0e0360, 0x0e4360, 0x0f0360, 0x0f4360)
PIB_LEN = 0x1f90
HDR_LEN = 0x60
SUB_LEN = 0x90
PAYLOAD_LEN = 0x4b0f8
FILTERS = [{"id": lzma.FILTER_LZMA1, "dict_size": 4096, "lc": 3, "lp": 0, "pb": 2}]

PATCH = """
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
AAQAtiYJ6g==
"""


def fail(msg):
    print("\nERROR: " + msg)
    print("No image was written.")
    sys.exit(1)


def sha(b):
    return hashlib.sha256(b).hexdigest()


def checksum32(buf):
    x = 0
    for (w,) in struct.iter_unpack("<I", buf):
        x ^= w
    return ~x & 0xFFFFFFFF


def masked(flash):
    m = bytearray(flash)
    for h in PIB_HEADERS:
        m[h:h + HDR_LEN + PIB_LEN] = bytes(HDR_LEN + PIB_LEN)
    return bytes(m)


def firmware_hash(flash):
    return sha(b"".join(bytes(flash[s:s + HDR_LEN + PAYLOAD_LEN]) for s in SLOTS))


def parse_mac(text):
    raw = text.replace(":", "").replace("-", "")
    try:
        mac = bytes.fromhex(raw)
    except ValueError:
        mac = b""
    if len(mac) != 6 or mac[0] & 1:
        raise argparse.ArgumentTypeError("not a unicast MAC: " + text)
    return mac


def main():
    ap = argparse.ArgumentParser(description="Build the QCA7005 flash image " + TAG + " for ccs32dave.")
    ap.add_argument("dump", nargs="?", default=INPUT_NAME,
                    help="the original 2 MB flash image (default " + INPUT_NAME + ")")
    ap.add_argument("--mac", type=parse_mac, default=parse_mac(DEFAULT_MAC),
                    help="MAC address of the modem (default " + DEFAULT_MAC + ")")
    ap.add_argument("--out", default=OUT_NAME, help="output file (default " + OUT_NAME + ")")
    args = ap.parse_args()

    try:
        flash = bytearray(open(args.dump, "rb").read())
    except OSError as e:
        fail("cannot read the input %s (%s).\nPass the original 2 MB flash image with the firmware %s "
             "as the first argument." % (args.dump, e.strerror, FW_VERSION))
    expected_input = ("The expected input is an original 2 MB flash image with the firmware %s,\n"
                      "identical to the tested one apart from the per-device parameter blocks." % FW_VERSION)
    if len(flash) != INPUT_LEN:
        fail("the input has %d bytes, expected %d.\n%s" % (len(flash), INPUT_LEN, expected_input))
    for h in PIB_HEADERS:
        if struct.unpack_from("<I", flash, h + 0x24)[0] != 0x0F or \
                struct.unpack_from("<I", flash, h + 0x10)[0] != PIB_LEN or \
                checksum32(bytes(flash[h:h + HDR_LEN])) != 0:
            fail("the parameter blocks are not where expected.\n" + expected_input)
    if sha(masked(flash)) != MASKED_SHA256:
        fail("the input differs from the tested original outside the per-device parameter blocks.\n"
             + expected_input)
    input_sha = sha(flash)
    print("[1] input OK (identical to the tested original apart from the parameter blocks)")

    first = SLOTS[0] + HDR_LEN
    dec = lzma.LZMADecompressor(format=lzma.FORMAT_RAW, filters=FILTERS)
    image = bytearray(dec.decompress(bytes(flash[first + SUB_LEN:first + PAYLOAD_LEN]), max_length=IMAGE_LEN))
    if sha(image) != STOCK_SHA256:
        fail("the firmware in the dump does not decompress as expected (SHA-256 %s)" % sha(image))
    print("[2] firmware decompressed, %d bytes" % len(image))

    patch = base64.b64decode(PATCH)
    pos = 0
    while pos < len(patch):
        off, n = struct.unpack_from("<IH", patch, pos)
        pos += 6
        image[off:off + n] = patch[pos:pos + n]
        pos += n
    if sha(image) != PATCHED_SHA256:
        fail("the patched firmware is not as expected (SHA-256 %s)" % sha(image))
    print("[3] patch applied")

    comp = lzma.LZMACompressor(format=lzma.FORMAT_RAW, filters=FILTERS)
    stream = comp.compress(bytes(image)) + comp.flush()
    if len(stream) != STREAM_LEN or sha(stream) != STREAM_SHA256:
        fail("the LZMA compressor of this Python gives a different result than on the tested systems.\n"
             "  expected: %d bytes, SHA-256 %s\n"
             "  got:      %d bytes, SHA-256 %s\n"
             "  this system: Python %s on %s %s\n"
             "The modem only boots a compressed firmware of exactly the tested size, and only the tested\n"
             "result was tried on hardware. Please run this script on one of the tested systems:\n  - %s\n"
             "and report the lines above as an issue at %s"
             % (STREAM_LEN, STREAM_SHA256, len(stream), sha(stream), platform.python_version(),
                platform.system(), platform.machine(), "\n  - ".join(TESTED), ISSUES_URL))
    print("[4] firmware recompressed, %d bytes, identical to the tested build" % len(stream))

    stream += bytes(-len(stream) % 4)
    for slot in SLOTS:
        p = slot + HDR_LEN
        sub = bytearray(flash[p:p + SUB_LEN])
        struct.pack_into("<I", sub, 0x18, SUB_LEN + len(stream) - 0x80)
        struct.pack_into("<I", sub, 0x88, len(stream))
        struct.pack_into("<I", sub, 0x8c, len(image))
        payload = bytes(sub) + stream
        if len(payload) != PAYLOAD_LEN:
            fail("internal error: payload length %d" % len(payload))
        hdr = bytearray(flash[slot:slot + HDR_LEN])
        struct.pack_into("<I", hdr, 0x10, len(payload))
        struct.pack_into("<I", hdr, 0x14, checksum32(payload))
        struct.pack_into("<I", hdr, 0x5c, 0)
        struct.pack_into("<I", hdr, 0x5c, checksum32(bytes(hdr)))
        flash[slot:slot + HDR_LEN + PAYLOAD_LEN] = bytes(hdr) + payload
    print("[5] firmware written into both partitions")

    for h in PIB_HEADERS:
        n = struct.unpack_from("<I", flash, h + 0x10)[0]
        p = h + HDR_LEN
        flash[p + 0x0c:p + 0x12] = args.mac
        flash[p + 0x102] = 1
        flash[p + 0x103:p + 0x109] = args.mac
        struct.pack_into("<I", flash, h + 0x14, checksum32(bytes(flash[p:p + n])))
        struct.pack_into("<I", flash, h + 0x5c, checksum32(bytes(flash[h:h + 0x5c])))
    mac_text = ":".join("%02X" % b for b in args.mac)
    print("[6] MAC set to " + mac_text)

    for slot in SLOTS:
        p = slot + HDR_LEN
        payload = bytes(flash[p:p + PAYLOAD_LEN])
        if checksum32(bytes(flash[slot:slot + HDR_LEN])) != 0 or \
                checksum32(payload) != struct.unpack_from("<I", flash, slot + 0x14)[0]:
            fail("internal error: partition checksum")
        dec = lzma.LZMADecompressor(format=lzma.FORMAT_RAW, filters=FILTERS)
        if sha(dec.decompress(payload[SUB_LEN:], max_length=IMAGE_LEN)) != PATCHED_SHA256:
            fail("internal error: partition does not decompress to the patched firmware")
    for h in PIB_HEADERS:
        n = struct.unpack_from("<I", flash, h + 0x10)[0]
        if checksum32(bytes(flash[h:h + HDR_LEN])) != 0 or \
                checksum32(bytes(flash[h + HDR_LEN:h + HDR_LEN + n])) != struct.unpack_from("<I", flash, h + 0x14)[0]:
            fail("internal error: PIB checksum")
    if firmware_hash(flash) != FIRMWARE_SHA256:
        fail("internal error: the firmware partitions are not as expected")
    if input_sha == REF_INPUT_SHA256 and mac_text == DEFAULT_MAC.upper() and sha(flash) != OUTPUT_SHA256:
        fail("internal error: the output image is not as expected (SHA-256 %s)" % sha(flash))
    print("[7] self-check OK")

    with open(args.out, "wb") as f:
        f.write(flash)
    print("\nwritten: %s (%d bytes, SHA-256 %s)" % (args.out, len(flash), sha(flash)))
    print("Keep a backup of your modem's original flash before writing this image.")


if __name__ == "__main__":
    main()
