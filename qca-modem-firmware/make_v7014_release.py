#!/usr/bin/env python3
"""
make_v7014_release.py - build the QCA7005 flash image 261102-V7014-0730 for ccs32dave.

This is the "unjoined in-modem decrypt" sniffer firmware (V7014). Unlike PINGPONG-RELEASE-5, the
modem does NOT join the powerline network. It passively overhears the ISO 15118 / DIN 70121
charging traffic, recovers the session key in the modem, decrypts the payload in the modem, and
delivers the plaintext to the host as native IPv6 (0x86DD) frames plus a 250 ms status frame.

This script contains NO firmware. It needs an original 2 MB SPI flash image with the QCA7005
firmware MAC-QCA7005-1.1.0.730-04-20140815-CS, which you obtain yourself. Apart from the per-device parameter blocks (PIB:
MAC address, keys), the image must be identical to the one this release was tested with; the script
checks this and refuses anything else. The file name doesn't matter; the default is original_1.1.0.730-04-CS-20140815.bin.
It applies the ccs32dave changes (a byte diff of only the changed bytes - no original firmware) to
the firmware in that dump, sets a new MAC address and writes a complete 2 MB image for the
QCA7005's SPI flash. Only Python 3 is needed, no packages. The exact same original dump works for
both this release and make_pingpong_release_5.py.

Usage:
    python make_v7014_release.py
    python make_v7014_release.py other_name.bin --mac 04:65:65:FF:FF:12 --out my.bin

Default output: qca_flash_V7014.bin. Default MAC: 04:65:65:FF:FF:11. The MAC must differ from every other modem
on the same line: images of this firmware often carry 04:65:65:FF:FF:FF, which many other boards
(e.g. Foccci) also use, and two modems with the same MAC break the traffic in confusing ways.

Flashing is up to you (any SPI flash programmer). One way: a Raspberry Pi with flashrom, the
QCA7005 held in reset while flashing. Read and keep a backup of the original flash first.

Tested on (the compressor must produce a bit-exact result, the script checks this):
    - Windows 10 x64, Python 3.11.7 (python.org)

DISCLAIMER - use at your own risk, no warranty of any kind:
  * Flashing can leave the modem unusable until it is flashed again with a working image.
  * This firmware is a PASSIVE SNIFFER that decrypts overheard charging traffic in the modem. It is
    meant for analysis on your OWN equipment. Recovering and decrypting communications you are not
    authorised to observe may be unlawful in your jurisdiction. Use it only where you are permitted
    to, and be aware that even a passive, non-joining station in range can in principle affect a
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

TAG = "261102-V7014-0730"
INPUT_NAME = "original_1.1.0.730-04-CS-20140815.bin"
FW_VERSION = "MAC-QCA7005-1.1.0.730-04-20140815-CS"
REF_INPUT_SHA256 = "fb8b44adfd73b3ed11e048787d244fbca5057b192453f4164f08d6f5c938c2db"
MASKED_SHA256 = "8175b217a4a793256593d530ff04e45e0b0f9328dbbfc5f546bbb764076b910a"
INPUT_LEN = 2097152
STOCK_SHA256 = "6f4656c7354111cf253a78a25cb9ba4e4cb4b9ad0daac7ea6014fb60717ae382"
PATCHED_SHA256 = "08f67177a67e9f3a9fb3b171028fa0d49d71d263c2b3ac5a091a2ddf8f0482a7"
IMAGE_LEN = 487732
STREAM_LEN = 307301
STREAM_SHA256 = "616d4a975dce3efec19559aa24ad990c88ffc4a31e8165a6b62b618ba4cf2aef"
FIRMWARE_SHA256 = "788b222f5aef3c503e6d1b9c80a0a22fffe11bd016dd567aa2323c33edbfa52f"
OUTPUT_SHA256 = "dc5606fa7380cea0641c0fe38c0650f52984299d18286adf6f9ebca096e4694e"
DEFAULT_MAC = "04:65:65:FF:FF:11"
OUT_NAME = "qca_flash_V7014.bin"
ISSUES_URL = "https://github.com/uhi22/ccs32dave/issues"
TESTED = [
    'Windows 10 x64, Python 3.11.7 (python.org)',
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
aAApINESSQpoS2iSC5sEGkMQSQtoHyELQFsBDklbGBpgDkkJaFlgAQAYOQpommBKaNpgimgaYYJo
WmHKaJphBEkKaAEyCmAPvJHmlHAlAABBEQCkcCUAAK8wAJBwJQAKtBJIAmgfIxNA2wEQSBsYEEgp
AUAYBCIBaBlgBDAEMwE6+dEBmIBoCCIBaBlgBDAEMwE6+dEBmMBoCCIBaBlgBDAEMwE6+dEKvFrg
tHAlAACzMACgjC8AD7RaSABoACgA0KvgAC0D0GZIAWgBMQFgVUgBaAExAWBYSAJoQ2iSC5sEGkNW
SAJgUEgCaD8jE0BbAU9IGxgZYF1gBpkIaJhgSGjYYIhoGGHIaFhhCGmYYTBo2GFFSAFoATEBYAaZ
SmhESAFoHyMLQIwgWEOAAEJJQxg9SABoGGBaYEJIkgCAGABomGCAadhgQWkZYT5IAWA7SABoWGEI
DQIoDtMEKAzSfucDIIFDIDOBIgAqBdAIaBhgBDEEMwE69+cuSAFoATEBYDFIAWgAKUjQCA0CKEXT
BChD0oh8iChA0ch84Sg90Up9iH0AAgJDKEiCQgDR7+YnSIJCBdEnSAAjA2AmSAEjA2AlSANoI0gA
aAArAtEPIxhAAeAQKCHSAwIgSVsYEkkJaBlgWGCaYBdJCWjZYBRIAGgYYQMggUMYMzoiACoF0Aho
GGAEMQQzATr35xFIAWgBMQFgEkgBaAExAWAPvAiYAHxwR5RwJQCQcCUAsHAlAAAnMAC0cCUAAC8w
AABBEQDUcCUA6LsIAMxwJQAGYAAAfWAAAMRwJQDIcCUAAI0wAMBwJQDccCUAez8GAAEA4D5ABgAC
AMBGaEAGAAQAVvLm+Ih/BgAEAFLyePkemgYABABQ8uH8qp0GAAQAUPLd+VTsBgAEAEvySPoc8AYA
BABL8uD4vAUHAAQARHcI6g==
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
