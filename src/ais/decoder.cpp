#include "decoder.h"
#include <cmath>

namespace ais {

    // ---------------------------------------------------------------------
    // Bit-field helpers. The reconstructed message bytes form a big-endian
    // bitfield (bit 0 = MSB of byte 0), per the AIS / ITU-R M.1371 convention.
    // ---------------------------------------------------------------------
    static uint64_t getUint(const std::vector<uint8_t>& b, int start, int len) {
        uint64_t v = 0;
        for (int i = 0; i < len; i++) {
            int bit = start + i;
            uint8_t x = (b[bit >> 3] >> (7 - (bit & 7))) & 1;
            v = (v << 1) | x;
        }
        return v;
    }

    static int64_t getInt(const std::vector<uint8_t>& b, int start, int len) {
        uint64_t v = getUint(b, start, len);
        if (len < 64 && (v & (1ULL << (len - 1)))) {
            v |= ~((1ULL << len) - 1); // sign extend
        }
        return (int64_t)v;
    }

    static char sixbitChar(int v) {
        return (v < 32) ? (char)(v + 64) : (char)v;
    }

    static std::string getString(const std::vector<uint8_t>& b, int start, int chars, int totalBits) {
        std::string s;
        for (int i = 0; i < chars; i++) {
            int pos = start + 6 * i;
            if (pos + 6 > totalBits) break;
            s += sixbitChar((int)getUint(b, pos, 6));
        }
        // Trim trailing pad ('@') and spaces.
        while (!s.empty() && (s.back() == '@' || s.back() == ' ')) s.pop_back();
        // Replace leading/embedded '@' that some encoders use as padding.
        for (auto& c : s) { if (c == '@') c = ' '; }
        return s;
    }

    // RFC 1662 / X.25 HDLC frame check sequence (reflected CRC-16-CCITT).
    // Run over the de-stuffed bits in air order (data + 16-bit FCS).
    // A valid frame yields the magic residual 0xF0B8.
    static uint16_t hdlcResidual(const std::vector<uint8_t>& airbits) {
        uint16_t fcs = 0xFFFF;
        for (uint8_t b : airbits) {
            bool bit = (b ^ (fcs & 1)) & 1;
            fcs >>= 1;
            if (bit) fcs ^= 0x8408;
        }
        return fcs;
    }

    // ---------------------------------------------------------------------
    // Streaming entry point.
    // ---------------------------------------------------------------------
    void Decoder::process(const uint8_t* bits, int count) {
        for (int i = 0; i < count; i++) {
            uint8_t ch = bits[i] & 1;
            if (prevChannelBit < 0) { prevChannelBit = ch; continue; }
            // NRZI decode: no transition -> 1, transition -> 0.
            // Immune to global FM-discriminator polarity inversion.
            uint8_t decoded = (ch == (uint8_t)prevChannelBit) ? 1 : 0;
            prevChannelBit = ch;
            pushDecodedBit(decoded);
        }
    }

    void Decoder::reset() {
        prevChannelBit = -1;
        inFrame = false;
        onesCount = 0;
        frameBits.clear();
    }

    // HDLC bit-level receiver with on-the-fly de-stuffing and flag detection.
    void Decoder::pushDecodedBit(uint8_t d) {
        if (d == 1) {
            onesCount++;
            if (inFrame) frameBits.push_back(1);
        }
        else { // d == 0
            if (onesCount == 5) {
                // Stuffed zero: drop it (do not store).
            }
            else if (onesCount == 6) {
                // Flag 0x7E (01111110) detected.
                if (inFrame) {
                    // Remove the flag's leading 0 and its six 1s that we just
                    // appended (7 bits total) before validating.
                    int rm = 7;
                    while (rm-- > 0 && !frameBits.empty()) frameBits.pop_back();
                    handleFrame();
                }
                // A flag also opens the next frame.
                inFrame = true;
                frameBits.clear();
            }
            else if (onesCount >= 7) {
                // Abort sequence / line error: resync.
                inFrame = false;
                frameBits.clear();
            }
            else {
                // Regular data zero.
                if (inFrame) frameBits.push_back(0);
            }
            onesCount = 0;
        }

        // Guard against runaway buffers on noise (longest AIS frame << 1024 bits).
        if (frameBits.size() > 2048) {
            inFrame = false;
            frameBits.clear();
        }
    }

    void Decoder::handleFrame() {
        const int bitLen = (int)frameBits.size();
        // Minimum: smallest AIS payload (160 bits) + 16-bit FCS.
        if (bitLen < (160 + 16)) return;

        // FCS check over the full air-order bitstream.
        if (hdlcResidual(frameBits) != 0xF0B8) {
            framesCrcFail++;
            return;
        }
        framesOk++;

        // Data portion (everything but the trailing 16-bit FCS).
        int dataBits = bitLen - 16;
        if (dataBits % 8 != 0) return;     // AIS messages are octet aligned
        int nBytes = dataBits / 8;

        // Reconstruct message bytes. Each octet was transmitted LSB-first,
        // so pack received bits LSB-first to recover the big-endian bitfield.
        std::vector<uint8_t> bytes(nBytes, 0);
        for (int n = 0; n < nBytes; n++) {
            for (int i = 0; i < 8; i++) {
                if (frameBits[8 * n + i]) bytes[n] |= (1 << i);
            }
        }

        Message msg;
        msg.bitLength = dataBits;
        if (parse(bytes, dataBits, msg)) {
            messagesParsed++;
            if (onMessage) onMessage(msg);
        }
    }

    bool Decoder::parse(const std::vector<uint8_t>& b, int bitLen, Message& msg) {
        if (bitLen < 38) return false;
        msg.type = (int)getUint(b, 0, 6);
        msg.mmsi = (uint32_t)getUint(b, 8, 30);
        if (msg.type < 1 || msg.type > 27) return false;

        auto setPosition = [&](int lonStart, int latStart) {
            // 1/10000-minute units. The "not available" sentinels are 181 deg
            // (lon) and 91 deg (lat); both are caught by the range check below.
            double lon = getInt(b, lonStart, 28) / 600000.0;
            double lat = getInt(b, latStart, 27) / 600000.0;
            if (lon < -180.0 || lon > 180.0 || lat < -90.0 || lat > 90.0) return;
            msg.lon = lon; msg.lat = lat; msg.hasPosition = true;
        };
        auto setSpeed = [&](int sogStart) {
            int sog = (int)getUint(b, sogStart, 10);
            if (sog != 1023) { msg.sog = sog / 10.0; msg.hasSpeed = true; }
        };
        auto setCourse = [&](int cogStart) {
            int cog = (int)getUint(b, cogStart, 12);
            if (cog != 3600 && cog <= 3599) { msg.cog = cog / 10.0; msg.hasCog = true; }
        };
        auto setHeading = [&](int hdgStart) {
            int hdg = (int)getUint(b, hdgStart, 9);
            if (hdg != 511) msg.heading = hdg;
        };

        switch (msg.type) {
        case 1: case 2: case 3: // Class A position report
            if (bitLen < 168) return false;
            msg.navStatus = (int)getUint(b, 38, 4);
            setSpeed(50);
            setPosition(61, 89);
            setCourse(116);
            setHeading(128);
            return true;

        case 4:   // Base station report
        case 11:  // UTC/date response (same layout)
            if (bitLen < 168) return false;
            setPosition(79, 107);
            return true;

        case 5: { // Class A static & voyage related data
            if (bitLen < 424) return false;
            msg.callsign    = getString(b, 70, 7, bitLen);
            msg.name        = getString(b, 112, 20, bitLen);
            msg.shipType    = (int)getUint(b, 232, 8);
            msg.destination = getString(b, 302, 20, bitLen);
            return true;
        }

        case 18: // Class B position report
            if (bitLen < 168) return false;
            setSpeed(46);
            setPosition(57, 85);
            setCourse(112);
            setHeading(124);
            return true;

        case 19: // Class B extended position report
            if (bitLen < 312) return false;
            setSpeed(46);
            setPosition(57, 85);
            setCourse(112);
            setHeading(124);
            msg.name     = getString(b, 143, 20, bitLen);
            msg.shipType = (int)getUint(b, 263, 8);
            return true;

        case 21: // Aid-to-Navigation report
            if (bitLen < 272) return false;
            msg.aidType = (int)getUint(b, 38, 5);
            msg.name    = getString(b, 43, 20, bitLen);
            setPosition(164, 192);
            return true;

        case 24: { // Static data report (Part A or Part B)
            if (bitLen < 160) return false;
            int part = (int)getUint(b, 38, 2);
            if (part == 0) {                          // Part A: name
                msg.name = getString(b, 40, 20, bitLen);
            }
            else if (part == 1 && bitLen >= 168) {    // Part B: type + callsign
                msg.shipType = (int)getUint(b, 40, 8);
                msg.callsign = getString(b, 90, 7, bitLen);
            }
            return true;
        }

        default:
            // Known type but not specifically parsed; still report MMSI so the
            // map at least sees the contact exists.
            return true;
        }
    }
}
