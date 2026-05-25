// Round-trip validation of ais::Decoder (no SDR++ deps).
// Encodes known AIS messages into HDLC+NRZI channel bits exactly the way the
// over-the-air signal would carry them, feeds them through the decoder, and
// checks the parsed fields. This validates: NRZI decode, HDLC flag sync,
// bit de-stuffing, X.25 FCS check, byte reconstruction and field parsing.
#include "ais/decoder.h"
#include <vector>
#include <string>
#include <cstdio>
#include <cstdint>
#include <cmath>
#include <cassert>

// ---- bitfield writers (mirror of the decoder's getUint, MSB-first) --------
static void setUint(std::vector<uint8_t>& b, int start, int len, uint64_t val) {
    for (int i = 0; i < len; i++) {
        int bit = start + i;
        int byteIdx = bit >> 3;
        if ((int)b.size() <= byteIdx) { b.resize(byteIdx + 1, 0); }
        uint8_t x = (val >> (len - 1 - i)) & 1;
        if (x) { b[byteIdx] |= (1 << (7 - (bit & 7))); }
    }
}
static void setInt(std::vector<uint8_t>& b, int start, int len, int64_t val) {
    setUint(b, start, len, (uint64_t)val & ((len < 64) ? ((1ULL << len) - 1) : ~0ULL));
}
static void setString(std::vector<uint8_t>& b, int start, int chars, const std::string& s) {
    for (int i = 0; i < chars; i++) {
        int c = (i < (int)s.size()) ? (unsigned char)s[i] : 0; // '@' pad = 0
        int v = (c >= 64) ? (c - 64) : c;                      // inverse of sixbitChar
        setUint(b, start + 6 * i, 6, (uint64_t)(v & 0x3F));
    }
}

// ---- HDLC transmit FCS (reflected CRC-16/X.25), produces the 16 air bits ---
static void appendFCS(std::vector<uint8_t>& airbits) {
    uint16_t crc = 0xFFFF;
    for (uint8_t d : airbits) {
        bool bit = (d ^ (crc & 1)) & 1;
        crc >>= 1;
        if (bit) { crc ^= 0x8408; }
    }
    crc ^= 0xFFFF;                          // ones-complement
    for (int j = 0; j < 16; j++) {          // transmitted LSB-first
        airbits.push_back((crc >> j) & 1);
    }
}

// Encode a big-endian bitfield message (length bits) into NRZI channel bits.
static std::vector<uint8_t> encode(const std::vector<uint8_t>& bytes, int bitLen) {
    // 1) data air bits: each byte transmitted LSB-first
    std::vector<uint8_t> air;
    int nBytes = (bitLen + 7) / 8;
    for (int n = 0; n < nBytes; n++) {
        uint8_t by = (n < (int)bytes.size()) ? bytes[n] : 0;
        for (int i = 0; i < 8; i++) { air.push_back((by >> i) & 1); }
    }
    // 2) FCS
    appendFCS(air);

    // sanity: receiver residual must be the X.25 "good" value
    {
        uint16_t fcs = 0xFFFF;
        for (uint8_t d : air) { bool bit = (d ^ (fcs & 1)) & 1; fcs >>= 1; if (bit) fcs ^= 0x8408; }
        assert(fcs == 0xF0B8 && "FCS generation incorrect");
    }

    // 3) bit-stuff (insert 0 after five consecutive 1s)
    std::vector<uint8_t> stuffed;
    int ones = 0;
    for (uint8_t bit : air) {
        stuffed.push_back(bit);
        if (bit) { if (++ones == 5) { stuffed.push_back(0); ones = 0; } }
        else { ones = 0; }
    }

    // 4) wire bits: preamble + flag + stuffed + flag
    std::vector<uint8_t> wire;
    for (int i = 0; i < 24; i++) { wire.push_back(i & 1); }   // 010101... preamble
    const uint8_t flag[8] = { 0, 1, 1, 1, 1, 1, 1, 0 };       // 0x7E
    for (int i = 0; i < 8; i++) { wire.push_back(flag[i]); }
    for (uint8_t bit : stuffed) { wire.push_back(bit); }
    for (int i = 0; i < 8; i++) { wire.push_back(flag[i]); }
    for (int i = 0; i < 8; i++) { wire.push_back(0); }        // trailing

    // 5) NRZI encode: bit 1 -> no transition, bit 0 -> transition
    std::vector<uint8_t> chan;
    uint8_t level = 0;
    chan.push_back(level);                                    // initial reference
    for (uint8_t w : wire) {
        if (w == 0) { level ^= 1; }
        chan.push_back(level);
    }
    return chan;
}

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { printf("  FAIL: %s\n", msg); failures++; } else { printf("  ok:   %s\n", msg); } } while(0)

int main() {
    // ================= Test 1: Type 1 position report =================
    printf("== Type 1 (position report) ==\n");
    {
        std::vector<uint8_t> b(21, 0); // 168 bits
        setUint(b, 0, 6, 1);                 // type 1
        setUint(b, 6, 2, 0);                 // repeat
        setUint(b, 8, 30, 227006760);        // MMSI
        setUint(b, 38, 4, 0);                // nav status: under way
        setUint(b, 50, 10, 100);             // SOG = 10.0 kn
        setInt (b, 61, 28, (int64_t)llround(5.37 * 600000.0));   // lon
        setInt (b, 89, 27, (int64_t)llround(43.295 * 600000.0)); // lat
        setUint(b, 116, 12, 875);            // COG = 87.5
        setUint(b, 128, 9, 88);              // heading

        ais::Decoder dec;
        ais::Message got; bool fired = false;
        dec.onMessage = [&](const ais::Message& m) { got = m; fired = true; };
        auto chan = encode(b, 168);
        dec.process(chan.data(), (int)chan.size());

        CHECK(fired, "message fired");
        CHECK(dec.framesOk == 1 && dec.framesCrcFail == 0, "exactly one CRC-valid frame");
        CHECK(got.type == 1, "type == 1");
        CHECK(got.mmsi == 227006760u, "mmsi == 227006760");
        CHECK(got.navStatus == 0, "navStatus == 0");
        CHECK(got.hasSpeed && fabs(got.sog - 10.0) < 1e-6, "sog == 10.0");
        CHECK(got.hasPosition && fabs(got.lon - 5.37) < 1e-4, "lon == 5.37");
        CHECK(got.hasPosition && fabs(got.lat - 43.295) < 1e-4, "lat == 43.295");
        CHECK(got.hasCog && fabs(got.cog - 87.5) < 1e-6, "cog == 87.5");
        CHECK(got.heading == 88, "heading == 88");
    }

    // ================= Test 2: Type 5 static/voyage =================
    printf("== Type 5 (static & voyage) ==\n");
    {
        std::vector<uint8_t> b(53, 0); // 424 bits
        setUint(b, 0, 6, 5);                 // type 5
        setUint(b, 8, 30, 227006760);        // MMSI
        setString(b, 70, 7, "TEST123");        // callsign
        setString(b, 112, 20, "TEST VESSEL");// name
        setUint(b, 232, 8, 70);              // ship type: cargo
        setString(b, 302, 20, "PORTVILLE");  // destination

        ais::Decoder dec;
        ais::Message got; bool fired = false;
        dec.onMessage = [&](const ais::Message& m) { got = m; fired = true; };
        auto chan = encode(b, 424);
        dec.process(chan.data(), (int)chan.size());

        CHECK(fired, "message fired");
        CHECK(dec.framesOk == 1 && dec.framesCrcFail == 0, "exactly one CRC-valid frame");
        CHECK(got.type == 5, "type == 5");
        CHECK(got.mmsi == 227006760u, "mmsi == 227006760");
        CHECK(got.name == "TEST VESSEL", ("name == 'TEST VESSEL' (got '" + got.name + "')").c_str());
        CHECK(got.callsign == "TEST123", ("callsign == 'TEST123' (got '" + got.callsign + "')").c_str());
        CHECK(got.shipType == 70, "shipType == 70");
        CHECK(got.destination == "PORTVILLE", ("destination == 'PORTVILLE' (got '" + got.destination + "')").c_str());
    }

    // ================= Test 3: deliberately corrupted CRC =================
    printf("== Corrupted frame (must be rejected) ==\n");
    {
        std::vector<uint8_t> b(21, 0);
        setUint(b, 0, 6, 1);
        setUint(b, 8, 30, 123456789);
        setInt (b, 61, 28, (int64_t)llround(1.0 * 600000.0));
        setInt (b, 89, 27, (int64_t)llround(2.0 * 600000.0));
        auto chan = encode(b, 168);
        // Flip one channel bit in the middle of the payload to break the CRC.
        chan[chan.size() / 2] ^= 1;

        ais::Decoder dec;
        bool fired = false;
        dec.onMessage = [&](const ais::Message&) { fired = true; };
        dec.process(chan.data(), (int)chan.size());

        CHECK(!fired, "corrupted message NOT delivered");
        CHECK(dec.framesOk == 0, "no CRC-valid frames");
    }

    printf("\n%s (%d failure(s))\n", failures == 0 ? "ALL TESTS PASSED" : "TESTS FAILED", failures);
    return failures == 0 ? 0 : 1;
}
