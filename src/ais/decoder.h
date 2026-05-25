#pragma once
#include <stdint.h>
#include <string>
#include <vector>
#include <functional>

namespace ais {

    // A decoded AIS message. Fields that aren't present in a given message type
    // (or are flagged "not available" over the air) are left at their default /
    // "has*" flags are false.
    struct Message {
        int type = 0;                  // AIS message type (1..27)
        uint32_t mmsi = 0;             // Maritime Mobile Service Identity

        bool hasPosition = false;
        double lat = 0.0;              // degrees North (valid if hasPosition)
        double lon = 0.0;              // degrees East  (valid if hasPosition)

        bool hasSpeed = false;
        double sog = 0.0;              // speed over ground, knots

        bool hasCog = false;
        double cog = 0.0;              // course over ground, degrees

        int heading = -1;              // true heading in degrees, -1 = N/A
        int navStatus = -1;            // navigation status code, -1 = N/A

        std::string name;              // vessel / AtoN name (types 5/19/21/24A)
        std::string callsign;          // call sign (type 5/24B)
        std::string destination;       // voyage destination (type 5)
        int shipType = -1;             // ship/cargo type code, -1 = N/A
        int aidType = -1;              // aid-to-navigation type (type 21), -1 = N/A

        int bitLength = 0;             // length of the message in bits
    };

    // Streaming HDLC + AIS decoder.
    //
    // Feed it the demodulated NRZI channel bits (one slice per symbol) via
    // process(). For every frame that passes the HDLC FCS (CRC-16/X.25) check
    // and is successfully parsed, onMessage() is invoked.
    class Decoder {
    public:
        // bits: array of 0/1 channel bits straight from the binary slicer
        //       (these are still NRZI encoded; this class does the NRZI decode).
        void process(const uint8_t* bits, int count);

        // Reset all HDLC/NRZI state (e.g. when restarting the DSP).
        void reset();

        // Callback fired for each valid, parsed message.
        std::function<void(const Message&)> onMessage;

        // Running statistics (read-only, useful for the GUI).
        uint64_t framesOk = 0;         // frames that passed CRC
        uint64_t framesCrcFail = 0;    // frames that failed CRC
        uint64_t messagesParsed = 0;   // frames that produced a Message

    private:
        void pushDecodedBit(uint8_t d);
        void handleFrame();
        bool parse(const std::vector<uint8_t>& bytes, int bitLen, Message& msg);

        // NRZI state
        int prevChannelBit = -1;

        // HDLC framing state
        bool inFrame = false;
        int onesCount = 0;
        std::vector<uint8_t> frameBits; // de-stuffed payload bits (air order, incl. FCS)
    };
}
