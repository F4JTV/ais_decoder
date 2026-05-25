#pragma once
#include <dsp/processor.h>
#include <dsp/demod/quadrature.h>
#include <dsp/correction/dc_blocker.h>
#include <dsp/loop/fast_agc.h>
#include <dsp/filter/fir.h>
#include <dsp/taps/root_raised_cosine.h>
#include <dsp/clock_recovery/mm.h>
#include <dsp/digital/binary_slicer.h>
#include <vector>

// DSP front-end for AIS (ITU-R M.1371): 9600 baud GMSK, h=0.5, BT=0.4.
//
// Pipeline (per VFO output sample):
//   complex IQ -> AGC -> FM discriminator -> DC block -> RRC matched filter
//              -> M&M symbol clock recovery -> soft symbols -> binary slicer
//
// Outputs:
//   out  : one hard channel bit (0/1) per recovered symbol  (still NRZI encoded;
//          the ais::Decoder performs the NRZI decode + HDLC framing)
class AISDSP : public dsp::Processor<dsp::complex_t, uint8_t> {
    using base_type = dsp::Processor<dsp::complex_t, uint8_t>;
public:
    AISDSP() {}
    AISDSP(dsp::stream<dsp::complex_t>* in, double samplerate, double baudrate) {
        init(in, samplerate, baudrate);
    }

    void init(dsp::stream<dsp::complex_t>* in, double samplerate, double baudrate) {
        _samplerate = samplerate;
        _baudrate = baudrate;

        // AGC to normalize amplitude before the discriminator.
        agc.init(NULL, 1.0, 1e6, 0.01);
        // FM discriminator. AIS peak deviation ~2400 Hz; scale so symbols ~ +/-0.5.
        demod.init(NULL, 4800.0, samplerate);
        // Remove residual frequency offset so the slicer threshold stays at 0.
        dcBlock.init(NULL, 0.0001);
        // Root-raised-cosine matched filter.
        rrcTaps = dsp::taps::rootRaisedCosine<float>(31, 0.4, baudrate, samplerate);
        rrc.init(NULL, rrcTaps);
        // Mueller & Muller symbol clock recovery.
        recov.init(NULL, samplerate / baudrate, 1e-6, 0.01, 0.01);

        // We drive the blocks manually in process(); free their output streams
        // (demod.out is reused as a scratch buffer, so keep it).
        agc.out.free();
        dcBlock.out.free();
        rrc.out.free();
        recov.out.free();

        base_type::init(in);
    }

    int process(int count, dsp::complex_t* in, float* softOut, uint8_t* out) {
        agc.process(count, in, in);
        demod.process(count, in, demod.out.readBuf);
        dcBlock.process(count, demod.out.readBuf, demod.out.readBuf);
        rrc.process(count, demod.out.readBuf, demod.out.readBuf);
        count = recov.process(count, demod.out.readBuf, softOut);
        dsp::digital::BinarySlicer::process(count, softOut, out);
        return count;
    }

    int run() {
        int count = base_type::_in->read();
        if (count < 0) { return -1; }

        count = process(count, base_type::_in->readBuf, softScratch, base_type::out.writeBuf);

        base_type::_in->flush();
        if (count) {
            if (!base_type::out.swap(count)) { return -1; }
        }
        return count;
    }

private:
    dsp::loop::FastAGC<dsp::complex_t> agc;
    dsp::demod::Quadrature demod;
    dsp::correction::DCBlocker<float> dcBlock;
    dsp::tap<float> rrcTaps;
    dsp::filter::FIR<float, float> rrc;
    dsp::clock_recovery::MM<float> recov;

    // Scratch buffer for recovered soft symbols. Required as the M&M output
    // target, but not exposed: there is no constellation/eye diagram anymore,
    // so we don't publish a stream (which would otherwise need a consumer to
    // avoid blocking on swap()). Heap-allocated (STREAM_BUFFER_SIZE is large).
    std::vector<float> softScratchBuf = std::vector<float>(STREAM_BUFFER_SIZE);
    float* softScratch = softScratchBuf.data();

    double _samplerate = 48000.0;
    double _baudrate = 9600.0;
};
