// Flanger.h
// HH Electronics Clockwork Concubine flanger (drawing by Dirk Hendrik,
// rev A, 5 October 2007 / 21 September 2008), modelled from the schematic:
//
//   input (J1 pin 2) -> C4 100n / R14 100K -> Q1 emitter follower (22K)
//   -> C8 6u8 -> R19 100K into IC2's inverting input
//   IC2 (741): 47K-47K T network feedback with 4K7 + 4n7 to ground
//      (pre-emphasis, about +15 dB above 1.2 kHz), and the RESONANCE
//      feedback on its + input:
//        wet node -> RX1 6K8 -> (C18 47n + R37 470) and VR4 10K lin to ground
//        -> VR4 wiper -> C5 4n7 / R20 100K -> IC2 pin 3
//   -> C12 100n (R30 100K to the VR3 bias trim) -> U1 SAD1024, both
//      512-stage sections in parallel (inputs 2 and 15 joined, outputs 6 and
//      12 joined), loaded by R32 10K and C13 10n
//   -> C16 47n -> wet node
//   IC5 (741) mixer: dry from IC2 through R31 47K, wet through RX2 10K /
//      CX 47n + R39 2K2 to ground / R40 22K (de-emphasis), feedback
//      R33 22K || CX2 6n8
//   -> C19 6u8 / R38 47K -> S1.A (effect / bypass) -> J1 pin 4
//
//   Clock: IC1.A, Q2 (BC208 current source) and C1 470p make a ramp; IC1.D
//   compares it with the DEPTH pot's wiper, so the ramp time (the BBD clock
//   period) follows the LFO; U2 (4013) divides it into the BBD's clock.
//   LFO: IC1.B / C2 680n integrator, IC1.C and the other half of U2 as the
//   Schmitt trigger, FREQUENCY = VR2 1M reverse log.
//
// Modelled as a circuit: every resistor and capacitor in the audio path,
// the 741s on the +/-6V8 zener rails (swing and slew), the BBD as a clocked
// sample-and-hold bucket chain (two sections, clocks in antiphase, outputs
// averaged), its clock aliasing and staircase output.
// Modelled behaviourally (from what the drawing shows, values estimated): the
// clock oscillator's delay range, the LFO's rate range and shape, the
// SAD1024's gain, output impedance and clipping level. All are named
// constants in Flanger.cpp.
//
// Volts in, volts out (1.0 = 1 V). Runs at 4x with Oversampler4x.
// Plain C++98, no allocation after construction.

#ifndef TBCC_FLANGER_H
#define TBCC_FLANGER_H

#include "Filters.h"
#include "Oversampler4x.h"

namespace tbcc
{

class Flanger
{
public:
    Flanger();

    void prepare (double sampleRate);
    void reset();

    // Front panel (0..1 pot rotation)
    void setFrequency (double v);       // VR2 1M rev log: LFO rate
    void setDepth (double v);           // VR1 100K lin: sweep depth
    void setResonance (double v);       // VR4 10K lin: feedback
    void setEffectOn (bool on);         // S1: effect in / bypass
    // Plugin only
    void setInputGainDb (double dB);
    void setOutputGainDb (double dB);

    // Mono in, mono out
    void process (const float* in, float* out, int numSamples);

    static int latencySamples() { return Oversampler4x::latencySamples(); }

    // Introspection for the panel and tests
    static double lfoHz (double frequencyPot);
    static double delayAtDepthZeroMs();
    static double delayRangeMs (double depthPot, bool longest);   // full LFO swing
    double currentDelayMs() const;
    double currentClockHz() const;      // the BBD clock (each section)
    double lfoValue() const { return lfoTri; }                   // -1..1
    double peakBbd() const { return peakBbdLevel; }
    void   clearPeakBbd() { peakBbdLevel = 0.0; }

    enum { kStages = 512, kRing = 2048 };

private:
    double tickOs (double x);
    void   updateFeedbackFilter();

    double fsBase, fsOs;
    Oversampler4x up, down;

    // Targets and smoothed values
    double freqT, depthT, resoT, effectT, inT, outT;
    double freq, depth, reso, effect, inG, outG;
    double smKnob, smRate, smSwitch;

    // Audio path
    FirstOrder inHp;            // C4 / R14
    FirstOrder couple;          // C8 into R19
    FirstOrder preEmph;         // IC2 T network / R19
    FirstOrder fbNet;           // RX1, C18 + R37, VR4 track
    FirstOrder fbHp;            // C5 / R20
    FirstOrder bbdHp;           // C12 / R30
    FirstOrder bbdLp;           // SAD1024 output impedance into C13
    FirstOrder wetHp;           // C16 into the wet node's load
    FirstOrder deEmph;          // RX2, CX + R39, R40
    FirstOrder mixLp;           // IC5: R33 || CX2
    FirstOrder outHp;           // C19 / R38
    Slew slewIc2, slewIc5;

    // BBD: one stream at twice the clock (the two sections interleaved)
    float ring[kRing];
    int ringPos;
    double tickPhase, bbdHeld, lastBbdIn, wetNode;

    // LFO
    double lfoPhase, lfoTri;

    double lastDelay;
    double peakBbdLevel;
};

} // namespace tbcc

#endif
