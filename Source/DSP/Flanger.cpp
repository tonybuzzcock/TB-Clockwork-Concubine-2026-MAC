// Flanger.cpp
// See Flanger.h for the signal path. Component values are from the drawing;
// the estimates (things the drawing doesn't fix) are marked.

#include "Flanger.h"

#include <cmath>
#include <cstring>

namespace tbcc
{

namespace
{
    // Supplies: D3 / D4 6V8 zeners from the external supply -> 741s on +/-6.8 V
    const double kRail     = 5.3;              // 741 output swing on +/-6.8 V
    const double kSlewRate = 0.5e6;            // V/s (741)
    const double kQ1Swing  = 5.5;              // emitter follower headroom

    // Input: C4 100n into R14 100K; C8 6u8 into R19 100K
    const double kC4 = 100.0e-9, kR14 = 100.0e3, kC8 = 6.8e-6, kR19 = 100.0e3;
    // IC2 T network: R21 47K, R26 47K, R24 4K7 + C11 4n7 to ground
    const double kR21 = 47.0e3, kR26 = 47.0e3, kR24 = 4.7e3, kC11 = 4.7e-9;
    // Resonance network: RX1 6K8, C18 47n + R37 470, VR4 10K lin, C5 4n7 / R20 100K
    const double kRX1 = 6.8e3, kC18 = 47.0e-9, kR37 = 470.0, kVR4 = 10.0e3, kC5 = 4.7e-9, kR20 = 100.0e3;
    // BBD input coupling: C12 100n, R30 100K
    const double kC12 = 100.0e-9, kR30 = 100.0e3;
    // BBD output: R32 10K load, C13 10n; C16 47n into the wet node
    const double kC13 = 10.0e-9, kC16 = 47.0e-9;
    // IC5 mixer: R31 47K dry; RX2 10K, CX 47n + R39 2K2, R40 22K wet; R33 22K || CX2 6n8
    const double kR31 = 47.0e3, kRX2 = 10.0e3, kCX = 47.0e-9, kR39 = 2.2e3, kR40 = 22.0e3;
    const double kR33 = 22.0e3, kCX2 = 6.8e-9;
    // Output: C19 6u8 into R38 47K
    const double kC19 = 6.8e-6, kR38 = 47.0e3;
    // LFO: VR2 1M rev log in series with the Thevenin resistance of R22 15K / R23 27K
    const double kVR2 = 1.0e6, kRth = 15.0e3 * 27.0e3 / 42.0e3;

    // --- Estimates (not fixed by the drawing) ---------------------------------
    // Clock oscillator: with DEPTH at 0 the BBD delay sits at kDelay0; the
    // ramp time follows the threshold linearly, so the delay swings
    // kDelay0 x (1 +/- kSweep) at full DEPTH (about 0.45 - 5.6 ms).
    const double kDelay0 = 3.0e-3;             // s
    const double kSweep  = 0.85;
    // LFO rate fully anticlockwise (VR2 at 1M); the top end follows from
    // the resistance ratio (about 10 Hz)
    const double kLfoSlowHz = 0.1;
    // SAD1024: gain through the chain, output resistance into C13, clipping
    const double kBbdGain = 0.9;
    const double kBbdRout = 1.5e3;
    const double kBbdClip = 1.6;               // V peak at the input, VR3 centred
    // The wet node's load seen by C16: (RX1 + VR4) || (RX2 + R40)
    const double kWetLoad = (kRX1 + kVR4) * (kRX2 + kR40) / (kRX1 + kVR4 + kRX2 + kR40);

    inline double audioTaper (double x)
    {
        const double k = 4.394449154672439;    // 10% of the track at mid-rotation
        if (x <= 0.0) return 0.0;
        if (x >= 1.0) return 1.0;
        return (std::exp (k * x) - 1.0) / (std::exp (k) - 1.0);
    }

    inline double clamp01 (double x) { return x < 0.0 ? 0.0 : (x > 1.0 ? 1.0 : x); }

    // The SAD1024's soft limit around its bias point
    inline double bbdClip (double v)
    {
        const double knee = 0.7 * kBbdClip;
        const double a = std::fabs (v);
        if (a <= knee) return v;
        const double c = knee + (kBbdClip - knee) * std::tanh ((a - knee) / (kBbdClip - knee));
        return v < 0.0 ? -c : c;
    }

    inline double highpassFc (double r, double c) { return 1.0 / (2.0 * kPi * r * c); }
}

//==============================================================================
double Flanger::lfoHz (double pot)
{
    // Rev-log track: the resistance in circuit falls fast at first as the knob
    // turns clockwise, so the rate climbs evenly on a log scale.
    const double r = kVR2 * audioTaper (1.0 - clamp01 (pot));
    return kLfoSlowHz * (kVR2 + kRth) / (r + kRth);
}

double Flanger::delayAtDepthZeroMs() { return 1000.0 * kDelay0; }

double Flanger::delayRangeMs (double depthPot, bool longest)
{
    const double d = clamp01 (depthPot) * kSweep;
    return 1000.0 * kDelay0 * (longest ? 1.0 + d : 1.0 - d);
}

double Flanger::currentDelayMs() const { return 1000.0 * lastDelay; }

double Flanger::currentClockHz() const
{
    return lastDelay > 0.0 ? (kStages / 2) / lastDelay : 0.0;
}

//==============================================================================
Flanger::Flanger()
    : fsBase (44100.0), fsOs (176400.0),
      freqT (0.4), depthT (0.7), resoT (0.5), effectT (1.0), inT (1.0), outT (1.0),
      ringPos (0), tickPhase (0.0), bbdHeld (0.0), lastBbdIn (0.0), wetNode (0.0),
      lfoPhase (0.25), lfoTri (0.0), lastDelay (kDelay0), peakBbdLevel (0.0)
{
    prepare (44100.0);
}

void Flanger::prepare (double sampleRate)
{
    fsBase = sampleRate > 1000.0 ? sampleRate : 44100.0;
    fsOs = fsBase * Oversampler4x::kFactor;

    smKnob   = std::exp (-1.0 / (0.020 * fsBase));
    smRate   = std::exp (-1.0 / (0.150 * fsBase));
    smSwitch = std::exp (-1.0 / (0.006 * fsBase));

    inHp.highpass (highpassFc (kR14, kC4), fsOs);
    couple.highpass (highpassFc (kR19, kC8), fsOs);

    // IC2: Zt / R19 with Zt = R21 + R26 + R21 R26 / (R24 + 1/sC11)
    preEmph.setAnalog (kC11 * ((kR21 + kR26) * kR24 + kR21 * kR26) / kR19, (kR21 + kR26) / kR19,
                       kR24 * kC11, 1.0, fsOs);

    // Wet node -> RX1 -> node with (R37 + C18) || VR4 to ground:
    //   (VR4 R37 C s + VR4) / ((RX1 (VR4 + R37) + VR4 R37) C s + RX1 + VR4)
    fbNet.setAnalog (kVR4 * kR37 * kC18, kVR4,
                     (kRX1 * (kVR4 + kR37) + kVR4 * kR37) * kC18, kRX1 + kVR4, fsOs);
    fbHp.highpass (highpassFc (kR20, kC5), fsOs);

    bbdHp.highpass (highpassFc (kR30, kC12), fsOs);
    {
        double f = 1.0 / (2.0 * kPi * kBbdRout * kC13);
        if (f > 0.45 * fsOs) f = 0.45 * fsOs;
        bbdLp.lowpass (f, fsOs);
    }
    wetHp.highpass (highpassFc (kWetLoad, kC16), fsOs);

    // RX2 into R40 || (R39 + CX), normalised to the current into IC5 x R40:
    //   (R39 CX s + 1) / ((RX2 (R40 + R39) + R40 R39) CX s / R40 + (RX2 + R40) / R40)
    deEmph.setAnalog (kR39 * kCX, 1.0,
                      (kRX2 * (kR40 + kR39) + kR40 * kR39) * kCX / kR40, (kRX2 + kR40) / kR40, fsOs);
    mixLp.lowpass (1.0 / (2.0 * kPi * kR33 * kCX2), fsOs);
    outHp.highpass (highpassFc (kR38, kC19), fsOs);

    slewIc2.step = slewIc5.step = kSlewRate / fsOs;

    reset();
}

void Flanger::reset()
{
    freq = freqT; depth = depthT; reso = resoT; effect = effectT; inG = inT; outG = outT;

    up.reset(); down.reset();
    inHp.reset(); couple.reset(); preEmph.reset(); fbNet.reset(); fbHp.reset();
    bbdHp.reset(); bbdLp.reset(); wetHp.reset(); deEmph.reset(); mixLp.reset(); outHp.reset();
    slewIc2.reset(); slewIc5.reset();

    std::memset (ring, 0, sizeof (ring));
    ringPos = 0;
    tickPhase = 0.0; bbdHeld = 0.0; lastBbdIn = 0.0; wetNode = 0.0;
    lfoPhase = 0.25; lfoTri = 0.0;
    lastDelay = kDelay0;
    peakBbdLevel = 0.0;
}

void Flanger::setFrequency (double v)      { freqT = clamp01 (v); }
void Flanger::setDepth (double v)          { depthT = clamp01 (v); }
void Flanger::setResonance (double v)      { resoT = clamp01 (v); }
void Flanger::setEffectOn (bool on)        { effectT = on ? 1.0 : 0.0; }
void Flanger::setInputGainDb (double dB)   { inT = std::pow (10.0, dB / 20.0); }
void Flanger::setOutputGainDb (double dB)  { outT = std::pow (10.0, dB / 20.0); }

//==============================================================================
double Flanger::tickOs (double x)
{
    // LFO (integrator + Schmitt: a linear triangle)
    lfoPhase += lfoHz (freq) / fsOs;
    if (lfoPhase >= 1.0) lfoPhase -= std::floor (lfoPhase);
    lfoTri = lfoPhase < 0.5 ? 4.0 * lfoPhase - 1.0 : 3.0 - 4.0 * lfoPhase;

    // Clock: the ramp time follows the DEPTH wiper's threshold
    const double delay = kDelay0 * (1.0 + depth * kSweep * lfoTri);
    lastDelay = delay;

    // Input buffer Q1
    double v = railClip (inHp.tick (x * inG), kQ1Swing);
    v = couple.tick (v);

    // RESONANCE: the wet node (one sample back; the BBD delay is far longer)
    const double vp = reso * fbHp.tick (fbNet.tick (wetNode));

    // IC2: out = vp - (Zt/R19) (vin - vp)
    const double ic2 = slewIc2.tick (railClip (vp - preEmph.tick (v - vp), kRail));

    // SAD1024: both sections sample at the clock, half a period apart, so the
    // joined outputs carry a sample stream at twice the clock, each output
    // value the average of the two newest buckets 512 stages back.
    const double bin = bbdClip (bbdHp.tick (ic2));
    {
        const double a = std::fabs (bin) / kBbdClip;
        if (a > peakBbdLevel) peakBbdLevel = a;
    }
    double step = 2.0 * ((kStages / 2) / delay) / fsOs;      // buckets per oversampled sample
    if (step < 1.0e-3) step = 1.0e-3;
    const double inv = 1.0 / step;
    double pos = (1.0 - tickPhase) * inv;                     // where in this sample the next tick falls
    double last = 0.0, acc = 0.0;
    int guard = 0;
    while (pos <= 1.0 && guard < 256)
    {
        acc += bbdHeld * (pos - last);
        const double s = lastBbdIn + (bin - lastBbdIn) * pos;
        ring[ringPos] = (float) s;
        const int a = (ringPos - kStages) & (kRing - 1);
        const int b = (ringPos - kStages - 1) & (kRing - 1);
        bbdHeld = 0.5 * ((double) ring[a] + (double) ring[b]);
        ringPos = (ringPos + 1) & (kRing - 1);
        last = pos;
        pos += inv;
        ++guard;
    }
    acc += bbdHeld * (1.0 - last);                             // the staircase averaged over the sample
    tickPhase = 1.0 - (pos - 1.0) * step;
    if (tickPhase < 0.0) tickPhase = 0.0;
    if (tickPhase >= 1.0) tickPhase = 0.999999;
    lastBbdIn = bin;

    // Output network and the wet node
    wetNode = wetHp.tick (bbdLp.tick (kBbdGain * acc));

    // IC5: -(R33 || CX2) (dry / R31 + wet current)
    const double i = ic2 / kR31 + deEmph.tick (wetNode) / kR40;
    const double ic5 = slewIc5.tick (railClip (-kR33 * mixLp.tick (i), kRail));
    const double wetOut = outHp.tick (ic5);

    // S1: effect in, or straight through
    return effect * wetOut + (1.0 - effect) * x;
}

void Flanger::process (const float* in, float* out, int numSamples)
{
    const double aK = 1.0 - smKnob, aR = 1.0 - smRate, aW = 1.0 - smSwitch;
    double xs[Oversampler4x::kFactor], ys[Oversampler4x::kFactor];

    for (int n = 0; n < numSamples; ++n)
    {
        freq += (freqT - freq) * aR;
        depth += (depthT - depth) * aK;
        reso += (resoT - reso) * aK;
        effect += (effectT - effect) * aW;
        inG += (inT - inG) * aK;
        outG += (outT - outG) * aK;

        float a = in[n];
        if (! (a == a)) a = 0.0f;
        up.upsample ((double) a, xs);
        for (int k = 0; k < Oversampler4x::kFactor; ++k)
            ys[k] = tickOs (xs[k]);
        out[n] = (float) (down.downsample (ys) * outG);
    }
}

} // namespace tbcc
